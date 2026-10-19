"""Offline 2F-85 geometry from the user-supplied description. No hardware imports.

The vendor collision STL is retained for clearance and rendering. Point/mesh
queries use a conservative convex hull PER LINK, not a box for the whole hand.
Mounting and active TCP are separate transforms. Opening is inferred from pad
geometry and mimic joints, never from an assumed linear mapping of raw POS.
"""
from copy import deepcopy
from dataclasses import dataclass
from functools import lru_cache
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import brentq, linprog
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation
import trimesh

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT/'config/robotiq_2f85.json'


def rigid(xyz, rotation=None):
    result = np.eye(4)
    result[:3, 3] = xyz
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def pose_matrix(pose):
    pose = np.asarray(pose, float)
    return rigid(pose[:3], Rotation.from_rotvec(pose[3:]).as_matrix())


def matrix_pose(matrix):
    return np.r_[matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_rotvec()]


def _name(value):
    return value.replace('${prefix}', '')


@dataclass
class LinkGeometry:
    name: str
    mesh: trimesh.Trimesh
    equations: np.ndarray
    origin: np.ndarray


class Robotiq85:
    def __init__(self, config=DEFAULT_CONFIG):
        self.config_path = Path(config).resolve()
        self.config = json.loads(self.config_path.read_text())
        if self.config.get('model') != 'robotiq_2f_85':
            raise ValueError('Expected robotiq_2f_85 configuration')
        if type(self.config.get('mounting_verified')) is not bool:
            raise ValueError('mounting_verified must be a boolean')
        for key in ('maximum_opening_m', 'opening_clearance_m', 'table_clearance_m',
                    'scene_voxel_m', 'scene_penetration_m', 'graspnet_finger_length_m'):
            value = self.config[key]
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError('Invalid positive geometry setting: '+key)
        for key, minimum in (('minimum_collision_points', 1), ('closure_samples', 2)):
            if type(self.config[key]) is not int or self.config[key] < minimum:
                raise ValueError('Invalid integer geometry setting: '+key)
        if not 0 < self.config['max_top_down_tilt_deg'] <= 90:
            raise ValueError('Invalid top-down tilt limit')
        self.description = (ROOT/self.config['description_root']).resolve()
        self.macro_path = self.description/'urdf/robotiq_2f_85_macro.urdf.xacro'
        root = ET.parse(self.macro_path).getroot()
        self.macro = root.find('{http://wiki.ros.org/xacro}macro')
        self.joints = self.macro.findall('joint')
        self.links = {}
        self.source_hashes = {}
        self._record_source(self.macro_path)
        for link in self.macro.findall('link'):
            collision = link.find('collision')
            path = self.resolve_mesh(collision.find('geometry/mesh').get('filename'))
            mesh = trimesh.load(path, process=True)
            self._record_source(path)
            hull = ConvexHull(mesh.vertices)
            self.links[_name(link.get('name'))] = LinkGeometry(
                _name(link.get('name')), mesh, hull.equations, self.xml_origin(collision.find('origin')))
        adapter = self.description/'meshes/collision/2f_85/ur_to_robotiq_adapter.stl'
        self.adapter_mesh = trimesh.load(adapter, process=True)
        self.adapter_equations = ConvexHull(self.adapter_mesh.vertices).equations
        self._record_source(adapter)
        mounting = np.asarray(self.config['T_tool0_gripper_base_xyz_rpy'], float)
        tcp = np.asarray(self.config['active_tcp_tool0_xyz_rotvec'], float)
        if mounting.shape != (6,) or tcp.shape != (6,) or not np.isfinite(np.r_[mounting, tcp]).all():
            raise ValueError('Invalid mounting/TCP transform')
        self.T_tool0_base = rigid(mounting[:3], Rotation.from_euler('xyz', mounting[3:]).as_matrix())
        self.T_tool0_tcp = pose_matrix(tcp)
        self.T_tcp_base = np.linalg.inv(self.T_tool0_tcp)@self.T_tool0_base
        self.T_tcp_adapter = np.linalg.inv(self.T_tool0_tcp)@rigid([0, 0, 0], self.T_tool0_base[:3, :3])
        self.closed = float(self.config['closed_joint_rad'])
        if not 0 < self.closed <= .8:
            raise ValueError('Closed joint outside vendor URDF limits')
        self.scene_points = self.scene_tree = None

    def _record_source(self, path):
        self.source_hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()

    def resolve_mesh(self, name):
        prefix = 'package://robotiq_description/'
        if not name.startswith(prefix):
            raise ValueError('Unexpected mesh package')
        path = (self.description/name[len(prefix):]).resolve()
        if not path.is_relative_to(self.description):
            raise ValueError('Mesh escapes description package')
        return path

    @staticmethod
    def xml_origin(element):
        if element is None:
            return np.eye(4)
        return rigid(np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
                     Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix())

    def frames(self, angle):
        if not np.isfinite(angle) or not -.000001 <= angle <= .800001:
            raise ValueError('Joint angle outside URDF limits')
        frames = {'robotiq_85_base_link': np.eye(4)}
        pending = [j for j in self.joints if _name(j.find('child').get('link')) != 'robotiq_85_base_link']
        while pending:
            advanced = False
            for joint in pending[:]:
                parent = _name(joint.find('parent').get('link'))
                if parent not in frames:
                    continue
                transform = self.xml_origin(joint.find('origin'))
                if joint.get('type') == 'revolute':
                    mimic = joint.find('mimic')
                    factor = 1. if mimic is None else float(mimic.get('multiplier', '1'))
                    offset = 0. if mimic is None else float(mimic.get('offset', '0'))
                    q = factor*angle+offset
                    limits = joint.find('limit')
                    if not float(limits.get('lower'))-1e-6 <= q <= float(limits.get('upper'))+1e-6:
                        raise ValueError('Mimic joint outside URDF limits')
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                    transform = transform@rigid([0, 0, 0], Rotation.from_rotvec(q*axis).as_matrix())
                frames[_name(joint.find('child').get('link'))] = frames[parent]@transform
                pending.remove(joint)
                advanced = True
            if not advanced:
                raise ValueError('Disconnected vendor gripper joints')
        return frames

    @lru_cache(maxsize=256)
    def pads(self, angle):
        frames = self.frames(angle)
        pads = []
        for side in ('left', 'right'):
            name = f'robotiq_85_{side}_finger_tip_link'
            link = self.links[name]
            transform = frames[name]@link.origin
            points = link.mesh.vertices@transform[:3, :3].T+transform[:3, 3]
            edge = points[:, 0].min() if side == 'left' else points[:, 0].max()
            # Inner pad is a planar surface. Use its bounds, not the vertex mean
            # (mesh tessellation would otherwise bias the contact reference).
            surface = points[np.abs(points[:, 0]-edge) < .00002]
            pads.append({'x': float(edge), 'y': [float(surface[:, 1].min()), float(surface[:, 1].max())],
                         'z': [float(surface[:, 2].min()), float(surface[:, 2].max())]})
        return pads

    def opening(self, angle):
        pads = self.pads(float(angle))
        return pads[0]['x']-pads[1]['x']

    @lru_cache(maxsize=256)
    def angle_for_opening(self, width):
        if not np.isfinite(width) or not self.opening(self.closed)-1e-7 <= width <= self.opening(0.)+1e-7:
            raise ValueError(f'Opening {width:.6f}m outside vendor kinematic range')
        return float(brentq(lambda q: self.opening(q)-np.clip(width, self.opening(self.closed), self.opening(0.)),
                            0., self.closed, xtol=1e-12))

    def contact_center(self, angle):
        pads = self.pads(float(angle))
        return np.array([(pads[0]['x']+pads[1]['x'])/2,
                         np.mean([*pads[0]['y'], *pads[1]['y']]),
                         np.mean([*pads[0]['z'], *pads[1]['z']])])

    @lru_cache(maxsize=128)
    def parts_tcp(self, angle):
        frames = self.frames(float(angle))
        result = []
        for name, link in self.links.items():
            transform = self.T_tcp_base@frames[name]@link.origin
            vertices = link.mesh.vertices@transform[:3, :3].T+transform[:3, 3]
            result.append((name, vertices, link.equations, transform))
        vertices = self.adapter_mesh.vertices@self.T_tcp_adapter[:3, :3].T+self.T_tcp_adapter[:3, 3]
        result.append(('ur_to_robotiq_adapter', vertices, self.adapter_equations, self.T_tcp_adapter))
        return result

    def grasp_tcp_pose(self, grasp, T_world_camera, current_rotation):
        """Map MODEL contact reference to real pads, then to unchanged active TCP.

        GraspNet fingers span approach coordinates [depth-.06, depth]. Their
        midpoint is aligned to the actual inner pad midpoint at predicted width.
        Both 180-degree parallel-jaw symmetries keep the same model contact.
        """
        angle = self.angle_for_opening(float(grasp.width))
        reference = self.contact_center(angle)
        rotation = T_world_camera[:3, :3]@grasp.rotation_matrix
        z = rotation[:, 0]/np.linalg.norm(rotation[:, 0])
        x = rotation[:, 1]-z*(z@rotation[:, 1])
        x /= np.linalg.norm(x)
        primary = np.column_stack([x, np.cross(z, x), z])
        center = T_world_camera[:3, :3]@grasp.translation+T_world_camera[:3, 3]
        center = center+(float(grasp.depth)-self.config['graspnet_finger_length_m']/2)*z
        options = []
        T_base_tcp = np.linalg.inv(self.T_tool0_base)@self.T_tool0_tcp
        for flip in (False, True):
            orient = primary@np.diag([-1., -1., 1.]) if flip else primary
            base = rigid(center-orient@reference, orient)
            tcp = base@T_base_tcp
            distance = Rotation.from_matrix(current_rotation.T@tcp[:3, :3]).magnitude()
            options.append((distance, matrix_pose(tcp)))
        return min(options, key=lambda x: x[0])[1]

    def provenance(self):
        return {'config': str(self.config_path.relative_to(ROOT)) if self.config_path.is_relative_to(ROOT) else str(self.config_path),
                'config_sha256': hashlib.sha256(self.config_path.read_bytes()).hexdigest(),
                'source_sha256': self.source_hashes, 'mounting_verified': self.config['mounting_verified'],
                'scene_collision_method': 'per-link convex hulls from supplied collision STL',
                'surface_clearance_method': 'transformed original STL vertices',
                'grasp_reference': self.config['grasp_reference']}

    def set_scene(self, points_world):
        points = np.asarray(points_world, float)
        points = points[np.isfinite(points).all(axis=1)]
        voxel = float(self.config['scene_voxel_m'])
        _, indices = np.unique(np.floor(points/voxel).astype(np.int64), axis=0, return_index=True)
        self.scene_points = points[indices]
        self.scene_tree = cKDTree(self.scene_points)

    def screen(self, tcp_pose, angle, plane=None, scene=True):
        transform = pose_matrix(tcp_pose)
        minimum = float('inf')
        collisions = []
        for name, vertices, equations, local_transform in self.parts_tcp(float(angle)):
            world = vertices@transform[:3, :3].T+transform[:3, 3]
            if plane is not None:
                minimum = min(minimum, float(np.min(world@plane[:3]+plane[3])))
            if not scene or self.scene_tree is None:
                continue
            low, high = world.min(axis=0), world.max(axis=0)
            midpoint = (low+high)/2
            indices = self.scene_tree.query_ball_point(midpoint, np.linalg.norm(high-low)/2)
            if not indices:
                continue
            points = self.scene_points[indices]
            points = points[np.all((points >= low) & (points <= high), axis=1)]
            if not len(points):
                continue
            full = transform@local_transform
            local = (points-full[:3, 3])@full[:3, :3]
            # A point must penetrate every hull plane by this tolerance. Boundary
            # contact is not treated as penetration; no arbitrary object-wide mask.
            tolerance = float(self.config['scene_penetration_m'])
            inside = np.all(local@equations[:, :3].T+equations[:, 3] < -tolerance, axis=1)
            count = int(inside.sum())
            if count >= int(self.config['minimum_collision_points']):
                collisions.append({'link': name, 'penetrating_points': count})
        clearance = None if plane is None else minimum*1000
        if plane is not None and minimum < float(self.config['table_clearance_m']):
            return {'ok': False, 'reason': '2F85 collision mesh/table clearance', 'table_clearance_mm': clearance,
                    'scene_collisions': collisions}
        return {'ok': not collisions, 'reason': '2F85 link/scene penetration' if collisions else '',
                'table_clearance_mm': clearance, 'scene_collisions': collisions}

    def screen_closure(self, tcp_pose, start_angle, end_angle, plane=None):
        results = []
        for angle in np.linspace(start_angle, end_angle, int(self.config['closure_samples'])):
            check = self.screen(tcp_pose, float(angle), plane)
            results.append({'joint_rad': float(angle), **check})
            if not check['ok']:
                return {'ok': False, 'reason': check['reason'], 'samples': results}
        return {'ok': True, 'samples': results}

    def screen_arm(self, tcp_pose, angle, arm_frames, arm_meshes):
        """Conservative convex-hull overlap with non-adjacent robot links."""
        tcp = pose_matrix(tcp_pose)
        for name, vertices, equations, local_transform in self.parts_tcp(float(angle)):
            full = tcp@local_transform
            gv = vertices@tcp[:3, :3].T+tcp[:3, 3]
            gn = equations[:, :3]@full[:3, :3].T
            go = equations[:, 3]-gn@full[:3, 3]
            for arm, (av, ae) in arm_meshes.items():
                # These links physically meet at the flange, not self obstacles.
                if arm == 'wrist_3_link' and name in ('robotiq_85_base_link', 'ur_to_robotiq_adapter'):
                    continue
                frame = arm_frames[arm]
                world = av@frame[:3, :3].T+frame[:3, 3]
                if np.any(gv.max(axis=0) < world.min(axis=0)) or np.any(world.max(axis=0) < gv.min(axis=0)):
                    continue
                an = ae[:, :3]@frame[:3, :3].T
                ao = ae[:, 3]-an@frame[:3, 3]
                fit = linprog(np.zeros(3), A_ub=np.r_[gn, an], b_ub=-np.r_[go, ao],
                              bounds=[(None, None)]*3, method='highs')
                if fit.success:
                    return {'ok': False, 'reason': f'2F85/arm hull overlap: {name}/{arm}'}
                if fit.status != 2:
                    raise RuntimeError('Gripper/arm overlap solver failed: '+fit.message)
        return {'ok': True}

    def screen_arm_closure(self, tcp_pose, start, end, arm_frames, arm_meshes):
        for angle in np.linspace(start, end, self.config['closure_samples']):
            result = self.screen_arm(tcp_pose, float(angle), arm_frames, arm_meshes)
            if not result['ok']:
                return dict(result, joint_rad=float(angle))
        return {'ok': True}

    def export_urdf(self, output, robot_urdf=None):
        root = ET.parse(robot_urdf).getroot() if robot_urdf else ET.Element('robot', name='robotiq_2f85_offline')
        if robot_urdf is None:
            ET.SubElement(root, 'link', name='tool0')
        # Description only: retain robot's existing ros2_control; never append the
        # vendor serial driver or simulation ros2_control block.
        for link in self.macro.findall('link'):
            element = deepcopy(link)
            element.set('name', _name(element.get('name')))
            for mesh in element.findall('.//mesh'):
                mesh.set('filename', str(self.resolve_mesh(mesh.get('filename'))))
            for collision in element.findall('collision'):
                for surface in collision.findall('surface'):
                    collision.remove(surface)
            root.append(element)
        for joint in self.joints:
            element = deepcopy(joint)
            if _name(element.find('child').get('link')) == 'robotiq_85_base_link':
                continue
            for item in element.iter():
                for key, value in item.attrib.copy().items():
                    item.set(key, _name(value))
            root.append(element)
        link = ET.SubElement(root, 'link', name='robotiq_85_adapter_link')
        for kind in ('visual', 'collision'):
            shape = ET.SubElement(link, kind)
            geometry = ET.SubElement(shape, 'geometry')
            ET.SubElement(geometry, 'mesh', filename=str(self.description/'meshes/collision/2f_85/ur_to_robotiq_adapter.stl'))
        for name, parent, child, matrix in [
            ('tool0_to_robotiq_adapter', 'tool0', 'robotiq_85_adapter_link', rigid([0, 0, 0], self.T_tool0_base[:3, :3])),
            ('robotiq_85_base_joint', 'robotiq_85_adapter_link', 'robotiq_85_base_link',
             rigid(self.T_tool0_base[:3, :3].T@self.T_tool0_base[:3, 3]))]:
            joint = ET.SubElement(root, 'joint', name=name, type='fixed')
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=child)
            ET.SubElement(joint, 'origin', xyz=' '.join(map(str, matrix[:3, 3])),
                          rpy=' '.join(map(str, Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz'))))
        ET.SubElement(root, 'link', name='robotiq_active_tcp')
        joint = ET.SubElement(root, 'joint', name='tool0_to_robotiq_active_tcp', type='fixed')
        ET.SubElement(joint, 'parent', link='tool0')
        ET.SubElement(joint, 'child', link='robotiq_active_tcp')
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, self.T_tool0_tcp[:3, 3])),
                      rpy=' '.join(map(str, Rotation.from_matrix(self.T_tool0_tcp[:3, :3]).as_euler('xyz'))))
        tree = ET.ElementTree(root)
        ET.indent(tree, space='  ')
        tree.write(output, encoding='utf-8', xml_declaration=True)
