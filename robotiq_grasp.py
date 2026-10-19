"""Shared 2F-85 grasp mapping, collision screening and Open3D meshes.

GraspNet continues to predict its trained grasp representation. The real pad
midpoint, mounting transform and active TCP come from Robotiq85's configuration.
All geometry below uses the same frame as the input scene/grasp coordinates.
"""
from functools import lru_cache
import logging
import threading

import numpy as np
from scipy.spatial.transform import Rotation

from robotiq_2f85_model import Robotiq85, pose_matrix

_models = threading.local()


def gripper_model():
    # Each perception/viewer thread owns its scene and geometry caches.
    if not hasattr(_models, 'gripper'):
        _models.gripper = Robotiq85()
    return _models.gripper


def grasp_tcp_pose(grasp, camera_transform, current_rotation):
    return gripper_model().grasp_tcp_pose(grasp, camera_transform, current_rotation)


def filter_grasps(grasps, scene_points, approach_distance, max_candidates=120):
    """Check real links while approaching open and closing to predicted width.

    Both parallel-jaw orientations must clear the observed scene, so execution
    can retain its nearest-orientation choice. These sampled checks do not
    certify the arm path or obstacles absent from the camera point cloud.
    """
    model = gripper_model()
    model.set_scene(scene_points)
    keep = []
    width_rejected = 0
    clearance_rejected = 0
    minimum_width = max(0., model.opening(model.closed))
    maximum_width = min(model.config['maximum_opening_m'], model.opening(0.))
    for index, grasp in enumerate(grasps[:max_candidates]):
        if not minimum_width <= grasp.width <= maximum_width - model.config['opening_clearance_m']:
            width_rejected += 1
            continue
        angle = model.angle_for_opening(float(grasp.width))
        primary = np.column_stack([grasp.rotation_matrix[:, 1],
                                   grasp.rotation_matrix[:, 2],
                                   grasp.rotation_matrix[:, 0]])
        clear = True
        for rotation in (primary, primary @ np.diag([-1., -1., 1.])):
            pose = model.grasp_tcp_pose(grasp, np.eye(4), rotation)
            if not model.screen_closure(pose, 0., angle)['ok']:
                clear = False
                break
            direction = Rotation.from_rotvec(pose[3:]).as_matrix()[:, 2]
            for distance in np.linspace(0., approach_distance, 5):
                approach = pose.copy()
                approach[:3] -= distance * direction
                if not model.screen(approach, 0.)['ok']:
                    clear = False
                    break
            if not clear:
                break
        if clear:
            keep.append(index)
        else:
            clearance_rejected += 1
    logging.info(
        "2F-85 filter: kept=%d, width_rejected=%d, clearance_rejected=%d",
        len(keep), width_rejected, clearance_rejected)
    return grasps[np.asarray(keep, dtype=int)]


@lru_cache(maxsize=128)
def _local_meshes(angle):
    """Original collision STL surfaces placed in the active TCP frame."""
    import open3d as o3d
    model = gripper_model()
    meshes = []
    for name, vertices, _, _ in model.parts_tcp(angle):
        faces = (model.adapter_mesh if name == 'ur_to_robotiq_adapter'
                 else model.links[name].mesh).faces
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.25, 0.45, 0.7])
        meshes.append(mesh)
    return meshes


def grasp_meshes(grasps):
    """Render the same physical geometry and contact mapping used for screening."""
    from copy import deepcopy
    model = gripper_model()
    result = []
    maximum = min(model.config['maximum_opening_m'], model.opening(0.))
    minimum = max(0., model.opening(model.closed))
    for grasp in grasps:
        # Raw debug predictions can exceed the physical stroke. Omit those.
        if not minimum <= grasp.width <= maximum:
            continue
        angle = model.angle_for_opening(float(grasp.width))
        pose = model.grasp_tcp_pose(grasp, np.eye(4), np.eye(3))
        transform = pose_matrix(pose)
        for source in _local_meshes(angle):
            mesh = deepcopy(source)
            mesh.transform(transform)
            result.append(mesh)
    return result
