#!/usr/bin/env python3
"""Minimal geometry-only Cat CAD -> partial-depth registration probe.

Requires numpy and open3d. No robot, ROS, RGB matching, GraspNet or camera access.
The provided Cat mesh must geometrically match the eventual physical object.
"""
import argparse
from pathlib import Path
import numpy as np
import open3d as o3d


def sample_mesh(path, scale, count):
    mesh = o3d.io.read_triangle_mesh(str(path))
    if mesh.is_empty() or not mesh.has_triangles():
        raise ValueError('Expected a triangle-mesh PLY/OBJ/STL with faces')
    mesh.scale(scale, center=(0, 0, 0))
    mesh.translate(-mesh.get_center())  # Keep the centered model frame for both modes
    diameter = np.linalg.norm(mesh.get_max_bound() - mesh.get_min_bound())
    if not 0.015 <= diameter <= 1.0:
        raise ValueError(f'CAD diagonal {diameter:.4f} m is implausible; check --cad-scale')
    cloud = mesh.sample_points_uniformly(number_of_points=count)
    return mesh, cloud


def render_partial(mesh, voxel):
    """Raycast an RGB-free depth view from the *same* mesh; no invisible faces."""
    diag = np.linalg.norm(mesh.get_max_bound() - mesh.get_min_bound())
    eye = np.array([0.0, -0.25 * diag, 2.7 * diag])
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=60.0, center=[0.0, 0.0, 0.0], eye=eye.tolist(),
        up=[0.0, 1.0, 0.0], width_px=320, height_px=240,
    ).numpy()
    t = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()
    valid = np.isfinite(t)
    xyz = rays[..., :3][valid] + rays[..., 3:][valid] * t[valid, None]
    if len(xyz) < 40:
        raise RuntimeError('Raycast returned too few visible surface points')
    # Small position noise mimics imperfect depth, not an entire realistic camera model.
    rng = np.random.default_rng(17)
    xyz += rng.normal(0, voxel / 8, xyz.shape)
    observed = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    transform = np.eye(4)
    transform[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0.15, -0.22, 0.13))
    transform[:3, 3] = [0.015, -0.01, 0.025]
    observed.transform(transform)
    return observed, transform


def features(cloud, voxel):
    down = cloud.voxel_down_sample(voxel)
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*5, max_nn=80))
    return down, fpfh


def align(cad, observed, voxel):
    src, fs = features(cad, voxel)
    tgt, ft = features(observed, voxel)
    if len(src.points) < 20 or len(tgt.points) < 20:
        raise RuntimeError('Too few downsampled points; reduce --voxel')
    reg = o3d.pipelines.registration
    coarse = reg.registration_ransac_based_on_feature_matching(
        src, tgt, fs, ft, mutual_filter=True,
        max_correspondence_distance=voxel*2,
        estimation_method=reg.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[reg.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                  reg.CorrespondenceCheckerBasedOnDistance(voxel*2)],
        criteria=reg.RANSACConvergenceCriteria(30000, 0.999),
    )
    observed.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2, max_nn=30))
    cad.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2, max_nn=30))
    refined = reg.registration_icp(
        cad, observed, voxel*2, coarse.transformation,
        reg.TransformationEstimationPointToPlane(),
        reg.ICPConvergenceCriteria(max_iteration=80))
    return coarse, refined


def coverage(observed, aligned, voxel):
    tree = o3d.geometry.KDTreeFlann(aligned)
    distances = []
    for p in np.asarray(observed.points):
        n, _, d2 = tree.search_knn_vector_3d(p, 1)
        if n:
            distances.append(np.sqrt(d2[0]))
    distances = np.asarray(distances)
    return np.mean(distances < voxel), np.sqrt(np.mean(distances**2))


def main():
    pa = argparse.ArgumentParser(description=__doc__)
    pa.add_argument('--cad', required=True, type=Path, help='LINEMOD Cat mesh, e.g. models/obj_000006.ply')
    pa.add_argument('--observed', type=Path, help='Optional real, object-only pointcloud in metres')
    pa.add_argument('--cad-scale', type=float, default=0.001, help='BOP/LINEMOD models commonly use mm; mm->m = 0.001')
    pa.add_argument('--voxel', type=float, default=0.0035, help='Voxel size in metres')
    pa.add_argument('--output-dir', type=Path, default=Path('cat_probe_out'))
    pa.add_argument('--show', action='store_true')
    args = pa.parse_args()
    mesh, cad = sample_mesh(args.cad, args.cad_scale, 30000)
    gt = None
    if args.observed:
        observed = o3d.io.read_point_cloud(str(args.observed))
        if observed.is_empty():
            raise ValueError('Observed point cloud is empty')
    else:
        observed, gt = render_partial(mesh, args.voxel)
    coarse, result = align(cad, observed, args.voxel)
    aligned = o3d.geometry.PointCloud(cad)
    aligned.transform(result.transformation)
    covered, rmse = coverage(observed, aligned, args.voxel*2)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(args.output_dir/'aligned_cat.ply'), aligned)
    o3d.io.write_point_cloud(str(args.output_dir/'observation.ply'), observed)
    np.savetxt(args.output_dir/'cad_to_observation.txt', result.transformation, fmt='%.8f')
    print('Coarse feature-matching fitness:', round(coarse.fitness, 4))
    print('Refined full-CAD->partial fitness:', round(result.fitness, 4))
    print('Observation covered by aligned CAD within tolerance:', f'{100*covered:.1f}%')
    print('Observation-to-CAD nearest distance RMSE:', f'{1000*rmse:.2f} mm')
    if gt is not None:
        relative = result.transformation[:3, :3] @ gt[:3, :3].T
        angle = np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1)))
        translation_mm = 1000*np.linalg.norm(result.transformation[:3, 3] - gt[:3, 3])
        print(f'Synthetic GT errors: rotation {angle:.2f} deg, translation {translation_mm:.2f} mm')
        print('Note: synthetic registration is NOT physical grasp validation.')
    if args.show:
        aligned.paint_uniform_color([0.1, 0.5, 0.9])
        observed.paint_uniform_color([0.9, 0.3, 0.2])
        o3d.visualization.draw_geometries([observed, aligned])


if __name__ == '__main__':
    main()
