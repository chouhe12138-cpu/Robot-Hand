"""Frame-aligned CAD observation outputs; no camera or robot connections."""

import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d


def depth_to_points(depth_m, K, color_bgr, mask=None):
    """Return unfiltered XYZ (metres) and RGB samples in the left optical frame."""
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if mask is not None:
        valid &= np.asarray(mask, dtype=bool)
    yy, xx = np.nonzero(valid)
    z = depth_m[yy, xx].astype(np.float32)
    points = np.stack(((xx - K[0, 2]) * z / K[0, 0],
                       (yy - K[1, 2]) * z / K[1, 1], z), axis=-1)
    colors = color_bgr[yy, xx][:, ::-1].copy()
    return points, colors


def save_cloud(path, depth_m, K, color_bgr, mask=None):
    points, colors = depth_to_points(depth_m, K, color_bgr, mask)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255)
    o3d.io.write_point_cloud(str(path), cloud)
    return int(len(points))


def mask_overlay(left_bgr, masks, prompts):
    """Human-reviewable overlay; original images and binary masks stay unmodified."""
    view = left_bgr.copy()
    for obj_id, color in ((1, (255, 80, 60)), (2, (40, 220, 40))):
        mask = masks.get(obj_id)
        if mask is not None:
            colored = view.copy()
            colored[mask] = color
            view = cv2.addWeighted(view, 0.7, colored, 0.3, 0)
            contours, _ = cv2.findContours(mask.astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(view, contours, -1, color, 1)
    for item in prompts:
        color = (255, 80, 60) if item['id'] == 1 else (40, 220, 40)
        if 'bbox' in item:
            x1, y1, x2, y2 = item['bbox']
            cv2.rectangle(view, (x1, y1), (x2, y2), color, 2)
        else:
            cv2.circle(view, tuple(item['point']), 5, color, 2)
    return view


def save_sample(folder, *, left_full_bgr, right_full_bgr, left_bgr, right_bgr,
                depth_m, disparity, K, masks, prompts, metadata):
    """Save aligned assets from a single frozen stereo frame."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(folder / 'left_full.png'), left_full_bgr)
    cv2.imwrite(str(folder / 'right_full.png'), right_full_bgr)
    cv2.imwrite(str(folder / 'left.png'), left_bgr)
    cv2.imwrite(str(folder / 'right.png'), right_bgr)
    cv2.imwrite(str(folder / 'depth_mm.png'), np.clip(depth_m * 1000, 0, 65535).astype(np.uint16))
    np.save(folder / 'disparity_px.npy', disparity.astype(np.float32))
    counts = {'full': save_cloud(folder / 'pointcloud_full.ply', depth_m, K, left_bgr)}
    for obj_id, name in ((1, 'hand'), (2, 'object')):
        if obj_id in masks:
            mask = masks[obj_id].astype(bool)
            cv2.imwrite(str(folder / f'mask_{name}.png'), mask.astype(np.uint8) * 255)
            counts[name] = save_cloud(folder / f'pointcloud_{name}.ply', depth_m, K, left_bgr, mask)
    cv2.imwrite(str(folder / 'mask_preview.png'), mask_overlay(left_bgr, masks, prompts))
    metadata['valid_point_counts'] = counts
    metadata['mask_pixel_counts'] = {str(k): int(m.sum()) for k, m in masks.items()}
    (folder / 'meta.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return counts
