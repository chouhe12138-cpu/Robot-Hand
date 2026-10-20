"""Shared calibration mathematics; no camera, ROS or robot connections."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def transform(r, t):
    out = np.eye(4)
    out[:3, :3] = cv2.Rodrigues(np.asarray(r, dtype=float))[0] if np.asarray(r).size == 3 else r
    out[:3, 3] = np.asarray(t).reshape(3)
    return out


def mean_pose(poses):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_matrix(np.array([p[:3, :3] for p in poses])).mean().as_matrix()
    out[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return out


def errors(poses, ref):
    return {
        'translation_mm': [float(np.linalg.norm(p[:3, 3] - ref[:3, 3]) * 1000) for p in poses],
        'rotation_deg': [float(np.rad2deg(Rotation.from_matrix(ref[:3, :3].T @ p[:3, :3]).magnitude())) for p in poses],
    }


def solve(robot, board, method):
    r, t = cv2.calibrateHandEye([p[:3, :3] for p in robot], [p[:3, 3] for p in robot],
                               [p[:3, :3] for p in board], [p[:3, 3] for p in board], method=method)
    out = transform(r, t)
    if not np.isfinite(out).all():
        raise ValueError('non-finite solution')
    return out
