"""Shared grasp candidate screening and READY validity for R2H/H2R.

This module does not command hardware. Callers retain responsibility for their
own camera direction filter, hand/scene segmentation, and motion execution.
"""
import time

import numpy as np

from robotiq_grasp import filter_grasps


def filter_by_camera_direction(
    grasps, camera_rot_mat, max_tilt_deg, max_closing_forward_cos=None,
):
    """Filter by camera-axis approach angle, retaining each task's optional closing-axis rule."""
    if len(grasps) == 0:
        return grasps
    camera_forward_base = camera_rot_mat[:, 2]
    camera_forward_base = camera_forward_base / (np.linalg.norm(camera_forward_base) + 1e-8)
    max_tilt_cos = np.cos(np.deg2rad(max_tilt_deg))
    keep = []
    for grasp in grasps:
        rot_mat = grasp.rotation_matrix
        approach_camera = rot_mat[:, 0]
        approach_camera = approach_camera / (np.linalg.norm(approach_camera) + 1e-8)
        approach_base = camera_rot_mat @ approach_camera
        approach_base = approach_base / (np.linalg.norm(approach_base) + 1e-8)
        cos_tilt = float(np.clip(np.dot(approach_base, camera_forward_base), -1.0, 1.0))
        allowed = cos_tilt >= max_tilt_cos
        if max_closing_forward_cos is not None:
            closing_camera = rot_mat[:, 1]
            closing_camera = closing_camera / (np.linalg.norm(closing_camera) + 1e-8)
            closing_base = camera_rot_mat @ closing_camera
            closing_base = closing_base / (np.linalg.norm(closing_base) + 1e-8)
            allowed = allowed and abs(np.dot(closing_base, camera_forward_base)) <= max_closing_forward_cos
        keep.append(allowed)
    return grasps[np.asarray(keep, dtype=bool)]


def prepare_ready_grasps(
    direction_filtered, scene_points, approach_distance,
    build_plan, validate_plan, cancelled=None, on_rejected=None,
):
    """Screen physical 2F-85 symmetries and endpoint IK, preserving the chosen pose.

    The order of input grasps is preserved. Both valid gripper symmetries are
    independent candidates. Returns None when a selection is cancelled.
    """
    physical, physical_flips = filter_grasps(
        direction_filtered, scene_points, approach_distance, return_flips=True)
    reachable = []
    rejected = []
    first_plan = None
    for index, candidate in enumerate(physical):
        if cancelled is not None and cancelled():
            return None
        flip = physical_flips[index]
        try:
            plan = build_plan(candidate, force_flip=flip)
            validate_plan(plan)
        except ValueError as error:
            rejection = {"candidate": index, "force_flip": flip, "reason": str(error)}
            rejected.append(rejection)
            if on_rejected is not None:
                on_rejected(index, flip, error)
            continue
        reachable.append(index)
        if first_plan is None:
            first_plan = plan
    if cancelled is not None and cancelled():
        return None
    selected = physical[np.asarray(reachable, dtype=int)]
    selected_flips = [physical_flips[i] for i in reachable]
    return selected, selected_flips, first_plan, len(physical), rejected


def ready_rejection_reason(
    expected_q, ready_at, actual_q, *,
    now=None, camera_frame_times=(), joint_velocities=None,
    max_age_s=30.0, max_camera_age_s=2.0, max_joint_delta_rad=.01,
    max_joint_velocity_rad_s=.003,
):
    """Check whether a READY grasp is still eligible for operator confirmation.

    Returns a reason string or None. These are staleness/stationary checks,
    not a substitute for hand collision or continuous-path validation.
    """
    if now is None:
        now = time.monotonic()
    if expected_q is None or now - ready_at > max_age_s:
        return "READY expired"
    if any(now - frame_time > max_camera_age_s for frame_time in camera_frame_times):
        return "Camera frame is stale"
    if np.max(np.abs(np.asarray(actual_q) - np.asarray(expected_q))) > max_joint_delta_rad:
        return "Robot moved since READY"
    if (joint_velocities is not None
            and np.max(np.abs(np.asarray(joint_velocities))) > max_joint_velocity_rad_s):
        return "Robot is moving"
    return None
