"""Hardware-independent checks and eye-to-hand solving (metres, radians)."""
from dataclasses import dataclass, asdict
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from object_hand.calibration import transform, solve, mean_pose, errors


@dataclass(frozen=True)
class BoardSpec:
    columns: int = 7
    rows: int = 5
    square_m: float = .030
    marker_m: float = .022
    dictionary: str = 'DICT_4X4_50'

    def create(self):
        if self.columns < 3 or self.rows < 3 or not 0 < self.marker_m < self.square_m:
            raise ValueError('Invalid ChArUco board dimensions')
        if not hasattr(cv2.aruco, self.dictionary):
            raise ValueError('Unknown ArUco dictionary: '+self.dictionary)
        return cv2.aruco.CharucoBoard((self.columns, self.rows), self.square_m, self.marker_m,
                                     cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)))

    def metadata(self):
        return asdict(self)


def detect_board(image, k, distortion, spec, min_corners=8, max_rms_px=1.5):
    board = spec.create()
    corners, ids, _, marker_ids = cv2.aruco.CharucoDetector(board).detectBoard(image)
    n = 0 if ids is None else len(ids)
    result = {'accepted': False, 'corners': n, 'reason': 'Show a sharp, unobstructed board'}
    if n < min_corners or board.checkCharucoCornersCollinear(ids):
        return result
    markers = [] if marker_ids is None else marker_ids.flatten().tolist()
    if len(markers) != len(set(markers)):
        result['reason'] = 'Duplicate marker IDs; show only one board'
        return result
    obj, pixels = board.matchImagePoints(corners, ids)
    ok, rv, tv = cv2.solvePnP(obj, pixels, k, distortion)
    if not ok or not np.isfinite(np.r_[rv.ravel(), tv.ravel()]).all():
        result['reason'] = 'PnP failed'
        return result
    projected, _ = cv2.projectPoints(obj, rv, tv, k, distortion)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-pixels.reshape(-1, 2))**2, axis=1))))
    cb = transform(rv, tv)
    in_front = np.all((np.asarray(obj).reshape(-1, 3) @ cb[:3, :3].T+cb[:3, 3])[:, 2] > 0)
    result.update(accepted=bool(rms <= max_rms_px and in_front), reason='OK' if rms <= max_rms_px and in_front else 'PnP quality rejected',
                  reprojection_rms_px=rms, charuco_corners=corners.tolist(), charuco_ids=ids.flatten().tolist(),
                  T_camera_board=cb.tolist(), rvec=rv.ravel().tolist(), tvec=tv.ravel().tolist())
    return result


def pose_matrix(pose):
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('TCP pose must contain six finite numbers')
    return transform(values[3:], values[:3])


def new_pose(pose, saved, min_rotation_deg=6., min_translation_m=.025):
    """Compare to every saved pose, not just the previous one."""
    current = pose_matrix(pose)
    for previous in saved:
        other = pose_matrix(previous)
        angle = float(np.rad2deg(Rotation.from_matrix(other[:3, :3].T @ current[:3, :3]).magnitude()))
        distance = float(np.linalg.norm(other[:3, 3]-current[:3, 3]))
        if not (angle >= min_rotation_deg or (distance >= min_translation_m and angle >= 2.)):
            return False
    return True


def observability(robot_poses):
    if len(robot_poses) < 3:
        return {'ok': False, 'rotation_span_deg': 0., 'second_axis_rms_deg': 0.}
    ref = robot_poses[0][:3, :3]
    rv = np.array([Rotation.from_matrix(ref.T @ p[:3, :3]).as_rotvec() for p in robot_poses[1:]])
    singular = np.linalg.svd(rv, compute_uv=False)/np.sqrt(len(rv))
    span = float(np.rad2deg(np.max(np.linalg.norm(rv, axis=1))))
    second = float(np.rad2deg(singular[1]))
    return {'ok': bool(span >= 20 and second >= 5), 'rotation_span_deg': span, 'second_axis_rms_deg': second}


def error_stats(poses, reference):
    return {key: {'rms': float(np.sqrt(np.mean(np.square(values)))), 'max': float(max(values)), 'values': values}
            for key, values in errors(poses, reference).items()}


def valid_transform(value):
    return (np.asarray(value).shape == (4, 4) and np.isfinite(value).all()
            and np.allclose(value[3], [0, 0, 0, 1], atol=1e-8)
            and np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-5)
            and np.isclose(np.linalg.det(value[:3, :3]), 1., atol=1e-5))


def solve_eye_to_hand(robot_poses, camera_boards, required_samples=20,
                      max_fit_mm=8., max_holdout_mm=15., max_rotation_deg=2., max_loo_mm=10.):
    """B_i Y = X C_i; invert B_i for OpenCV, returning X = T_ur_base_camera.

    Quality limits are internal consistency checks, not a physical accuracy guarantee.
    """
    if len(robot_poses) != len(camera_boards) or len(robot_poses) < max(8, required_samples):
        raise ValueError(f'Need at least {max(8, required_samples)} paired samples')
    if not all(valid_transform(p) for p in [*robot_poses, *camera_boards]):
        raise ValueError('Invalid input rigid transform')
    diversity = observability(robot_poses)
    if not diversity['ok']:
        raise ValueError('Insufficient orientation diversity: tilt around at least two different axes; '+str(diversity))
    limits = {'fit_rms_mm': max_fit_mm, 'holdout_max_mm': max_holdout_mm,
              'rotation_rms_deg': max_rotation_deg, 'leave_one_out_max_mm': max_loo_mm}
    if not all(np.isfinite(v) and v > 0 for v in limits.values()):
        raise ValueError('Quality limits must be finite and positive')
    inverse_robot = [np.linalg.inv(p) for p in robot_poses]
    held = [i for i in range(len(robot_poses)) if i % 5 == 4]
    train = [i for i in range(len(robot_poses)) if i not in held]
    # Do not put the only second-axis pose entirely into the holdout set.
    if not observability([robot_poses[i] for i in train])['ok']:
        for phase in range(4):
            alternate = [i for i in range(len(robot_poses)) if i % 5 == phase]
            training = [i for i in range(len(robot_poses)) if i not in alternate]
            if observability([robot_poses[i] for i in training])['ok']:
                held, train = alternate, training
                break
        else:
            raise ValueError('Orientation coverage insufficient for a separate train/holdout split')
    results = {}
    for name in ['TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS']:
        try:
            method = getattr(cv2, 'CALIB_HAND_EYE_'+name)
            x = solve(inverse_robot, camera_boards, method)
            if not valid_transform(x):
                raise ValueError('Invalid solver output')
            fixed = [g @ x @ c for g, c in zip(inverse_robot, camera_boards)]
            full = error_stats(fixed, mean_pose(fixed))
            xt = solve([inverse_robot[i] for i in train], [camera_boards[i] for i in train], method)
            reference = mean_pose([inverse_robot[i] @ xt @ camera_boards[i] for i in train])
            test = error_stats([inverse_robot[i] @ xt @ camera_boards[i] for i in held], reference)
            loo = []
            for excluded in range(len(robot_poses)):
                idx = [i for i in range(len(robot_poses)) if i != excluded]
                value = solve([inverse_robot[i] for i in idx], [camera_boards[i] for i in idx], method)
                if not valid_transform(value):
                    raise ValueError('Invalid leave-one-out solution')
                loo.append(value)
            changes = error_stats(loo, x)
            passed = (full['translation_mm']['rms'] <= max_fit_mm
                      and test['translation_mm']['max'] <= max_holdout_mm
                      and full['rotation_deg']['rms'] <= max_rotation_deg
                      and test['rotation_deg']['rms'] <= max_rotation_deg
                      and changes['translation_mm']['max'] <= max_loo_mm)
            results[name] = {'T_ur_base_camera': x.tolist(), 'T_active_tcp_board_mean': mean_pose(fixed).tolist(),
                             'rigid_board_consistency': full, 'heldout_errors': test,
                             'leave_one_out_transform_changes': changes, 'quality_passed': bool(passed),
                             'score': test['translation_mm']['rms']+changes['translation_mm']['max']}
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            results[name] = {'error': str(error), 'quality_passed': False}
    eligible = [name for name, value in results.items() if value.get('quality_passed')]
    available = [name for name, value in results.items() if 'T_ur_base_camera' in value]
    if not available:
        raise ValueError('All hand-eye solvers failed')
    selected = min(eligible or available, key=lambda name: results[name]['score'])
    return {'status': 'CALIBRATED_INTERNAL_CHECKS_PASSED' if eligible else 'QUALITY_FAILED',
            'selected_method': selected, 'quality_passed': bool(eligible), 'quality_limits': limits,
            'pose_diversity': diversity, 'heldout_indices': held, 'solvers': results}
