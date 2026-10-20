"""Reusable offline eye-to-hand solver. No hardware connections or motion."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from object_hand.calibration import transform
from d435_handeye_core import BoardSpec, detect_board, pose_matrix, solve_eye_to_hand, error_stats

# Kept for validate_handeye_d435.py and existing callers.
stats = error_stats


def calibrate_session(root, required_samples=20, max_fit_mm=8., max_holdout_mm=15.,
                      max_rotation_deg=2., max_loo_mm=10.):
    root = Path(root)
    metadata = json.loads((root/'session.json').read_text())
    if metadata.get('calibration_type') != 'eye_to_hand':
        raise ValueError('Session must be eye_to_hand: camera fixed, board rigid on end effector')
    k = np.asarray(metadata['camera_matrix'], float)
    d = np.asarray(metadata['distortion_coefficients'], float)
    if 'inverse_brown' in metadata['distortion_model'] and np.any(d):
        raise ValueError('Nonzero inverse Brown coefficients require undistortion before OpenCV PnP')
    spec = BoardSpec(**metadata['board'])
    output = root/'calibrated_result'
    output.mkdir(exist_ok=True)
    rows, robot, camera, qs = [], [], [], []
    for path in sorted(root.glob('*.png')):
        if not path.with_suffix('.json').is_file():
            continue
        image = cv2.imread(str(path))
        if image is None:
            rows.append({'image': path.name, 'accepted': False, 'reason': 'Unreadable image'})
            continue
        detected = detect_board(image, k, d, spec)
        row = {'image': path.name, **detected}
        raw = json.loads(path.with_suffix('.json').read_text())
        before, after = raw['robot_before_image'], raw['robot_after_image']
        dq = float(np.max(np.abs(np.asarray(after['actual_q_rad'])-before['actual_q_rad'])))
        speed = max(float(np.max(np.abs(s['actual_qd_rad_s']))) for s in [before, after])
        # Old sessions store the same fields; reject motion even if PnP is good.
        timing = after['host_monotonic_s']-before['host_monotonic_s']
        if dq > .001 or speed >= .002 or not 0 <= timing <= .2:
            row.update(accepted=False, reason='Robot moved or image/state bracket exceeded 200ms')
        if row['accepted']:
            pose = after['actual_tcp_pose_m_rotvec_rad']
            robot.append(pose_matrix(pose))
            camera.append(np.asarray(row['T_camera_board']))
            qs.append(after['actual_q_rad'])
            cv2.aruco.drawDetectedCornersCharuco(image, np.array(row['charuco_corners'], np.float32),
                                                np.array(row['charuco_ids'], np.int32))
            cv2.drawFrameAxes(image, k, d, np.array(row['rvec']), np.array(row['tvec']), .05)
        cv2.imwrite(str(output/path.name), image)
        rows.append(row)
    report = {'sample_count': len(rows), 'accepted_samples': len(robot), 'samples': rows,
              'source_session': str(root.resolve()), 'assumptions': metadata.get('assumptions', []),
              'board_model': spec.metadata(), 'required_samples': required_samples,
              'independent_physical_validation_performed': False}
    try:
        result = solve_eye_to_hand(robot, camera, required_samples, max_fit_mm, max_holdout_mm,
                                   max_rotation_deg, max_loo_mm)
        report.update(result, joint_ranges_deg=np.rad2deg(np.ptp(qs, axis=0)).tolist())
        accepted = [r for r in rows if r['accepted']]
        report['heldout_images'] = [accepted[i]['image'] for i in result['heldout_indices']]
    except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
        report.update(status='CALIBRATION_FAILED', quality_passed=False, error=str(error))
        (output/'report.json').write_text(json.dumps(report, indent=2))
        # Replace any previous success marker so a failed re-solve cannot leave stale success.
        (output/'d435_base_calibration.json').write_text(json.dumps({
            'status': report['status'], 'quality_passed': False, 'error': str(error)}, indent=2))
        for name in ['publish_tf.bash', 'eye_to_hand_d435.launch.py']:
            (output/name).unlink(missing_ok=True)
        (output/'summary.txt').write_text('标定失败：'+str(error)+'\n请检查 report.json，补充多轴倾斜姿态或重新采集。\n')
        raise
    report_path = output/'report.json'
    report_path.write_text(json.dumps(report, indent=2))
    selected = result['solvers'][result['selected_method']]
    base_camera = np.asarray(selected['T_ur_base_camera'])
    base_link_base = transform([0, 0, np.pi], [0, 0, 0])
    base_link_camera = base_link_base @ base_camera
    q = Rotation.from_matrix(base_link_camera[:3, :3]).as_quat()
    if q[3] < 0:
        q = -q
    calibration = {'status': result['status'], 'quality_passed': result['quality_passed'],
                   'parent_frame': 'base_link', 'child_frame': 'rs_tohand_color_optical_frame',
                   'method': result['selected_method'], 'T_ur_base_camera': base_camera.tolist(),
                   'T_base_link_ur_base': base_link_base.tolist(), 'T_base_link_camera': base_link_camera.tolist(),
                   'translation_m': base_link_camera[:3, 3].tolist(), 'quaternion_xyzw': q.tolist(),
                   'validation': selected, 'source_report_sha256': hashlib.sha256(report_path.read_bytes()).hexdigest(),
                   'source_session': str(root.resolve()), 'camera_serial': metadata['camera_serial'],
                   'board_model': spec.metadata(), 'independent_physical_validation_performed': False,
                   'transform_convention': 'T_parent_child maps coordinates from child to parent',
                   'note': 'Internal consistency pass does not certify physical accuracy; camera/base and board/TCP must remain rigid.'}
    target = output/'d435_base_calibration.json'
    target.write_text(json.dumps(calibration, indent=2))
    (output/'summary.txt').write_text(
        f"状态：{'通过内部一致性检查' if result['quality_passed'] else '质量检查未通过，请勿用于运动'}\n"
        f"有效样本：{len(robot)}/{len(rows)}；自动选择算法：{result['selected_method']}\n"
        f"固定板位置一致性 RMS：{selected['rigid_board_consistency']['translation_mm']['rms']:.3f} mm\n"
        f"留出样本位置误差最大值：{selected['heldout_errors']['translation_mm']['max']:.3f} mm\n"
        f"逐一删除样本后外参位置变化最大值：{selected['leave_one_out_transform_changes']['translation_mm']['max']:.3f} mm\n"
        'T_ur_base_camera：将相机彩色光学坐标中的点转换到 UR 控制器基座，长度单位为米。\n'
        +np.array2string(base_camera, precision=10)+'\n'
        'ROS base_link 到相机的平移（米）：'+str(calibration['translation_m'])+'\n'
        '四元数 xyzw：'+str(calibration['quaternion_xyzw'])+'\n'
        '这是内部一致性验证，不等于独立实测的绝对精度。相机换位置后必须重新标定。\n')
    for name in ['publish_tf.bash', 'eye_to_hand_d435.launch.py']:
        (output/name).unlink(missing_ok=True)
    if result['quality_passed']:
        flags = ['--frame-id', calibration['parent_frame'], '--child-frame-id', calibration['child_frame']]
        for key, value in zip(['--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'],
                              [*calibration['translation_m'], *q]):
            flags.extend([key, format(float(value), '.12g')])
        (output/'publish_tf.bash').write_text('#!/usr/bin/env bash\nset -euo pipefail\n'
                                            '# Source ROS first. Only one publisher per child frame.\n'
                                            'exec ros2 run tf2_ros static_transform_publisher '+' '.join(flags)+'\n')
        (output/'eye_to_hand_d435.launch.py').write_text(
            'from launch import LaunchDescription\nfrom launch_ros.actions import Node\n\n'
            'def generate_launch_description():\n'
            '    return LaunchDescription([Node(package="tf2_ros", executable="static_transform_publisher",\n'
            f'        name="d435_handeye_tf", arguments={flags!r}, output="screen")])\n')
    print(f"{result['status']}: {len(robot)}/{len(rows)} samples; method={result['selected_method']}", flush=True)
    print('Camera -> UR base (metres):\n', base_camera, flush=True)
    print('Result:', target.resolve(), flush=True)
    print('Report:', report_path.resolve(), flush=True)
    return calibration


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--required-samples', type=int, default=20)
    parser.add_argument('--max-fit-mm', type=float, default=8.)
    parser.add_argument('--max-holdout-mm', type=float, default=15.)
    parser.add_argument('--max-rotation-deg', type=float, default=2.)
    parser.add_argument('--max-loo-mm', type=float, default=10.)
    args = parser.parse_args(argv)
    try:
        result = calibrate_session(args.session, args.required_samples, args.max_fit_mm,
                                   args.max_holdout_mm, args.max_rotation_deg, args.max_loo_mm)
    except (ValueError, RuntimeError, OSError, cv2.error) as error:
        print('CALIBRATION_FAILED:', error, flush=True)
        return 2
    return 0 if result['quality_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
