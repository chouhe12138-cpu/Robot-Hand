"""Convert offline TCP-camera estimates to tool0-camera; never connects to hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from object_hand.calibration import transform, solve


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--tcp-offset', nargs=6, type=float, required=True,
                        help='UR flange/all-zero-tool to active TCP: metres and rotation-vector radians')
    parser.add_argument('--method', default='PARK', choices=['TSAI','PARK','HORAUD','ANDREFF','DANIILIDIS'])
    args = parser.parse_args()
    finalize_session(args.session, args.tcp_offset, args.method)


def finalize_session(root, tcp_offset, method='PARK', write_launch=True):
    root = Path(root)
    report_path = root/'preliminary_analysis/report.json'
    report = json.loads(report_path.read_text())
    chosen = report['solvers'][method]
    if 'T_active_tcp_camera' not in chosen:
        raise RuntimeError('No valid solver result')
    t_tool0_tcp = transform(tcp_offset[3:], tcp_offset[:3])
    t_tcp_camera = np.array(chosen['T_active_tcp_camera'])
    t_tool0_camera = t_tool0_tcp @ t_tcp_camera
    # UR's physical flange coordinate system (zero TCP) corresponds to ROS tool0,
    # not to the separately oriented ROS-Industrial link named flange.
    tool0_poses, board_poses = [], []
    for row in report['images']:
        if not row['accepted']:
            continue
        raw = json.loads((root/row['image']).with_suffix('.json').read_text())
        pose = raw['robot_after_image']['actual_tcp_pose_m_rotvec_rad']
        t_base_tcp = transform(pose[3:], pose[:3])
        t_base_tool0 = t_base_tcp @ np.linalg.inv(t_tool0_tcp)
        tool0_poses.append(t_base_tool0)
        board_poses.append(np.array(row['T_camera_board']))
        # Changing the effector reference must preserve base->board for every sample.
        np.testing.assert_allclose(t_base_tcp @ t_tcp_camera @ board_poses[-1],
                                   t_base_tool0 @ t_tool0_camera @ board_poses[-1], atol=1e-10)
    direct = solve(tool0_poses, board_poses, getattr(cv2,'CALIB_HAND_EYE_'+method))
    np.testing.assert_allclose(direct, t_tool0_camera, atol=1e-8)
    r = t_tool0_camera[:3,:3]
    np.testing.assert_allclose(r.T @ r, np.eye(3), atol=1e-8)
    assert abs(np.linalg.det(r)-1) < 1e-8
    quaternion = Rotation.from_matrix(r).as_quat()
    if quaternion[3] < 0:
        quaternion = -quaternion
    held = chosen['heldout_errors']
    fixed = chosen['fixed_board_consistency']
    data = {
        'status': 'OFFLINE_CALIBRATED_REQUIRES_INDEPENDENT_PHYSICAL_VALIDATION',
        'parent_frame': 'tool0', 'child_frame': 'zed_inhand_camera_frame_optical',
        'solver': method, 'sample_count': report['accepted_samples'],
        'T_tool0_active_tcp': t_tool0_tcp.tolist(),
        'T_tool0_camera': t_tool0_camera.tolist(),
        'translation_m': t_tool0_camera[:3,3].tolist(), 'quaternion_xyzw': quaternion.tolist(),
        'fixed_board_position_rms_mm': float(np.sqrt(np.mean(np.square(fixed['translation_mm'])))),
        'fixed_board_rotation_rms_deg': float(np.sqrt(np.mean(np.square(fixed['rotation_deg'])))),
        'heldout_position_errors_mm': held['translation_mm'],
        'heldout_rotation_errors_deg': held['rotation_deg'],
        'leave_one_out_max_translation_change_mm': max(chosen['leave_one_out_transform_changes']['translation_mm']),
        'evidence': {'board_dimensions': report['board_model'],
                     'stationary_board': 'Assumes board remained fixed during capture',
                     'tcp_offset_m_rotvec_rad': list(tcp_offset),
                     'tcp_during_capture': 'Operator-supplied TCP must remain active throughout capture'},
        'board_model': report['board_model'],
        'source_session': str(root.resolve()),
        'source_report_sha256': hashlib.sha256(report_path.read_bytes()).hexdigest(),
        'frame_reference': 'https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/robot_frames.html',
        'limitations': ['Holdout samples are from the same acquisition session, not independent physical validation',
                        'Robot base vs ROS base_link differs by pi about Z; this must be respected by motion conversions',
                        'ROS FK/TF must use the calibration of this physical robot for accurate base->tool0'],
    }
    output = root/'calibrated_result'
    output.mkdir(exist_ok=True)
    (output/'zed_tool0_calibration.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
    if write_launch:
        flags = ['--frame-id','tool0','--child-frame-id','zed_inhand_camera_frame_optical']
        for name,value in zip(['--x','--y','--z','--qx','--qy','--qz','--qw'],[*data['translation_m'],*quaternion]):
            flags.extend([name,format(float(value),'.12g')])
        launch = Path(__file__).resolve().parents[2]/'src/calibration_result/ur3e_eye_in_hand/eye_in_hand_zed2i_local.launch.py'
        launch.write_text('"""Local ZED calibration, 2026-10-07, 20 samples; independent validation pending.\n'
                          'Publishes only TF. Do not run alongside another publisher for the same child frame.\n"""\n'
                          'from launch import LaunchDescription\nfrom launch_ros.actions import Node\n\n'
                          'def generate_launch_description():\n'
                          '    return LaunchDescription([Node(package="tf2_ros", executable="static_transform_publisher",\n'
                          '                                   name="zed_local_handeye_tf", output="log",\n'
                          f'                                   arguments={flags!r})])\n', encoding='utf-8')
    print('Result:', output/'zed_tool0_calibration.json', flush=True)
    return data



if __name__ == '__main__':
    main()
