"""Manual ZED hand-eye acquisition. Uses RTDE receive only; never commands motion."""
import argparse
from collections import deque
from datetime import datetime
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
from d435_handeye_core import BoardSpec, detect_board, new_pose, observability, pose_matrix


def state(receiver):
    if not receiver.isConnected():
        raise RuntimeError('RTDE disconnected; no sample can be saved')
    return {
        'host_monotonic_s': time.monotonic(),
        'robot_timestamp_s': receiver.getTimestamp(),
        'actual_q_rad': receiver.getActualQ(),
        'actual_qd_rad_s': receiver.getActualQd(),
        'actual_tcp_pose_m_rotvec_rad': receiver.getActualTCPPose(),
        'actual_tcp_speed': receiver.getActualTCPSpeed(),
    }


def stable(history):
    if len(history) < 2 or history[-1]['host_monotonic_s'] - history[0]['host_monotonic_s'] < 1.0:
        return False
    if history[-1]['robot_timestamp_s'] <= history[0]['robot_timestamp_s']:
        return False
    joints = np.array([s['actual_q_rad'] for s in history])
    velocities = np.array([s['actual_qd_rad_s'] for s in history])
    return bool(np.max(np.ptp(joints, axis=0)) < 0.001 and np.max(np.abs(velocities)) < 0.002)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.10')
    parser.add_argument('--check', action='store_true', help='Headless frame/RTDE check; saves nothing')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--solve', action='store_true', help='Solve automatically after target sample count')
    parser.add_argument('--square-mm', type=float, default=30.)
    parser.add_argument('--marker-mm', type=float, default=22.)
    parser.add_argument('--tcp-offset', type=float, nargs=6,
                        help='Optional verified active flange-to-TCP offset: metres, rotation vector radians')
    args = parser.parse_args(argv)
    if args.samples < 8:
        parser.error('--samples must be at least 8; 20 recommended')
    if args.solve and (args.tcp_offset is None or not np.isfinite(args.tcp_offset).all()):
        parser.error('--solve requires a verified --tcp-offset x y z rx ry rz')
    spec = BoardSpec(square_m=args.square_mm/1000, marker_m=args.marker_mm/1000)
    spec.create()
    import pyzed.sl as sl
    import rtde_receive
    receiver = None
    camera = sl.Camera()
    complete = False
    saved_poses = []
    try:
        # No RTDEControlInterface: this connects to state output only.
        receiver = rtde_receive.RTDEReceiveInterface(args.robot_ip, 125.0)
        initial = state(receiver)
        print('RTDE receive connected. Robot TCP:', initial['actual_tcp_pose_m_rotvec_rad'], flush=True)
        params = sl.InitParameters()
        params.camera_resolution = sl.RESOLUTION.HD720
        params.camera_fps = 30
        params.depth_mode = sl.DEPTH_MODE.NONE
        params.sensors_required = False
        if os.environ.get('URHO_ZED_SETTINGS_PATH'):
            params.optional_settings_path = os.environ['URHO_ZED_SETTINGS_PATH']
        status = camera.open(params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'ZED open failed: {status}')
        info = camera.get_camera_information()
        intr = info.camera_configuration.calibration_parameters.left_cam
        metadata = {
            'camera_serial': info.serial_number,
            'image_view': 'ZED rectified LEFT optical frame',
            'camera_matrix': [[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]],
            'distortion_coefficients': [0.0] * 5,
            'resolution': [1280, 720],
            'robot_ip': args.robot_ip,
            'robot_pose_frame': 'UR controller base -> ACTIVE TCP; not automatically ROS base_link -> tool0',
            'flange_to_active_tcp_offset_m_rotvec_rad': args.tcp_offset,
            'tcp_offset_verified': args.tcp_offset is not None,
            'board': spec.metadata(),
            'capture_target': args.samples,
            'note': 'Fixed board, constant active TCP; board dimensions supplied by operator. Manual robot positioning only.',
        }
        runtime, left = sl.RuntimeParameters(), sl.Mat()
        history = deque()
        failures = 0
        folder = None
        count = 0
        if not args.check:
            folder = args.output or Path(__file__).resolve().parents[2] / 'calibration_data' / ('zed_eye_in_hand_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            folder.mkdir(parents=True, exist_ok=False)
            (folder / 'session.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
            print('Save folder:', folder, flush=True)
            print('Focus preview window: SPACE/S=save; Q/Esc=quit. Keep board fixed; vary robot orientation.', flush=True)
            cv2.namedWindow('ZED hand-eye capture', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ZED hand-eye capture', 1100, 680)
        while True:
            before = state(receiver)
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                failures += 1
                if failures >= 30:
                    raise RuntimeError('30 consecutive ZED grab failures')
                continue
            failures = 0
            camera.retrieve_image(left, sl.VIEW.LEFT)
            image = cv2.cvtColor(left.get_data(), cv2.COLOR_BGRA2BGR)
            image_timestamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            after = state(receiver)
            history.append(after)
            while len(history) > 2 and after['host_monotonic_s'] - history[1]['host_monotonic_s'] > 1.2:
                history.popleft()
            if args.check:
                print('PASS: ZED image', image.shape, '; RTDE timestamp', after['robot_timestamp_s'], flush=True)
                print('PASS: intrinsics', metadata['camera_matrix'], flush=True)
                return
            detection = detect_board(image, np.array(metadata['camera_matrix']), np.zeros(5), spec)
            pose = after['actual_tcp_pose_m_rotvec_rad']
            diverse = new_pose(pose, saved_poses)
            ready = stable(history) and detection['accepted'] and diverse
            display = image.copy()
            if detection['accepted']:
                cv2.drawFrameAxes(display, np.array(metadata['camera_matrix']), np.zeros(5),
                                  np.array(detection['rvec']), np.array(detection['tvec']), .05)
            reason = 'stop robot >1s' if not stable(history) else detection['reason'] if not detection['accepted'] else 'change robot pose'
            text = f'SPACE=save Q=quit | {count}/{args.samples} | ' + ('READY' if ready else 'WAIT: '+reason)
            cv2.putText(display, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 220, 0) if ready else (0, 170, 255), 2)
            cv2.imshow('ZED hand-eye capture', display)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), ord('Q'), 27):
                break
            if cv2.getWindowProperty('ZED hand-eye capture', cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (32, ord('s'), ord('S')):
                if not ready or np.max(np.abs(np.array(before['actual_q_rad']) - np.array(after['actual_q_rad']))) > 0.001:
                    print('Not saved: '+reason, flush=True)
                    continue
                if after['host_monotonic_s'] - before['host_monotonic_s'] > 0.20:
                    print('Not saved: image/state acquisition delay >200ms', flush=True)
                    continue
                if count+1 == args.samples and not observability([pose_matrix(p) for p in saved_poses+[pose]])['ok']:
                    print('Not saved: tilt around at least two axes before final sample', flush=True)
                    continue
                sample_id = f'{count + 1:04d}'
                image_path = folder / f'{sample_id}.png'
                if not cv2.imwrite(str(image_path), image):
                    raise RuntimeError(f'Failed writing {image_path}')
                sample = {'image': image_path.name, 'image_timestamp_ns': image_timestamp,
                          'saved_at': datetime.now().astimezone().isoformat(),
                          'robot_before_image': before, 'robot_after_image': after,
                          'stationary_window_s': after['host_monotonic_s'] - history[0]['host_monotonic_s']}
                try:
                    (folder / f'{sample_id}.json').write_text(json.dumps(sample, indent=2), encoding='utf-8')
                except Exception:
                    image_path.unlink()
                    raise
                count += 1
                saved_poses.append(pose)
                print(f'SAVED {sample_id}: image + robot pose', flush=True)
                if count >= args.samples:
                    complete = True
                    break
    finally:
        camera.close()
        if receiver is not None:
            receiver.disconnect()
        if not args.check:
            cv2.destroyAllWindows()
    if complete and args.solve:
        from tools.calibration.analyze_handeye_zed import analyze_session
        from tools.calibration.finalize_handeye_zed import finalize_session
        report = analyze_session(folder, args.square_mm, args.marker_mm)
        if report['accepted_samples'] < args.samples:
            raise RuntimeError('Insufficient accepted samples; see preliminary_analysis/report.json')
        chosen = report['solvers'].get('PARK', {})
        if 'T_active_tcp_camera' not in chosen:
            raise RuntimeError('PARK solver failed; see preliminary_analysis/report.json')
        rms = lambda values: float(np.sqrt(np.mean(np.square(values))))
        if (rms(chosen['fixed_board_consistency']['translation_mm']) > 8 or
                rms(chosen['fixed_board_consistency']['rotation_deg']) > 2 or
                max(chosen['heldout_errors']['translation_mm']) > 15):
            raise RuntimeError('Calibration consistency failed; inspect preliminary_analysis/report.json')
        finalize_session(folder, args.tcp_offset, write_launch=False)


if __name__ == '__main__':
    main()
