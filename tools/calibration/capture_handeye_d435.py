"""Fixed D435i eye-to-hand capture. RTDE RECEIVE ONLY; never commands motion."""
import argparse
from collections import deque
from datetime import datetime
import json
from pathlib import Path
import time
import cv2
import numpy as np
from d435_handeye_core import BoardSpec, detect_board, new_pose, pose_matrix, observability


def state(receiver):
    if not receiver.isConnected():
        raise RuntimeError('RTDE disconnected')
    return {'host_monotonic_s': time.monotonic(), 'robot_timestamp_s': receiver.getTimestamp(),
            'actual_q_rad': receiver.getActualQ(), 'actual_qd_rad_s': receiver.getActualQd(),
            'actual_tcp_pose_m_rotvec_rad': receiver.getActualTCPPose(),
            'actual_tcp_speed': receiver.getActualTCPSpeed()}


def stable(history, seconds=1.):
    if len(history) < 2 or history[-1]['host_monotonic_s']-history[0]['host_monotonic_s'] < seconds:
        return False
    if history[-1]['robot_timestamp_s'] <= history[0]['robot_timestamp_s']:
        return False
    return bool(np.max(np.ptp([s['actual_q_rad'] for s in history], axis=0)) < .001
                and np.max(np.abs([s['actual_qd_rad_s'] for s in history])) < .002)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip', default='192.168.1.10')
    parser.add_argument('--serial', help='Default: automatically select the sole connected D435/D435i')
    parser.add_argument('--check', action='store_true', help='Headless camera + RTDE connection check')
    parser.add_argument('--auto', action='store_true', help='Save each new stationary valid pose automatically')
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--solve', action='store_true', help='Automatically solve when sample target is reached')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--columns', type=int, default=7)
    parser.add_argument('--rows', type=int, default=5)
    parser.add_argument('--square-mm', type=float, default=30.)
    parser.add_argument('--marker-mm', type=float, default=22.)
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--min-rotation-deg', type=float, default=6.)
    parser.add_argument('--min-translation-mm', type=float, default=25.)
    parser.add_argument('--settle-seconds', type=float, default=1.)
    args = parser.parse_args(argv)
    if args.samples < 8:
        parser.error('--samples must be at least 8; 20 recommended')
    if not all(np.isfinite(v) and v > 0 for v in [args.square_mm, args.marker_mm, args.min_rotation_deg,
                                                   args.min_translation_mm, args.settle_seconds]):
        parser.error('Dimensions and capture thresholds must be finite and positive')
    if args.output and args.resume:
        parser.error('--output and --resume are mutually exclusive')
    if args.headless and not (args.auto or args.check):
        parser.error('--headless capture requires --auto')
    BoardSpec(args.columns, args.rows, args.square_mm/1000, args.marker_mm/1000, args.dictionary).create()
    return args


def main(argv=None):
    args = parse_args(argv)
    # Device SDKs are imported only for live capture, not offline solve/tests.
    import pyrealsense2 as rs
    import rtde_receive
    receiver, pipeline = None, rs.pipeline()
    started = False
    folder = None
    complete = False
    saved = []
    window = 'D435i eye-to-hand: '+('AUTO capture' if args.auto else 'SPACE to capture')
    spec = BoardSpec(args.columns, args.rows, args.square_mm/1000, args.marker_mm/1000, args.dictionary)
    try:
        devices = list(rs.context().query_devices())
        if args.serial:
            matching = [d for d in devices if d.get_info(rs.camera_info.serial_number) == args.serial]
        else:
            matching = [d for d in devices if 'D435' in d.get_info(rs.camera_info.name)]
        if len(matching) != 1:
            raise RuntimeError('Expected one D435/D435i; connect camera or specify --serial')
        serial = matching[0].get_info(rs.camera_info.serial_number)
        receiver = rtde_receive.RTDEReceiveInterface(args.robot_ip, 125.)
        state(receiver)
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = pipeline.start(config)
        started = True
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        k = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1.]])
        distortion = np.array(intr.coeffs)
        if 'inverse_brown' in str(intr.model) and np.any(distortion):
            raise RuntimeError('Nonzero inverse Brown coefficients need explicit image undistortion')
        metadata = {'calibration_type': 'eye_to_hand', 'camera_serial': serial,
                    'camera_name': profile.get_device().get_info(rs.camera_info.name),
                    'image_view': 'RealSense native color optical frame', 'resolution': [intr.width, intr.height],
                    'camera_matrix': k.tolist(), 'distortion_model': str(intr.model),
                    'distortion_coefficients': distortion.tolist(), 'robot_ip': args.robot_ip,
                    'robot_pose_frame': 'UR controller base -> active TCP', 'board': spec.metadata(),
                    'flange_to_active_tcp_offset_m_rotvec_rad': None,
                    'tcp_evidence': 'Unknown fixed offset is absorbed into TCP-to-board; never change active TCP during acquisition',
                    'capture_target': args.samples, 'capture_mode': 'automatic_stationary' if args.auto else 'manual_key',
                    'assumptions': ['Camera and robot base remain fixed relative to each other',
                                    'Board is rigidly attached to the robot end effector, not held by a person',
                                    'Active TCP and board attachment remain unchanged',
                                    'Printed board matches the configured physical dimensions',
                                    'Only one board with these marker IDs is visible']}
        if not args.check:
            folder = args.resume or args.output or Path(__file__).resolve().parents[2]/'calibration_data'/('d435_eye_to_hand_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            if args.resume:
                previous = json.loads((folder/'session.json').read_text())
                if (previous['camera_serial'] != serial or previous['board'] != spec.metadata()
                        or previous.get('calibration_type') != 'eye_to_hand'
                        or not np.allclose(previous['camera_matrix'], k, atol=1e-5, rtol=0)
                        or not np.allclose(previous['distortion_coefficients'], distortion, atol=1e-8, rtol=0)):
                    raise RuntimeError('Resume session camera/board/intrinsics mismatch')
                for sample in sorted(folder.glob('[0-9][0-9][0-9][0-9].json')):
                    raw = json.loads(sample.read_text())
                    if not sample.with_suffix('.png').is_file():
                        raise RuntimeError('Incomplete sample: '+str(sample))
                    saved.append(raw['robot_after_image']['actual_tcp_pose_m_rotvec_rad'])
                if len(saved) > args.samples:
                    raise RuntimeError('Resume contains more samples than --samples')
            else:
                folder.mkdir(parents=True, exist_ok=False)
                (folder/'session.json').write_text(json.dumps(metadata, indent=2))
            (folder/'capture_status.json').write_text(json.dumps({'status': 'CAPTURING', 'saved': len(saved), 'target': args.samples}))
            print('SESSION:', folder.resolve(), flush=True)
            print('Camera fixed; board rigid on TCP. Change pose manually, then pause. Q/Esc quits; S saves in manual mode.', flush=True)
            if not args.headless:
                cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(window, 960, 720)
        for _ in range(15):
            pipeline.wait_for_frames(5000)
        history = deque()
        board_history = deque()
        last_print = 0.
        while len(saved) < args.samples or args.check:
            before = state(receiver)
            frame = pipeline.wait_for_frames(5000).get_color_frame()
            image = np.asanyarray(frame.get_data()).copy()
            frame_host = time.monotonic()
            after = state(receiver)
            history.append(after)
            while len(history) > 2 and after['host_monotonic_s']-history[1]['host_monotonic_s'] > args.settle_seconds+.2:
                history.popleft()
            detected = detect_board(image, k, distortion, spec)
            if args.check:
                print(json.dumps({'camera': metadata['camera_name'], 'serial': serial, 'image_shape': list(image.shape),
                                  'robot_connected': True, 'charuco_corners': detected['corners'],
                                  'board_accepted': detected['accepted'], 'motion_commands_sent': False}), flush=True)
                return 0
            reason = detected['reason']
            ready = stable(history, args.settle_seconds) and detected['accepted']
            pose = after['actual_tcp_pose_m_rotvec_rad']
            if not stable(history, args.settle_seconds):
                reason = 'WAIT: robot must stop for %.1fs' % args.settle_seconds
            if ready and not new_pose(pose, saved, args.min_rotation_deg, args.min_translation_mm/1000):
                ready = False
                reason = 'WAIT: change tilt/position; duplicate pose'
            if detected['accepted']:
                board_history.append((frame_host, np.array(detected['T_camera_board'])))
            else:
                board_history.clear()
            while len(board_history) > 2 and frame_host-board_history[1][0] > .7:
                board_history.popleft()
            if ready:
                board_settled = (len(board_history) > 1 and frame_host-board_history[0][0] >= .5
                                 and max(np.linalg.norm(t[:3, 3]-board_history[-1][1][:3, 3]) for _, t in board_history) < .003)
                if not board_settled:
                    ready = False
                    reason = 'WAIT: board pose must be stable'
            diversity = observability([pose_matrix(p) for p in [*saved, pose]])
            if ready and len(saved)+1 == args.samples and not diversity['ok']:
                ready = False
                reason = 'WAIT: tilt around another axis (not translation only)'
            dq = np.max(np.abs(np.asarray(after['actual_q_rad'])-before['actual_q_rad']))
            if ready and (dq > .001 or after['host_monotonic_s']-before['host_monotonic_s'] > .2
                          or time.monotonic()-frame_host > .3):
                ready = False
                reason = 'WAIT: image/robot timing or motion rejected'
            key = -1
            if not args.headless:
                display = image.copy()
                if detected['corners']:
                    # Corner data is present only when PnP succeeded.
                    if 'charuco_corners' in detected:
                        cv2.aruco.drawDetectedCornersCharuco(display, np.array(detected['charuco_corners'], np.float32), np.array(detected['charuco_ids'], np.int32))
                color = (0, 220, 0) if ready else (0, 160, 255)
                text = f'{len(saved)}/{args.samples} | corners {detected["corners"]} | '+('READY' if ready else reason)
                cv2.putText(display, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1)
                cv2.putText(display, f'Rotation span {diversity["rotation_span_deg"]:.0f} deg | second axis {diversity["second_axis_rms_deg"]:.1f} deg', (8, 43), cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1)
                cv2.putText(display, 'AUTO capture | Q: quit' if args.auto else 'SPACE: save one sample | Q: quit', (8, 64), cv2.FONT_HERSHEY_SIMPLEX, .43, color, 1)
                cv2.imshow(window, display)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), ord('Q'), 27) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if time.monotonic()-last_print >= 2:
                print(f'{len(saved)}/{args.samples}: '+('READY' if ready else reason), flush=True)
                last_print = time.monotonic()
            capture_pressed = key in (32, ord('s'), ord('S'))
            if capture_pressed and not ready:
                print('NOT SAVED: '+reason+'; wait for READY and press SPACE again.', flush=True)
            if not ready or not (args.auto or capture_pressed):
                continue
            # Recheck after GUI processing; never save a stale image/state pair.
            if time.monotonic()-frame_host > .3:
                continue
            sample_id = f'{len(saved)+1:04d}'
            path = folder/(sample_id+'.png')
            if path.exists() or path.with_suffix('.json').exists():
                raise RuntimeError('Refusing to overwrite an existing sample: '+str(path))
            if not cv2.imwrite(str(path), image):
                raise RuntimeError('Image write failed')
            sample = {'image': path.name, 'saved_at': datetime.now().astimezone().isoformat(),
                      'frame_number': frame.get_frame_number(), 'camera_timestamp_ms': frame.get_timestamp(),
                      'camera_timestamp_domain': str(frame.get_frame_timestamp_domain()),
                      'host_frame_received_monotonic_s': frame_host, 'robot_before_image': before,
                      'robot_after_image': after, 'stationary_window_s': after['host_monotonic_s']-history[0]['host_monotonic_s'],
                      **detected}
            try:
                path.with_suffix('.json').write_text(json.dumps(sample, indent=2))
            except Exception:
                path.unlink()
                raise
            saved.append(pose)
            (folder/'capture_status.json').write_text(json.dumps({'status': 'CAPTURING', 'saved': len(saved), 'target': args.samples}))
            print(f'SAVED {len(saved)}/{args.samples}: {detected["corners"]} corners; RMS {detected["reprojection_rms_px"]:.3f}px', flush=True)
        complete = len(saved) == args.samples
        (folder/'capture_status.json').write_text(json.dumps({'status': 'CAPTURE_COMPLETE' if complete else 'INCOMPLETE', 'saved': len(saved), 'target': args.samples}))
    except KeyboardInterrupt:
        print('Capture interrupted; saved samples retained. Use --resume SESSION.', flush=True)
        if folder:
            (folder/'capture_status.json').write_text(json.dumps({'status': 'INCOMPLETE', 'saved': len(saved), 'target': args.samples}))
        return 130
    except Exception as error:
        if folder is not None and folder.is_dir():
            (folder/'capture_status.json').write_text(json.dumps({'status': 'CAPTURE_FAILED', 'saved': len(saved),
                                                                 'target': args.samples, 'error': str(error)}))
        raise
    finally:
        if started:
            try:
                pipeline.stop()
            except RuntimeError:
                # librealsense may already have stopped after a USB disconnect.
                # Preserve the original acquisition error instead of masking it.
                pass
        if receiver is not None:
            receiver.disconnect()
        if not args.headless and not args.check:
            cv2.destroyAllWindows()
    if complete and args.solve:
        from tools.calibration.calibrate_handeye_d435 import calibrate_session
        (folder/'capture_status.json').write_text(json.dumps({'status': 'SOLVING', 'saved': len(saved), 'target': args.samples}))
        try:
            result = calibrate_session(folder, required_samples=args.samples)
        except (ValueError, RuntimeError, OSError, cv2.error) as error:
            print('CALIBRATION_FAILED:', error, flush=True)
            (folder/'capture_status.json').write_text(json.dumps({'status': 'CALIBRATION_FAILED', 'saved': len(saved),
                                                                 'target': args.samples, 'error': str(error)}))
            return 2
        (folder/'capture_status.json').write_text(json.dumps({'status': result['status'], 'saved': len(saved),
                                                             'target': args.samples, 'quality_passed': result['quality_passed']}))
        return 0 if result['quality_passed'] else 2
    return 0 if complete else 3


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError) as error:
        print('CAPTURE_FAILED:', error, flush=True)
        raise SystemExit(2)
