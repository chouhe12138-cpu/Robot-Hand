#!/usr/bin/env python3
"""Manual ZED stereo collection for CAD objects and hand-held CAD objects.

Read-only capture: never imports robot control or sends UR / gripper commands.
Run from the project root after: source activate_handover.bash
"""

import argparse
from datetime import datetime
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FFS_DIR = ROOT / 'src' / 'Fast-FoundationStereo'
SAM_DIR = ROOT / 'src' / 'SAM2_streaming'
for path in (ROOT, FFS_DIR, SAM_DIR):
    sys.path.insert(0, str(path))

from cad_capture_io import save_sample
from h2r_interaction import manual_segmentation_prompts, unique_hand

WIDTH, HEIGHT = 640, 360
MODEL_FFS = FFS_DIR / 'weights/23-36-37/model_best_bp2_serialize.pth'
MODEL_SAM = SAM_DIR / 'checkpoints/sam2.1/sam2.1_hiera_small.pt'
SAM_CFG = 'sam2.1/sam2.1_hiera_s.yaml'
# The current H2R camera is not mirrored: physical LEFT => MediaPipe "Right".
PHYSICAL_LEFT_LABEL = 'Right'


class ReadOnlyPose:
    """Optional pose snapshot, without starting a controller or moving the robot."""

    def __init__(self, with_tf, robot_ip):
        self.node = self.listener = self.buffer = self.rtde = None
        if with_tf:
            import rclpy
            from rclpy.node import Node
            from tf2_ros import Buffer, TransformListener
            rclpy.init()
            self.node = Node('cad_capture_tf_reader')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self.node, spin_thread=True)
        if robot_ip:
            from rtde_receive import RTDEReceiveInterface
            self.rtde = RTDEReceiveInterface(robot_ip)

    def snapshot(self):
        data = {'base_to_zed_optical': None, 'robot_joint_rad': None,
                'robot_tcp_controller_pose': None}
        if self.buffer is not None:
            try:
                import rclpy
                transform = self.buffer.lookup_transform(
                    'base_link', 'zed_inhand_camera_frame_optical', rclpy.time.Time())
                t, q = transform.transform.translation, transform.transform.rotation
                data['base_to_zed_optical'] = {
                    'parent': 'base_link', 'child': 'zed_inhand_camera_frame_optical',
                    'translation_m': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                    'tf_stamp': {'sec': transform.header.stamp.sec, 'nanosec': transform.header.stamp.nanosec},
                }
            except Exception as exc:
                data['tf_error'] = str(exc)
        if self.rtde is not None:
            try:
                data['robot_joint_rad'] = list(map(float, self.rtde.getActualQ()))
                data['robot_tcp_controller_pose'] = list(map(float, self.rtde.getActualTCPPose()))
            except Exception as exc:
                data['robot_pose_error'] = str(exc)
        return data

    def close(self):
        if self.node is not None:
            if hasattr(self.listener, 'unregister'):
                self.listener.unregister()
            self.node.destroy_node()
            import rclpy
            rclpy.shutdown()
        if self.rtde is not None:
            self.rtde.disconnect()


class CaptureUI:
    def __init__(self):
        self.target = 'object'
        self.object_prompt = None
        self.hand_override = None
        self.drag_origin = None
        self.drag_current = None

    def mouse(self, event, x, y, flags, userdata):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_origin = (x, y)
            self.drag_current = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_origin is not None:
            self.drag_current = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_origin is not None:
            x0, y0 = self.drag_origin
            self.drag_origin = self.drag_current = None
            if self.target == 'hand':
                self.hand_override = (x, y)
                self.target = 'object'
            elif abs(x - x0) > 8 and abs(y - y0) > 8:
                self.object_prompt = {'bbox': [min(x, x0), min(y, y0), max(x, x0), max(y, y0)]}
            else:
                self.object_prompt = {'point': [x, y]}

    def reset(self):
        self.object_prompt = self.hand_override = None
        self.target = 'object'

    def show(self, left_bgr, mode):
        view = left_bgr.copy()
        if self.object_prompt is not None:
            if 'bbox' in self.object_prompt:
                x1, y1, x2, y2 = self.object_prompt['bbox']
                cv2.rectangle(view, (x1, y1), (x2, y2), (0, 220, 0), 2)
            else:
                cv2.circle(view, tuple(self.object_prompt['point']), 5, (0, 220, 0), 2)
        if self.hand_override is not None:
            cv2.circle(view, self.hand_override, 5, (255, 80, 50), 2)
        if self.drag_origin is not None and self.drag_current is not None:
            cv2.rectangle(view, self.drag_origin, self.drag_current,
                          (255, 80, 50) if self.target == 'hand' else (0, 220, 0), 1)
        cv2.rectangle(view, (0, 0), (WIDTH, 53), (24, 24, 24), -1)
        cv2.putText(view, f'{mode} | click/box OBJECT | space=capture | r=reset | q=exit',
                    (7, 21), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
        cv2.putText(view, f'click target: {self.target} | h=manual HAND point | o=OBJECT',
                    (7, 42), cv2.FONT_HERSHEY_SIMPLEX, .46, (240, 240, 240), 1)
        return view


def open_zed():
    import pyzed.sl as sl
    zed = sl.Camera()
    settings = sl.InitParameters()
    settings.camera_resolution = sl.RESOLUTION.HD720
    settings.camera_fps = 30
    settings.depth_mode = sl.DEPTH_MODE.NONE
    settings.coordinate_units = sl.UNIT.METER
    err = zed.open(settings)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f'ZED open failed: {err}')
    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    cam = calib.left_cam
    full_K = np.array([[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]], np.float64)
    K = full_K.copy()
    K[0, :] *= WIDTH / 1280.0
    K[1, :] *= HEIGHT / 720.0
    baseline = float(calib.get_camera_baseline())
    return zed, K, full_K, baseline, sl


def load_models(mode):
    import torch
    from core.utils.utils import InputPadder
    from Utils import AMP_DTYPE
    from sam2.build_sam import build_sam2_camera_predictor

    if not torch.cuda.is_available():
        raise RuntimeError('FFS and SAM2 require the configured CUDA environment')
    ffs = torch.load(MODEL_FFS, map_location='cpu', weights_only=False)
    ffs.args.valid_iters = 8
    ffs.args.max_disp = 192
    ffs = ffs.cuda().eval()
    sam = build_sam2_camera_predictor(SAM_CFG, str(MODEL_SAM))
    sam.fill_hole_area = 0
    hands = None
    if mode == 'handover':
        import mediapipe as mp
        hands = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=2,
                                        min_detection_confidence=.7)
    return torch, InputPadder, AMP_DTYPE, ffs, sam, hands


def predict_depth(left_bgr, right_bgr, K, baseline, torch, Padder, amp_dtype, ffs):
    left_rgb = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)
    right_rgb = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB)
    im0 = torch.as_tensor(left_rgb.copy(), device='cuda').float()[None].permute(0, 3, 1, 2)
    im1 = torch.as_tensor(right_rgb.copy(), device='cuda').float()[None].permute(0, 3, 1, 2)
    padder = Padder(im0.shape, divis_by=32, force_square=False)
    im0, im1 = padder.pad(im0, im1)
    with torch.inference_mode(), torch.autocast('cuda', dtype=amp_dtype):
        result = ffs.forward(im0, im1, iters=8, test_mode=True, optimize_build_volume='pytorch1')
    disp = padder.unpad(result.float()).cpu().numpy().reshape(HEIGHT, WIDTH).clip(0, None)
    xx = np.arange(WIDTH)[None, :]
    disp[xx - disp < 0] = np.inf
    with np.errstate(divide='ignore', invalid='ignore'):
        depth = K[0, 0] * baseline / disp
    depth[~np.isfinite(depth) | (depth < .1) | (depth > 2.0)] = 0
    return disp, depth.astype(np.float32)


def get_hand_prompt(left_bgr, hands, override):
    if override is not None:
        return {'id': 1, 'point': list(override), 'point_candidates': [list(override)],
                'source': 'manual'}
    if hands is None:
        return None
    results = hands.process(cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB))
    hand, score = unique_hand(results, PHYSICAL_LEFT_LABEL, .7)
    if hand is None:
        return None
    ids = (0, 5, 17, 13, 9, 8, 12, 16, 20)
    pts = [[int(hand.landmark[i].x * WIDTH), int(hand.landmark[i].y * HEIGHT)] for i in ids]
    pts = [[min(max(x, 0), WIDTH - 1), min(max(y, 0), HEIGHT - 1)] for x, y in pts]
    return {'id': 1, 'point': pts[4], 'point_candidates': pts,
            'source': 'mediapipe', 'confidence': float(score)}


def segment(left_bgr, object_prompt, mode, hand_prompt, torch, sam):
    """Independent SAM2 initialization on each frozen frame; no tracking drift between samples."""
    obj = {'id': 2, **object_prompt}
    if mode == 'handover':
        if hand_prompt is None:
            raise ValueError('No physical LEFT hand detected: press h and click visible hand, then retry')
        hand_args, obj_args = manual_segmentation_prompts(hand_prompt, obj)
        prompts = [hand_prompt, obj]
        arguments = [(1, hand_args), (2, obj_args)]
    else:
        prompts = [obj]
        arguments = [(2, {'bbox': obj['bbox']} if 'bbox' in obj else
                         {'points': [obj['point']], 'labels': [1]})]
    masks = {}
    effective = []
    with torch.inference_mode(), torch.autocast('cuda', dtype=torch.bfloat16):
        # The existing Robot-Hand camera predictor is initialized with BGR frames.
        sam.load_first_frame(left_bgr)
        for obj_id, kwargs in arguments:
            _, ids, logits = sam.add_new_prompt(frame_idx=0, obj_id=obj_id, **kwargs)
            masks = {int(idx): (logit > 0).cpu().numpy().squeeze().astype(bool)
                     for idx, logit in zip(ids, logits)}
            effective.append({'id': obj_id, 'arguments': kwargs})
    return masks, prompts, effective


def capture(args, frames, pose, zed_info, models, ui, session_dir, index):
    torch, Padder, amp_dtype, ffs, sam, hands = models
    full_left, full_right, left, right, zed_timestamp = frames
    K, full_K, baseline, serial = zed_info
    now = datetime.now().astimezone()
    folder = session_dir / f'sample_{index:04d}_{now.strftime("%Y%m%d_%H%M%S_%f")}'
    metadata = {
        'sample_id': folder.name, 'object_id': args.object_id, 'mode': args.mode,
        'captured_at': now.isoformat(), 'zed_image_timestamp_ns': zed_timestamp,
        'cad': {'path': str(Path(args.cad).expanduser()) if args.cad else None,
                'unit': args.cad_unit, 'note': 'CAD is not aligned and is not pose ground truth'},
        'camera': {'name': 'ZED 2i', 'serial': serial,
                   'frame': 'zed_inhand_camera_frame_optical',
                   'intrinsics_640x360': K.tolist(), 'intrinsics_1280x720': full_K.tolist(),
                   'baseline_m': baseline, 'depth_scale_m_per_unit': .001,
                   'rectified': True, 'pointcloud_frame': 'left optical frame, metres'},
        'depth': {'model': 'Fast-FoundationStereo/23-36-37', 'iterations': 8,
                  'max_disparity_px': 192, 'valid_range_m': [.1, 2.0]},
        'segmentation': {'model': 'SAM2.1 Hiera Small', 'status': 'not_run'},
        'pose_snapshot': pose,
    }
    try:
        disparity, depth = predict_depth(left, right, K, baseline, torch, Padder, amp_dtype, ffs)
    except Exception as exc:
        disparity = np.full((HEIGHT, WIDTH), np.nan, np.float32)
        depth = np.zeros((HEIGHT, WIDTH), np.float32)
        metadata['depth']['error'] = str(exc)

    masks, prompts = {}, [{'id': 2, **ui.object_prompt}]
    if args.mode == 'handover':
        hand = get_hand_prompt(left, hands, ui.hand_override)
        if hand:
            prompts.insert(0, hand)
    else:
        hand = None
    try:
        masks, prompts, effective = segment(left, ui.object_prompt, args.mode, hand, torch, sam)
        metadata['segmentation'] = {'model': 'SAM2.1 Hiera Small', 'status': 'ok',
                                    'effective_prompts': effective}
    except Exception as exc:
        metadata['segmentation'] = {'model': 'SAM2.1 Hiera Small', 'status': 'failed',
                                    'error': str(exc)}
        print(f'Warning: segmentation failed, raw stereo/depth still saved: {exc}')
    metadata['prompts'] = prompts
    counts = save_sample(folder, left_full_bgr=full_left, right_full_bgr=full_right,
                         left_bgr=left, right_bgr=right, depth_m=depth, disparity=disparity,
                         K=K, masks=masks, prompts=prompts, metadata=metadata)
    cv2.imshow('Last capture Mask', cv2.imread(str(folder / 'mask_preview.png')))
    print(f'Saved {folder} | point counts: {counts} | segmentation: {metadata["segmentation"]["status"]}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--object-id', required=True, help='Stable identity, e.g. linemod_cat')
    parser.add_argument('--mode', choices=('object', 'handover'), default='object')
    parser.add_argument('--cad', help='Associated CAD path, recorded only; never modified or copied')
    parser.add_argument('--cad-unit', choices=('m', 'mm'), default='mm')
    parser.add_argument('--output-root', type=Path, default=ROOT / 'captured_data' / 'cad_samples')
    parser.add_argument('--with-tf', action='store_true', help='Read latest ROS TF without motion')
    parser.add_argument('--robot-ip', help='Optional read-only RTDE joint/TCP snapshot; no control connection')
    args = parser.parse_args()

    import pyzed.sl as sl
    pose = zed = hands = None
    try:
        pose = ReadOnlyPose(args.with_tf, args.robot_ip)
        zed, K, full_K, baseline, _ = open_zed()
        serial = int(zed.get_camera_information().serial_number)
        models = load_models(args.mode)
        hands = models[-1]
        zed_info = (K, full_K, baseline, serial)
        left_mat, right_mat = sl.Mat(), sl.Mat()
        ui = CaptureUI()
        cv2.namedWindow('ZED CAD Capture', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback('ZED CAD Capture', ui.mouse)
        session = args.output_root / args.object_id / datetime.now().strftime('%Y%m%d_%H%M%S')
        count = 0
        print('Manual-only: position robot/camera yourself. Click or box OBJECT; SPACE saves.')
        if args.mode == 'handover':
            print('Physical LEFT hand auto prompt; press h then click visible HAND to override.')
        print('Keys: h=manual hand point, o=object, r=reset selection, SPACE=save, q/ESC=exit')
        while True:
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                continue
            zed.retrieve_image(left_mat, sl.VIEW.LEFT)
            zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
            full_left = left_mat.get_data()[:, :, :3].copy()
            full_right = right_mat.get_data()[:, :, :3].copy()
            left = cv2.resize(full_left, (WIDTH, HEIGHT))
            right = cv2.resize(full_right, (WIDTH, HEIGHT))
            image_stamp = int(zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
            cv2.imshow('ZED CAD Capture', ui.show(left, args.mode))
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break
            if key == ord('r'):
                ui.reset()
            elif key == ord('h') and args.mode == 'handover':
                ui.target = 'hand'
            elif key == ord('o'):
                ui.target = 'object'
            elif key == 32:
                if ui.object_prompt is None:
                    print('Select the OBJECT in the left ZED image before capture.')
                    continue
                pose_snapshot = pose.snapshot()
                count += 1
                capture(args, (full_left, full_right, left, right, image_stamp),
                        pose_snapshot, zed_info, models, ui, session, count)
                ui.reset()
    finally:
        if hands is not None:
            hands.close()
        if zed is not None:
            zed.close()
        if pose is not None:
            pose.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
