#!/usr/bin/env python3
"""Manual, motion-free ZED/FFS CAD-object and hand-occlusion sample capture.

SPACE freezes one stereo pair. On frozen image: left click OBJECT; right click HAND
(optional); s saves a sample; r clears prompts; c resumes camera; q quits.
No UR, RTDE or robot control imports are used.
"""
import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pyzed.sl as sl
import torch

ROOT = Path(__file__).resolve().parents[1]
FFS_DIR = ROOT / "src" / "Fast-FoundationStereo"
SAM_DIR = ROOT / "src" / "SAM2_streaming"
sys.path.insert(0, str(FFS_DIR))
sys.path.insert(0, str(SAM_DIR))
sys.path.insert(0, str(ROOT))
from core.utils.utils import InputPadder
from Utils import AMP_DTYPE
from h2r_interaction import manual_segmentation_prompts

WIDTH, HEIGHT = 640, 360


def stereo_depth(model, left_bgr, right_bgr, K, baseline):
    left = torch.as_tensor(left_bgr[:, :, ::-1].copy(), device="cuda").float()[None].permute(0, 3, 1, 2)
    right = torch.as_tensor(right_bgr[:, :, ::-1].copy(), device="cuda").float()[None].permute(0, 3, 1, 2)
    padder = InputPadder(left.shape, divis_by=32, force_square=False)
    a, b = padder.pad(left, right)
    with torch.no_grad(), torch.amp.autocast("cuda", enabled=True, dtype=AMP_DTYPE):
        disp = model.forward(a, b, iters=8, test_mode=True, optimize_build_volume="pytorch1")
    disp = padder.unpad(disp.float()).detach().cpu().numpy().reshape(HEIGHT, WIDTH).clip(0, None)
    xs = np.broadcast_to(np.arange(WIDTH)[None, :], disp.shape)
    disp[(xs - disp) < 0] = np.inf
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = K[0, 0] * baseline / disp
    depth[~np.isfinite(depth) | (depth < .1) | (depth > 2.)] = 0
    return disp.astype(np.float32), depth.astype(np.float32)


def points_from_depth(depth, mask, left_bgr, K):
    valid = (depth > 0) & mask.astype(bool)
    ys, xs = np.nonzero(valid)
    z = depth[ys, xs]
    xyz = np.stack(((xs - K[0, 2]) * z / K[0, 0],
                    (ys - K[1, 2]) * z / K[1, 1], z), axis=-1).astype(np.float64)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(left_bgr[ys, xs, ::-1].astype(np.float64) / 255.)
    return cloud


def segment(left_bgr, object_prompt, hand_point, predictor):
    if predictor is None:
        raise RuntimeError("SAM2 was not loaded")
    if hasattr(predictor, "condition_state") and "point_inputs_per_obj" in predictor.condition_state:
        predictor.reset_state()
    predictor.load_first_frame(left_bgr.copy())
    if hand_point is not None:
        # Reuse the hand-object cross-negative logic already used by H2R.
        hand_args, obj_args = manual_segmentation_prompts(
            {"point": hand_point}, object_prompt.copy()
        )
        inputs = ((1, hand_args), (2, obj_args))
    elif "bbox" in object_prompt:
        inputs = ((2, {"bbox": np.asarray(object_prompt["bbox"], dtype=np.float32).reshape(2, 2)}),)
    else:
        inputs = ((2, {"points": [object_prompt["point"]], "labels": [1]}),)
    masks = {}
    for obj_id, args in inputs:
        if "bbox" in args:
            args["bbox"] = np.asarray(args["bbox"], dtype=np.float32)
        _, ids, logits = predictor.add_new_prompt(frame_idx=0, obj_id=obj_id, **args)
        for mid, logit in zip(ids, logits):
            masks[int(mid)] = (logit.detach().cpu().numpy().squeeze() > 0).astype(np.uint8)
    return masks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--object-id", required=True, help="CAD object identifier e.g. cat_06")
    parser.add_argument("--output", default="collected_cad_data", help="Output dataset root")
    parser.add_argument("--mode", choices=("object", "handover"), default="object")
    args = parser.parse_args()
    out_root = Path(args.output) / args.object_id / args.mode

    model_path = FFS_DIR / "weights/23-36-37/model_best_bp2_serialize.pth"
    ffs = torch.load(model_path, map_location="cpu", weights_only=False)
    ffs.args.valid_iters, ffs.args.max_disp = 8, 192
    ffs.cuda().eval()
    from sam2.build_sam import build_sam2_camera_predictor
    predictor = build_sam2_camera_predictor(
        "sam2.1/sam2.1_hiera_s.yaml",
        str(SAM_DIR / "checkpoints/sam2.1/sam2.1_hiera_small.pt"),
    )
    predictor.fill_hole_area = 0

    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")
    left_mat, right_mat = sl.Mat(), sl.Mat()
    calib = zed.get_camera_information().camera_configuration.calibration_parameters
    camera = calib.left_cam
    scale = .5
    K = np.array([[camera.fx * scale, 0, camera.cx * scale],
                  [0, camera.fy * scale, camera.cy * scale],
                  [0, 0, 1]], dtype=np.float32)
    baseline = float(calib.get_camera_baseline())
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    frozen = None
    prompts = {"object": None, "hand": None}
    cv2.namedWindow("CAD Capture", cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, flags, _):
        if frozen is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            prompts["object"] = {"point": (x, y)}
        elif event == cv2.EVENT_RBUTTONDOWN and args.mode == "handover":
            prompts["hand"] = (x, y)
    cv2.setMouseCallback("CAD Capture", on_mouse)

    print("SPACE freeze | left click object | right click hand (handover) | s save | r reset | c resume | q quit")
    try:
        while True:
            if frozen is None:
                if zed.grab() != sl.ERROR_CODE.SUCCESS:
                    continue
                zed.retrieve_image(left_mat, sl.VIEW.LEFT)
                zed.retrieve_image(right_mat, sl.VIEW.RIGHT)
                left_bgr = cv2.resize(left_mat.get_data()[:, :, :3], (WIDTH, HEIGHT)).copy()
                right_bgr = cv2.resize(right_mat.get_data()[:, :, :3], (WIDTH, HEIGHT)).copy()
                display = left_bgr.copy()
                cv2.putText(display, "SPACE: freeze", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
            else:
                display = frozen[0].copy()
                if prompts["object"]:
                    cv2.circle(display, prompts["object"]["point"], 6, (0, 255, 0), 2)
                if prompts["hand"]:
                    cv2.circle(display, prompts["hand"], 6, (255, 80, 30), 2)
                cv2.putText(display, "L: object  R: hand  S: save  C: resume", (8, 23),
                            cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1)
            cv2.imshow("CAD Capture", display)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q")):
                break
            if frozen is None:
                if key == 32:
                    frozen = (left_bgr.copy(), right_bgr.copy())
                    prompts = {"object": None, "hand": None}
                continue
            if key == ord("c"):
                frozen = None
                continue
            if key == ord("r"):
                prompts = {"object": None, "hand": None}
                continue
            if key != ord("s"):
                continue
            if prompts["object"] is None:
                print("Click on object before saving")
                continue
            if args.mode == "handover" and prompts["hand"] is None:
                print("Handover mode: right-click a visible HAND area before saving")
                continue
            try:
                masks = segment(frozen[0], prompts["object"], prompts["hand"], predictor)
                if 2 not in masks or not masks[2].any():
                    raise RuntimeError("Object mask empty; choose another prompt")
                disp, depth = stereo_depth(ffs, frozen[0], frozen[1], K, baseline)
                capture_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                destination = out_root / session / capture_id
                # Process everything first; never create a partially processed sample as 'valid'.
                depth_mm = (depth * 1000).astype(np.uint16)
                cloud_all = points_from_depth(depth, depth > 0, frozen[0], K)
                cloud_obj = points_from_depth(depth, masks[2], frozen[0], K)
                cloud_hand = points_from_depth(depth, masks.get(1, np.zeros_like(masks[2])), frozen[0], K)
                destination.mkdir(parents=True, exist_ok=False)
                cv2.imwrite(str(destination / "rgb_left.png"), frozen[0])
                cv2.imwrite(str(destination / "rgb_right.png"), frozen[1])
                cv2.imwrite(str(destination / "depth_mm.png"), depth_mm)
                np.save(destination / "disparity_px.npy", disp)
                cv2.imwrite(str(destination / "mask_object.png"), masks[2] * 255)
                if 1 in masks:
                    cv2.imwrite(str(destination / "mask_hand.png"), masks[1] * 255)
                o3d.io.write_point_cloud(str(destination / "cloud_scene.ply"), cloud_all)
                o3d.io.write_point_cloud(str(destination / "cloud_object.ply"), cloud_obj)
                if 1 in masks:
                    o3d.io.write_point_cloud(str(destination / "cloud_hand.ply"), cloud_hand)
                meta = {
                    "schema_version": 1, "capture_id": capture_id, "session": session,
                    "object_id": args.object_id, "mode": args.mode,
                    "capture_time_local": datetime.now().astimezone().isoformat(),
                    "camera": "ZED 2i", "frame": "zed_left_camera_frame_optical",
                    "depth_algorithm": "Fast-FoundationStereo", "depth_unit": "mm",
                    "pointcloud_unit": "m", "image_size_wh": [WIDTH, HEIGHT],
                    "K": K.tolist(), "baseline_m": baseline, "prompt": prompts,
                    "points": {"scene": len(cloud_all.points), "object": len(cloud_obj.points),
                               "hand": len(cloud_hand.points)},
                    "pose_gt_available": False,
                    "robot_pose_recorded": False,
                    "notes": "No robot commands; manually positioned camera/arm. Masks require human inspection."
                }
                (destination / "metadata.json").write_text(
                    json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
                print(f"Saved {destination} | object points={len(cloud_obj.points)}")
                frozen = None
            except Exception as exc:
                print(f"Capture not saved: {exc}. Reset prompts with r and retry.")
    finally:
        cv2.destroyAllWindows()
        zed.close()


if __name__ == "__main__":
    main()
