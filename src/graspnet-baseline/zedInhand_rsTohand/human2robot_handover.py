import numpy as np
import cv2
import torch
import open3d as o3d
import os
import sys
import argparse
import datetime
import scipy.io as scio
import time
import logging
import base64
import re
import json
import threading
from pathlib import Path

# ====== Volcengine SDK ======
try:
    from volcenginesdkarkruntime import Ark
except ImportError:
    Ark = None

# ====== ZED Camera ======
import pyzed.sl as sl

# ====== ROS2 & RTDE Imports ======
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from robotiq_2f_urcap_adapter.action import GripperCommand
# =================================
import mediapipe as mp
import pyrealsense2 as rs

logging.basicConfig(level=logging.INFO, format='%(message)s')

# ===== GPU config (SAM2 requires bfloat16) =====
torch.autocast(device_type="cuda", dtype=torch.bfloat16).__enter__()
if torch.cuda.get_device_properties(0).major >= 8:
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True

# === 路径设置 ===
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
BASELINE_DIR = ROOT_DIR if os.path.isdir(os.path.join(ROOT_DIR, "models")) else os.path.dirname(ROOT_DIR)
SRC_DIR = os.path.dirname(BASELINE_DIR)
sys.path.insert(0, os.path.dirname(SRC_DIR))
from robotiq_grasp import grasp_meshes, grasp_tcp_pose, filter_grasps
from grasp_preparation import prepare_ready_grasps, ready_rejection_reason, filter_by_camera_direction
from h2r_interaction import FistConfirmation, unique_hand, hands_separated, segmentation_rejection_reason, manual_segmentation_prompts
from camera_view import create_camera_view, draw_handover_hint, draw_physical_hand
from handover_runtime import load_home_joints, move_home, load_motion_settings

sys.path.append(BASELINE_DIR)
sys.path.append(os.path.join(BASELINE_DIR, 'models'))
sys.path.append(os.path.join(BASELINE_DIR, 'dataset'))
sys.path.append(os.path.join(BASELINE_DIR, 'utils'))

from models.graspnet import GraspNet, pred_decode
from graspnetAPI import GraspGroup

# SAM2 path
SAM2_DIR = os.path.join(SRC_DIR, "SAM2_streaming")
sys.path.insert(0, SAM2_DIR)
from sam2.build_sam import build_sam2_camera_predictor

# FFS path
FFS_DIR = os.path.join(SRC_DIR, "Fast-FoundationStereo")
sys.path.append(FFS_DIR)
from core.utils.utils import InputPadder
from Utils import AMP_DTYPE, vis_disparity

# FFS Constants
MODEL_DIR = os.path.join(FFS_DIR, "weights/23-36-37/model_best_bp2_serialize.pth")
VALID_ITERS = 8
MAX_DISP = 192
ZFAR = 5.0
ZNEAR = 0.2

from data_utils import CameraInfo, create_point_cloud_from_depth_image

# ===== MLLM Config (大模型) =====
ARK_API_KEY = os.getenv("ARK_API_KEY")
ARK_BASE_URL = os.getenv("ARK_BASE_URL", "https://ark.cn-beijing.volces.com/api/v3")
DOUBAO_MODEL_NAME = os.getenv("DOUBAO_MODEL_NAME", "doubao-seed-2-0-mini-260215")
REASONING_EFFORT = os.getenv("REASONING_EFFORT", "minimal")

client = Ark(
    base_url=ARK_BASE_URL,
    api_key=ARK_API_KEY,
) if ARK_API_KEY and Ark is not None else None

# ===== MediaPipe Config ======
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
PHYSICAL_RIGHT_HAND_MP_LABEL = "Left"
PHYSICAL_LEFT_HAND_MP_LABEL = "Right"
HANDOVER_LEFT_HAND_TIMEOUT = 5.0
HANDOVER_LEFT_HAND_MAX_AGE = 3.0
FOLLOW_MOVE_SPEED = 0.05
FOLLOW_MOVE_ACCEL = 0.05
FOLLOW_MAX_XY_STEP = 0.03
FOLLOW_MAX_Z_STEP = 0.02
WORKSPACE_OBSERVE_BASE_OFFSET = 0.10
WORKSPACE_OBSERVE_HEIGHT = 0.30
HANDOVER_OBSERVE_HEIGHT = 0.20  # TCP height above the palm; workspace floor still applies.
WORKSPACE_RELEASE_HEIGHT = 0.10
SAM2_SETTLE_SECONDS = 1.0
DEBUG_MODE = False

# ===== ZED Downscale 参数 =====
SCALE_FACTOR = 0.5
IMG_WIDTH = int(1280 * SCALE_FACTOR) # 640
IMG_HEIGHT = int(720 * SCALE_FACTOR) # 360
MASK_ALPHA = 0.5
MASK_COLORS_BGR = {1: [75, 70, 203], 2: [203, 192, 75]}
DENOISE_VOXEL_SIZE = 0.001
DENOISE_NB_POINTS = 30
DENOISE_RADIUS = 0.03
CAMERA_GRASP_MAX_TILT_DEG = 30.0
CAMERA_GRASP_MAX_CLOSING_FORWARD_COS = 0.75
BASE_TO_CONTROLLER_ROT = np.diag([-1.0, -1.0, 1.0]).astype(np.float32)
GRASP_BACKOFF_DIST = 0.10

# === 全局变量 ===
vlm_running = False
doubao_trigger = False
workspace_bbox = None
pending_targets = []
global ROS_NODE
ROS_NODE = None
global STATE
STATE = "IDLE"
OPEN3D_VIEWER = None


class Open3DLiveViewer:
    def __init__(self, window_name="Open3D Grasp Viewer"):
        self.window_name = window_name
        self.vis = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready_event = threading.Event()
        self.pending_update = None
        self.cloud = o3d.geometry.PointCloud()
        self.cloud_added = False
        self.gripper_geometries = []
        self.view_initialized = False
        self.render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self.render_thread.start()

    def _render_loop(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=self.window_name, width=1280, height=720, visible=True)
        self.ready_event.set()

        while not self.stop_event.is_set():
            with self.lock:
                update_payload = self.pending_update
                self.pending_update = None

            if update_payload is not None:
                points, colors, gg_array, top_k, force_flips = update_payload
                self.cloud.points = o3d.utility.Vector3dVector(points)
                self.cloud.colors = o3d.utility.Vector3dVector(colors)

                if not self.cloud_added:
                    self.vis.add_geometry(self.cloud)
                    self.cloud_added = True
                else:
                    self.vis.update_geometry(self.cloud)

                for geom in self.gripper_geometries:
                    self.vis.remove_geometry(geom, reset_bounding_box=False)
                self.gripper_geometries = []

                gg = (GraspGroup(gg_array) if force_flips is not None
                      else GraspGroup(gg_array).nms().sort_by_score())
                if len(gg) > 0:
                    show_n = min(len(gg), max(int(top_k), 1))
                    shown_flips = None if force_flips is None else force_flips[:show_n]
                    self.gripper_geometries = grasp_meshes(gg[:show_n], force_flips=shown_flips)
                    for geom in self.gripper_geometries:
                        self.vis.add_geometry(geom, reset_bounding_box=False)

                if not self.view_initialized:
                    ctr = self.vis.get_view_control()
                    ctr.set_front([0, 0, -1])
                    ctr.set_lookat([0, 0, 0.5])
                    ctr.set_up([0, -1, 0])
                    ctr.set_zoom(0.8)
                    self.view_initialized = True

            self.vis.poll_events()
            self.vis.update_renderer()
            time.sleep(0.02)

        self.vis.destroy_window()
        self.vis = None

    def update(self, points, colors, gg_array, top_k=1, force_flips=None):
        self.ready_event.wait()
        with self.lock:
            self.pending_update = (
                np.asarray(points, dtype=np.float64),
                np.asarray(colors, dtype=np.float64),
                np.asarray(gg_array),
                top_k,
                force_flips,
            )

    def close(self):
        self.stop_event.set()
        if self.render_thread.is_alive():
            self.render_thread.join(timeout=1.0)
        with self.lock:
            self.pending_update = None
            self.cloud_added = False
            self.gripper_geometries = []
            self.view_initialized = False


def get_open3d_viewer():
    global OPEN3D_VIEWER
    if OPEN3D_VIEWER is None:
        OPEN3D_VIEWER = Open3DLiveViewer()
    return OPEN3D_VIEWER


def close_open3d_viewer():
    global OPEN3D_VIEWER
    if OPEN3D_VIEWER is not None:
        OPEN3D_VIEWER.close()
        OPEN3D_VIEWER = None

# --- ROS2 机器人控制节点 ---
class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_2f_urcap_adapter/gripper_command', callback_group=self.callback_group)
        self.ur_ip = "192.168.1.10"
        
        # 使用 Socket 提前检测机器人的连通性，防止 ur_rtde 底层因无法连接而引发内部崩溃或退出
        import socket
        robot_reachable = False
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            if sock.connect_ex((self.ur_ip, 30004)) == 0:
                robot_reachable = True
            sock.close()
        except:
            pass

        self.rtde_c = None
        self.rtde_r = None
        self.dynamic_start_pose = None
        self.dynamic_workspace_center_pose = None
        self.left_hand_base_pos = None
        self.left_hand_seen_time = 0.0
        self.left_hand_lock = threading.Lock()
        self.handover_follow_pose = None
        self.hand_observation_error = ""
        self.zed_selection_mode_callback = None
        self.motion_failed = threading.Event()
        self.home_revision = 0
        self.gripper_force = load_motion_settings()["gripper_force_N"]
        self.motion_lock = threading.Lock()
        self.motion_error = ""
        self.recovery_pending = False
        if robot_reachable:
            try:
                self.rtde_c = RTDEControlInterface(self.ur_ip)
                self.rtde_r = RTDEReceiveInterface(self.ur_ip)
                self.get_logger().info("✓ Connected to UR3e robot arm.")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize RTDE: {e}")
                self.rtde_c = None
        else:
            self.get_logger().warning(f"UR3e robot arm at {self.ur_ip} is unreachable! Running in VISION-ONLY mode.")

    def hold_motion_error(self, error):
        global STATE
        first_failure = not self.motion_failed.is_set()
        self.motion_error = str(error)
        self.motion_failed.set()
        if first_failure:
            self.recovery_pending = True
        STATE = "ERROR"
        self.get_logger().error(f"H2R stopped; Home recovery requires a normal stationary controller: {error}")

    def ensure_motion_ready(self):
        if self.motion_failed.is_set():
            raise RuntimeError(self.motion_error or "H2R stopped")
        if (self.rtde_c is None or self.rtde_r is None
                or not self.rtde_c.isConnected() or not self.rtde_r.isConnected()):
            raise RuntimeError("RTDE disconnected")
        if self.rtde_r.getSafetyMode() != 1 or self.rtde_r.getRobotMode() != 7:
            raise RuntimeError("Robot is not in normal running/safety state")
        if not self.rtde_c.isProgramRunning():
            raise RuntimeError("RTDE control script stopped; restart explicitly after checking the cause")

    def checked_move(self, target, speed, acceleration, joint=False):
        """Check controller state/results; never clear a safety stop or restart a script."""
        if not self.motion_lock.acquire(blocking=False):
            raise RuntimeError("Another H2R motion is still running")
        try:
            self.ensure_motion_ready()
            if len(target) != 6 or not np.all(np.isfinite(target)):
                raise ValueError("Invalid motion target")
            if joint:
                if not self.rtde_c.isJointsWithinSafetyLimits(target):
                    raise RuntimeError("Joint safety check rejected target")
            else:
                start_q = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                seed = start_q.copy()
                start_pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
                start_rot = R.from_rotvec(start_pose[3:])
                turn = (start_rot.inv() * R.from_rotvec(target[3:])).as_rotvec()
                samples = max(1, math.ceil(np.linalg.norm(np.asarray(target[:3]) - start_pose[:3]) / .01),
                              math.ceil(np.linalg.norm(turn) / math.radians(5)))
                for fraction in np.linspace(0., 1., samples + 1)[1:]:
                    self.ensure_motion_ready()
                    pose = np.r_[start_pose[:3] + fraction * (np.asarray(target[:3]) - start_pose[:3]),
                                 (start_rot * R.from_rotvec(fraction * turn)).as_rotvec()].tolist()
                    if not self.rtde_c.getInverseKinematicsHasSolution(pose, seed.tolist()):
                        raise RuntimeError(f"No IK solution along Cartesian path: {pose}")
                    joints = np.asarray(self.rtde_c.getInverseKinematics(pose, seed.tolist()), dtype=float)
                    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                        raise RuntimeError("Invalid path IK result")
                    joints += 2 * math.pi * np.round((seed - joints) / (2 * math.pi))
                    if np.max(np.abs(joints - seed)) > math.radians(15):
                        raise RuntimeError("Cartesian path IK branch jump / excessive joint step")
                    if (not self.rtde_c.isPoseWithinSafetyLimits(pose)
                            or not self.rtde_c.isJointsWithinSafetyLimits(joints.tolist())):
                        raise RuntimeError("Cartesian path rejected by controller safety limits")
                    seed = joints
                if np.max(np.abs(np.asarray(self.rtde_r.getActualQ()) - start_q)) > .01:
                    raise RuntimeError("Robot moved during Cartesian path validation")
            move = self.rtde_c.moveJ if joint else self.rtde_c.moveL
            if not move(target, speed, acceleration):
                raise RuntimeError("Joint motion failed" if joint else "Cartesian motion failed")
            self.ensure_motion_ready()
            actual = self.rtde_r.getActualQ() if joint else self.rtde_r.getActualTCPPose()
            if joint:
                reached = np.max(np.abs(np.asarray(actual) - target)) <= .01
            else:
                reached = (np.linalg.norm(np.asarray(actual[:3]) - target[:3]) <= .005
                           and (R.from_rotvec(actual[3:]).inv() * R.from_rotvec(target[3:])).magnitude() <= math.radians(1))
            if not reached:
                raise RuntimeError("Motion returned without reaching target")
        except Exception as error:
            self.hold_motion_error(error)
            raise
        finally:
            self.motion_lock.release()

    def move_linear(self, target, speed, acceleration):
        return self.checked_move(target, speed, acceleration)

    def move_joint(self, target, speed, acceleration):
        return self.checked_move(target, speed, acceleration, joint=True)

    def move_to_observation(self, pose):
        """Use joint motion from Home, whose straight elbow is singular for moveL."""
        self.ensure_motion_ready()
        seed = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        if not self.rtde_c.getInverseKinematicsHasSolution(pose, seed.tolist()):
            raise RuntimeError("No IK solution for workspace observation pose")
        joints = np.asarray(self.rtde_c.getInverseKinematics(pose, seed.tolist()), dtype=float)
        if joints.shape != (6,) or not np.all(np.isfinite(joints)):
            raise RuntimeError("Invalid workspace observation IK result")
        # Match R2H: use the controller's seeded IK result directly for moveJ.
        if not self.rtde_c.isJointsWithinSafetyLimits(joints.tolist()):
            raise RuntimeError("Workspace observation IK solution violates controller joint safety limits")
        if np.max(np.abs(np.asarray(self.rtde_r.getActualQ()) - seed)) > .01:
            raise RuntimeError("Robot moved during observation planning")
        self.get_logger().info("Moving to workspace observation with checked joint motion")
        self.move_joint(joints.tolist(), .30, .30)

    def send_gripper_command(self, position, speed=0.15, force=None):
        self.ensure_motion_ready()
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Gripper action server unavailable")
            return False
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = self.gripper_force if force is None else force
        goal_msg.command.max_speed = speed
        deadline = time.monotonic() + 8.0
        future = self._action_client.send_goal_async(goal_msg)
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                self.get_logger().error("Gripper goal acknowledgement timed out; check gripper service")
                return False
            time.sleep(.05)
        goal = future.result()
        if goal is None or not goal.accepted:
            return False
        result = goal.get_result_async()
        while not result.done():
            if time.monotonic() >= deadline:
                goal.cancel_goal_async()
                self.get_logger().error("Gripper action result timed out; cancellation requested")
                return False
            time.sleep(.05)
        response = result.result()
        return bool(response and (response.result.reached_goal or response.result.stalled))

    def recover_home(self, release=False):
        """Recover only with normal controller state; never reset a protective stop."""
        global STATE
        self.recovery_pending = False
        try:
            if self.motion_lock.locked():
                raise RuntimeError("Motion still running; Home recovery deferred")
            # Ignore only the software error latch for this explicit recovery.
            self.motion_failed.clear()
            if (self.rtde_c is None or self.rtde_r is None
                    or not self.rtde_c.isConnected() or not self.rtde_r.isConnected()):
                raise RuntimeError("RTDE disconnected")
            if self.rtde_r.getSafetyMode() != 1 or self.rtde_r.getRobotMode() != 7:
                raise RuntimeError("Robot is not in normal running/safety state")
            if max(abs(v) for v in self.rtde_r.getActualQd()) > .003:
                raise RuntimeError("Robot is still moving")
            # Only the explicit 0-key recovery may restart a stopped RTDE script.
            if release and not self.rtde_c.isProgramRunning():
                if not self.rtde_c.reuploadScript():
                    raise RuntimeError("RTDE control script restart failed")
            self.ensure_motion_ready()
            if release and not self.send_gripper_command(.085):
                self.get_logger().warning(
                    "Gripper release unconfirmed; returning Home on explicit 0 request. "
                    "The gripper may still be holding the object.")
            # A gripper service timeout must not cancel an explicitly requested Home.
            # Recheck the robot after waiting for the independent gripper service.
            self.ensure_motion_ready()
            move_home(self.rtde_c, self.rtde_r, load_home_joints())
            if not self.send_gripper_command(.085):
                raise RuntimeError("Home reached but gripper opening was not confirmed")
            self.motion_error = ""
            self.handover_follow_pose = None
            with self.left_hand_lock:
                self.left_hand_base_pos = None
                self.left_hand_seen_time = 0.0
            self.home_revision += 1
            STATE = "IDLE"
            self.get_logger().info("Home reached. Select workspace for a new round.")
            return True
        except Exception as error:
            self.motion_failed.set()
            self.motion_error = str(error)
            STATE = "ERROR"
            self.get_logger().error(f"Home recovery blocked: {error}. Check controller; press 0 to retry explicitly.")
            return False
        finally:
            # An exiting worker may report the same failure during recovery.
            # Consume that request too, rather than retrying on every frame.
            self.recovery_pending = False

    def update_left_hand_base_pos(self, hand_base_pos):
        with self.left_hand_lock:
            self.left_hand_base_pos = np.array(hand_base_pos, dtype=float)
            self.left_hand_seen_time = time.time()

    def get_recent_left_hand_base_pos(self, max_age=HANDOVER_LEFT_HAND_MAX_AGE):
        with self.left_hand_lock:
            if self.left_hand_base_pos is None:
                return None
            if time.time() - self.left_hand_seen_time > max_age:
                return None
            return self.left_hand_base_pos.copy()

    def wait_for_left_hand_base_pos(self, timeout=HANDOVER_LEFT_HAND_TIMEOUT):
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.ensure_motion_ready()
            hand_base_pos = self.get_recent_left_hand_base_pos(max_age=HANDOVER_LEFT_HAND_MAX_AGE)
            if hand_base_pos is not None:
                return hand_base_pos
            time.sleep(0.05)
        return None

    def build_follow_left_hand_pose(self, hand_base_pos, limit_step=True):
        """Observe from above using the workspace view, not a same-height folded pose.

        Height/offset are observation targets, not collision-clearance guarantees.
        Raise before translating when below the requested observation height.
        """
        hand = np.asarray(hand_base_pos, dtype=float)
        if hand.shape != (3,) or not np.all(np.isfinite(hand)):
            raise ValueError("Invalid left-hand position for overhead observation")
        reference = self.handover_follow_pose or self.dynamic_start_pose
        if self.dynamic_start_pose is None or reference is None:
            raise RuntimeError("Workspace overhead observation reference is missing")
        rotation = R.from_rotvec(reference[3:]).as_matrix()
        if rotation[2, 2] > -0.99:
            raise RuntimeError("H2R observation reference must keep the tool pointing down")

        current = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
        hand_xy = hand[:2]
        radius = np.linalg.norm(hand_xy)
        observe_xy = hand_xy.copy()
        if radius >= 1e-5:
            observe_xy -= hand_xy / radius * WORKSPACE_OBSERVE_BASE_OFFSET
        target_xy = -observe_xy  # ROS base_link -> controller base XY.
        # Never follow a low hand below the selected workspace observation plane.
        observe_z = max(float(hand[2]) + HANDOVER_OBSERVE_HEIGHT,
                        float(self.dynamic_start_pose[2]))
        if not limit_step:
            return [*target_xy.tolist(), observe_z, *reference[3:]]
        target_z = current[2] + np.clip(observe_z - current[2],
                                      -FOLLOW_MAX_Z_STEP, FOLLOW_MAX_Z_STEP)
        delta_xy = target_xy - current[:2]
        distance_xy = np.linalg.norm(delta_xy)
        if distance_xy > FOLLOW_MAX_XY_STEP:
            target_xy = current[:2] + delta_xy / distance_xy * FOLLOW_MAX_XY_STEP
        # Match the existing 5 mm endpoint tolerance: prioritize the upward leg.
        if observe_z - current[2] > .005:
            target_xy = current[:2].copy()
        return [*target_xy.tolist(), float(target_z), *reference[3:]]

    def hand_observation_rejection_reason(self, hand_base_pos):
        """Read-only endpoint preview shared by the READY UI and actual guidance."""
        self.ensure_motion_ready()
        goal = self.build_follow_left_hand_pose(hand_base_pos, limit_step=False)
        seed = self.rtde_r.getActualQ()
        if not self.rtde_c.getInverseKinematicsHasSolution(goal, seed):
            return "Overhead view unreachable: reposition LEFT hand"
        joints = self.rtde_c.getInverseKinematics(goal, seed)
        if (len(joints) != 6 or not np.all(np.isfinite(joints))
                or not self.rtde_c.isPoseWithinSafetyLimits(goal)
                or not self.rtde_c.isJointsWithinSafetyLimits(joints)):
            return "Overhead view rejected by controller limits: reposition LEFT hand"
        return None

    def follow_left_hand(self, hand_base_pos):
        self.ensure_motion_ready()
        # Check the full overhead target before issuing even the first lift step.
        # An unreachable hand must not cause a fallback to the former low view.
        error = self.hand_observation_rejection_reason(hand_base_pos)
        if error:
            if error != self.hand_observation_error:
                self.get_logger().warning(error)
            self.hand_observation_error = error
            return False
        self.hand_observation_error = ""
        follow_pose = self.build_follow_left_hand_pose(hand_base_pos)
        self.get_logger().info(f"Observing physical left hand from above (workspace orientation): {follow_pose}")
        self.move_linear(follow_pose, FOLLOW_MOVE_SPEED, FOLLOW_MOVE_ACCEL)
        return True

    def build_grasp_plan(self, grasp, force_flip=None):
        """Convert the selected and collision-checked gripper orientation into TCP poses."""
        self.ensure_motion_ready()
        tf_msg = self.tf_buffer.lookup_transform(
            'base_link', 'zed_inhand_camera_frame_optical', rclpy.time.Time())
        tf_trans = np.array([tf_msg.transform.translation.x,
                             tf_msg.transform.translation.y,
                             tf_msg.transform.translation.z])
        tf_rot = R.from_quat([tf_msg.transform.rotation.x,
                              tf_msg.transform.rotation.y,
                              tf_msg.transform.rotation.z,
                              tf_msg.transform.rotation.w])
        camera_transform = np.eye(4)
        camera_transform[:3, :3] = BASE_TO_CONTROLLER_ROT @ tf_rot.as_matrix()
        camera_transform[:3, 3] = BASE_TO_CONTROLLER_ROT @ tf_trans
        current_rotation = R.from_rotvec(self.rtde_r.getActualTCPPose()[3:]).as_matrix()
        target_pose = grasp_tcp_pose(
            grasp, camera_transform, current_rotation, force_flip=force_flip).tolist()
        tool_z = R.from_rotvec(target_pose[3:]).as_matrix()[:, 2]
        approach_pose = list(target_pose)
        for i in range(3):
            approach_pose[i] -= GRASP_BACKOFF_DIST * tool_z[i]
        return {"approach": approach_pose, "grasp": target_pose, "retreat": list(approach_pose)}

    def validate_grasp_plan(self, plan):
        """Endpoint IK and controller-limit screening; does not certify full path clearance."""
        self.ensure_motion_ready()
        seed = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        for name in ("approach", "grasp", "retreat"):
            pose = plan[name]
            if not self.rtde_c.getInverseKinematicsHasSolution(pose, seed.tolist()):
                raise ValueError(f"{name}: no IK solution; xyz={pose[:3]}")
            joints = np.asarray(self.rtde_c.getInverseKinematics(pose, seed.tolist()), dtype=float)
            if joints.shape != (6,) or not np.all(np.isfinite(joints)):
                raise ValueError(f"{name}: invalid IK result; xyz={pose[:3]}")
            if (not self.rtde_c.isPoseWithinSafetyLimits(pose)
                    or not self.rtde_c.isJointsWithinSafetyLimits(joints.tolist())):
                raise ValueError(f"{name}: controller safety check rejected pose; xyz={pose[:3]}")
            seed = joints

    def execute_grasp(self, gg, viz_process=None, require_confirmation=True, prepared_plan=None):
        if not self.rtde_c: 
            self.get_logger().error("RTDE not connected.")
            return

        def close_viz_process():
            if viz_process is None:
                return
            try:
                if viz_process.is_alive():
                    viz_process.terminate()
                    viz_process.join(timeout=1.0)
            except Exception as e:
                self.get_logger().warning(f"Failed to close Open3D visualization process: {e}")
        try:
            plan = prepared_plan if prepared_plan is not None else self.build_grasp_plan(gg[0])
            self.validate_grasp_plan(plan)
            target_pose = plan["grasp"]
            approach_pose = plan["approach"]
            
            self.get_logger().info(f"Target pose (Controller): {target_pose}")

            if not self.send_gripper_command(0.085):
                raise RuntimeError("Gripper open failed")
            
            if require_confirmation:
                ans = input("\nReady to grasp? Press 'y' to continue, 'n' to cancel: ")
                if ans.lower() != 'y':
                    close_viz_process()
                    self.get_logger().info("Grasp cancelled.")
                    return
                close_viz_process()

            self.get_logger().info("Moving to approach pose...")
            self.move_linear(approach_pose, 0.05, 0.05)
            self.get_logger().info("Moving to target pose...")
            self.move_linear(target_pose, 0.05, 0.05)
            self.get_logger().info("Closing gripper...")
            if not self.send_gripper_command(0.0):
                raise RuntimeError("Gripper close failed")
            
            self.get_logger().info("Moving back to approach pose...")
            self.move_linear(approach_pose, 0.05, 0.05)
            
            # Retire the folded-pose/workspace-place route. Release only after Home.
            # Use the existing Home configuration and checked joint-motion wrapper.
            global STATE
            STATE = "RETURNING_HOME"
            self.get_logger().info("Object received and retreated. Returning Home before releasing the object...")
            self.move_joint(load_home_joints(), 0.30, 0.30)
            if not self.send_gripper_command(.085):
                raise RuntimeError("Home reached but gripper release was not confirmed")
            self.handover_follow_pose = None
            STATE = "HOME_READY"
            self.home_revision += 1
            self.get_logger().info("Home reached; gripper open. Resetting selection for the next round.")

        except Exception as e:
            self.hold_motion_error(e)

    def deliver_to_hand(self, hand_base_pos):
        # Called when MediaPipe finds the hand and user triggers delivery
        try:
            current_tcp_pose = self.rtde_r.getActualTCPPose()
            rx, ry, rz = current_tcp_pose[3], current_tcp_pose[4], current_tcp_pose[5]
            
            DELIVER_X_OFFSET = 0.02
            
            c_x = -hand_base_pos[0] + DELIVER_X_OFFSET
            c_y = -hand_base_pos[1]
            c_z = hand_base_pos[2]

            deliver_target_pose = [c_x, c_y, c_z, rx, ry, rz]

            self.get_logger().info(f"Delivering to Hand at Controller Pose: {deliver_target_pose}")
            self.move_linear(deliver_target_pose, 0.05, 0.05)
            
            if not self.send_gripper_command(0.085):
                raise RuntimeError("Gripper open failed")
            self.get_logger().info("Object Released to Human.")
            time.sleep(0.5)
            
            try:
                current_q = self.rtde_r.getActualQ()
                initial_q = [current_q[0], -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
                self.get_logger().info(f"Returning to initial joint shape before workspace start: {initial_q}")
                self.move_joint(initial_q, 0.15, 0.15)
            except Exception as e:
                raise RuntimeError("Failed to return to initial joint shape") from e

            if self.dynamic_start_pose:
                self.get_logger().info("Moving from initial joint shape to workspace start pose...")
                self.move_linear(self.dynamic_start_pose, 0.15, 0.15)
            else:
                start_joint_q = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
                self.move_joint(start_joint_q, 0.15, 0.15)
            self.get_logger().info("Returned to workspace/start position.")
            
            global STATE
            STATE = "IDLE"
            
        except Exception as e:
            self.get_logger().error(f"Delivery failed: {e}")

# === MLLM 工作台识别 API 调用 ===
def call_doubao_workspace(image_bgr):
    if client is None:
        logging.error("MLLM unavailable: configure ARK_API_KEY and the Ark SDK, or use manual selection.")
        return None

    _, buffer = cv2.imencode('.jpg', image_bgr)
    img_base64 = base64.b64encode(buffer).decode('utf-8')
    data_url = f"data:image/jpeg;base64,{img_base64}"

    grounding_text = (
        "Instructions: \n"
        "1. Detect the main workspace, desk, table, or flat surface where objects are located for manipulation.\n"
        "2. Provide its location in JSON format as follows: \n"
        "   {\"workspace\": [ymin, xmin, ymax, xmax]}\n"
        "   Coordinates should be normalized 0-1000. \n"
        "3. If no clear workspace is detected, return exactly: 'No workspace detected'."
    )
    
    try:
        kwargs = {
            "model": MODEL_NAME,
            "input": [
                {
                    "role": "system",
                    "content": [{"type": "input_text", "text": "You are a professional vision assistant for robot perception."}]
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "input_image", "image_url": data_url},
                        {"type": "input_text", "text": grounding_text},
                    ],
                }
            ]
        }
        if REASONING_EFFORT != "minimal":
            kwargs["reasoning_effort"] = REASONING_EFFORT

        response = client.responses.create(**kwargs)
        
        content = ""
        if hasattr(response, 'choices') and len(response.choices) > 0:
            content = response.choices[0].message.content
        elif hasattr(response, 'output') and len(response.output) > 1:
             content = response.output[1].content[0].text
        else:
            content = str(response)

        logging.info(f"Doubao Workspace Response: {content}")

        if "No workspace detected" in content:
            logging.warning("MLLM: Workspace not found in frame.")
            return None

        try:
            json_str = re.search(r'\{.*\}', content, re.DOTALL)
            if json_str:
                data = json.loads(json_str.group())
                h, w = image_bgr.shape[:2]
                
                if "workspace" in data:
                    ymin, xmin, ymax, xmax = data["workspace"]
                    bbox = (int(xmin*w/1000), int(ymin*h/1000), int(xmax*w/1000), int(ymax*h/1000))
                    return bbox
        except Exception as je:
            logging.error(f"JSON Parse failed: {je}")
            
        return None
    except Exception as e:
        logging.error(f"API call failed: {e}")
        return None

def vlm_thread_worker(image_bgr):
    global workspace_bbox, vlm_running
    try:
        bbox = call_doubao_workspace(image_bgr)
        if bbox:
            workspace_bbox = bbox
            logging.info(f"MLLM Worker: Found workspace at {bbox}")
    finally:
        vlm_running = False

# === 工作台平面法向量评估 ===
def estimate_workspace_normal(bbox, depth_img, camera_info, margin=15):
    """基于工作区域边界框，提取局部点云并用 RANSAC 拟合工作平面提取法向量"""
    x1, y1, x2, y2 = bbox
    x1 = max(0, x1 + margin)
    y1 = max(0, y1 + margin)
    x2 = min(depth_img.shape[1], x2 - margin)
    y2 = min(depth_img.shape[0], y2 - margin)
    
    crop_mask = np.zeros(depth_img.shape, dtype=bool)
    crop_mask[y1:y2, x1:x2] = True
    
    # 获取非空深度的掩码点
    mask = (crop_mask) & (depth_img > 0)
    
    # 构建完整点云，提取框内的有效点
    cloud = create_point_cloud_from_depth_image(depth_img, camera_info, organized=False)
    depth_flat = depth_img.flatten()
    mask_flat = mask.flatten()
    points = cloud[mask_flat]
    
    # 滤除原点异常值
    points = points[np.linalg.norm(points, axis=1) > 0.1]
    
    default_normal = np.array([0., 0., -1.]) # 默认朝向相机

    if len(points) < 500:
        logging.warning("Workspace points too few, using default Z-axis logic.")
        return default_normal
        
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size=0.01)
    
    if len(pcd.points) < 30:
        return default_normal
        
    try:
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        normal = np.array(plane_model[:3])
        
        # 相机视野为光学坐标系(Z向前)，平面法向应该是指向相机的，使得其 Z 轴分量一般为负
        if normal[2] > 0:
            normal = -normal
            
        logging.info(f"Estimated workspace normal (in camera frame): {normal}")
        return normal
    except Exception as e:
        logging.warning(f"Plane estimation failed: {e}. Fallback to default.")
        return default_normal


def start_zed():
    logging.info("Initializing ZED Camera...")
    zed = sl.Camera()
    init_params = sl.InitParameters()
    if os.getenv("URHO_ZED_SETTINGS_PATH"):
        init_params.optional_settings_path = os.environ["URHO_ZED_SETTINGS_PATH"]
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.coordinate_units = sl.UNIT.METER

    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to open ZED Camera: {err}")

    cam_info = zed.get_camera_information()
    calib = cam_info.camera_configuration.calibration_parameters
    left_cam = calib.left_cam
    baseline = calib.get_camera_baseline()

    fx_scaled = left_cam.fx * SCALE_FACTOR
    fy_scaled = left_cam.fy * SCALE_FACTOR
    cx_scaled = left_cam.cx * SCALE_FACTOR
    cy_scaled = left_cam.cy * SCALE_FACTOR

    K = np.array([
        [fx_scaled, 0, cx_scaled],
        [0, fy_scaled, cy_scaled],
        [0, 0, 1]
    ], dtype=np.float32)

    camera_info_graspnet = CameraInfo(float(IMG_WIDTH), float(IMG_HEIGHT), fx_scaled, fy_scaled, cx_scaled, cy_scaled, 1000.0)
    
    return zed, K, baseline, camera_info_graspnet


# --- SAM2 Global State & Mouse Callback ---
drawing = False
ix, iy, fx_mouse, fy_mouse = -1, -1, -1, -1
pending_bbox = None
pending_point = None
zed_drawing = False
zed_ix, zed_iy, zed_fx_mouse, zed_fy_mouse = -1, -1, -1, -1
zed_pending_bbox = None
zed_pending_point = None

rs_selection_locked = False

def mouse_callback(event, x, y, flags, param):
    global drawing, ix, iy, fx_mouse, fy_mouse, pending_bbox, pending_point, rs_selection_locked

    if STATE != "IDLE" or rs_selection_locked or num_targets > 0 or pending_bbox is not None or pending_point is not None:
        if event == cv2.EVENT_LBUTTONDOWN:
            logging.info("D435 target locked. Press r before selecting another region.")
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        fx_mouse, fy_mouse = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            fx_mouse, fy_mouse = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        if not drawing:
            return
        rs_selection_locked = True
        drawing = False
        fx_mouse, fy_mouse = x, y
        dx = abs(fx_mouse - ix)
        dy = abs(fy_mouse - iy)
        if dx > 8 and dy > 8:
            x1, y1 = min(ix, fx_mouse), min(iy, fy_mouse)
            x2, y2 = max(ix, fx_mouse), max(iy, fy_mouse)
            pending_bbox = (x1, y1, x2, y2)
        else:
            pending_point = (x, y)

def zed_mouse_callback(event, x, y, flags, param):
    global zed_drawing, zed_ix, zed_iy, zed_fx_mouse, zed_fy_mouse, zed_pending_bbox, zed_pending_point

    if STATE != "HANDOVER_SEGMENTATION":
        zed_drawing = False
        if event == cv2.EVENT_LBUTTONDOWN:
            logging.info("ZED selection locked: finish positioning with RIGHT fist 2s, then select the object in segmentation stage.")
        return
    if event == cv2.EVENT_LBUTTONDOWN:
        zed_drawing = True
        zed_ix, zed_iy = x, y
        zed_fx_mouse, zed_fy_mouse = x, y
    elif event == cv2.EVENT_MOUSEMOVE:
        if zed_drawing:
            zed_fx_mouse, zed_fy_mouse = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        if not zed_drawing:
            return
        zed_drawing = False
        zed_fx_mouse, zed_fy_mouse = x, y
        dx = abs(zed_fx_mouse - zed_ix)
        dy = abs(zed_fy_mouse - zed_iy)
        if dx > 8 and dy > 8:
            x1, y1 = min(zed_ix, zed_fx_mouse), min(zed_iy, zed_fy_mouse)
            x2, y2 = max(zed_ix, zed_fx_mouse), max(zed_iy, zed_fy_mouse)
            zed_pending_bbox = (x1, y1, x2, y2)
        else:
            zed_pending_point = (x, y)


def select_physical_hand(results, mp_label):
    return unique_hand(results, mp_label)


def select_right_hand(results):
    return select_physical_hand(results, PHYSICAL_RIGHT_HAND_MP_LABEL)

def select_left_hand(results):
    return select_physical_hand(results, PHYSICAL_LEFT_HAND_MP_LABEL)

def landmark_dist2d(lm1, lm2):
    return math.hypot(lm1.x - lm2.x, lm1.y - lm2.y)

def is_fist(hand_landmarks):
    wrist_lm = hand_landmarks.landmark[0]
    tips = [8, 12, 16, 20]
    mcps = [5, 9, 13, 17]
    curled_count = sum(
        1
        for tip_idx, mcp_idx in zip(tips, mcps)
        if landmark_dist2d(hand_landmarks.landmark[tip_idx], wrist_lm)
        < landmark_dist2d(hand_landmarks.landmark[mcp_idx], wrist_lm) * 1.2
    )
    return curled_count == 4

def is_open_palm(hand_landmarks):
    wrist = hand_landmarks.landmark[0]
    return all(
        landmark_dist2d(hand_landmarks.landmark[tip], wrist)
        > landmark_dist2d(hand_landmarks.landmark[mcp], wrist) * 1.5
        for tip, mcp in zip((8, 12, 16, 20), (5, 9, 13, 17))
    )


def is_thumbs_up(hand_landmarks):
    lm = hand_landmarks.landmark
    wrist_lm = lm[0]
    thumb_tip = lm[4]
    thumb_ip = lm[3]
    thumb_mcp = lm[2]

    fingers_curled = all(
        landmark_dist2d(lm[tip_idx], wrist_lm) < landmark_dist2d(lm[mcp_idx], wrist_lm) * 1.25
        for tip_idx, mcp_idx in zip([8, 12, 16, 20], [5, 9, 13, 17])
    )
    thumb_extended = landmark_dist2d(thumb_tip, wrist_lm) > landmark_dist2d(thumb_mcp, wrist_lm) * 1.15
    thumb_points_up = thumb_tip.y < thumb_ip.y and thumb_tip.y < lm[5].y

    return fingers_curled and thumb_extended and thumb_points_up

def calc_hand_base_pos_from_ffs(l_rgb, r_rgb, K_s, bl, pt_x, pt_y, max_depth=1.5, log_prefix="hand"):
    logging.info(f"Calculating depth for {log_prefix} via FFS...")
    with torch.autograd.set_grad_enabled(False):
        ffs_model = torch.load(MODEL_DIR, map_location='cpu', weights_only=False)
        ffs_model.args.valid_iters = VALID_ITERS
        ffs_model.args.max_disp = MAX_DISP
        ffs_model.cuda().eval()
        
        img0 = torch.as_tensor(l_rgb).cuda().float()[None].permute(0, 3, 1, 2)
        img1 = torch.as_tensor(r_rgb).cuda().float()[None].permute(0, 3, 1, 2)
        
        padder = InputPadder(img0.shape, divis_by=32, force_square=False)
        img0_p, img1_p = padder.pad(img0, img1)

        with torch.amp.autocast('cuda', enabled=True, dtype=AMP_DTYPE):
            disp = ffs_model.forward(img0_p, img1_p, iters=VALID_ITERS, test_mode=True, optimize_build_volume='pytorch1')
        
        disp = padder.unpad(disp.float())
        disp = disp.data.cpu().numpy().reshape(IMG_HEIGHT, IMG_WIDTH).clip(0, None)
    
    del ffs_model
    del img0, img1, img0_p, img1_p
    torch.cuda.empty_cache()

    xx = np.arange(IMG_WIDTH)[None, :].repeat(IMG_HEIGHT, axis=0)
    invalid = (xx - disp) < 0
    disp[invalid] = np.inf

    with np.errstate(divide='ignore', invalid='ignore'):
        depth_ffs = K_s[0, 0] * bl / disp

    pt_x = int(np.clip(pt_x, 0, IMG_WIDTH - 1))
    pt_y = int(np.clip(pt_y, 0, IMG_HEIGHT - 1))
    depth_val = depth_ffs[pt_y, pt_x]
    
    if not (0.1 < depth_val < max_depth):
        logging.warning(f"Calculated depth for {log_prefix} is invalid/out-of-bounds.")
        return None

    fx = K_s[0, 0]
    fy = K_s[1, 1]
    px = K_s[0, 2]
    py = K_s[1, 2]
    x_cam = (pt_x - px) * depth_val / fx
    y_cam = (pt_y - py) * depth_val / fy
    z_cam = depth_val
    point_3d_cam = np.array([x_cam, y_cam, z_cam])
    
    tf_msg = ROS_NODE.tf_buffer.lookup_transform('base_link', 'zed_left_camera_frame_optical', rclpy.time.Time())
    tf_rot = R.from_quat([tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w])
    tf_trans = np.array([tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z])
    
    return tf_rot.apply(point_3d_cam) + tf_trans


def compute_ffs_depth_map(l_rgb, r_rgb, K_s, bl):
    depth_mm, _ = compute_ffs_depth_and_disp(l_rgb, r_rgb, K_s, bl)
    return depth_mm


def compute_ffs_depth_and_disp(l_rgb, r_rgb, K_s, bl):
    with torch.autograd.set_grad_enabled(False):
        ffs_model = torch.load(MODEL_DIR, map_location='cpu', weights_only=False)
        ffs_model.args.valid_iters = VALID_ITERS
        ffs_model.args.max_disp = MAX_DISP
        ffs_model.cuda().eval()

        img0 = torch.as_tensor(l_rgb).cuda().float()[None].permute(0, 3, 1, 2)
        img1 = torch.as_tensor(r_rgb).cuda().float()[None].permute(0, 3, 1, 2)

        padder = InputPadder(img0.shape, divis_by=32, force_square=False)
        img0_p, img1_p = padder.pad(img0, img1)

        with torch.amp.autocast('cuda', enabled=True, dtype=AMP_DTYPE):
            disp = ffs_model.forward(img0_p, img1_p, iters=VALID_ITERS, test_mode=True, optimize_build_volume='pytorch1')

        disp = padder.unpad(disp.float())
        disp = disp.data.cpu().numpy().reshape(IMG_HEIGHT, IMG_WIDTH).clip(0, None)

    del ffs_model
    del img0, img1, img0_p, img1_p
    torch.cuda.empty_cache()

    xx = np.arange(IMG_WIDTH)[None, :].repeat(IMG_HEIGHT, axis=0)
    invalid = (xx - disp) < 0
    disp[invalid] = np.inf

    with np.errstate(divide='ignore', invalid='ignore'):
        depth_m = K_s[0, 0] * bl / disp
    depth_m[~np.isfinite(depth_m)] = 0.0
    depth_m[(depth_m < 0.1) | (depth_m > 2.0)] = 0.0
    return (depth_m * 1000.0).astype(np.uint16), disp


def add_colorbar_and_text(vis_bgr, vmin, vmax, title, invalid_mask=None, invert_labels=False):
    h, w = vis_bgr.shape[:2]
    bar_w = 30
    gap = 10
    top_pad = 32
    bottom_pad = 26
    canvas_h = h + top_pad + bottom_pad
    canvas_w = w + gap + bar_w + 90
    canvas = np.full((canvas_h, canvas_w, 3), 255, dtype=np.uint8)
    canvas[top_pad: top_pad + h, :w] = vis_bgr

    bar_x0 = w + gap
    bar_x1 = bar_x0 + bar_w
    grad = np.linspace(255, 0, h, dtype=np.uint8).reshape(h, 1)
    grad = np.repeat(grad, bar_w, axis=1)
    bar = cv2.applyColorMap(grad, cv2.COLORMAP_TURBO)
    canvas[top_pad: top_pad + h, bar_x0:bar_x1] = bar

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, title, (8, 24), font, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
    top_label = f"min: {vmin:.3f}" if invert_labels else f"max: {vmax:.3f}"
    bottom_label = f"max: {vmax:.3f}" if invert_labels else f"min: {vmin:.3f}"
    cv2.putText(canvas, top_label, (bar_x1 + 8, top_pad + 14), font, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(canvas, bottom_label, (bar_x1 + 8, top_pad + h - 6), font, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    if invalid_mask is not None:
        invalid_ratio = float(invalid_mask.mean()) * 100.0
        cv2.putText(canvas, f"invalid: {invalid_ratio:.1f}%", (8, canvas_h - 8), font, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def save_depth_vis(depth_m, out_path):
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not np.any(valid):
        vis = np.full((depth_m.shape[0], depth_m.shape[1], 3), 255, dtype=np.uint8)
        dmin, dmax = ZNEAR, ZFAR
    else:
        d = depth_m.copy()
        d[~valid] = 0
        dmin = max(ZNEAR, float(d[valid].min()))
        dmax = min(ZFAR, float(d[valid].max()))
        denom = max(dmax - dmin, 1e-6)
        inv = (1.0 - (d - dmin) / denom).clip(0, 1)
        vis = cv2.applyColorMap((inv * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        vis[~valid] = 255

    vis = add_colorbar_and_text(vis, dmin, dmax, "Depth (m)", invalid_mask=~valid, invert_labels=True)
    cv2.imwrite(out_path, vis)


def save_disparity_vis(disp, out_path):
    disp_stats = {}
    disp_vis_rgb = vis_disparity(disp, invalid_thres=np.inf, other_output=disp_stats)
    disp_valid = np.isfinite(disp) & (disp > 0)
    disp_vis_bgr = cv2.cvtColor(disp_vis_rgb, cv2.COLOR_RGB2BGR)
    disp_vis_bgr[~disp_valid] = 255

    if disp_stats.get("min_val") is None or disp_stats.get("max_val") is None:
        disp_min, disp_max = 0.0, float(MAX_DISP)
    else:
        disp_min = float(disp_stats["min_val"])
        disp_max = float(disp_stats["max_val"])

    disp_vis_bgr = add_colorbar_and_text(disp_vis_bgr, disp_min, disp_max, "Disparity (px)", invalid_mask=~disp_valid)
    cv2.imwrite(out_path, disp_vis_bgr)


def save_pointcloud_xyzrgb(points, colors_rgb, out_path):
    if len(points) == 0:
        return
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb.astype(np.float64))
    o3d.io.write_point_cloud(out_path, pcd)


def save_zed_grasp_capture_bundle(left_rgb, tracking_bgr, depth_mm, disp, camera, workspace_mask, gg_before, gg_after, note="human2robot", diagnostics=None, force_flips=None):
    if not DEBUG_MODE:
        return None
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = Path.cwd() / "captured_data" / ts
    save_dir.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(save_dir / "sam2_input.png"), tracking_bgr)
    cv2.imwrite(str(save_dir / "left_rgb.png"), cv2.cvtColor(left_rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(str(save_dir / "depth_raw_mm.png"), depth_mm)
    save_disparity_vis(disp, str(save_dir / "disparity.png"))
    save_depth_vis(depth_mm.astype(np.float32) / 1000.0, str(save_dir / "depth.png"))
    cv2.imwrite(str(save_dir / "workspace_mask_used.png"), workspace_mask.astype(np.uint8) * 255)

    cloud = create_point_cloud_from_depth_image(depth_mm, camera, organized=True)
    color = left_rgb.astype(np.float32) / 255.0
    valid_depth_mask = depth_mm > 0
    full_points = cloud[valid_depth_mask]
    full_colors = color[valid_depth_mask]
    save_pointcloud_xyzrgb(full_points, full_colors, str(save_dir / "pointcloud_full.ply"))

    used_valid = workspace_mask & (depth_mm > 0)
    used_points = cloud[used_valid]
    used_colors = color[used_valid]
    save_pointcloud_xyzrgb(used_points, used_colors, str(save_dir / "pointcloud_workspace_used.ply"))

    gg_before_array = gg_before.grasp_group_array if len(gg_before) > 0 else np.empty((0, 17), dtype=np.float32)
    gg_after_array = gg_after.grasp_group_array if len(gg_after) > 0 else np.empty((0, 17), dtype=np.float32)
    np.save(str(save_dir / "grasp_before_filter.npy"), gg_before_array)
    np.save(str(save_dir / "grasp_after_filter.npy"), gg_after_array)
    save_open3d_compare_images(full_points, full_colors, gg_before_array, gg_after_array, str(save_dir), force_flips=force_flips)

    filter_info = {
        "timestamp": ts,
        "note": note,
        "raw_count": int(len(gg_before)),
        "selected_count": int(len(gg_after)),
        "image_size": {"width": int(left_rgb.shape[1]), "height": int(left_rgb.shape[0])},
    }
    filter_info.update(diagnostics or {})
    with open(save_dir / "grasp_filter_info.json", "w", encoding="utf-8") as f:
        json.dump(filter_info, f, ensure_ascii=False, indent=2)

    meta = {
        'intrinsic_matrix': np.array([
            [camera.fx, 0, camera.cx],
            [0, camera.fy, camera.cy],
            [0, 0, 1]
        ]),
        'factor_depth': np.array([[camera.scale]])
    }
    scio.savemat(str(save_dir / "meta.mat"), meta)
    logging.info(f"Saved ZED grasp capture to {save_dir}")
    return save_dir


def calc_point_base_from_rs_depth(depth_img, camera, pt_x, pt_y, max_depth=1.5, log_prefix="hand"):
    h, w = depth_img.shape[:2]
    pt_x = int(np.clip(pt_x, 0, w - 1))
    pt_y = int(np.clip(pt_y, 0, h - 1))

    x1, x2 = max(0, pt_x - 2), min(w, pt_x + 3)
    y1, y2 = max(0, pt_y - 2), min(h, pt_y + 3)
    patch = depth_img[y1:y2, x1:x2].astype(np.float32)
    valid = patch[patch > 0]
    if valid.size == 0:
        logging.warning(f"Depth around {log_prefix} is invalid.")
        return None

    depth_raw = float(np.median(valid))
    depth_scale = getattr(camera, "factor_depth", getattr(camera, "scale", None))
    if depth_scale is None or depth_scale <= 0:
        logging.error(f"Camera depth scale is invalid for {log_prefix}.")
        return None
    depth_m = depth_raw / depth_scale
    if not (0.1 < depth_m < max_depth):
        logging.warning(f"Depth for {log_prefix} is out-of-bounds: {depth_m:.3f} m")
        return None

    x_cam = (pt_x - camera.cx) * depth_m / camera.fx
    y_cam = (pt_y - camera.cy) * depth_m / camera.fy
    point_3d_cam = np.array([x_cam, y_cam, depth_m], dtype=np.float64)

    tf_msg = ROS_NODE.tf_buffer.lookup_transform('base_link', 'rs_tohand_color_optical_frame', rclpy.time.Time())
    tf_rot = R.from_quat([tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w])
    tf_trans = np.array([tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z], dtype=np.float64)
    return tf_rot.apply(point_3d_cam) + tf_trans


def calc_hand_base_pos_from_rs_depth(depth_img, camera, pt_x, pt_y, max_depth=1.5, log_prefix="hand"):
    return calc_point_base_from_rs_depth(depth_img, camera, pt_x, pt_y, max_depth=max_depth, log_prefix=log_prefix)




def encode_image_to_base64(image_bgr):
    _, buffer = cv2.imencode(".jpg", image_bgr)
    return base64.b64encode(buffer).decode("utf-8")

def build_hand_object_prompt():
    return (
        "Instructions: \n"
        "1. Detect the human hand in the image. If no hand is present, reply exactly: 'No hand detected'.\n"
        "2. If a hand is present, detect both the 'hand' and the 'object' being held or interactived with by that hand.\n"
        "3. Provide their locations in JSON format as follows: \n"
        "   {\"hand\": [ymin, xmin, ymax, xmax], \"object\": [ymin, xmin, ymax, xmax], \"object_name\": \"name\"}\n"
        "4. object_name should be a short noun phrase for the detected object.\n"
        "   Coordinates should be normalized 0-1000. \n"
        "Return ONLY the JSON or 'No hand detected'."
    )


def save_hand_object_bbox_artifacts(image_bgr, targets, prefix="handover_vlm"):
    """Save parsed hand/object bbox JSON and visualization image under ./llm_output/<timestamp>/."""
    if not DEBUG_MODE:
        return None, None
    if image_bgr is None or not targets:
        return None, None

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    cwd = os.getcwd()
    output_dir = os.path.join(cwd, "llm_output", ts)
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, f"{prefix}.json")
    img_path = os.path.join(output_dir, f"{prefix}.jpg")

    payload = {
        "timestamp": ts,
        "image_size": {"width": int(image_bgr.shape[1]), "height": int(image_bgr.shape[0])},
        "targets": [],
    }
    vis = image_bgr.copy()

    for target in targets:
        tid = int(target.get("id", -1))
        label = str(target.get("label", f"id_{tid}"))
        entry = {"id": tid, "label": label}
        color = (0, 255, 0) if tid == 2 else (255, 0, 0)
        if "bbox" in target:
            x1, y1, x2, y2 = map(int, target["bbox"])
            entry["bbox_xyxy"] = [x1, y1, x2, y2]
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        else:
            x1, y1 = map(int, target["point"])
            entry["point_xy"] = [x1, y1]
            cv2.circle(vis, (x1, y1), 6, color, 2)
        payload["targets"].append(entry)
        cv2.putText(vis, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    cv2.imwrite(img_path, vis)
    logging.info(f"Saved MLLM bbox JSON: {json_path}")
    logging.info(f"Saved MLLM bbox image: {img_path}")
    return json_path, img_path

def save_mediapipe_intent_frame(image_bgr, prefix="mediapipe_intent", detected_text="Detected intent"):
    """Save the current MediaPipe-annotated frame under ./mediapipe_output/<timestamp>/."""
    if not DEBUG_MODE:
        return None
    if image_bgr is None:
        return None

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out_dir = Path.cwd() / "mediapipe_output" / ts
    out_dir.mkdir(parents=True, exist_ok=True)
    img_path = out_dir / f"{prefix}.jpg"
    vis = image_bgr.copy()
    cv2.putText(vis, detected_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
    if cv2.imwrite(str(img_path), vis):
        logging.info(f"Saved MediaPipe intent frame: {img_path}")
        return str(img_path)

    logging.warning("Failed to save MediaPipe intent frame.")
    return None

def extract_doubao_text(response):
    if hasattr(response, "choices") and len(response.choices) > 0:
        return response.choices[0].message.content
    if hasattr(response, "output") and len(response.output) > 1:
        return response.output[1].content[0].text
    return str(response)

def parse_hand_object_response(content, image_bgr):
    if "No hand detected" in content:
        logging.warning("MLLM: Hand not found in frame.")
        return []

    try:
        json_str = re.search(r"\{.*\}", content, re.DOTALL)
        if not json_str:
            logging.warning("MLLM response does not contain JSON.")
            return []

        data = json.loads(json_str.group())
        results = []
        h, w = image_bgr.shape[:2]

        if "hand" in data:
            vals = [float(v) for v in data["hand"]]
            ymin, xmin, ymax, xmax = vals
            bbox = (int(xmin * w / 1000), int(ymin * h / 1000), int(xmax * w / 1000), int(ymax * h / 1000))
            results.append({"id": 1, "label": "hand", "bbox": bbox})

        if "object" in data:
            vals = [float(v) for v in data["object"]]
            ymin, xmin, ymax, xmax = vals
            bbox = (int(xmin * w / 1000), int(ymin * h / 1000), int(xmax * w / 1000), int(ymax * h / 1000))
            object_name = str(data.get("object_name") or "object").strip() or "object"
            results.append({"id": 2, "label": object_name, "bbox": bbox})

        return results
    except Exception as je:
        logging.error(f"JSON Parse failed: {je}")
        return []

def call_doubao_hand_object(image_bgr):
    if client is None:
        logging.error("MLLM unavailable: configure ARK_API_KEY and the Ark SDK, or use manual selection.")
        return []

    img_base64 = encode_image_to_base64(image_bgr)
    data_url = f"data:image/jpeg;base64,{img_base64}"
    grounding_text = build_hand_object_prompt()

    try:
        kwargs = {
            "model": DOUBAO_MODEL_NAME,
            "input": [
                {
                    "role": "system",
                    "content": [{"type": "input_text", "text": "You are a professional vision assistant for robot hand-object interaction."}]
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "input_image", "image_url": data_url},
                        {"type": "input_text", "text": grounding_text},
                    ],
                }
            ]
        }
        if REASONING_EFFORT != "minimal":
            kwargs["reasoning_effort"] = REASONING_EFFORT

        response = client.responses.create(**kwargs)
        content = extract_doubao_text(response)
        logging.info(f"Doubao Response: {content}")
        return parse_hand_object_response(content, image_bgr)
    except Exception as e:
        logging.error(f"API call failed: {e}")
        return []

def call_vlm_hand_object(image_bgr):
    return call_doubao_hand_object(image_bgr)

def vlm_thread_worker_hand(image_bgr, result_callback=None, done_callback=None):
    try:
        found_targets = call_vlm_hand_object(image_bgr)
        if found_targets and result_callback is not None:
            result_callback(found_targets)
            logging.info(f"MLLM Worker: Found {len(found_targets)} targets.")
    finally:
        if done_callback is not None:
            done_callback()


def prompt_zed_selection_mode(default_mode="mllm"):
    if client is None:
        logging.info("MLLM unavailable; using manual hand/object selection.")
        return "manual"
    mode = (default_mode or "mllm").strip().lower()
    while True:
        prompt = "Select ZED hand/object selection mode before workspace selection [1=mllm, 2=manual]"
        if mode in ("mllm", "manual"):
            prompt += f" (default: {mode})"
        prompt += ": "
        ans = input(prompt).strip().lower()
        if not ans:
            return mode if mode in ("mllm", "manual") else "mllm"
        if ans in ("1", "mllm"):
            return "mllm"
        if ans in ("2", "manual"):
            return "manual"
        print("Please enter 1/mllm or 2/manual.")

def get_net(checkpoint_path, num_view=300):
    net = GraspNet(input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    checkpoint = torch.load(checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    net.eval()
    return net, device

def resolve_checkpoint_path():
    ckpt_rs = os.path.join(BASELINE_DIR, 'logs', 'log_rs', 'checkpoint-rs.tar')
    ckpt_fallback = os.path.join(BASELINE_DIR, 'logs', 'log_rs', 'checkpoint.tar')
    if os.path.exists(ckpt_rs):
        return ckpt_rs
    if os.path.exists(ckpt_fallback):
        return ckpt_fallback
    raise FileNotFoundError(
        f"GraspNet checkpoint not found. Tried:\n  - {ckpt_rs}\n  - {ckpt_fallback}"
    )


def denoise_points(points, colors):
    if len(points) == 0:
        return points, colors

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

    pcd = pcd.voxel_down_sample(voxel_size=DENOISE_VOXEL_SIZE)
    if len(pcd.points) == 0:
        return points, colors

    _, ind = pcd.remove_radius_outlier(
        nb_points=DENOISE_NB_POINTS,
        radius=DENOISE_RADIUS
    )
    if len(ind) == 0:
        return points, colors

    pcd = pcd.select_by_index(ind)
    return np.asarray(pcd.points, dtype=np.float32), np.asarray(pcd.colors, dtype=np.float32)


def filter_collision_grasps(gg, scene_points):
    return filter_grasps(gg, scene_points, GRASP_BACKOFF_DIST)


def legacy_realsense_main():
    global vlm_running, pending_targets, doubao_trigger, ROS_NODE
    
    # Init ROS2
    rclpy.init()
    ROS_NODE = RobotControllerNode()
    ROS_NODE.ensure_motion_ready()
    executor = MultiThreadedExecutor()
    executor.add_node(ROS_NODE)
    t_ros = threading.Thread(target=executor.spin, daemon=True)
    t_ros.start()

    if ROS_NODE.rtde_c:
        ROS_NODE.get_logger().info("【初始化】移动机械臂到交接观测位置...")
        start_joint_q = [math.radians(100), math.radians(60), math.radians(-150), math.radians(-90), math.radians(-90), math.radians(0)]
        ROS_NODE.move_joint(start_joint_q, 0.5, 0.5)
        
        time.sleep(1.0)
        ROS_NODE.get_logger().info("【初始化】释放夹爪...")
        success = ROS_NODE.send_gripper_command(0.085)
        if not success:
            raise RuntimeError("H2R startup aborted: gripper open failed or timed out")
            
        ROS_NODE.get_logger().info("【初始化完成】机械臂已就位，启动相机流...")
    
    checkpoint_path = resolve_checkpoint_path()
    net, device = get_net(checkpoint_path)
    
    SAM2_CHECKPOINT = os.path.join(SAM2_DIR, "checkpoints/sam2.1/sam2.1_hiera_small.pt")
    SAM2_CFG = "sam2.1/sam2.1_hiera_s.yaml"
    logging.info("Loading SAM2 model...")
    sam2_predictor = build_sam2_camera_predictor(SAM2_CFG, SAM2_CHECKPOINT)
    sam2_predictor.fill_hole_area = 0
    sam2_predictor_rs = build_sam2_camera_predictor(SAM2_CFG, SAM2_CHECKPOINT)
    sam2_predictor_rs.fill_hole_area = 0
    logging.info("SAM2 model loaded")
    
    pipeline, align, camera_info = start_realsense()
    cv2.namedWindow("Realsense Viewer", cv2.WINDOW_AUTOSIZE)

    logging.info("Camera Started. UI Controls (focus on OpenCV window):")
    logging.info("  - v: Trigger Doubao prompt for hand and object selection")
    logging.info("  - Space: Pass Object Mask (ID 2) to GraspNet")
    logging.info("  - r: Reset SAM2 selection")
    logging.info("  - q: Quit")
    
    num_targets = 0 
    rs_selection_locked = False
    need_reset = False
    current_masks = {} 
    all_prompts = []
    
    try:
        while True:
            t0 = time.time()
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame: continue
            
            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())
            
            color_img_bgr = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
            tracking_img = color_img_bgr.copy()

            # --- SAM2: Reset ---
            if need_reset:
                if hasattr(sam2_predictor, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor.condition_state:
                    sam2_predictor.reset_state()
                num_targets = 0
                need_reset = False
                current_masks = {}
                all_prompts = []
                pending_targets = []
                logging.info("Reset tracking targets.")

            # --- Doubao Grounding ---
            if doubao_trigger and not vlm_running:
                vlm_snapshot = color_img_bgr.copy() # Snapshot taken at the moment of trigger
                vlm_running = True
                threading.Thread(target=vlm_thread_worker, args=(vlm_snapshot,), daemon=True).start()
                doubao_trigger = False

            # --- SAM2: Initialize targets ---
            if pending_targets:
                if hasattr(sam2_predictor, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor.condition_state:
                    sam2_predictor.reset_state()
                
                sam2_predictor.load_first_frame(vlm_snapshot)
                all_prompts = []
                
                for tid, bbox in pending_targets:
                    x1, y1, x2, y2 = bbox
                    bbox_arr = np.array([[x1, y1], [x2, y2]], dtype=np.float32)
                    sam2_predictor.add_new_prompt(frame_idx=0, obj_id=tid, bbox=bbox_arr)
                    all_prompts.append({'id': tid, 'bbox': bbox})
                
                num_targets = len(all_prompts)
                pending_targets = []

            # --- SAM2: Track ---
            if num_targets > 0:
                out_obj_ids, out_mask_logits = sam2_predictor.track(tracking_img)
                current_masks = {}
                for i in range(len(out_obj_ids)):
                    obj_id = out_obj_ids[i]
                    current_masks[obj_id] = (out_mask_logits[i] > 0.0).cpu().numpy().squeeze()

            # --- Visualization ---
            display = tracking_img.copy()
            for obj_id, mask in current_masks.items():
                if mask is not None:
                    color = MASK_COLORS_BGR.get(obj_id, [0, 255, 0])
                    overlay = display.copy()
                    overlay[mask > 0] = color
                    display = cv2.addWeighted(display, 1 - MASK_ALPHA, overlay, MASK_ALPHA, 0)
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(display, contours, -1, (0, 255, 0), 2)
                    
                    label = "Hand" if obj_id == 1 else "Object"
                    pos = np.argwhere(mask)
                    if len(pos) > 0:
                        y, x = pos[len(pos)//2]
                        cv2.putText(display, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            if vlm_running:
                cv2.putText(display, "Processing", (IMG_WIDTH // 2 - 80, IMG_HEIGHT // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            t1 = time.time()
            fps = 1.0 / (t1 - t0)
            cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            status = f"Targets: {num_targets} | v=Seg | SPACE=GraspNet | r=Reset | ESC=Exit"
            cv2.putText(display, status, (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Realsense Viewer", display)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 32: 
                print("Processing Frame for GraspNet...")
                # 获取物体的掩码 (ID 为 2)
                object_mask = None
                if 2 in current_masks and current_masks[2] is not None:
                    object_mask = current_masks[2] > 0
                
                if object_mask is None or not np.any(object_mask):
                    print("No Object mask found (ID 2), using default workspace heuristic!")
                else:
                    print("Object mask found. Proceeding with grasped object mask.")
                
                def process_grasp(c_img, d_img, cam_info, dev, c_mask, t_img):
                    end_points, cloud_o3d, workspace_mask = process_frame(c_img, d_img, cam_info, dev, sam_mask=c_mask)
                    if end_points is None:
                        print("No points found in workspace mask.")
                        return
                    
                    if DEBUG_MODE:
                        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        save_dir = os.path.join("captured_data", timestamp)
                        os.makedirs(save_dir, exist_ok=True)
                        
                        cv2.imwrite(os.path.join(save_dir, 'color.png'), t_img)
                        cv2.imwrite(os.path.join(save_dir, 'depth.png'), d_img)
                        
                        mask_img = (workspace_mask.astype(np.uint8) * 255)
                        cv2.imwrite(os.path.join(save_dir, 'workspace_mask.png'), mask_img)
                        
                        try:
                            meta = {
                                'intrinsic_matrix': np.array([
                                    [cam_info.fx, 0, cam_info.cx],
                                    [0, cam_info.fy, cam_info.cy],
                                    [0, 0, 1]
                                ]),
                                'factor_depth': np.array([[cam_info.scale]])
                            }
                            scio.savemat(os.path.join(save_dir, 'meta.mat'), meta)
                            print(f"Data successfully saved to {save_dir}")
                        except Exception as e:
                            print("Failed to save meta.mat:", e)
                    
                    with torch.no_grad():
                        with torch.autocast(device_type="cuda", dtype=torch.float32):
                            end_points = net(end_points)
                            grasp_preds = pred_decode(end_points)
                    
                    gg_array = grasp_preds[0].detach().cpu().numpy()
                    
                    show_open3d_process(
                        np.asarray(cloud_o3d.points),
                        np.asarray(cloud_o3d.colors),
                        gg_array
                    )

                    gg_filtered = GraspGroup(gg_array).nms().sort_by_score()
                    if len(gg_filtered) > 0 and ROS_NODE:
                        ROS_NODE.execute_grasp(gg_filtered)

                t = threading.Thread(target=process_grasp, args=(
                    color_img.copy(), depth_img.copy(), camera_info, device, 
                    object_mask.copy() if object_mask is not None else None, 
                    tracking_img.copy()))
                t.daemon = True
                t.start()
                
            elif key == ord('v'):
                doubao_trigger = client is not None
            elif key == ord('r'):
                need_reset = True
            elif key == ord('q'):
                break
                
    finally:
        pipeline.stop()
        if ROS_NODE and ROS_NODE.rtde_c: ROS_NODE.rtde_c.stopScript()
        rclpy.shutdown()
        close_open3d_viewer()
        cv2.destroyAllWindows()

def start_realsense():
    pipeline = rs.pipeline()
    config = rs.config()
    rs_width = 640
    rs_height = 480
    config.enable_stream(rs.stream.depth, rs_width, rs_height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, rs_width, rs_height, rs.format.rgb8, 30)
    
    profile = pipeline.start(config)
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale() 
    factor_depth = 1.0 / depth_scale 
    
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    intrinsics = color_profile.get_intrinsics()
    
    camera = CameraInfo(float(rs_width), float(rs_height), intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, factor_depth)
    return pipeline, align, camera

def process_frame(color_img, depth_img, camera, device, sam_mask=None, num_point=20000):
    color = color_img.astype(np.float32) / 255.0
    
    if sam_mask is not None and np.any(sam_mask):
        workspace_mask = (sam_mask > 0) & (depth_img > 0) & (depth_img < 2000)
    else:
        z_min_mm, z_max_mm = 200, 1000 
        workspace_mask = (depth_img > z_min_mm) & (depth_img < z_max_mm)
    
    cloud = create_point_cloud_from_depth_image(depth_img, camera, organized=True)
    mask = (workspace_mask & (depth_img > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]
    
    if len(cloud_masked) == 0:
        return None, None, None

    cloud_masked_denoised, color_masked_denoised = denoise_points(cloud_masked, color_masked)
    if len(cloud_masked_denoised) > 0:
        cloud_masked = cloud_masked_denoised
        color_masked = color_masked_denoised
        
    if len(cloud_masked) >= num_point:
        idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
        
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # 保存用于可视化的完整点云
    valid_depth_mask = depth_img > 0
    cloud_full = cloud[valid_depth_mask]
    color_full = color[valid_depth_mask]
    cloud_full_denoised, color_full_denoised = denoise_points(cloud_full, color_full)
    if len(cloud_full_denoised) > 0:
        cloud_full = cloud_full_denoised
        color_full = color_full_denoised
    
    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_full.astype(np.float32))
    cloud_o3d.colors = o3d.utility.Vector3dVector(color_full.astype(np.float32))
    
    end_points = dict()
    cloud_sampled_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device, dtype=torch.float32)
    with torch.autocast(device_type="cuda", dtype=torch.float32):
        pass 
    end_points['point_clouds'] = cloud_sampled_tensor
    end_points['cloud_colors'] = color_sampled
    
    return end_points, cloud_o3d, workspace_mask

def show_open3d_process(points, colors, gg_array, force_flips=None):
    get_open3d_viewer().update(points, colors, gg_array, top_k=1, force_flips=force_flips)


def _render_open3d_snapshot(points, colors, gg_array, out_path, window_name, top_k=1, force_flips=None):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)

    gg = (GraspGroup(gg_array) if force_flips is not None
          else GraspGroup(gg_array).nms().sort_by_score())
    geoms = [cloud]
    if len(gg) > 0:
        show_n = min(len(gg), max(int(top_k), 1))
        shown_flips = None if force_flips is None else force_flips[:show_n]
        geoms.extend(grasp_meshes(gg[:show_n], force_flips=shown_flips))

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=1280, height=720, visible=False)
    for g in geoms:
        vis.add_geometry(g)

    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])
    ctr.set_lookat([0, 0, 0.5])
    ctr.set_up([0, -1, 0])
    ctr.set_zoom(0.8)

    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(out_path, do_render=True)
    vis.destroy_window()


def save_open3d_compare_images(points, colors, gg_before_array, gg_after_array, save_dir, force_flips=None):
    before_path = os.path.join(save_dir, "open3d_before_filter.png")
    after_path = os.path.join(save_dir, "open3d_after_filter.png")

    try:
        _render_open3d_snapshot(points, colors, gg_before_array, before_path, "Open3D Before Filter", top_k=20)
        _render_open3d_snapshot(points, colors, gg_after_array, after_path, "Open3D After Filter", top_k=1, force_flips=force_flips)

        img_before = cv2.imread(before_path)
        img_after = cv2.imread(after_path)
        if img_before is not None and img_after is not None and img_before.shape == img_after.shape:
            cv2.imwrite(os.path.join(save_dir, "open3d_filter_compare.png"), cv2.hconcat([img_before, img_after]))
    except Exception as e:
        logging.warning(f"Open3D screenshot save failed: {e}")

def main(debug=False, manual=False, all_directions=False):
    global DEBUG_MODE
    DEBUG_MODE = debug
    global need_reset, num_targets, pending_bbox, pending_point, current_masks, all_prompts, drawing, ix, iy, fx_mouse, fy_mouse
    global vlm_running, doubao_trigger, workspace_bbox, pending_targets, ROS_NODE, STATE, rs_selection_locked
    global zed_drawing, zed_ix, zed_iy, zed_fx_mouse, zed_fy_mouse, zed_pending_bbox, zed_pending_point

    rclpy.init()
    ROS_NODE = RobotControllerNode()
    ROS_NODE.ensure_motion_ready()
    executor = MultiThreadedExecutor()
    executor.add_node(ROS_NODE)
    t_ros = threading.Thread(target=executor.spin, daemon=True)
    t_ros.start()

    ROS_NODE.get_logger().info("Startup: returning to shared Home, then opening the gripper...")
    move_home(ROS_NODE.rtde_c, ROS_NODE.rtde_r, load_home_joints())
    if not ROS_NODE.send_gripper_command(.085):
        raise RuntimeError("Startup Home reached but gripper opening was not confirmed")
    ROS_NODE.get_logger().info("Home reached, gripper open; starting camera streams.")

    SAM2_CHECKPOINT = os.path.join(SAM2_DIR, "checkpoints/sam2.1/sam2.1_hiera_small.pt")
    SAM2_CFG = "sam2.1/sam2.1_hiera_s.yaml"
    logging.info("Loading SAM2 model...")
    sam2_predictor = build_sam2_camera_predictor(SAM2_CFG, SAM2_CHECKPOINT)
    sam2_predictor.fill_hole_area = 0
    sam2_predictor_rs = build_sam2_camera_predictor(SAM2_CFG, SAM2_CHECKPOINT)
    sam2_predictor_rs.fill_hole_area = 0
    logging.info("SAM2 model loaded")

    zed_selection_mode = "mllm"
    
    zed, K_scaled, baseline, camera_info = start_zed()
    
    # GraspNet Init
    checkpoint_path = resolve_checkpoint_path()
    net, device = get_net(checkpoint_path)
    pipeline, align, camera_info_rs = start_realsense()
    image_left = sl.Mat()
    image_right = sl.Mat()

    show_realsense = create_camera_view("Realsense Viewer", 960, 720, mouse_callback)
    show_zed = create_camera_view("ZED Inhand Viewer", 960, 540, zed_mouse_callback)


    logging.info("Camera Started. UI Controls (focus on OpenCV window):")
    logging.info("  - w: Trigger Doubao to detect Workpace")
    logging.info("  - Left-click drag: Draw object bounding box -> track")
    logging.info("  - Left-click: Select foreground object point -> track")
    logging.info("  - v: Reselect ZED hand/object only within segmentation stage")
    logging.info("  - RIGHT open palm then fist 2s: start tracking / stop tracking / confirm READY; SPACE disabled")
    logging.info("  - 0: explicitly release and return Home when idle; check terminal on failure")
    logging.info("  - r: Reset current selection")
    logging.info("  - ESC: Stop and exit without releasing or returning Home (q also exits)")
    
    num_targets = 0 
    rs_selection_locked = False
    need_reset = False
    current_masks = {}
    all_prompts = []
    zed_need_reset = False
    zed_vlm_running = False
    zed_pending_targets = []
    zed_doubao_trigger = False
    zed_current_masks = {}
    zed_all_prompts = []
    zed_vlm_snapshot = None
    zed_viewer_requested = False
    zed_grasp_request_pending = False
    zed_segmentation_ready_time = 0.0
    zed_selection_source = None
    zed_sam2_initialized = False
    zed_selection_mode_default = "mllm"
    
    initialized_dynamic_start = False
    handover_start_calc_running = False
    follow_calc_running = False
    handover_grasp_running = False
    workspace_calc_running = False
    prepared_grasp = None
    last_zed_frame_time = 0.0
    last_rs_frame_time = 0.0
    recovery_running = False
    release_on_recovery = False
    preparation_revision = 0
    fist_confirmation = FistConfirmation()
    # Separate detector: do not mix temporal tracking between the two cameras.
    zed_hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2,
                               min_detection_confidence=0.7, min_tracking_confidence=0.5)
    zed_left_available = False
    zed_left_since = None
    zed_left_center = None
    zed_hand_checked_at = 0.0
    last_mask_time = 0.0
    grasp_status = "Select the object after the LEFT hand is identified"
    grasp_snapshot_bgr = None
    observation_preview = None
    observation_preview_running = False
    observation_preview_started = 0.0
    observation_preview_revision = 0
    seen_home_revision = ROS_NODE.home_revision

    def reset_workspace_selection():
        """Clear round UI state on the main thread, as R2H does after Home."""
        global STATE, workspace_bbox, pending_bbox, pending_point, rs_selection_locked
        global drawing, need_reset, num_targets, current_masks, all_prompts, pending_targets
        global zed_pending_bbox, zed_pending_point, zed_drawing
        nonlocal prepared_grasp, preparation_revision, zed_need_reset
        nonlocal zed_pending_targets, zed_grasp_request_pending, zed_doubao_trigger
        nonlocal observation_preview, observation_preview_revision, fist_confirmation
        nonlocal zed_left_available, zed_left_since, zed_left_center, grasp_snapshot_bgr
        preparation_revision += 1
        observation_preview_revision += 1
        prepared_grasp = observation_preview = grasp_snapshot_bgr = None
        fist_confirmation = FistConfirmation()
        workspace_bbox = pending_bbox = pending_point = None
        rs_selection_locked = drawing = False
        num_targets = 0
        current_masks, all_prompts, pending_targets = {}, [], []
        zed_pending_targets = []
        zed_pending_bbox = zed_pending_point = None
        zed_drawing = False
        zed_grasp_request_pending = zed_doubao_trigger = zed_left_available = False
        zed_left_since = zed_left_center = None
        need_reset = zed_need_reset = True
        ROS_NODE.dynamic_start_pose = ROS_NODE.dynamic_workspace_center_pose = None
        ROS_NODE.handover_follow_pose = None
        STATE = "IDLE"
        logging.info("Ready for next round: select a new D435 workspace, then press W.")

    def select_zed_mode_for_round():
        nonlocal zed_selection_mode, zed_selection_mode_default
        zed_selection_mode = "manual" if manual else prompt_zed_selection_mode(default_mode=zed_selection_mode_default)
        zed_selection_mode_default = zed_selection_mode
        if ROS_NODE is not None:
            ROS_NODE.get_logger().info(f"ZED hand/object selection mode for this round: {zed_selection_mode}")
        return zed_selection_mode

    if ROS_NODE is not None:
        ROS_NODE.zed_selection_mode_callback = select_zed_mode_for_round

    def launch_handover_grasp(c_img, c_bgr, l_rgb, r_rgb, masks):
        nonlocal handover_grasp_running, prepared_grasp, grasp_status, grasp_snapshot_bgr
        if ROS_NODE.motion_failed.is_set():
            return
        if handover_grasp_running:
            return
        handover_grasp_running = True
        prepared_grasp = None
        request_revision = preparation_revision
        grasp_status = segmentation_rejection_reason(masks)
        if grasp_status:
            handover_grasp_running = False
            return
        grasp_snapshot_bgr = c_bgr.copy()
        grasp_status = "Computing depth and grasp candidates; holding position"

        object_mask = masks[2] > 0

        logging.info("Processing ZED frame for handover GraspNet...")

        def process_handover_grasp(c_img_i, cam_info_i, dev_i, c_mask_i, t_img_i, left_i, right_i):
            nonlocal handover_grasp_running, zed_need_reset, prepared_grasp, grasp_status
            try:
                from graspnetAPI import GraspGroup

                if request_revision != preparation_revision or ROS_NODE.motion_failed.is_set():
                    return
                d_img_i, zed_disp = compute_ffs_depth_and_disp(left_i, right_i, K_scaled, baseline)
                if request_revision != preparation_revision or ROS_NODE.motion_failed.is_set():
                    return
                end_points, cloud_o3d, workspace_mask = process_frame(c_img_i, d_img_i, cam_info_i, dev_i, sam_mask=c_mask_i)
                if end_points is None:
                    raise RuntimeError("No valid points found for GraspNet")

                if DEBUG_MODE:
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    save_dir = os.path.join("captured_data", timestamp)
                    os.makedirs(save_dir, exist_ok=True)

                    cv2.imwrite(os.path.join(save_dir, 'color.png'), t_img_i)
                    cv2.imwrite(os.path.join(save_dir, 'depth.png'), d_img_i)
                    cv2.imwrite(os.path.join(save_dir, 'workspace_mask.png'), workspace_mask.astype(np.uint8) * 255)

                    try:
                        meta = {
                            'intrinsic_matrix': np.array([
                                [cam_info_i.fx, 0, cam_info_i.cx],
                                [0, cam_info_i.fy, cam_info_i.cy],
                                [0, 0, 1]
                            ]),
                            'factor_depth': np.array([[cam_info_i.scale]])
                        }
                        scio.savemat(os.path.join(save_dir, 'meta.mat'), meta)
                        logging.info(f"Saved ZED inhand capture to {save_dir}")
                    except Exception as e:
                        logging.warning(f"Failed to save meta.mat: {e}")

                with torch.no_grad():
                    with torch.autocast(device_type="cuda", dtype=torch.float32):
                        end_points = net(end_points)
                        grasp_preds = pred_decode(end_points)

                gg_array = grasp_preds[0].detach().cpu().numpy()
                gg_nms = GraspGroup(gg_array).nms().sort_by_score()
                raw_count = len(gg_nms)

                try:
                    tf_msg = ROS_NODE.tf_buffer.lookup_transform('base_link', 'zed_inhand_camera_frame_optical', rclpy.time.Time())
                    tf_rot = R.from_quat([
                        tf_msg.transform.rotation.x,
                        tf_msg.transform.rotation.y,
                        tf_msg.transform.rotation.z,
                        tf_msg.transform.rotation.w
                    ])
                    camera_rot_mat = tf_rot.as_matrix()
                except Exception as e:
                    raise RuntimeError("H2R camera orientation TF unavailable; cannot prepare grasp") from e

                gg_camera = (gg_nms if all_directions
                             else filter_by_camera_direction(gg_nms, camera_rot_mat, CAMERA_GRASP_MAX_TILT_DEG,
                                                             CAMERA_GRASP_MAX_CLOSING_FORWARD_COS).sort_by_score())
                orient_count = len(gg_camera)
                # Same physical symmetry + endpoint IK screening as R2H.
                prepared_candidates = prepare_ready_grasps(
                    gg_camera, np.asarray(cloud_o3d.points), GRASP_BACKOFF_DIST,
                    ROS_NODE.build_grasp_plan, ROS_NODE.validate_grasp_plan,
                    cancelled=lambda: (request_revision != preparation_revision
                                       or ROS_NODE.motion_failed.is_set()),
                    on_rejected=lambda index, flip, error: logging.warning(
                        "H2R candidate %d (force_flip=%s) rejected before READY: %s", index, flip, error),
                )
                if prepared_candidates is None:
                    return
                gg_filtered, selected_flips, ready_plan, physical_count, endpoint_rejections = prepared_candidates
                logging.info(
                    "H2R grasp filtering: nms=%d, direction=%d, physical_orientations=%d, ready=%d",
                    raw_count, orient_count, physical_count, len(gg_filtered))

                show_open3d_process(
                    np.asarray(cloud_o3d.points),
                    np.asarray(cloud_o3d.colors),
                    gg_filtered.grasp_group_array if len(gg_filtered) > 0 else np.empty((0, 17), dtype=np.float32),
                    force_flips=selected_flips,
                )
                p = None

                save_zed_grasp_capture_bundle(
                    left_rgb=c_img_i,
                    tracking_bgr=t_img_i,
                    depth_mm=d_img_i,
                    disp=zed_disp,
                    camera=cam_info_i,
                    workspace_mask=workspace_mask,
                    gg_before=gg_nms,
                    gg_after=gg_filtered,
                    note="human2robot",
                    diagnostics={
                        "nms_count": raw_count,
                        "direction_count": orient_count,
                        "all_directions": all_directions,
                        "physical_orientation_count": physical_count,
                        "selected_symmetry_flips": selected_flips,
                        "endpoint_rejections": endpoint_rejections,
                    },
                    force_flips=selected_flips,
                )

                if request_revision != preparation_revision:
                    logging.info("Discarding grasp prepared before hand identity/selection reset.")
                    return
                if len(gg_filtered) == 0:
                    grasp_status = (f"No grasp: direction={orient_count}, physical={physical_count}, "
                                    "reachable=0; adjust object, then retry or reselect")
                    if endpoint_rejections:
                        grasp_status += "; " + endpoint_rejections[0]["reason"]
                    logging.warning(grasp_status)
                    return
                prepared_grasp = (gg_filtered, np.array(ROS_NODE.rtde_r.getActualQ()),
                                  time.monotonic(), ready_plan)
                grasp_status = "Grasp plan available; review hand/object and gripper preview"
                logging.info("READY: review preview, keep LEFT hand/object still; open RIGHT palm then hold fist 2s.")
            except Exception as error:
                if request_revision == preparation_revision:
                    # Preparation commands no motion: vision/no-candidate failures
                    # stay at the observation pose instead of automatically going Home.
                    grasp_status = f"Preparation failed: {error}; retry or reselect"
                    logging.exception(grasp_status)
            finally:
                handover_grasp_running = False

        t = threading.Thread(
            target=process_handover_grasp,
            args=(
                c_img.copy(),
                camera_info,
                device,
                object_mask.copy() if object_mask is not None else None,
                c_bgr.copy(),
                l_rgb.copy(),
                r_rgb.copy(),
            ),
            daemon=True,
        )
        t.start()

    def trigger_zed_segmentation(reason="manual trigger"):
        nonlocal zed_doubao_trigger, zed_need_reset, zed_grasp_request_pending, follow_calc_running, zed_selection_source, zed_sam2_initialized, prepared_grasp
        global STATE
        nonlocal preparation_revision, grasp_status, zed_left_available, zed_left_since, zed_left_center
        if STATE not in ("HANDOVER_SELECTION_PENDING", "HANDOVER_SEGMENTATION"):
            logging.warning("Segmentation locked: finish positioning with RIGHT fist 2s first.")
            return
        if (ROS_NODE.motion_failed.is_set() or follow_calc_running or handover_grasp_running
                or handover_start_calc_running or ROS_NODE.motion_lock.locked()):
            logging.warning("Cannot segment while motion is running or H2R is stopped.")
            return
        preparation_revision += 1
        prepared_grasp = None
        grasp_status = "Identify LEFT hand, then select object"
        zed_left_available = False
        zed_left_since = zed_left_center = None
        STATE = "HANDOVER_SEGMENTATION"
        zed_need_reset = True
        zed_grasp_request_pending = False
        zed_sam2_initialized = False
        follow_calc_running = False
        if zed_selection_mode == "mllm":
            zed_doubao_trigger = True
            zed_selection_source = "mllm"
            logging.info(f"{reason}: triggering ZED hand/object detection in MLLM mode.")
        else:
            zed_doubao_trigger = False
            zed_selection_source = "manual"
            logging.info(f"{reason}: entering ZED manual hand/object selection mode. LEFT hand is selected automatically; click or box-select only the object.")
    
    def request_observation_preview(depth, camera, pixel):
        nonlocal observation_preview_running, observation_preview_started, observation_preview
        if observation_preview_running or time.monotonic() - observation_preview_started < .5:
            return
        observation_preview_running = True
        observation_preview_started = time.monotonic()
        revision = observation_preview_revision
        request_state = STATE
        source_time = time.monotonic()

        def preview():
            nonlocal observation_preview_running, observation_preview_started, observation_preview
            locked = False
            try:
                hand = calc_hand_base_pos_from_rs_depth(depth, camera, *pixel,
                                                       log_prefix="LEFT observation preview")
                reason = "LEFT hand depth invalid"
                if hand is not None:
                    locked = ROS_NODE.motion_lock.acquire(blocking=False)
                    if not locked:
                        return
                    reason = ROS_NODE.hand_observation_rejection_reason(hand)
                if revision == observation_preview_revision and STATE == request_state:
                    if observation_preview is None or observation_preview["reason"] != reason:
                        goal = (ROS_NODE.build_follow_left_hand_pose(hand, limit_step=False)
                                if hand is not None else None)
                        logging.info("Observation preview: %s; hand_base=%s; tcp_goal=%s; check=%.3fs",
                                     reason or "reachable", hand, goal, time.monotonic() - source_time)
                    observation_preview = {"time": time.monotonic(), "source_time": source_time, "pixel": pixel,
                                           "hand": hand, "reason": reason}
            except Exception as error:
                if revision == observation_preview_revision and STATE == request_state:
                    observation_preview = {"time": time.monotonic(), "source_time": source_time, "pixel": pixel,
                                           "hand": None, "reason": str(error)}
            finally:
                if locked:
                    ROS_NODE.motion_lock.release()
                observation_preview_started = time.monotonic()
                observation_preview_running = False
        threading.Thread(target=preview, daemon=True).start()

    # 刚启动时，自动触发工作台检测
    doubao_trigger = client is not None and not manual
    
    try:
        while True:
            t0 = time.time()
            if ROS_NODE.motion_failed.is_set():
                STATE = "ERROR"
            if (seen_home_revision != ROS_NODE.home_revision
                    and not ROS_NODE.motion_failed.is_set()
                    and not handover_grasp_running and not recovery_running
                    and not workspace_calc_running and not follow_calc_running
                    and not handover_start_calc_running and not ROS_NODE.motion_lock.locked()):
                reset_workspace_selection()
                seen_home_revision = ROS_NODE.home_revision
            if (STATE == "HANDOVER_SELECTION_PENDING" and not follow_calc_running
                    and not handover_start_calc_running and not ROS_NODE.motion_lock.locked()):
                trigger_zed_segmentation(reason="Tracking stopped; gesture/selection accepted")
            if (ROS_NODE.recovery_pending and not recovery_running
                    and not workspace_calc_running and not follow_calc_running
                    and not handover_start_calc_running
                    and not handover_grasp_running):
                prepared_grasp = None
                zed_grasp_request_pending = False
                recovery_running = True
                STATE = "RECOVERING"
                def recover_round():
                    nonlocal recovery_running, release_on_recovery
                    try:
                        ROS_NODE.recover_home(release=release_on_recovery)
                    finally:
                        release_on_recovery = False
                        recovery_running = False
                threading.Thread(target=recover_round, daemon=True).start()
            if zed.grab() != sl.ERROR_CODE.SUCCESS:
                continue
                
            zed.retrieve_image(image_left, sl.VIEW.LEFT)
            zed.retrieve_image(image_right, sl.VIEW.RIGHT)

            raw_left_bgr = image_left.get_data()[:, :, :3]
            raw_right_bgr = image_right.get_data()[:, :, :3]

            left_bgr = cv2.resize(raw_left_bgr, (IMG_WIDTH, IMG_HEIGHT))
            right_bgr = cv2.resize(raw_right_bgr, (IMG_WIDTH, IMG_HEIGHT))

            left_rgb = left_bgr[:, :, ::-1].copy()
            right_rgb = right_bgr[:, :, ::-1].copy()
            last_zed_frame_time = time.monotonic()

            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            rs_color_frame = aligned_frames.get_color_frame()
            rs_depth_frame = aligned_frames.get_depth_frame()
            if not rs_color_frame or not rs_depth_frame:
                continue
            rs_color_image = np.asanyarray(rs_color_frame.get_data())
            rs_depth_image = np.asanyarray(rs_depth_frame.get_data())
            last_rs_frame_time = time.monotonic()
            rs_color_image_bgr = cv2.cvtColor(rs_color_image, cv2.COLOR_RGB2BGR)

            color_img_bgr = rs_color_image_bgr.copy()
            tracking_img = color_img_bgr.copy()
            color_img = rs_color_image.copy()
            display = tracking_img.copy()
            results = hands.process(rs_color_image)
            left_hand_landmarks, left_hand_score = select_left_hand(results)
            right_hand_landmarks, right_hand_score = select_right_hand(results)
            if not hands_separated(left_hand_landmarks, right_hand_landmarks):
                left_hand_landmarks = right_hand_landmarks = None
            if (right_hand_landmarks is not None
                    and STATE in ("WAITING_HANDOVER_START_POSE", "HANDOVER_START_REQUESTED",
                                  "MOVING_TO_HANDOVER", "WAITING_DELIVERY", "FOLLOWING_HAND",
                                  "HANDOVER_SEGMENTATION")):
                draw_physical_hand(display, right_hand_landmarks, right_hand_score,
                                   "Right", mp_drawing, mp_hands.HAND_CONNECTIONS)

            # --- ZED in-hand viewer / segmentation ---
            # Keep ZED stream visible in all states (startup, grasping, post-grasp).
            try:
                zed_color_image = left_rgb.copy()
                zed_color_image_bgr = left_bgr.copy()
                zed_display = (grasp_snapshot_bgr.copy() if handover_grasp_running
                               and grasp_snapshot_bgr is not None else zed_color_image_bgr.copy())

                if zed_need_reset:
                    if hasattr(sam2_predictor_rs, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor_rs.condition_state:
                        sam2_predictor_rs.reset_state()
                    zed_current_masks = {}
                    last_mask_time = 0.0
                    zed_all_prompts = []
                    zed_pending_targets = []
                    zed_pending_bbox = None
                    zed_pending_point = None
                    zed_drawing = False
                    zed_grasp_request_pending = False
                    zed_segmentation_ready_time = 0.0
                    zed_sam2_initialized = False
                    zed_need_reset = False
                    logging.info("Reset ZED hand/object segmentation targets.")

                if STATE == "HANDOVER_SEGMENTATION" and zed_doubao_trigger and not zed_vlm_running:
                    zed_vlm_snapshot = zed_color_image_bgr.copy()
                    zed_vlm_running = True

                    def on_rs_targets(found_targets):
                        nonlocal zed_pending_targets
                        zed_pending_targets = found_targets

                    def on_rs_done():
                        nonlocal zed_vlm_running
                        zed_vlm_running = False

                    threading.Thread(
                        target=vlm_thread_worker_hand,
                        args=(zed_vlm_snapshot, on_rs_targets, on_rs_done),
                        daemon=True,
                    ).start()
                    zed_doubao_trigger = False

                if (STATE == "HANDOVER_SEGMENTATION" and zed_selection_mode == "manual"
                        and not zed_sam2_initialized and not handover_grasp_running
                        and time.monotonic() - zed_hand_checked_at >= .1):
                    zed_hand_checked_at = time.monotonic()
                    zed_left_available = False
                    zed_results = zed_hands.process(left_rgb)
                    zed_left, _ = select_left_hand(zed_results)
                    zed_right, _ = select_right_hand(zed_results)
                    if zed_left is not None:
                        coords = np.array([(lm.x, lm.y) for lm in zed_left.landmark])
                        valid = np.all(np.isfinite(coords)) and np.all((coords >= 0) & (coords <= 1))
                        # Reject overlapping hands: handedness alone is unreliable at crossings.
                        valid = valid and hands_separated(zed_left, zed_right)
                        center = coords[[0, 5, 9, 13, 17]].mean(0)
                        now = time.monotonic()
                        if valid:
                            if zed_left_center is None or np.linalg.norm(center - zed_left_center) > .05:
                                zed_left_since = now
                            zed_left_center = center
                            zed_left_available = zed_left_since is not None and now - zed_left_since >= .5
                            if (zed_left_available and zed_selection_mode == "manual"
                                    and not zed_sam2_initialized and not handover_grasp_running):
                                palm = zed_left.landmark[9]
                                hand_prompt = {"id": 1, "label": "physical_left_hand",
                                               "point": (int(palm.x * IMG_WIDTH), int(palm.y * IMG_HEIGHT)),
                                               "point_candidates": [(int(coords[i, 0] * IMG_WIDTH),
                                                                     int(coords[i, 1] * IMG_HEIGHT))
                                                                    for i in (0, 5, 17, 13, 9, 8, 12, 16, 20)]}
                                zed_all_prompts = [hand_prompt] + [p for p in zed_all_prompts if p["id"] != 1]
                        else:
                            zed_left_since = zed_left_center = None
                    else:
                        zed_left_since = zed_left_center = None
                    if not zed_left_available:
                        prepared_grasp = None
                        if not zed_sam2_initialized:
                            zed_all_prompts = []
                            zed_pending_bbox = zed_pending_point = None
                        cv2.putText(zed_display, "Waiting for unambiguous LEFT hand; keep hands apart",
                                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 165, 255), 2)
                elif STATE != "HANDOVER_SEGMENTATION":
                    zed_left_available = False
                    zed_left_since = zed_left_center = None

                if zed_selection_mode == "manual" and STATE == "HANDOVER_SEGMENTATION":
                    if (zed_left_available and zed_all_prompts
                            and (zed_pending_bbox is not None or zed_pending_point is not None)
                            and len(zed_all_prompts) < 2):
                        next_id = 2
                        next_label = "hand" if next_id == 1 else "object"
                        target = {"id": next_id, "label": next_label}
                        if zed_pending_bbox is not None:
                            target["bbox"] = zed_pending_bbox
                            zed_pending_bbox = None
                        elif zed_pending_point is not None:
                            target["point"] = zed_pending_point
                            zed_pending_point = None
                        zed_all_prompts.append(target)
                        logging.info(f"Added manual ZED target {next_id}: {next_label}")

                    prompt_args = None
                    if len(zed_all_prompts) == 2 and not zed_sam2_initialized:
                        try:
                            prompt_args = manual_segmentation_prompts(*zed_all_prompts)
                        except ValueError as error:
                            grasp_status = str(error)
                            logging.warning(grasp_status)
                            zed_all_prompts = zed_all_prompts[:1]
                    if (prompt_args is not None and not zed_sam2_initialized
                            and not handover_grasp_running):
                        if hasattr(sam2_predictor_rs, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor_rs.condition_state:
                            sam2_predictor_rs.reset_state()
                        zed_vlm_snapshot = zed_color_image_bgr.copy()
                        sam2_predictor_rs.load_first_frame(zed_vlm_snapshot)
                        zed_sam2_initialized = True

                        for prompt, arguments in zip(zed_all_prompts, prompt_args):
                            if "bbox" in arguments:
                                arguments["bbox"] = np.asarray(arguments["bbox"], dtype=np.float32)
                            sam2_predictor_rs.add_new_prompt(frame_idx=0, obj_id=int(prompt["id"]),
                                                            **arguments)

                        save_hand_object_bbox_artifacts(zed_vlm_snapshot, zed_all_prompts, prefix="human2robot_manual_bbox")
                        zed_grasp_request_pending = True
                        zed_segmentation_ready_time = time.time()
                        logging.info("Manual ZED hand/object prompts are ready. Waiting for SAM2 to stabilize before GraspNet.")

                if STATE == "HANDOVER_SEGMENTATION" and zed_pending_targets:
                    if hasattr(sam2_predictor_rs, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor_rs.condition_state:
                        sam2_predictor_rs.reset_state()

                    save_hand_object_bbox_artifacts(zed_vlm_snapshot, zed_pending_targets, prefix="human2robot_bbox")
                    sam2_predictor_rs.load_first_frame(zed_vlm_snapshot)
                    zed_sam2_initialized = True
                    zed_all_prompts = []

                    for target in zed_pending_targets:
                        tid = int(target.get("id", -1))
                        bbox = target.get("bbox")
                        if bbox is None:
                            continue
                        x1, y1, x2, y2 = bbox
                        bbox_arr = np.array([[x1, y1], [x2, y2]], dtype=np.float32)
                        sam2_predictor_rs.add_new_prompt(frame_idx=0, obj_id=tid, bbox=bbox_arr)
                        zed_all_prompts.append({'id': tid, 'bbox': bbox, 'label': target.get("label", f"id_{tid}")})

                    zed_pending_targets = []
                    zed_grasp_request_pending = True
                    zed_segmentation_ready_time = time.time()

                if (STATE == "HANDOVER_SEGMENTATION" and zed_sam2_initialized
                        and zed_all_prompts and not handover_grasp_running):
                    out_obj_ids, out_mask_logits = sam2_predictor_rs.track(zed_color_image_bgr)
                    zed_current_masks = {}
                    for i in range(len(out_obj_ids)):
                        obj_id = out_obj_ids[i]
                        zed_current_masks[obj_id] = (out_mask_logits[i] > 0.0).cpu().numpy().squeeze()
                    last_mask_time = time.monotonic()
                    zed_left_available = segmentation_rejection_reason(zed_current_masks) is None
                    if not zed_left_available:
                        prepared_grasp = None
                        grasp_status = segmentation_rejection_reason(zed_current_masks)

                for obj_id, mask in zed_current_masks.items():
                    if mask is not None and mask.shape == zed_display.shape[:2]:
                        m_color = [0, 255, 0] if obj_id == 2 else [255, 0, 0] # 2 is object, 1 is hand
                        overlay = zed_display.copy()
                        overlay[mask > 0] = m_color
                        zed_display = cv2.addWeighted(zed_display, 0.7, overlay, 0.3, 0)
                        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(zed_display, contours, -1, m_color, 2)
                        label = next((p.get("label") for p in zed_all_prompts if p.get("id") == obj_id), str(obj_id))
                        if contours:
                            x, y, _, _ = cv2.boundingRect(contours[0])
                            cv2.putText(zed_display, str(label), (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, m_color, 2)

                if zed_drawing and zed_ix >= 0:
                    cv2.rectangle(zed_display, (zed_ix, zed_iy), (zed_fx_mouse, zed_fy_mouse), (255, 200, 0), 2)
                for prompt in zed_all_prompts:
                    color = (255, 0, 0) if prompt["id"] == 1 else (0, 255, 0)
                    if "bbox" in prompt:
                        x1, y1, x2, y2 = prompt["bbox"]
                        cv2.rectangle(zed_display, (x1, y1), (x2, y2), color, 2)
                        label_pos = (x1, max(20, y1 - 8))
                    else:
                        px, py = prompt["point"]
                        cv2.circle(zed_display, (px, py), 6, color, 2)
                        label_pos = (px, max(20, py - 8))
                    cv2.putText(zed_display, prompt["label"], label_pos, cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
                
                if zed_vlm_running:
                    cv2.putText(zed_display, "MLLM Detecting Hand/Object...", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                elif zed_selection_mode == "manual" and STATE == "HANDOVER_SEGMENTATION":
                    if len(zed_all_prompts) == 0:
                        zed_manual_hint = "Auto-selecting physical LEFT hand"
                    elif len(zed_all_prompts) == 1:
                        zed_manual_hint = "LEFT selected automatically: select OBJECT only"
                    else:
                        zed_manual_hint = "Manual ZED Seg: hand/object selected"
                    cv2.putText(zed_display, zed_manual_hint, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

                if STATE == "HANDOVER_SEGMENTATION":
                    if (prepared_grasp is not None
                            and time.monotonic() - prepared_grasp[2] > 30.0):
                        prepared_grasp = None
                        grasp_status = "Grasp plan expired: hold RIGHT fist to prepare again"
                        logging.info("H2R READY expired; prepare the grasp again.")
                    if zed_grasp_request_pending:
                        wait_left = max(0.0, SAM2_SETTLE_SECONDS - (time.time() - zed_segmentation_ready_time))
                        rs_hint = f"ZED Segmentation | waiting {wait_left:.1f}s -> GraspNet | r=Reset | ESC=Exit"
                    else:
                        if zed_selection_mode == "manual":
                            rs_hint = "ZED Segmentation(manual) | auto LEFT; click/box OBJECT | RIGHT fist=Confirm | 0=Home | r=Reset | ESC=Exit"
                        else:
                            rs_hint = "ZED Segmentation(MLLM) | RIGHT fist=Confirm | 0=Home | r=Reset | ESC=Exit"
                else:
                    if STATE in ("FOLLOWING_HAND", "WAITING_DELIVERY"):
                        rs_hint = "RIGHT fist 2s: stop tracking | auto LEFT; select OBJECT"
                    elif STATE == "HANDOVER_SELECTION_PENDING":
                        rs_hint = "Stopping tracking... selection queued"
                    else:
                        rs_hint = f"ZED Live | state={STATE} | mode={zed_selection_mode} | ESC=Exit"
                rs_status = rs_hint
                cv2.putText(zed_display, rs_status, (10, zed_display.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                if (STATE == "HANDOVER_SEGMENTATION"
                        and zed_grasp_request_pending and zed_all_prompts and zed_left_available
                        and not handover_grasp_running):
                    if time.time() - zed_segmentation_ready_time >= SAM2_SETTLE_SECONDS:
                        zed_grasp_request_pending = False
                        launch_handover_grasp(
                            zed_color_image.copy(),
                            zed_color_image_bgr.copy(),
                            left_rgb.copy(),
                            right_rgb.copy(),
                            zed_current_masks,
                        )
            except Exception as e:
                logging.error(f"ZED inhand segmentation read error: {e}")

            if STATE in ("WAITING_HANDOVER_START_POSE", "WAITING_DELIVERY"):
                if left_hand_landmarks is None:
                    observation_preview = None
                    observation_preview_revision += 1
                else:
                    palm = left_hand_landmarks.landmark[9]
                    pixel = (int(palm.x * rs_depth_image.shape[1]),
                             int(palm.y * rs_depth_image.shape[0]))
                    request_observation_preview(rs_depth_image.copy(), camera_info_rs, pixel)
            else:
                observation_preview = None
                observation_preview_revision += 1

            # --- MediaPipe Left-Hand Handover ---
            if STATE in ("WAITING_HANDOVER_START_POSE", "HANDOVER_START_REQUESTED", "WAITING_DELIVERY", "FOLLOWING_HAND"):
                if left_hand_landmarks:
                    mp_drawing.draw_landmarks(display, left_hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    
                    h, w, _ = display.shape
                    wrist_lm = left_hand_landmarks.landmark[0]
                    palm_center_lm = left_hand_landmarks.landmark[9]
                    cx, cy = int(palm_center_lm.x * w), int(palm_center_lm.y * h)
                    cv2.circle(display, (cx, cy), 10, (255, 0, 0), cv2.FILLED)
                    cv2.putText(display, f"Physical Left hand {left_hand_score:.2f}", (cx-70, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    
                    def calc_rs_and_update_handover_start(d_img, cam_info, pt_x, pt_y):
                        nonlocal handover_start_calc_running
                        global STATE
                        try:
                            hand_base_pos = calc_hand_base_pos_from_rs_depth(d_img, cam_info, pt_x, pt_y, max_depth=1.5, log_prefix="left hand handover start")
                            if hand_base_pos is not None and STATE == "HANDOVER_START_REQUESTED":
                                ROS_NODE.ensure_motion_ready()
                                ROS_NODE.update_left_hand_base_pos(hand_base_pos)
                                if ROS_NODE.dynamic_start_pose is None:
                                    raise RuntimeError("Workspace observation pose is missing")
                                ROS_NODE.handover_follow_pose = list(ROS_NODE.dynamic_start_pose)
                                ROS_NODE.hand_observation_error = ""
                                STATE = "MOVING_TO_HANDOVER"
                                # Start with the same bounded lift-first step as subsequent frames;
                                # never switch to the former fixed folded joint configuration.
                                ROS_NODE.follow_left_hand(hand_base_pos)
                                STATE = "FOLLOWING_HAND"
                                logging.info("Overhead hand guidance started. Open RIGHT palm, then fist 2s to stop tracking.")
                        except Exception as error:
                            ROS_NODE.hold_motion_error(error)
                        finally:
                            handover_start_calc_running = False

                    def calc_rs_and_follow(d_img, cam_info, pt_x, pt_y):
                        nonlocal follow_calc_running
                        try:
                            hand_base_pos = calc_hand_base_pos_from_rs_depth(d_img, cam_info, pt_x, pt_y, max_depth=1.5, log_prefix="left hand follow")
                            if hand_base_pos is not None and STATE == "FOLLOWING_HAND":
                                ROS_NODE.update_left_hand_base_pos(hand_base_pos)
                                ROS_NODE.follow_left_hand(hand_base_pos)
                        except Exception as error:
                            ROS_NODE.hold_motion_error(error)
                        finally:
                            follow_calc_running = False

                    if STATE == "HANDOVER_START_REQUESTED":
                        cv2.putText(display, "Preparing overhead LEFT-hand observation...", (cx-90, cy-35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        if not handover_start_calc_running:
                            handover_start_calc_running = True
                            t = threading.Thread(target=calc_rs_and_update_handover_start, args=(rs_depth_image.copy(), camera_info_rs, cx, cy), daemon=True)
                            t.start()

                    elif STATE == "FOLLOWING_HAND":
                        if not follow_calc_running:
                            follow_calc_running = True
                            t = threading.Thread(target=calc_rs_and_follow, args=(rs_depth_image.copy(), camera_info_rs, cx, cy), daemon=True)
                            t.start()


            # 这里原本是 MLLM 调用，现已废弃

            # --- SAM2: Reset ---
            if need_reset:
                if hasattr(sam2_predictor, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor.condition_state:
                    sam2_predictor.reset_state()
                num_targets = 0
                need_reset = False
                pending_bbox = None
                pending_point = None
                current_masks = {}
                all_prompts = []
                workspace_bbox = None  # The workspace selection outline is not a live overlay.
                logging.info("Reset, select new targets.")

            # --- SAM2: Initialize object tracking ---
            if (pending_bbox is not None or pending_point is not None) and num_targets < 1 and STATE == "IDLE":
                rs_selection_locked = True
                target_id = num_targets + 1
                if pending_bbox is not None:
                    all_prompts.append({'id': target_id, 'bbox': pending_bbox})
                    pending_bbox = None
                elif pending_point is not None:
                    all_prompts.append({'id': target_id, 'point': pending_point})
                    pending_point = None
                num_targets += 1

                if hasattr(sam2_predictor, 'condition_state') and 'point_inputs_per_obj' in sam2_predictor.condition_state:
                    sam2_predictor.reset_state()
                
                sam2_predictor.load_first_frame(tracking_img)
                for p in all_prompts:
                    tid = p['id']
                    if 'bbox' in p:
                        x1, y1, x2, y2 = p['bbox']
                        bbox_arr = np.array([[x1, y1], [x2, y2]], dtype=np.float32)
                        sam2_predictor.add_new_prompt(frame_idx=0, obj_id=tid, bbox=bbox_arr)
                    elif 'point' in p:
                        px, py = p['point']
                        pts_arr = np.array([[px, py]], dtype=np.float32)
                        lbl_arr = np.array([1], dtype=np.int32)
                        sam2_predictor.add_new_prompt(frame_idx=0, obj_id=tid, points=pts_arr, labels=lbl_arr)

            # --- SAM2: Track ---
            if num_targets > 0:
                out_obj_ids, out_mask_logits = sam2_predictor.track(tracking_img)
                current_masks = {}
                for i in range(len(out_obj_ids)):
                    obj_id = out_obj_ids[i]
                    current_masks[obj_id] = (out_mask_logits[i] > 0.0).cpu().numpy().squeeze()

            # --- Visualization ---
            for obj_id, mask in current_masks.items():
                if mask is not None:
                    color = MASK_COLORS_BGR.get(obj_id, [0, 255, 0])
                    overlay = display.copy()
                    overlay[mask > 0] = color
                    display = cv2.addWeighted(display, 1 - MASK_ALPHA, overlay, MASK_ALPHA, 0)
                    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(display, contours, -1, (0, 255, 0), 2)
            
            # 画当前工作台框 (Rotated Rect from SAM2)
            if workspace_bbox is not None and STATE == "IDLE":
                cv2.polylines(display, [workspace_bbox], isClosed=True, color=(0, 165, 255), thickness=2, lineType=cv2.LINE_AA)
                cv2.putText(display, "Workspace", (workspace_bbox[0][0], max(20, workspace_bbox[0][1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                
            if drawing and ix >= 0:
                cv2.rectangle(display, (ix, iy), (fx_mouse, fy_mouse), (255, 200, 0), 2)
                
            if vlm_running:
                cv2.putText(display, "Detecting Workspace...", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            t1 = time.time()
            fps = 1.0 / (t1 - t0)
            cv2.putText(display, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            status = f"Targets: {num_targets} | STATE: {STATE} | w=Workspace | RIGHT fist=Confirm | 0=Home | r=reset | ESC=Exit"
            cv2.putText(display, status, (10, display.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            now = time.monotonic()
            gesture_context = None
            stage = f"H2R | {STATE}"
            blocked = ""
            status = ""
            action = ""
            progress = 0.0
            color = (0, 200, 255)
            guidance_states = ("WAITING_HANDOVER_START_POSE", "WAITING_DELIVERY", "FOLLOWING_HAND")
            if STATE in guidance_states:
                action = "stop tracking and select object" if STATE == "FOLLOWING_HAND" else "start overhead guidance"
                stage = f"H2R | {action.upper()}"
                if left_hand_landmarks is None:
                    blocked = "LEFT hand missing/uncertain or hands overlap"
                elif STATE == "FOLLOWING_HAND":
                    blocked = ROS_NODE.hand_observation_error
                elif (observation_preview is None or now - observation_preview["time"] > 2.0
                      or now - observation_preview["source_time"] > 4.0
                      or np.linalg.norm(np.array(pixel) - observation_preview["pixel"]) >= 25):
                    blocked = "Checking LEFT depth and observation reachability..."
                else:
                    blocked = observation_preview["reason"] or ""
                if not blocked and right_hand_landmarks is None:
                    blocked = "Show RIGHT confirmation hand; keep hands apart"
                if not blocked and STATE != "FOLLOWING_HAND":
                    try:
                        ROS_NODE.ensure_motion_ready()
                        if np.max(np.abs(ROS_NODE.rtde_r.getActualQd())) > .003:
                            blocked = "Robot is moving: wait for it to stop"
                    except Exception as error:
                        blocked = str(error)
                if not blocked:
                    gesture_context = (STATE, None)
                    status = f"READY: {action}"
            elif STATE == "HANDOVER_SEGMENTATION":
                stage = "H2R | REVIEW HAND/OBJECT AND GRASP"
                blocked = segmentation_rejection_reason(zed_current_masks) or ""
                if handover_grasp_running:
                    blocked = "Computing depth / candidates; snapshot preview, robot holding"
                elif zed_grasp_request_pending:
                    blocked = "Waiting for segmentation to settle..."
                elif not zed_sam2_initialized:
                    blocked = "Identifying LEFT hand / waiting for object selection"
                elif now - last_mask_time > 2.0:
                    blocked = "Segmentation frame is stale"
                if not blocked and prepared_grasp is not None:
                    _, expected_q, ready_time, _ = prepared_grasp
                    try:
                        rejection = ready_rejection_reason(
                            expected_q, ready_time, ROS_NODE.rtde_r.getActualQ(), now=now,
                            camera_frame_times=(last_rs_frame_time, last_zed_frame_time),
                            joint_velocities=ROS_NODE.rtde_r.getActualQd())
                        ROS_NODE.ensure_motion_ready()
                    except Exception as error:
                        rejection = str(error)
                    if rejection:
                        blocked = rejection
                        if rejection in ("READY expired", "Robot moved since READY"):
                            prepared_grasp = None
                            grasp_status = rejection + ": prepare again"
                    else:
                        action = "execute grasp"
                        status = "READY: grasp pose checked; review gripper preview"
                        gesture_context = (STATE, ready_time)
                elif not blocked:
                    # No candidate is a stationary retry, never a return to tracking/Home.
                    action = "retry preparation"
                    status = grasp_status or "No grasp: adjust object and retry"
                    gesture_context = (STATE, "prepare")
                    try:
                        ROS_NODE.ensure_motion_ready()
                        if np.max(np.abs(ROS_NODE.rtde_r.getActualQd())) > .003:
                            blocked = "Robot is moving: wait before preparing"
                    except Exception as error:
                        blocked = str(error)
                if gesture_context is not None and right_hand_landmarks is None:
                    blocked = "Show RIGHT confirmation hand"
            else:
                blocked = (ROS_NODE.motion_error if STATE == "ERROR" else
                           "Select workspace and press W" if STATE == "IDLE" else
                           "Operation in progress; wait")
            if ROS_NODE.motion_failed.is_set():
                blocked = ROS_NODE.motion_error or "Robot stopped: check controller"
            confirmation_ready = not blocked
            if blocked:
                # Preserve the open-palm arming within this stage across preview jitter.
                # A current valid check is still required for the full fist timer.
                if (STATE in guidance_states and left_hand_landmarks is not None
                        and right_hand_landmarks is not None and not ROS_NODE.motion_failed.is_set()):
                    gesture_context = (STATE, None)
                else:
                    gesture_context = None
                status = "NOT READY: " + blocked
                color = (0, 100, 255)
            elif action == "execute grasp" or STATE in guidance_states:
                color = (0, 255, 0)
            gesture_confirm = fist_confirmation.update(
                gesture_context, visible=gesture_context is not None,
                fist=right_hand_landmarks is not None and is_fist(right_hand_landmarks),
                open_palm=right_hand_landmarks is not None and is_open_palm(right_hand_landmarks),
                now=now, ready=confirmation_ready,
            )
            hint, progress = fist_confirmation.feedback(now)
            detail = ""
            if STATE == "HOME_READY":
                hint = "HOME: gripper open"
                detail = "0=Reset workspace | ESC=Exit"
                color = (0, 255, 255)
            elif blocked:
                hint = ("NOT REACHABLE: move hand, then wait for READY"
                        if "unreachable" in blocked.lower() or "controller limits" in blocked.lower()
                        else "Checking hand position... hold still"
                        if blocked.startswith("Checking LEFT") else "NOT READY: " + blocked)
                detail = blocked if hint != "NOT READY: " + blocked else ""
            elif gesture_confirm:
                hint = "RIGHT FIST CONFIRMED"
            elif fist_confirmation.fist_since is not None:
                color = (0, 255, 0)
            elif fist_confirmation.armed:
                hint = "READY: hold RIGHT FIST 2s to confirm"
                detail = action
            else:
                hint = "Open RIGHT palm, then hold RIGHT FIST 2s"
                detail = status
            draw_handover_hint(display, hint, color, detail)
            draw_handover_hint(zed_display, hint, color, detail)
            show_realsense(display)
            show_zed(zed_display)

            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:
                ROS_NODE.motion_failed.set()
                logging.info("ESC: stopping and exiting without release or Home.")
                break
            elif key == ord('w'):
                if STATE != "IDLE" or workspace_calc_running or handover_grasp_running:
                    logging.warning("Workspace request ignored: H2R is busy or stopped.")
                    continue
                if num_targets > 0:
                    logging.info("Setting current SAM2 targets as workspace...")
                    if rs_depth_image is None:
                        logging.warning("RealSense workspace stream not ready.")
                        continue
                    combined_mask = np.zeros(rs_depth_image.shape[:2], dtype=bool)
                    for mask in current_masks.values():
                        if mask is not None:
                            combined_mask |= (mask > 0)
                    if np.any(combined_mask):
                        contour, _ = cv2.findContours(combined_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if len(contour) > 0:
                            c = max(contour, key=cv2.contourArea)
                            rect = cv2.minAreaRect(c)
                            box = cv2.boxPoints(rect)
                            box = np.int32(box)  # 4 corners of the rotated bounding box
                            workspace_bbox = box
                            
                            cx, cy = int(rect[0][0]), int(rect[0][1])
                            
                            # 获取最长边作为X轴平行的参考
                            edge1_len = np.linalg.norm(box[0] - box[1])
                            edge2_len = np.linalg.norm(box[1] - box[2])
                            if edge1_len > edge2_len:
                                edge_pt1, edge_pt2 = box[0], box[1]
                            else:
                                edge_pt1, edge_pt2 = box[1], box[2]
                            
                            def calc_rs_and_move_ws(d_img, cam_info, pt_x, pt_y, e_pt1, e_pt2):
                                nonlocal workspace_calc_running
                                global STATE
                                try:
                                    ROS_NODE.ensure_motion_ready()
                                    logging.info("Calculating workspace center and edges via RealSense depth...")
                                    ws_center_base = calc_hand_base_pos_from_rs_depth(
                                        d_img, cam_info, pt_x, pt_y, max_depth=2.0, log_prefix="workspace center")
                                    pt1_base = calc_hand_base_pos_from_rs_depth(
                                        d_img, cam_info, int(e_pt1[0]), int(e_pt1[1]), max_depth=2.0, log_prefix="workspace edge1")
                                    pt2_base = calc_hand_base_pos_from_rs_depth(
                                        d_img, cam_info, int(e_pt2[0]), int(e_pt2[1]), max_depth=2.0, log_prefix="workspace edge2")
                                    if ws_center_base is None or pt1_base is None or pt2_base is None:
                                        raise RuntimeError("Workspace depth invalid; select another region")

                                    V = pt2_base - pt1_base
                                    V[2] = 0
                                    V_norm = np.linalg.norm(V)
                                    if V_norm < 1e-5:
                                        V = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                                        V_norm = 1.0

                                    X_axis = V / V_norm
                                    Z_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)

                                    current_tcp = ROS_NODE.rtde_r.getActualTCPPose()
                                    curr_rvec = np.array([current_tcp[3], current_tcp[4], current_tcp[5]])
                                    curr_R, _ = cv2.Rodrigues(curr_rvec)
                                    curr_X_axis = curr_R[:, 0]
                                    if np.dot(X_axis, curr_X_axis) < 0:
                                        X_axis = -X_axis

                                    Y_axis = np.cross(Z_axis, X_axis)
                                    R_target = np.column_stack((X_axis, Y_axis, Z_axis))
                                    rvec, _ = cv2.Rodrigues(R_target)
                                    rx, ry, rz = rvec.flatten()

                                    ws_center_xy = np.array(ws_center_base[:2], dtype=np.float64)
                                    ws_center_xy_norm = np.linalg.norm(ws_center_xy)
                                    observe_xy = ws_center_xy.copy()
                                    if ws_center_xy_norm >= 1e-5:
                                        observe_xy = ws_center_xy - (ws_center_xy / ws_center_xy_norm) * WORKSPACE_OBSERVE_BASE_OFFSET
                                    else:
                                        logging.warning("Workspace center too close to base origin; skip 10cm base offset for observation pose.")

                                    dyn_center_pose = [-ws_center_base[0], -ws_center_base[1], ws_center_base[2] + WORKSPACE_OBSERVE_HEIGHT, rx, ry, rz]
                                    dyn_observe_pose = [-observe_xy[0], -observe_xy[1], ws_center_base[2] + WORKSPACE_OBSERVE_HEIGHT, rx, ry, rz]
                                    ROS_NODE.dynamic_workspace_center_pose = dyn_center_pose
                                    ROS_NODE.dynamic_start_pose = dyn_observe_pose
                                    ROS_NODE.get_logger().info(
                                        f"Moving to observation pose (30cm above workspace, 10cm toward base, tool0 X parallel to bbox edge): {dyn_observe_pose}"
                                    )
                                    ROS_NODE.move_to_observation(dyn_observe_pose)

                                    select_zed_mode_for_round()
                                    STATE = "WAITING_HANDOVER_START_POSE"
                                    ROS_NODE.get_logger().info("Workspace recognized. Show LEFT hand; open RIGHT palm then hold RIGHT fist 2s to start tracking.")
                                    with ROS_NODE.left_hand_lock:
                                        ROS_NODE.left_hand_base_pos = None
                                        ROS_NODE.left_hand_seen_time = 0.0
                                    logging.info("Workspace observation ready. Open RIGHT palm then fist 2s to start LEFT tracking; R reselects workspace.")
                                except Exception as error:
                                    ROS_NODE.hold_motion_error(error)
                                finally:
                                    workspace_calc_running = False
                                    if STATE == "OBSERVING":
                                        STATE = "IDLE"

                        workspace_calc_running = True
                        STATE = "OBSERVING"
                        t_start = threading.Thread(target=calc_rs_and_move_ws, args=(rs_depth_image.copy(), camera_info_rs, cx, cy, edge_pt1, edge_pt2), daemon=True)
                        t_start.start()
                        
                        # 清理现有 targets 留空进行抓取选择
                        need_reset = True
                else:
                    logging.warning("No SAM2 tracking target to set as workspace. Please specify target first.")
            elif key == ord('v'):
                if STATE == "HANDOVER_SEGMENTATION":
                    trigger_zed_segmentation(reason="Manual key v: reselect hand/object")
                else:
                    logging.warning("V cannot enter segmentation: finish positioning with RIGHT fist 2s first.")
            elif key == ord('0'):
                if recovery_running or handover_grasp_running or ROS_NODE.motion_lock.locked():
                    logging.warning("Wait for the active operation before pressing 0; use hardware stop for urgent stopping.")
                    continue
                ROS_NODE.motion_failed.set()
                ROS_NODE.recovery_pending = True
                release_on_recovery = True
                prepared_grasp = None
                STATE = "ERROR"
            elif gesture_confirm:
                if STATE == "WAITING_HANDOVER_START_POSE":
                    STATE = "HANDOVER_START_REQUESTED"
                    logging.info("RIGHT fist confirmed. Waiting for LEFT hand to start tracking.")
                    continue
                if STATE == "WAITING_DELIVERY":
                    STATE = "FOLLOWING_HAND"
                    continue
                if STATE == "FOLLOWING_HAND":
                    STATE = "HANDOVER_SELECTION_PENDING"
                    logging.info("RIGHT fist confirmed. Waiting for the current tracking move to finish before ZED selection.")
                    continue
                if STATE == "HANDOVER_SEGMENTATION" and prepared_grasp is not None:
                    if handover_grasp_running:
                        continue
                    selected, expected_q, ready_time, ready_plan = prepared_grasp
                    prepared_grasp = None
                    rejection = ready_rejection_reason(
                        expected_q, ready_time, ROS_NODE.rtde_r.getActualQ(),
                        camera_frame_times=(last_rs_frame_time, last_zed_frame_time),
                        joint_velocities=ROS_NODE.rtde_r.getActualQd(),
                    )
                    if rejection is not None:
                        grasp_status = rejection + ": release then hold RIGHT fist to prepare again"
                        logging.warning("READY cancelled: %s. Release then hold RIGHT fist to prepare again.", rejection)
                        continue
                    handover_grasp_running = True
                    STATE = "GRASPING"
                    def execute_ready(grasps, plan):
                        nonlocal handover_grasp_running, zed_need_reset
                        try:
                            zed_need_reset = True
                            ROS_NODE.execute_grasp(grasps, require_confirmation=False, prepared_plan=plan)
                        except Exception as error:
                            ROS_NODE.hold_motion_error(error)
                        finally:
                            handover_grasp_running = False
                    threading.Thread(target=execute_ready, args=(selected, ready_plan), daemon=True).start()
                    continue
                if STATE != "HANDOVER_SEGMENTATION":
                    logging.warning("Confirm with RIGHT fist while following to select the object.")
                    continue

                if 'zed_color_image' not in locals():
                    logging.warning("ZED frame not ready yet.")
                    continue

                if not zed_all_prompts:
                    logging.warning("No ZED SAM2 targets available yet.")
                    continue

                if zed_selection_mode == "manual" and len(zed_all_prompts) < 2:
                    logging.warning("Manual mode needs both hand and object selections before running GraspNet.")
                    continue

                if not zed_sam2_initialized:
                    logging.warning("ZED SAM2 is not initialized yet. Please finish target selection first.")
                    continue

                zed_grasp_request_pending = True
                if zed_segmentation_ready_time <= 0:
                    zed_segmentation_ready_time = time.time()
                wait_left = max(0.0, SAM2_SETTLE_SECONDS - (time.time() - zed_segmentation_ready_time))
                if wait_left > 0:
                    logging.info(f"Queued handover grasp generation. Waiting {wait_left:.1f}s for SAM2 to stabilize.")
                else:
                    logging.info("Queued handover grasp generation. SAM2 is ready.")
            elif key in (ord('r'), ord('R')):
                if handover_grasp_running or recovery_running:
                    logging.warning("Wait for the active operation before resetting selection.")
                    continue
                prepared_grasp = None
                if STATE == "HANDOVER_SEGMENTATION":
                    trigger_zed_segmentation(reason="Reset ZED hand/object segmentation")
                elif (STATE in ("IDLE", "WAITING_HANDOVER_START_POSE")
                      and not workspace_calc_running and not handover_start_calc_running
                      and not follow_calc_running and not ROS_NODE.motion_lock.locked()):
                    reset_workspace_selection()
                else:
                    logging.warning("Workspace reset blocked while busy or stopped; use 0 for Home.")
            elif key == ord('q'):
                break
                
    finally:
        ROS_NODE.motion_failed.set()
        zed_hands.close()
        zed.close()
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        if ROS_NODE and ROS_NODE.rtde_c:
            ROS_NODE.rtde_c.stopScript()
        try:
            executor.shutdown()
        except Exception:
            pass
        rclpy.shutdown()
        close_open3d_viewer()
        cv2.destroyAllWindows()

def parse_args():
    parser = argparse.ArgumentParser(description="Human-to-robot handover pipeline")
    parser.add_argument("--manual", action="store_true", help="Use manual workspace and hand/object selection without MLLM prompts")
    parser.add_argument("--debug", action="store_true", help="Save debug artifacts like captured_data, llm_output, mediapipe_output")
    parser.add_argument("--all-directions", action="store_true",
                        help="Opt in to directions outside ZED 30-degree cone; physical gripper/IK checks still apply")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    main(debug=args.debug, manual=args.manual, all_directions=args.all_directions)
