"""Shared configuration and guarded joint-space Home return for handover programs."""
import json
import math
from pathlib import Path

HOME_PATH = Path(__file__).resolve().parent / 'config' / 'robot_home.json'


def load_home_joints(path=HOME_PATH):
    joints = json.loads(Path(path).read_text())['joint_positions_rad']
    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
        raise ValueError('Home must contain six finite joint positions in radians')
    return list(joints)


def move_home(control, receiver, joints):
    """Return without changing the gripper; never reset a robot safety stop.

    Uses joint motion because the recorded extended Home can be singular in
    Cartesian space. Robot safety limits do not constitute collision planning.
    """
    if not control or not receiver or not receiver.isConnected():
        raise RuntimeError('Home return unavailable: RTDE disconnected')
    if not control.isConnected() or not control.isProgramRunning():
        raise RuntimeError('Home return blocked: RTDE control script is not running')
    if receiver.getRobotMode() != 7 or receiver.getSafetyMode() != 1:
        raise RuntimeError('Home return blocked by robot/safety state')
    if max(abs(v) for v in receiver.getActualQd()) > .01:
        raise RuntimeError('Home return blocked: previous motion has not stopped')
    if not control.isJointsWithinSafetyLimits(joints):
        raise RuntimeError('Home is outside configured robot joint safety limits')
    if max(abs(a-b) for a, b in zip(receiver.getActualQ(), joints)) < .002:
        return
    if not control.moveJ(joints, .30, .30):
        raise RuntimeError('Home motion did not complete')
    if max(abs(a-b) for a, b in zip(receiver.getActualQ(), joints)) > .01:
        raise RuntimeError('Home endpoint was not reached')


def load_motion_settings():
    settings = json.loads((HOME_PATH.parent / 'handover_motion.json').read_text())
    limits = {
        'gripper_force_N': (20., 235.),
        'post_grasp_lift_height_m': (.10, .40),
        'grasp_move_speed_m_s': (.01, .15),
        'grasp_move_acceleration_m_s2': (.01, .50),
    }
    for key, (lower, upper) in limits.items():
        value = settings[key]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f'Invalid motion setting: {key}')
    return settings


def post_grasp_lift_pose(target_pose, approach_pose, height):
    """Lift in controller base Z, independently of the gripper approach direction."""
    pose = list(approach_pose)
    pose[2] = max(pose[2], target_pose[2] + height)
    return pose
