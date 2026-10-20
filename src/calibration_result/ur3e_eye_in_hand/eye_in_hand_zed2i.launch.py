"""Publish the newest completed ZED eye-in-hand calibration.

The newest calibration_data/zed_eye_in_hand_*/calibrated_result/
zed_tool0_calibration.json is read at launch time, so a completed new
calibration is used on the next launch without editing this file.
Independent physical validation of the selected transform is still required.
Publish this child frame only once.
"""
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

PATTERN = ('calibration_data/zed_eye_in_hand_*/calibrated_result/'
           'zed_tool0_calibration.json')


def _project_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'calibration_data').is_dir():
            return parent
    raise RuntimeError(f'calibration_data not found above {__file__}')


def _latest_calibration():
    candidates = []
    for path in _project_root().glob(PATTERN):
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            translation = data['translation_m']
            quaternion = data['quaternion_xyzw']
        except (OSError, ValueError, KeyError, TypeError):
            continue
        if len(translation) != 3 or len(quaternion) != 4:
            continue
        if (data.get('parent_frame') != 'tool0'
                or data.get('child_frame') != 'zed_inhand_camera_frame_optical'):
            continue
        candidates.append((path.parent.parent.name, path, data))
    if not candidates:
        raise RuntimeError(f'No valid ZED calibration matches {PATTERN}')
    return max(candidates, key=lambda item: item[0])[1:]


def generate_launch_description():
    path, data = _latest_calibration()
    values = [*data['translation_m'], *data['quaternion_xyzw']]
    flags = ['--frame-id', 'tool0',
             '--child-frame-id', 'zed_inhand_camera_frame_optical']
    for name, value in zip(
            ('--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'), values):
        flags += [name, format(float(value), '.12g')]
    return LaunchDescription([
        LogInfo(msg=f'ZED eye-in-hand: using newest calibration {path}'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             output='log', arguments=flags),
    ])
