"""Local calibration from calibration_data/zed_eye_in_hand_20261019_140354_716625/calibrated_result/zed_tool0_calibration.json
Independent physical validation pending. Publish this child frame only once.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="tf2_ros", executable="static_transform_publisher",
        output="log",
        arguments=['--frame-id', 'tool0', '--child-frame-id', 'zed_inhand_camera_frame_optical', '--x', '-0.06002297375095697', '--y', '-0.05860468022657418', '--z', '0.0613009306353398', '--qx', '-0.019370027874477853', '--qy', '-0.0005445874980252557', '--qz', '0.002837277640723051', '--qw', '0.9998082092582499'],
    )])
