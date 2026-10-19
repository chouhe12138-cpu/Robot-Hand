"""Local calibration from calibration_data/d435_eye_to_hand_20261018_204340_085125/calibrated_result/d435_base_calibration.json
Independent physical validation pending. Publish this child frame only once.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(
        package="tf2_ros", executable="static_transform_publisher",
        output="log",
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'rs_tohand_color_optical_frame', '--x', '0.25494109016705735', '--y', '-0.238060878089931', '--z', '0.5480193704097971', '--qx', '-0.9411749721847735', '--qy', '0.24399914958516022', '--qz', '-0.05717021635647178', '--qw', '0.22668403802752268'],
    )])
