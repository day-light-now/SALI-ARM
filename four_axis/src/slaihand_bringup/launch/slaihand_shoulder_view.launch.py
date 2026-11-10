"""Additional shoulder-mounted RViz view; reuse an already running fake robot."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('slaihand_description'),
                          'config', 'slaihand_shoulder.rviz')
    # Installation ledger, 2026-10-08:
    # User diagram identifies base +Y as down. shoulder_world +Z is up.
    # Preserve base +X; rotate about X by -pi/2: base +Y -> world -Z,
    # base +Z -> world +Y. Translation is zero (display origin, NOT body height).
    # This TF is external to the canonical URDF; internal frames/signs/limits
    # and joint zero remain unchanged. No dynamics or real hardware is added.
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='slaihand_shoulder_mount', arguments=[
                 '--x', '0', '--y', '0', '--z', '0',
                 '--roll', str(-math.pi / 2), '--pitch', '0', '--yaw', '0',
                 '--frame-id', 'shoulder_world', '--child-frame-id', 'base_link'],
             output='screen'),
        Node(package='rviz2', executable='rviz2', name='slaihand_shoulder_rviz',
             arguments=['-d', config], output='screen'),
    ])
