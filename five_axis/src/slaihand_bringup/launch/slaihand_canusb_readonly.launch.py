"""Explicitly launched state-only CANUSB owner. No motor command interfaces."""
import os
import hashlib
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    port = LaunchConfiguration('port').perform(context).strip()
    if not port.startswith('/dev/'):
        raise RuntimeError('Specify port:=/dev/<actual-CANUSB-device>. Windows COM9 cannot be opened by Linux ROS.')
    description = get_package_share_directory('slaihand_description')
    bringup = get_package_share_directory('slaihand_bringup')
    mapping_file = os.path.join(bringup, 'config', 'hardware_mapping.yaml')
    model_file = os.path.join(description, 'urdf', 'slaihand_four_axis.urdf')
    def sha(path):
        with open(path, 'rb') as stream:
            return hashlib.file_digest(stream, 'sha256').hexdigest()
    xml = xacro.process_file(os.path.join(description, 'urdf', 'slaihand_canusb_readonly.urdf.xacro'),
        mappings={'port': port, 'mapping_file': mapping_file,
                  'log_directory': LaunchConfiguration('log_directory').perform(context),
                  'mapping_sha256': sha(mapping_file), 'model_sha256': sha(model_file)}).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='slaihand',
             parameters=[{'robot_description': ParameterValue(xml, value_type=str)}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             namespace='slaihand',
             parameters=[os.path.join(bringup, 'config', 'canusb_readonly_controllers.yaml')],
             remappings=[('~/robot_description', '/slaihand/robot_description'),
                         ('robot_description', '/slaihand/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             namespace='slaihand',
             arguments=['joint_state_broadcaster', '--controller-manager', '/slaihand/controller_manager'], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='', description='Explicit Linux/WSL serial device for the existing CANUSB'),
        DeclareLaunchArgument('log_directory', default_value=os.path.expanduser('~/.ros/slaihand/canusb_readonly')),
        OpaqueFunction(function=nodes),
    ])
