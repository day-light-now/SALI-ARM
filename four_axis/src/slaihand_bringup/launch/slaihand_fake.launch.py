import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory("slaihand_description")
    bringup_dir = get_package_share_directory("slaihand_bringup")
    xacro_file = os.path.join(description_dir, "urdf", "slaihand_four_axis.urdf.xacro")
    controllers_file = os.path.join(bringup_dir, "config", "slaihand_controllers.yaml")
    rviz_config = os.path.join(description_dir, "config", "slaihand.rviz")
    robot_description = ParameterValue(Command(["xacro ", xacro_file,
        " backend:=", LaunchConfiguration("backend")]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="openarm", choices=["openarm", "generic"]),
        DeclareLaunchArgument("gui", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers_file],
             remappings=[("robot_description", "/robot_description")], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["slaihand_joint_trajectory_controller", "--controller-manager", "/controller_manager"], output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config],
             condition=IfCondition(LaunchConfiguration("gui")), output="screen"),
    ])
