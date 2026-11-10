"""Candidate five-axis CANUSB motion launch. Starts disarmed; J5 motion is
gated by calibration_qualified=false until real zero/sign/limits/TIMEOUT exist."""
import hashlib
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def sha(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def nodes(context):
    port = LaunchConfiguration("port").perform(context).strip()
    if not port.startswith("/dev/"):
        raise RuntimeError("Explicit Linux/WSL CANUSB port required")
    description = get_package_share_directory("slaihand_description")
    bringup = get_package_share_directory("slaihand_bringup")
    mapping = os.path.join(bringup, "config", "hardware_mapping.yaml")
    policy = os.path.join(bringup, "config", "canusb_five_axis_motion_policy.yaml")
    model = os.path.join(description, "urdf", "slaihand_five_axis.urdf")
    with open(policy, encoding="utf-8") as stream:
        policy_data = yaml.safe_load(stream) or {}
    if policy_data.get("profile") != "candidate_five_axis_native_mit_j5_gated":
        raise RuntimeError("Require explicit five-axis candidate policy")
    # Additive fallback only: an explicit load_id launch argument lets the
    # candidate admission launch pass the authoritative PROVISIONAL load
    # declaration id.  When omitted (every existing test launch) the historical
    # policy-file load_id is used, so the old launch path is unchanged.
    load_id = (LaunchConfiguration("load_id").perform(context).strip()
               or str(policy_data.get("load_id", "")))
    xml = xacro.process_file(
        os.path.join(description, "urdf", "slaihand_canusb_five_axis.urdf.xacro"),
        mappings=dict(
            port=port, mapping_file=mapping, policy_file=policy,
            mapping_sha256=sha(mapping), model_sha256=sha(model),
            policy_sha256=sha(policy),
            mass_profile_sha256=LaunchConfiguration("mass_profile_sha256").perform(context),
            load_id=load_id,
            log_directory=LaunchConfiguration("log_directory").perform(context),
            j5_qualified=LaunchConfiguration("j5_qualified").perform(context),
            j5_calibration_sha256=LaunchConfiguration("j5_calibration_sha256").perform(context),
            j5_hardware_sign=LaunchConfiguration("j5_hardware_sign").perform(context),
            j5_zero_offset_rad=LaunchConfiguration("j5_zero_offset_rad").perform(context),
            j5_limit_lower=LaunchConfiguration("j5_limit_lower").perform(context),
            j5_limit_upper=LaunchConfiguration("j5_limit_upper").perform(context),
        ),
    ).toxml()
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             namespace="slaihand",
             parameters=[{"robot_description": ParameterValue(xml, value_type=str)}],
             output="screen"),
        Node(package="controller_manager", executable="ros2_control_node",
             namespace="slaihand",
             parameters=[os.path.join(bringup, "config", "canusb_five_axis_controllers.yaml")],
             remappings=[("~/robot_description", "/slaihand/robot_description"),
                         ("robot_description", "/slaihand/robot_description")],
             output="screen"),
        Node(package="controller_manager", executable="spawner", namespace="slaihand",
             arguments=["joint_state_broadcaster", "--controller-manager",
                        "/slaihand/controller_manager"], output="screen"),
        Node(package="controller_manager", executable="spawner", namespace="slaihand",
             arguments=["slaihand_joint_trajectory_controller", "--inactive",
                        "--controller-manager", "/slaihand/controller_manager"],
             output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("log_directory",
                              default_value=os.path.expanduser(
                                  "~/.ros/slaihand/canusb_five_axis_motion")),
        # Test/simulation qualification knobs; production defaults keep J5
        # feedback-only and unqualified.
        DeclareLaunchArgument("j5_qualified", default_value="false"),
        DeclareLaunchArgument("j5_calibration_sha256", default_value=""),
        DeclareLaunchArgument("j5_hardware_sign", default_value="1"),
        DeclareLaunchArgument("j5_zero_offset_rad", default_value="0.0"),
        DeclareLaunchArgument("j5_limit_lower", default_value="-0.5235987755982988"),
        DeclareLaunchArgument("j5_limit_upper", default_value="0.5235987755982988"),
        DeclareLaunchArgument("mass_profile_sha256", default_value=""),
        # Empty keeps the legacy policy-file load_id fallback; the candidate
        # launch passes the load declaration's authoritative load_id here.
        DeclareLaunchArgument("load_id", default_value=""),
        OpaqueFunction(function=nodes),
    ])
