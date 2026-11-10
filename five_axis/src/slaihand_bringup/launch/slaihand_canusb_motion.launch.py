"""Starts disarmed. No arm service is called and the standard JTC stays inactive."""
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _number(value):
    return format(float(value), '.17g')


def build_gravity_model_text(model_path, profile_path, gravity, j3_offset):
    """Compact model text consumed by GravityFfModel in the hardware component.

    Format: G:gx:gy:gz | L:name:mass:comx:comy:comz | J:name:parent:child:xyz(3):
    rpy(3):axis(3):offset:lower:upper:r|f, records separated by '|'.
    """
    root = ET.parse(model_path).getroot()
    sys.path.insert(0, os.path.join(get_package_prefix('slaihand_bringup'), 'lib', 'slaihand_bringup'))
    from control_panel_gravity import FourAxisGravityModel
    parsed_model = FourAxisGravityModel(model_path, profile_path, gravity,
        fixed_joint_offsets={'slaihand_joint3':float(j3_offset)})
    records = ['G:' + ':'.join(_number(value) for value in gravity)]
    for link in root.findall('link'):
        entry = parsed_model.masses.get(link.get('name'))
        if entry is None:
            continue
        effective = link.get('name') in parsed_model.first_moments
        values = parsed_model.first_moments[link.get('name')] if effective else entry[1]
        records.append(':'.join(['B' if effective else 'L', link.get('name'), _number(entry[0])]
                                + [_number(value) for value in values]))
    for joint in root.findall('joint'):
        kind = joint.get('type')
        if kind not in ('revolute', 'fixed'):
            raise RuntimeError('Unsupported joint type in gravity model: %s' % kind)
        name = joint.get('name')
        origin = joint.find('origin')
        xyz = [float(value) for value in (origin.get('xyz').split() if origin is not None
                                          and origin.get('xyz') else [0.0, 0.0, 0.0])]
        rpy = [float(value) for value in (origin.get('rpy').split() if origin is not None
                                          and origin.get('rpy') else [0.0, 0.0, 0.0])]
        axis = [0.0, 0.0, 1.0]
        offset = lower = upper = 0.0
        if kind == 'revolute':
            axis_node = joint.find('axis')
            axis = [float(value) for value in axis_node.get('xyz').split()]
            limit = joint.find('limit')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if name == 'slaihand_joint3':
                offset = float(j3_offset)
        records.append(':'.join(['J', name, joint.find('parent').get('link'), joint.find('child').get('link')]
                                + [_number(value) for value in xyz + rpy + axis]
                                + [_number(offset), _number(lower), _number(upper),
                                   'r' if kind == 'revolute' else 'f']))
    return '|'.join(records)


def nodes(context):
    port = LaunchConfiguration('port').perform(context).strip()
    if not port.startswith('/dev/'):
        raise RuntimeError('Explicit Linux/WSL CANUSB port required')
    description = get_package_share_directory('slaihand_description')
    bringup = get_package_share_directory('slaihand_bringup')
    paths = dict(mapping=os.path.join(bringup, 'config', 'hardware_mapping.yaml'),
                 model=os.path.join(description, 'urdf', 'slaihand_four_axis.urdf'),
                 profile=os.path.join(bringup, 'config', 'gravity_provisional.json'),
                 policy=os.path.join(bringup, 'config', 'canusb_motion_policy.yaml'))
    def sha(path):
        with open(path, 'rb') as stream:
            return hashlib.file_digest(stream, 'sha256').hexdigest()
    with open(paths['policy'], encoding='utf-8') as stream:
        policy = yaml.safe_load(stream) or {}
    profile_name = policy.get('gravity_profile', 'gravity_provisional.json')
    if not isinstance(profile_name, str) or os.path.basename(profile_name) != profile_name or '\\' in profile_name:
        raise RuntimeError('gravity_profile must name a config file')
    paths['profile'] = os.path.join(bringup, 'config', profile_name)
    with open(paths['profile'], encoding='utf-8') as stream:
        profile = json.load(stream)
    if profile.get('schema') == 'slaihand_identified_gravity_base/1':
        if profile['identity']['mapping_sha256'] != sha(paths['mapping']):
            raise RuntimeError('Identified gravity mapping does not match current configuration')
        if profile['identity']['load_id'] != policy.get('load_id'):
            raise RuntimeError('Identified gravity load declaration does not match')
        if any(profile.get('training_gains', {}).get(key) != policy.get(key) for key in ('kp', 'kd')):
            raise RuntimeError('Identified gravity trial gains differ from capture gains')
    gravity = [float(policy.get('gravity_base_x', 0.0)),
               float(policy.get('gravity_base_y', 9.80665)),
               float(policy.get('gravity_base_z', 0.0))]
    j3_offset = float(policy.get('gravity_j3_offset_rad', math.pi))
    gravity_model = ''
    if os.path.isfile(paths['profile']):
        gravity_model = build_gravity_model_text(paths['model'], paths['profile'], gravity, j3_offset)
    else:
        print('WARNING: gravity mass profile missing; feedforward model disabled: %s' % paths['profile'])
    alpha = LaunchConfiguration('gravity_ff_alpha').perform(context).strip()
    alpha = _number(policy.get('gravity_ff_alpha', 0.0)) if not alpha else _number(alpha)
    xml = xacro.process_file(os.path.join(description, 'urdf', 'slaihand_canusb_motion.urdf.xacro'),
        mappings=dict(port=port, mapping_file=paths['mapping'], policy_file=paths['policy'],
                      mapping_sha256=sha(paths['mapping']), model_sha256=sha(paths['model']),
                      policy_sha256=sha(paths['policy']),
                      mass_profile_sha256=sha(paths['profile']),
                      load_id=str(policy.get('load_id', '')),
                      gravity_model=gravity_model, gravity_ff_alpha=alpha,
                      log_directory=LaunchConfiguration('log_directory').perform(context))).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace='slaihand',
             parameters=[{'robot_description': ParameterValue(xml, value_type=str)}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node', namespace='slaihand',
             parameters=[os.path.join(bringup, 'config', 'canusb_motion_controllers.yaml')],
             remappings=[('~/robot_description', '/slaihand/robot_description'),
                         ('robot_description', '/slaihand/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace='slaihand',
             arguments=['joint_state_broadcaster', '--controller-manager', '/slaihand/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner', namespace='slaihand',
             arguments=['slaihand_joint_trajectory_controller', '--inactive', '--controller-manager', '/slaihand/controller_manager'], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=''),
        DeclareLaunchArgument('log_directory', default_value=os.path.expanduser('~/.ros/slaihand/canusb_motion')),
        DeclareLaunchArgument('gravity_ff_alpha', default_value='',
                              description='Override gravity feedforward alpha (default: policy value)'),
        OpaqueFunction(function=nodes),
    ])

