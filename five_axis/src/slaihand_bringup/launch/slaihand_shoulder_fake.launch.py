"""Start the original four-axis fake robot with the separate shoulder view."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('slaihand_bringup'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slaihand_fake.launch.py')),
            launch_arguments={'backend': 'openarm', 'gui': 'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slaihand_shoulder_view.launch.py'))),
    ])
