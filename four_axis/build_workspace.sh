#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /home/xuyu/slaihand_ros2_ws
cp -a "$(dirname "$0")/src" /home/xuyu/slaihand_ros2_ws/
cd /home/xuyu/slaihand_ros2_ws
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
xacro install/slaihand_description/share/slaihand_description/urdf/slaihand_four_axis.urdf.xacro -o /tmp/slaihand_expanded.urdf
python3 -c 'import xml.etree.ElementTree as E; r=E.parse("/tmp/slaihand_expanded.urdf").getroot(); assert len(r.findall("joint")) == 4; assert r.find("ros2_control/hardware/plugin").text == "slaihand_hardware/OpenArmJointFake"; print("Expanded four-axis configurable OpenArm offline adapter verified")'
