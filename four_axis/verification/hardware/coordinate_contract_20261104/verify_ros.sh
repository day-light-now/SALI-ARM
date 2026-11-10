#!/usr/bin/env bash
# Isolated offline build. Does not launch ROS nodes or open any serial/CAN port.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_ros2_ws/install/setup.bash
source_root="$(cd "$(dirname "$0")/../../.." && pwd)"
evidence="$source_root/verification/hardware/coordinate_contract_20261104"
check_ws="$(mktemp -d /tmp/slai-coordinate-contract-XXXXXXXX)"
mkdir -p "$check_ws/src"
cp -a "$source_root/src/slaihand_description" "$source_root/src/slaihand_hardware" "$source_root/src/slaihand_transport" "$check_ws/src/"
cd "$check_ws"
echo "Isolated check workspace: $check_ws"
colcon build --base-paths src --packages-select slaihand_transport slaihand_description slaihand_hardware --parallel-workers 2
source install/setup.bash
colcon test --packages-select slaihand_hardware --event-handlers console_direct+ | tee "$evidence/ros_ctest.log"
colcon test-result --verbose
for axes in four five; do
  xacro "install/slaihand_description/share/slaihand_description/urdf/slaihand_${axes}_axis.urdf.xacro" -o "$check_ws/$axes.urdf"
  "$check_ws/build/slaihand_hardware/urdf_limits_test" "$check_ws/$axes.urdf"
  cp "$check_ws/$axes.urdf" "$evidence/expanded_$axes.urdf"
done
check_urdf "$source_root/src/slaihand_description/urdf/slaihand_four_axis.urdf" | tee "$evidence/check_urdf.log"
