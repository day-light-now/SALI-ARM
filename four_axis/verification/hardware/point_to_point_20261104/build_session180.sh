#!/usr/bin/env bash
set -eo pipefail
root=/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws
target=/home/xuyu/slaihand_session180_ws_20261104
mkdir -p "$target/src"
for pkg in slaihand_hardware slaihand_description; do
  test ! -e "$target/src/$pkg"
  cp -a "/home/xuyu/slaihand_speed05_ws_20261104/src/$pkg" "$target/src/"
done
cp -a "$root/src/slaihand_bringup" "$target/src/"
for path in src/motion_supervisor.cpp include/slaihand_hardware/motion_supervisor.hpp test/motion_supervisor_test.cpp; do
  cp "$root/src/slaihand_hardware/$path" "$target/src/slaihand_hardware/$path"
done
source /home/xuyu/slaihand_speed05_ws_20261104/install/setup.bash
cd "$target"
colcon build --packages-select slaihand_hardware slaihand_description slaihand_bringup --event-handlers console_cohesion+
colcon test --packages-select slaihand_hardware --event-handlers console_cohesion+
colcon test-result --verbose
python3 "$root/verification/hardware/control_panel_20261104/test_control_panel_offline.py"
python3 "$root/verification/hardware/point_to_point_20261104/test_point_to_point.py"
