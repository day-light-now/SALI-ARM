#!/usr/bin/env bash
set -eo pipefail
root=/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws
target=/home/xuyu/slaihand_speed01_ws_20261104
mkdir -p "$target/src"
for package in slaihand_transport slaihand_description slaihand_hardware slaihand_bringup; do
  cp -a "$root/src/$package" "$target/src/"
done
source /home/xuyu/slaihand_canusb_motion_ws_20261104_kpMgtx/install/setup.bash
cd "$target"
colcon build --packages-select slaihand_transport slaihand_description slaihand_hardware slaihand_bringup \
  --allow-overriding slaihand_transport slaihand_description slaihand_hardware slaihand_bringup \
  --event-handlers console_cohesion+
source "$target/install/setup.bash"
colcon test --packages-select slaihand_transport slaihand_hardware --event-handlers console_direct+
colcon test-result --verbose
