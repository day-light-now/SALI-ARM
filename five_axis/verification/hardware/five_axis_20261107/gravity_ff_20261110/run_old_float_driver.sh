#!/bin/bash
# Launch the legacy four-axis gravity-float driver in the background with a log.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/logs"
set +u
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_control_panel_ws_float_20261105/install/setup.bash
set -u
export ROS_DOMAIN_ID=183
export RCUTILS_LOGGING_BUFFERED_STREAM=0
nohup ros2 launch slaihand_bringup slaihand_canusb_motion.launch.py port:=/dev/ttyACM0 \
  > "$HERE/logs/old_float_driver.log" 2>&1 &
echo $! > "$HERE/logs/old_float_driver.pid"
sleep 1
echo "old_float_driver_pid=$(cat "$HERE/logs/old_float_driver.pid")"
