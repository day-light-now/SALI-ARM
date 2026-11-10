#!/bin/bash
# Launch the 2026-11-06 gravitycal_deploy four-axis float driver.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
W=/home/xuyu/slaihand_control_panel_ws_gravitycal_deploy_20261106
mkdir -p "$HERE/logs"
set +u
source /opt/ros/jazzy/setup.bash
source "$W/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
export RCUTILS_LOGGING_BUFFERED_STREAM=0
nohup ros2 launch slaihand_bringup slaihand_canusb_motion.launch.py port:=/dev/ttyACM0 \
  > "$HERE/logs/gravitycal_deploy_driver.log" 2>&1 &
echo $! > "$HERE/logs/gravitycal_deploy_driver.pid"
sleep 1
echo "gravitycal_driver_pid=$(cat "$HERE/logs/gravitycal_deploy_driver.pid")"
