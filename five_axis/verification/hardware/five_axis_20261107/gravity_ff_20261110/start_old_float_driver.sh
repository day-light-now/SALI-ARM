#!/bin/bash
# Start the legacy four-axis gravity-float driver workspace (2026-11-05 build).
set -u
source /home/xuyu/slaihand_control_panel_ws_float_20261105/install/setup.bash
export ROS_DOMAIN_ID=183
export RCUTILS_LOGGING_BUFFERED_STREAM=0
exec ros2 launch slaihand_bringup slaihand_canusb_motion.launch.py port:=/dev/ttyACM0
