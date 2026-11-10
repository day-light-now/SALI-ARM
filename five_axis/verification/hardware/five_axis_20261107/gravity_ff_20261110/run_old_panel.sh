#!/bin/bash
# Launch the legacy four-axis control panel (float drag UI) on port 42665.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HERE/logs"
set +u
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_control_panel_ws_float_20261105/install/setup.bash
set -u
export ROS_DOMAIN_ID=183
nohup python3 /home/xuyu/slaihand_control_panel_ws_float_20261105/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py \
  --port 42665 > "$HERE/logs/old_float_panel.log" 2>&1 &
echo $! > "$HERE/logs/old_float_panel.pid"
sleep 1
echo "old_panel_pid=$(cat "$HERE/logs/old_float_panel.pid")"
