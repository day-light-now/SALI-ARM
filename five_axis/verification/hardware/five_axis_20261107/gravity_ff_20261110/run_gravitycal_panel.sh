#!/bin/bash
# Launch the 2026-11-06 gravitycal_deploy four-axis panel (KD/seconds float UI).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
W=/home/xuyu/slaihand_control_panel_ws_gravitycal_deploy_20261106
mkdir -p "$HERE/logs"
set +u
source /opt/ros/jazzy/setup.bash
source "$W/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
nohup python3 "$W/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py" \
  --port 42663 > "$HERE/logs/gravitycal_deploy_panel.log" 2>&1 &
echo $! > "$HERE/logs/gravitycal_deploy_panel.pid"
sleep 2
echo "gravitycal_panel_pid=$(cat "$HERE/logs/gravitycal_deploy_panel.pid")"
cat "$HERE/logs/gravitycal_deploy_panel.log"
