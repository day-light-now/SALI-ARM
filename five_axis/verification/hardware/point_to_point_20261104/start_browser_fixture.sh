#!/usr/bin/env bash
set -e
root=/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws
source /home/xuyu/slaihand_ptp_ws_20261104/install/setup.bash
export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
export PANEL_SCRIPT=/home/xuyu/slaihand_ptp_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py
if [ -f /tmp/panel-browser-fixture/fixture.json ]; then
  bash "$root/verification/hardware/control_panel_20261104/stop_browser_fixture.sh"
fi
cd "$root/verification/hardware/control_panel_20261104"
setsid nohup python3 browser_fixture.py >/tmp/ptp-browser-fixture.log 2>&1 < /dev/null &
sleep 1
