#!/usr/bin/env bash
SRC="/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/src/slaihand_bringup"
INST="/home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_bringup"
echo "=== installed vs source"
for f in scripts/control_panel.py scripts/control_panel_model.py scripts/control_panel_http.py scripts/commission_supported_trial.py scripts/joint_waypoint_plan.py; do
  a=$(sha256sum "$SRC/$f" | cut -d' ' -f1); b=$(sha256sum "$INST/lib/slaihand_bringup/$(basename $f)" | cut -d' ' -f1)
  [ "$a" = "$b" ] && echo "same $f" || echo "DIFF $f ($a vs $b)"
done
for f in index.html panel.css panel.js; do
  a=$(sha256sum "$SRC/web/control_panel/$f" | cut -d' ' -f1); b=$(sha256sum "$INST/share/slaihand_bringup/web/control_panel/$f" | cut -d' ' -f1)
  [ "$a" = "$b" ] && echo "same web/$f" || echo "DIFF web/$f"
done
echo "=== leftover test processes (should be none)"
ps -eo pid,etime,cmd | grep -E "control_panel.py|browser_fixture|port:=/dev/pts" | grep -v grep || echo none
echo "=== real stack"
ps -eo pid,etime,cmd | grep "dev/serial/by-id" | grep -v grep || echo "REAL_STACK_NOT_FOUND"
