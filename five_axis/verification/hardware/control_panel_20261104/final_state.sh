#!/usr/bin/env bash
echo "=== test processes"
ps -eo pid,etime,cmd | grep -E "browser_fixture|panel-browser-fixture|/dev/pts" | grep -v grep || echo none
echo "=== panels"
ps -eo pid,etime,cmd | grep "control_panel.py" | grep -v grep || echo none
echo "=== real stack"
ps -eo pid,etime,cmd | grep "dev/serial/by-id" | grep -v grep | cut -c1-140
echo "=== domain 176 publishers (should be unknown)"
export ROS_DOMAIN_ID=176 ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source /home/xuyu/slaihand_control_panel_ws_20261104/install/setup.bash
timeout 10 ros2 topic info /slaihand/robot_description 2>&1 | head -3
