#!/bin/bash
# Shared environment for the 2026-11-10 gravity-sag compensation session.
set +u
cd "/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain"
source /opt/ros/jazzy/setup.bash
source /home/xuyu/.cache/dsh_sched/install/setup.bash
set -u
export ROS_DOMAIN_ID=183
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1
