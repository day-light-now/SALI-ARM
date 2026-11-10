#!/bin/bash
# Shared environment for the GC05-GC08 acquisition (2026-11-09).
# ROS setup scripts reference unset variables, so relax -u while sourcing.
set +u
cd "/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain"
source /opt/ros/jazzy/setup.bash
source /home/xuyu/.cache/dsh_sched/install/setup.bash
set -u
export ROS_DOMAIN_ID=183
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export PYTHONUNBUFFERED=1
