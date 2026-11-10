#!/usr/bin/env bash
# 拖动示教测试环境：唯一教学候选工作空间 + 隔离域 176。
# Source 本文件，不要执行。
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX LD_LIBRARY_PATH PYTHONPATH \
      COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH AMENT_PYTHON_EXECUTABLE \
      ROS_LOCALHOST_ONLY ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export SLAIHAND_CANDIDATE_WS=/home/xuyu/slaihand_control_panel_ws_dragteaching_20261106
if [ ! -f "$SLAIHAND_CANDIDATE_WS/install/setup.bash" ]; then
  echo "candidate workspace is not built: $SLAIHAND_CANDIDATE_WS" >&2
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source "$SLAIHAND_CANDIDATE_WS/install/local_setup.bash"
export PANEL_SCRIPT="$SLAIHAND_CANDIDATE_WS/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py"
export PANEL_WORKSPACE="$SLAIHAND_CANDIDATE_WS"
export ROS_DOMAIN_ID=176
export ROS_LOCALHOST_ONLY=1
