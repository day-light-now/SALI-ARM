#!/usr/bin/env bash
# Deterministic ROS test environment.
# Drops inherited workspace overlays (other workspaces on this machine contain
# stale hardware libraries) and sources only the base ROS install plus the
# isolated candidate workspace. Source this file, do not execute it.
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX LD_LIBRARY_PATH PYTHONPATH \
      COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH AMENT_PYTHON_EXECUTABLE \
      ROS_LOCALHOST_ONLY ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
panel_candidate_ws="${SLAIHAND_CANDIDATE_WS:-/home/xuyu/slaihand_control_panel_ws_speed1_20261105}"
if [ ! -f "$panel_candidate_ws/install/setup.bash" ]; then
  echo "candidate workspace is not built: run stage_workspace.sh first" >&2
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source "$panel_candidate_ws/install/local_setup.bash"
