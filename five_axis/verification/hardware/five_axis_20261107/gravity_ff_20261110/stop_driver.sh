#!/bin/bash
# Gracefully stop the candidate driver started by 04_start_driver_bg.sh.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
PID="$(cat "$HERE/logs/driver.pid" 2>/dev/null || true)"
if [ -n "${PID:-}" ]; then
  kill -INT "$PID" 2>/dev/null || true
fi
sleep 2
for pattern in 'ros2_control_node --ros-args' \
               'ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate' \
               'robot_state_publisher --ros-args'; do
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    if [ "$pid" != "$$" ]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
done
sleep 2
pgrep -af 'ros2_control_node|ros2 launch slaihand' | grep -v pgrep || echo ALL_CLEAN
fuser -v /dev/ttyACM0 2>&1 || echo TTY_FREE
