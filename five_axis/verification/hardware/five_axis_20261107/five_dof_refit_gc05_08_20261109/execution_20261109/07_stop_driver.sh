#!/bin/bash
# Step 7: stop the candidate launch gracefully and verify the arm is disarmed.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh" 2>/dev/null || true
PID="$(cat "$HERE/logs/driver.pid" 2>/dev/null || true)"
echo "stored_driver_pid=${PID:-missing}"
if [ -n "${PID:-}" ] && kill -0 "$PID" 2>/dev/null; then
  kill -INT "$PID" 2>/dev/null || true
fi
for pattern in 'ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate' \
               'ros2_control_node'; do
  for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    [ "$pid" = "$$" ] && continue
    echo "SIGINT $pid ($pattern)"
    kill -INT "$pid" 2>/dev/null || true
  done
done
sleep 8
echo '--- remaining processes ---'
pgrep -af 'ros2 launch slaihand_bringup|ros2_control_node|slaihand_joint_trajectory|robot_state_publisher' \
  | grep -v pgrep || echo NO_SLAIHAND_ROS_PROCESS
