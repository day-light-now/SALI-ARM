#!/bin/bash
# Inspect driver processes and terminate stale launch instances (keeps driver.pid).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
echo '--- processes ---'
ps -eo pid,ppid,comm,args | grep -E 'ros2 launch slaihand|ros2_control_node|robot_state_publisher' | grep -v grep || echo NONE
echo '--- logs ---'
ls -l "$HERE/logs"
CURRENT="$(cat "$HERE/logs/driver.pid" 2>/dev/null || true)"
echo "current_driver_pid=${CURRENT:-missing}"
for pid in $(pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate.launch.py' 2>/dev/null); do
  if [ "$pid" != "${CURRENT:-none}" ]; then
    echo "TERM stale launch $pid"
    kill -TERM "$pid" 2>/dev/null || true
  fi
done
sleep 2
echo '--- after ---'
ps -eo pid,ppid,comm,args | grep -E 'ros2 launch slaihand|ros2_control_node|robot_state_publisher' | grep -v grep || echo NONE
echo '--- session1 statistics registrations ---'
grep -c 'Registering statistics' "$HERE/logs/driver_launch_session1.log" 2>/dev/null || echo no_session1_log
