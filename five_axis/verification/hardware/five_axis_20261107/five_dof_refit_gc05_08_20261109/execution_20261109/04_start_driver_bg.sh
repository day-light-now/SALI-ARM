#!/bin/bash
# Step 4: start the candidate launch detached; write its PID.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
nohup bash "$HERE/03_launch_driver.sh" > "$HERE/logs/driver_launch.log" 2>&1 &
PID=$!
echo "$PID" > "$HERE/logs/driver.pid"
sleep 1
echo "driver_launch_pid=$PID"
