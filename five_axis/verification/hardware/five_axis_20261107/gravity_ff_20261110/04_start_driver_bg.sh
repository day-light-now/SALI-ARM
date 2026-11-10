#!/bin/bash
# Start the candidate launch detached with a unique log path (never overwrite).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$HERE/logs/driver_launch_${STAMP}.log"
nohup bash "$HERE/03_launch_driver.sh" > "$LOG" 2>&1 &
PID=$!
echo "$PID" > "$HERE/logs/driver.pid"
sleep 1
echo "driver_launch_pid=$PID log=$LOG"
