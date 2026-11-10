#!/bin/bash
# Run the five single-joint bidirectional friction sweeps sequentially.
# Each plan is <=178 s; software reset between runs keeps the driver DISARMED.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for name in F1 F2 F3 F4 F5; do
  echo "=== friction run $name ==="
  python3 "$HERE/panel_reset.py" >/dev/null 2>&1 || true
  sleep 1
  bash "$HERE/run_plan.sh" "$HERE/plans/${name}_friction_20261110.json" "${name}fr" 143
  if ! pgrep -f 'ros2_control_node --ros-args' >/dev/null 2>&1; then
    echo "DRIVER_DEAD after $name"
    exit 1
  fi
done
echo "FRICTION_SESSIONS_DONE"
