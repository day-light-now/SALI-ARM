#!/bin/bash
# Run the three J1 hysteresis plans back-to-back (driver already admitted).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
for name in J1H_m8 J1H_0 J1H_p8; do
  plan="$HERE/plans/${name}_20261111.json"
  stage="$(python3 "$HERE/panel_stage.py" 2>/dev/null || echo x)"
  if [ "$stage" != "0" ]; then
    echo "stage=$stage; resetting panel"
    python3 "$HERE/panel_reset.py" || { echo "ABORT reset $name"; exit 1; }
    sleep 1
  fi
  echo "=== $name ==="
  bash "$HERE/run_plan.sh" "$plan" "$name" 92
  if [ "$(pgrep -f 'ros2_control_node --ros-args' | wc -l)" != "1" ]; then
    echo "ABORT driver died after $name"; exit 1
  fi
  python3 "$HERE/extract_windows.py" "$name" "$plan" > "$HERE/logs/${name}_extract.log" 2>&1 \
    || echo "WARN extract $name"
done
echo J1H_DONE
