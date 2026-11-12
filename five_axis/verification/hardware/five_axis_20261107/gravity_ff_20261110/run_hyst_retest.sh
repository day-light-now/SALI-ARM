#!/bin/bash
# Run hysteresis-compensated H2/H4 plans and extract windows.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"

run_one() {
  local name="$1" duration="$2"
  local plan="$HERE/plans/${name}_20261110.json"
  local stage
  stage="$(python3 "$HERE/panel_stage.py" 2>/dev/null || echo x)"
  if [ "$stage" != "0" ]; then
    echo "reset panel before $name (stage=$stage)"
    python3 "$HERE/panel_reset.py" >/dev/null 2>&1 || true
    sleep 1
  fi
  echo "=== $name ==="
  bash "$HERE/run_plan.sh" "$plan" "$name" "$duration"
  if [ "$(pgrep -f 'ros2_control_node --ros-args' | wc -l)" != "1" ]; then
    echo "ABORT driver died after $name"
    exit 1
  fi
  python3 "$HERE/extract_windows.py" "$name" "$plan" > "$HERE/logs/${name}_extract.log" 2>&1 \
    || echo "WARN extract $name"
}

run_one H2_hyst2 92
run_one H4_hyst2 116
echo HYST_RETEST_DONE
