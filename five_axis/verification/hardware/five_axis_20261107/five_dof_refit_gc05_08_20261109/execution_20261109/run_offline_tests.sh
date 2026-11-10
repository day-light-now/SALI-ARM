#!/bin/bash
# Run the remaining offline/PTY test suites used by the follow-up plan. No hardware.
set -u
cd "/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain"
run() {
  echo "--- $1 ($2)"
  python3 -B -m unittest discover -s "$1" -p "$2" 2>&1 | tail -2
}
run verification/hardware/five_axis_20261107 test_j5_timeout_setup.py
run verification/hardware/five_axis_20261107 test_j5_first_motion_plan.py
run verification/hardware/five_axis_20261107/five_dof_identification_20261109 test_five_dof.py
run verification/hardware/five_axis_20261107/identification_codex_20261109 test_identification.py
run verification/hardware/five_axis_20261107/five_axis_j5_model_20261109 test_j5_analysis.py
