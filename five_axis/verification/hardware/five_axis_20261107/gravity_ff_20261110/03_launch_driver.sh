#!/bin/bash
# Launch the five-axis candidate driver with today's fresh admission chain.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
ART="$HERE/artifacts/j5_motion_calibration_artifact_20261110.json"
EVI="$HERE/evidence/current_timeout_evidence.json"
ulimit -c unlimited 2>/dev/null || true
exec ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate.launch.py \
  port:=/dev/ttyACM0 \
  j5_artifact:="$ART" \
  timeout_evidence:="$EVI"
