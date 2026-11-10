#!/bin/bash
# GC09+ step 3: five-axis candidate launch with the fresh admission chain.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
ART="$HERE/artifacts/j5_motion_calibration_artifact_20261109_session4.json"
EVI="$HERE/evidence/j5_timeout_write_once_20261109.json"
ulimit -c unlimited 2>/dev/null || true
exec ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate.launch.py \
  port:=/dev/ttyACM0 \
  j5_artifact:="$ART" \
  timeout_evidence:="$EVI"
