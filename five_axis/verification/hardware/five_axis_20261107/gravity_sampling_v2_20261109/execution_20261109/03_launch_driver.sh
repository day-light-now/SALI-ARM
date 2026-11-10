#!/bin/bash
# Step 3: five-axis candidate launch with the fresh readback-only admission chain.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
ART="$HERE/artifacts/j5_motion_calibration_artifact_20261109_session2.json"
EVI="$HERE/evidence/j5_timeout_verify_existing_20261109_session2.json"
exec ros2 launch slaihand_bringup slaihand_canusb_five_axis_candidate.launch.py \
  port:=/dev/ttyACM0 \
  j5_artifact:="$ART" \
  timeout_evidence:="$EVI"
