#!/bin/bash
# Step 2: build the J5 admission artifact from the fresh readback-only evidence.
# Uses the installed package files, which are byte-identical to the worktree src
# files and are what the candidate launch validates against.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
INST=/home/xuyu/.cache/dsh_sched/install
EVIDENCE="$HERE/evidence/j5_timeout_verify_existing_20261109_session2.json"
ARTIFACT="$HERE/artifacts/j5_motion_calibration_artifact_20261109_session2.json"
python3 src/slaihand_bringup/scripts/j5_motion_artifact.py build \
  --mapping "$INST/slaihand_bringup/share/slaihand_bringup/config/hardware_mapping.yaml" \
  --model "$INST/slaihand_description/share/slaihand_description/urdf/slaihand_five_axis.urdf" \
  --policy "$INST/slaihand_bringup/share/slaihand_bringup/config/canusb_five_axis_motion_policy.yaml" \
  --load "$INST/slaihand_bringup/share/slaihand_bringup/config/load_3p5kg_provisional.yaml" \
  --evidence "$EVIDENCE" \
  --device /dev/ttyACM0 \
  --output "$ARTIFACT"
CODE=$?
echo "artifact_build_exit=$CODE artifact=$ARTIFACT"
exit $CODE
