#!/bin/bash
# GC09+ step 2: build a truthful admission artifact with the FIXED builder
# (deployed copy in the install cache), then validate it against the same files.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/env.sh"
INST=/home/xuyu/.cache/dsh_sched/install
ARTIFACT="$HERE/artifacts/j5_motion_calibration_artifact_20261109_session4.json"
EVIDENCE="$HERE/evidence/j5_timeout_write_once_20261109.json"
MAPPING="$INST/slaihand_bringup/share/slaihand_bringup/config/hardware_mapping.yaml"
POLICY="$INST/slaihand_bringup/share/slaihand_bringup/config/canusb_five_axis_motion_policy.yaml"
LOAD="$INST/slaihand_bringup/share/slaihand_bringup/config/load_3p5kg_provisional.yaml"
MODEL="$INST/slaihand_description/share/slaihand_description/urdf/slaihand_five_axis.urdf"
python3 "$INST/slaihand_bringup/lib/slaihand_bringup/j5_motion_artifact.py" build \
  --mapping "$MAPPING" --model "$MODEL" --policy "$POLICY" --load "$LOAD" \
  --evidence "$EVIDENCE" --device /dev/ttyACM0 --output "$ARTIFACT" || exit 1
python3 "$INST/slaihand_bringup/lib/slaihand_bringup/j5_motion_artifact.py" validate \
  "$ARTIFACT" \
  --mapping "$MAPPING" --model "$MODEL" --policy "$POLICY" --load "$LOAD" \
  --evidence "$EVIDENCE" --device /dev/ttyACM0 || exit 1
python3 - "$ARTIFACT" <<'PY'
import json, sys
payload = json.load(open(sys.argv[1], encoding='utf-8'))['payload']
timeout = payload['timeout']
print('artifact_timeout_block', json.dumps(timeout, ensure_ascii=False, sort_keys=True))
assert timeout['write_outcome'] in ('complete', 'verified_existing')
if timeout['write_outcome'] == 'verified_existing':
    assert timeout['timeout_before'] == 2000 and timeout['write_attempts'] == 0
    assert timeout['parameter_writes'] == 0
print('truthful_metadata=OK')
PY
echo "artifact=$ARTIFACT"
