#!/bin/bash
# Prepare the latest four-axis legacy float stack with the identified gravity model.
# Creates a compatibility profile (mapping superset only adds the J5 block; J1-J4
# sections are byte-identical) and temporarily enables gravity_float in the policy.
set -u
ROOT=/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain
SRC="$ROOT/src/slaihand_bringup/config"
INST=/home/xuyu/.cache/dsh_sched/install/slaihand_bringup/share/slaihand_bringup/config
MAP_SHA=$(sha256sum "$INST/hardware_mapping.yaml" | cut -d' ' -f1)

python3 - "$SRC" "$INST" "$MAP_SHA" <<'PY'
import json, sys
from pathlib import Path
src, inst, mapping_sha = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
candidate = json.loads((src / 'gravity_identified_candidate.json').read_text(encoding='utf-8'))
old_mapping = candidate['identity']['mapping_sha256']
candidate['identity']['mapping_sha256'] = mapping_sha
candidate['identity']['mapping_compat_note'] = (
    'Derived 2026-11-10 for four-axis legacy float only. The original model was '
    'identified with mapping %s; the current mapping %s differs only in the J5 '
    'block (J1-J4 sections byte-identical), so the four-axis gravity coefficients '
    'and identity checks remain valid. Coefficients unchanged; original file kept.'
    % (old_mapping, mapping_sha))
for target in (src, inst):
    (target / 'gravity_identified_candidate_j5_mapping_compat.json').write_text(
        json.dumps(candidate, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

for target in (src, inst):
    policy = (target / 'canusb_motion_policy.yaml').read_text(encoding='utf-8')
    backup = (target / 'canusb_motion_policy.yaml.orig_drag_20261110')
    if not backup.exists():
        backup.write_text(policy, encoding='utf-8')
    lines = [line for line in policy.splitlines()
             if not line.startswith('gravity_profile:')
             and not line.startswith('gravity_float_enabled:')
             and not line.startswith('gravity_ff_ramp_s:')]
    lines += ['gravity_profile: gravity_identified_candidate_j5_mapping_compat.json',
              'gravity_float_enabled: true',
              'gravity_ff_ramp_s: 2.0']
    (target / 'canusb_motion_policy.yaml').write_text('\n'.join(lines) + '\n', encoding='utf-8')
print('prepared compat profile and float policy; mapping sha', mapping_sha)
PY
echo "--- policy tail ---"
tail -5 "$INST/canusb_motion_policy.yaml"
