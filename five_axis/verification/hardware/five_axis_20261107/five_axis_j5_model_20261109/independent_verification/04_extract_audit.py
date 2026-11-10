"""Independent extraction from the 41 MB canusb audit file. Offline, read-only.

Streams the audit JSONL (never loads it whole), keeps only:
  * kind == SESSION (one record, run metadata incl. kp/kd/joint_policy), and
  * kind == CONTROL_CYCLE with command_id == 1 (the 2026-11-09 J5 excitation run).
Drops every other line. Writes a reduced gzip JSONL into independent_verification/.
"""
import gzip
import json
from collections import Counter
from pathlib import Path

SRC = '/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_qsrQke.jsonl'
OUT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
           'hardware/five_axis_20261107/five_axis_j5_model_20261109/'
           'independent_verification/audit_command1_cycles.jsonl.gz')

FIELDS = ('steady_s', 'cycle_id', 'command_id', 'control_state', 'target',
          'kp_scheduled', 'kp_scaled', 'kp_encoded', 'kp_output_state',
          'request_estimate_nm', 'request_estimate_state', 'request_estimate_at')

kinds = Counter()
n_session = 0
n_cc1 = 0
n_cc_other = 0
with open(SRC, 'r', encoding='utf-8', errors='replace') as src, \
        gzip.open(OUT, 'wt', encoding='utf-8') as dst:
    for line in src:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = row.get('kind')
        if kind == 'SESSION':
            kinds['SESSION'] += 1
            dst.write(json.dumps(row, ensure_ascii=False) + '\n')
            n_session += 1
        elif kind == 'CONTROL_CYCLE' and row.get('command_id') == 1:
            kinds['CONTROL_CYCLE:1'] += 1
            keep = {k: row.get(k) for k in FIELDS}
            dst.write(json.dumps(keep, ensure_ascii=False) + '\n')
            n_cc1 += 1
        elif kind == 'CONTROL_CYCLE':
            kinds['CONTROL_CYCLE:other'] += 1
            n_cc_other += 1
print('kept SESSION:', n_session, 'CONTROL_CYCLE command_id=1:', n_cc1,
      'dropped CONTROL_CYCLE other:', n_cc_other)
print('written', OUT, OUT.stat().st_size, 'bytes')
