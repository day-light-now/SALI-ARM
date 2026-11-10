"""Time-series analysis of J1-J4 following error (sag) and effort over the whole run.

READ-ONLY. Following error = feedback_positions - reference_positions from the
controller_state stream (same recorder clock); effort from the dynamic stream.
Outputs time series JSON + prints 0.5 s buckets over the full plan.
"""
import gzip
import json
from pathlib import Path

import numpy as np

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
RAW = ROOT / ('verification/hardware/five_axis_20261107/acquisition_20261109/raw/'
              'j5_excitation_20261109.jsonl.gz')
HERE = ROOT / ('verification/hardware/five_axis_20261107/five_axis_j5_model_20261109/'
               'independent_verification')
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]

cs, dyn = [], []
with gzip.open(RAW, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'controller_state':
            cs.append(r)
        elif r['kind'] == 'dynamic':
            dyn.append(r)


def arr(payload, field):
    v = dict(zip(payload['joint_names'], payload[field]))
    return np.array([v[n] for n in NAMES], float)


def dyn_arr(payload, field):
    v = payload['interfaces']
    return np.array([v[n][field] for n in NAMES], float)


tc = np.array([r['t'] for r in cs])
ref = np.array([arr(r['payload'], 'reference_positions') for r in cs])
fb = np.array([arr(r['payload'], 'feedback_positions') for r in cs])
err_deg = np.degrees(fb - ref)
td = np.array([r['t'] for r in dyn])
pos_d = np.array([dyn_arr(r['payload'], 'position') for r in dyn])
eff_d = np.array([dyn_arr(r['payload'], 'effort') for r in dyn])
motor_ok = np.array([[r['payload']['interfaces'][n]['motor_state'] == 1
                      and r['payload']['interfaces'][n]['control_state'] in (5, 6)
                      for n in NAMES] for r in dyn]).all(axis=1)

print('=== J1-J5 following error (deg) and effort (Nm), 0.5 s buckets ===')
print('   t     ref5(deg)  ' + ' '.join(f'err{i}' for i in range(1, 6)) +
      ' | ' + ' '.join(f'eff{i}' for i in range(1, 6)))
rows = []
for start in np.arange(7.8, 87.6, 0.5):
    sel = (tc >= start) & (tc < start + 0.5)
    sd = (td >= start) & (td < start + 0.5)
    if not sel.any():
        continue
    e = err_deg[sel].mean(axis=0)
    r5 = np.degrees(ref[sel, 4]).mean()
    ef = eff_d[sd].mean(axis=0) if sd.any() else np.full(5, np.nan)
    rows.append({'t': float(start), 'ref5_deg': float(r5), 'err_deg': e.tolist(),
                 'effort_nm': ef.tolist(), 'motor_ok': bool(motor_ok[sd].mean() > 0.9) if sd.any() else False})

prev = None
for row in rows:
    flag = ''
    if prev is not None:
        d = np.abs(np.array(row['err_deg'][:4]) - np.array(prev['err_deg'][:4]))
        if d.max() > 0.05:
            flag = '  <== J1-J4 sag step %.3f deg' % d.max()
    print(f"{row['t']:6.2f} {row['ref5_deg']:+9.3f}  " +
          ' '.join(f'{v:+8.3f}' for v in row['err_deg']) + ' | ' +
          ' '.join(f'{v:+8.4f}' for v in row['effort_nm']) + flag)
    prev = row

(HERE / 'time_series.json').write_text(json.dumps(rows, indent=2) + '\n', encoding='utf-8')
print('\nwritten', HERE / 'time_series.json')
