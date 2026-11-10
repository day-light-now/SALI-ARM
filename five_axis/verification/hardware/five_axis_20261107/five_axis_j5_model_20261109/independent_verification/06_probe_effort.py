"""Probe: effort semantics, quantization steps, and gain relation at sample level.

READ-ONLY on data. Uses only in-memory streaming of the recorder file.
"""
import gzip
import json
from pathlib import Path

import numpy as np

RAW = ('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
       'hardware/five_axis_20261107/acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz')
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]

rows = []
with gzip.open(RAW, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] != 'dynamic':
            continue
        a = r['payload']['interfaces']
        rows.append((r['t'],
                     [a[n]['position'] for n in NAMES],
                     [a[n]['velocity'] for n in NAMES],
                     [a[n]['effort'] for n in NAMES],
                     [a[n]['motor_state'] for n in NAMES],
                     [a[n]['control_state'] for n in NAMES]))
t = np.array([r[0] for r in rows])
pos = np.array([r[1] for r in rows])
vel = np.array([r[2] for r in rows])
eff = np.array([r[3] for r in rows])
motor = np.array([r[4] for r in rows])
cs = np.array([r[5] for r in rows])

hold = ((motor == 1) & ((cs == 5) | (cs == 6))).all(axis=1) & (t > 11.4) & (t < 13.3)
print('hold w0 samples', hold.sum())
print('w0 mean pos deg', np.degrees(pos[hold].mean(axis=0)))
print('w0 mean vel', vel[hold].mean(axis=0))
print('w0 std vel', vel[hold].std(axis=0))
print('w0 mean effort', eff[hold].mean(axis=0))
print('w0 std effort', eff[hold].std(axis=0))
print()

# quantization: smallest positive step between sorted distinct values over whole run
for i, n in enumerate(NAMES):
    vals = np.unique(eff[:, i])
    diffs = np.diff(vals)
    diffs = diffs[diffs > 1e-12]
    print(f'{n}: n_unique_effort={len(vals)} min_step={diffs.min() if len(diffs) else None:.10g} '
          f'median_step={np.median(diffs) if len(diffs) else None:.10g}')
    vals = np.unique(vel[:, i])
    diffs = np.diff(vals)
    diffs = diffs[diffs > 1e-12]
    print(f'    n_unique_vel={len(vals)} min_vel_step={diffs.min() if len(diffs) else None:.10g}')
    vals = np.unique(pos[:, i])
    diffs = np.diff(vals)
    diffs = diffs[diffs > 1e-12]
    print(f'    n_unique_pos={len(vals)} min_pos_step={diffs.min() if len(diffs) else None:.10g}')

print()
print('first 5 w0 samples: axis1..5 pos/vel/eff')
sel = np.flatnonzero(hold)[:5]
for k in sel:
    print('t=', round(t[k], 4))
    for i, n in enumerate(NAMES):
        print(f'   {n}: pos={pos[k,i]:.8f} vel={vel[k,i]:+.8f} eff={eff[k,i]:+.8f} '
              f'kd*v={5*vel[k,i]:+.8f}')
