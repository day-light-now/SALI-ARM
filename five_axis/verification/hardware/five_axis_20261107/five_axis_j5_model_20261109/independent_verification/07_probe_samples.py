"""Sample-level probe of effort vs (target-q) and velocity in a static hold.

READ-ONLY. Prints a compact table so the effort semantics can be judged from data.
"""
import gzip
import json

import numpy as np

RAW = ('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
       'hardware/five_axis_20261107/acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz')
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]

dyn = {}
t = []
with gzip.open(RAW, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] != 'dynamic':
            continue
        t.append(r['t'])
        dyn[r['t']] = r['payload']['interfaces']
t = np.array(t)
cs_rows = []
with gzip.open(RAW, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] != 'controller_state':
            continue
        cs_rows.append(r)
tc = np.array([r['t'] for r in cs_rows])
ref = {}
for r in cs_rows:
    p = r['payload']
    ref[r['t']] = np.array([p['reference_positions'][p['joint_names'].index(n)] for n in NAMES])

# target from plan: last 2 s of w0 = t in [11.34, 13.33]
print('t        | ' + ' | '.join(f'{n[-1]}: err(deg) v(mrad/s) eff(Nm)' for n in NAMES))
for k in np.flatnonzero((t > 11.4) & (t < 12.4)):
    dt = t[k]
    j = int(np.argmin(np.abs(tc - dt)))
    a = dyn[dt]
    s = f'{dt:8.3f} |'
    for i, n in enumerate(NAMES):
        err = np.degrees(0.0 - a[n]['position'])
        s += f' {err:+8.4f} {a[n]["velocity"]*1000:+8.2f} {a[n]["effort"]:+8.4f} |'
    print(s)

print()
# correlation of effort with kp*err and with -kd*v inside w0.window
sel = np.flatnonzero((t > 11.4) & (t < 13.3))
for i, n in enumerate(NAMES):
    e = np.array([dyn[tt][n]['effort'] for tt in t[sel]])
    v = np.array([dyn[tt][n]['velocity'] for tt in t[sel]])
    q = np.array([dyn[tt][n]['position'] for tt in t[sel]])
    print(f'{n}: eff mean {e.mean():+.5f} std {e.std():.5f}; '
          f'v mean {v.mean():+.5f} std {v.std():.5f}; q std_deg {np.degrees(q.std()):.5f}')
    # fit eff = a + b*(-v) and eff = a + c*(-q)
    A = np.column_stack([np.ones_like(v), -v])
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    pred = A @ coef
    print(f'   eff = {coef[0]:+.5f} + {coef[1]:.3f}*(-v),  residual std {np.std(e - pred):.5f}')
    A = np.column_stack([np.ones_like(q), -q])
    coef, *_ = np.linalg.lstsq(A, e, rcond=None)
    pred = A @ coef
    print(f'   eff = {coef[0]:+.5f} + {coef[1]:.3f}*(-q),  residual std {np.std(e - pred):.5f}')
