"""Diagnose GC06 feedback/target discrepancy. Offline, read-only."""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[name] for name in NAMES], dtype=float)


path = HERE / 'raw' / 'gc06_20261109.jsonl.gz'
refs, fbs, times = [], [], []
with gzip.open(path, 'rt', encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] != 'controller_state':
            continue
        try:
            refs.append(ordered(row['payload'], 'reference_positions'))
            fbs.append(ordered(row['payload'], 'feedback_positions'))
            times.append(row['t'])
        except (KeyError, TypeError, ValueError):
            continue
refs = np.degrees(np.array(refs))
fbs = np.degrees(np.array(fbs))
times = np.array(times)
print('records', len(times), 't', round(times[0], 2), '..', round(times[-1], 2))
for index, name in enumerate(NAMES):
    print(name,
          'ref range [%.3f, %.3f]' % (refs[:, index].min(), refs[:, index].max()),
          'fb range [%.3f, %.3f]' % (fbs[:, index].min(), fbs[:, index].max()))
print('\nreference samples around the expected J2 move:')
for k in range(0, len(times), max(1, len(times) // 40)):
    print('t=%7.2f' % times[k],
          'ref', ' '.join('%+8.3f' % v for v in refs[k]),
          ' fb', ' '.join('%+8.3f' % v for v in fbs[k]))
