"""J5 coupling analysis: upstream effort vs q5 at tilted base poses.

If the 3.5 kg box rotates with J5 and its COM is off-axis, upstream effort must
vary sinusoidally with q5; amplitude gives the payload's off-axis gravity
sensitivity. If amplitude is below the noise/code level, the J5 gravity term is
negligible (box COM near axis).
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
tag = sys.argv[1] if len(sys.argv) > 1 else 'J5COUP'
plan = json.loads((HERE / 'plans' / 'J5COUP_20261110.json').read_text(encoding='utf-8'))
desired = [w['degrees'] for w in plan['waypoints']]
data = json.loads((HERE / f'windows_{tag}.json').read_text(encoding='utf-8'))
rows = [w for w in data['windows']
        if w.get('effort_mean_nm') is not None and w.get('samples', 0) >= 150]

groups = defaultdict(list)
for window in rows:
    base = tuple(np.round(desired[window['index']][:4], 2).tolist())
    groups[base].append(window)

NAME = ['J1', 'J2', 'J3', 'J4', 'J5']
CODE = [0.0977, 0.0977, 0.0264, 0.0264, 0.0137]
result = {'schema': 'slaihand_j5_coupling/1', 'tag': tag, 'groups': {}}
print('J5 coupling: sinusoid fit of effort vs q5 (Nm), per base pose')
print('expected: 3.5 kg, 1 mm off-axis <=> 0.034 Nm peak (1 g lever);'
      ' box COM near J5 axis => amplitude < ~0.1 Nm')
for base, windows in groups.items():
    q5 = np.array([w['target_deg'][4] for w in windows])
    if len(q5) < 3 or np.ptp(q5) < 5:
        continue
    print('\nbase J1..J4 =', list(base), 'q5 =', np.round(q5, 1).tolist())
    entry = {'base_deg': list(base), 'q5_deg': q5.tolist(), 'joints': {}}
    for joint in range(5):
        y = np.array([w['effort_mean_nm'][joint] for w in windows])
        A = np.c_[np.sin(np.radians(q5)), np.cos(np.radians(q5)), np.ones(len(q5))]
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        residual = y - A @ coef
        amplitude = float(np.hypot(coef[0], coef[1]))
        entry['joints'][NAME[joint]] = {
            'amplitude_nm': amplitude,
            'offset_nm': float(coef[2]),
            'residual_rms_nm': float(np.sqrt(np.mean(residual ** 2))),
            'range_nm': float(np.ptp(y)),
            'code_nm': CODE[joint],
            'effort_nm': np.round(y, 4).tolist(),
            'phase_deg': float(np.degrees(np.arctan2(coef[1], coef[0]))),
        }
        print(' %s amp=%.4f off=%.4f res=%.4f range=%.4f code=%.4f' % (
            NAME[joint], amplitude, coef[2],
            entry['joints'][NAME[joint]]['residual_rms_nm'], np.ptp(y), CODE[joint]))
    result['groups'][str(list(base))] = entry

# aggregate evidence for J5 gravity term
upstream = [result['groups'][g]['joints'][name]['amplitude_nm']
            for g in result['groups'] for name in ('J1', 'J2', 'J3', 'J4')]
if upstream:
    result['max_upstream_amplitude_nm'] = max(upstream)
    result['j5_gravity_term_negligible'] = bool(max(upstream) < 0.15)
    print('\nmax upstream amplitude: %.4f Nm -> J5 gravity term %s' % (
        max(upstream), 'NEGLIGIBLE (<0.15 Nm)' if max(upstream) < 0.15 else 'NONZERO, needs fit'))
(HERE / f'j5_coupling_{tag}.json').write_text(
    json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('wrote', HERE / f'j5_coupling_{tag}.json')
