"""Compare uncompensated vs compensated validation residuals against desired poses.

Residual = settled actual - original desired pose (not the compensated command).
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def residuals(tag, plan_path):
    plan = read(plan_path)
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / f'windows_{tag}.json')
    items = []
    for window in data['windows']:
        if 'sag_deg' not in window or window.get('samples', 0) < 150:
            continue
        index = window['index']
        want = desired[index]
        actual = np.array(window['mean_fb_deg'], dtype=float)
        items.append({'index': index, 'desired': want.tolist(),
                      'actual': actual.tolist(), 'residual': (actual - want).tolist()})
    return items


def summary(items):
    residuals = np.array([item['residual'] for item in items])
    active = np.array([np.max(np.abs(item['desired'])) > 0.5 for item in items])
    if not active.any():
        active = np.ones(len(items), dtype=bool)
    return {'per_joint_rms': np.sqrt(np.mean(residuals[active] ** 2, axis=0)).tolist(),
            'per_joint_max': np.max(np.abs(residuals[active]), axis=0).tolist(),
            'overall_rms': float(np.sqrt(np.mean(residuals[active] ** 2))),
            'overall_max': float(np.max(np.abs(residuals[active]))),
            'waypoints': int(active.sum()),
            'all_waypoint_max': np.max(np.abs(residuals), axis=0).tolist()}


if __name__ == '__main__':
    plan_path = sys.argv[1]
    plain = residuals(sys.argv[2], plan_path)
    compensated = residuals(sys.argv[3], plan_path)
    a, b = summary(plain), summary(compensated)
    print('uncompensated:', json.dumps(a, ensure_ascii=False))
    print('compensated:  ', json.dumps(b, ensure_ascii=False))
    print('per-joint |residual| max over active waypoints (deg):')
    for joint in range(5):
        reduction = 100.0 * (1.0 - b['per_joint_max'][joint] / a['per_joint_max'][joint]) \
            if a['per_joint_max'][joint] > 1e-9 else float('nan')
        print(' J%d  %.4f -> %.4f  (%.0f%% reduction)' % (
            joint + 1, a['per_joint_max'][joint], b['per_joint_max'][joint], reduction))
    print('overall RMS %.4f -> %.4f deg; overall max %.4f -> %.4f deg' % (
        a['overall_rms'], b['overall_rms'], a['overall_max'], b['overall_max']))
    out = {'schema': 'slaihand_static_sag_comparison/1',
           'plan': str(plan_path),
           'uncompensated': a, 'compensated': b,
           'uncompensated_windows': plain, 'compensated_windows': compensated}
    (HERE / 'sag_comparison_20261110.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
