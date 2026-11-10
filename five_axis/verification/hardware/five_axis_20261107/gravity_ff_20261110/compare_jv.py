"""Compare JV baseline vs a compensated JV run (tag optional) against desired JV poses."""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def residuals(tag):
    plan = read(HERE / 'plans' / 'JV_20261110.json')
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / f'windows_{tag}.json')
    rows = []
    for window in data['windows']:
        if 'sag_deg' not in window or window.get('samples', 0) < 150:
            continue
        index = window['index']
        actual = np.array(window['mean_fb_deg'], dtype=float)
        rows.append({'index': index, 'desired': desired[index].tolist(),
                     'actual': actual.tolist(),
                     'residual': (actual - desired[index]).tolist()})
    return rows


def summary(rows):
    active = [row for row in rows
              if np.max(np.abs(row['desired'])) > 0.5 or abs(row['desired'][4]) > 0.5]
    residuals = np.array([row['residual'] for row in active])
    return {'per_joint_rms': np.sqrt(np.mean(residuals ** 2, axis=0)).tolist(),
            'per_joint_max': np.max(np.abs(residuals), axis=0).tolist(),
            'overall_rms': float(np.sqrt(np.mean(residuals ** 2))),
            'overall_max': float(np.max(np.abs(residuals))),
            'waypoints': len(active)}


if __name__ == '__main__':
    compensated_tag = sys.argv[1] if len(sys.argv) > 1 else 'JVc'
    base_rows = residuals('JV')
    comp_rows = residuals(compensated_tag)
    a, b = summary(base_rows), summary(comp_rows)
    print('baseline (JV)   :', json.dumps(a, ensure_ascii=False))
    print('compensated (%s):' % compensated_tag, json.dumps(b, ensure_ascii=False))
    print('per-joint |residual| max (deg):')
    for joint in range(5):
        before = a['per_joint_max'][joint]
        after = b['per_joint_max'][joint]
        reduction = 100.0 * (1.0 - after / before) if before > 1e-9 else float('nan')
        print(' J%d  %.4f -> %.4f  (%.0f%% reduction)' % (joint + 1, before, after, reduction))
    print('overall RMS %.4f -> %.4f deg; max %.4f -> %.4f deg' % (
        a['overall_rms'], b['overall_rms'], a['overall_max'], b['overall_max']))
    out = {'schema': 'slaihand_j5_compensation_result/1',
           'baseline': a, 'compensated': b,
           'baseline_windows': base_rows, 'compensated_windows': comp_rows}
    (HERE / 'j5_compensation_result_20261110.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote j5_compensation_result_20261110.json')
