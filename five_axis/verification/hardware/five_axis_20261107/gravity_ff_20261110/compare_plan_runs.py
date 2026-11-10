"""Compare one or more runs against the ORIGINAL (uncompensated) plan.

Used by the model-compensation one-click path, where no uncompensated baseline
run is performed: residual = actual feedback - original desired pose.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def stats_for(plan_name, tag):
    plan = read(HERE / 'plans' / ('%s_20261110.json' % plan_name))
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % tag))
    rows = []
    for window in data['windows']:
        if 'sag_deg' not in window or window.get('samples', 0) < 150:
            continue
        index = window['index']
        actual = np.array(window['mean_fb_deg'], dtype=float)
        rows.append({'index': index, 'desired': desired[index].tolist(),
                     'actual': actual.tolist(),
                     'residual': (actual - desired[index]).tolist()})
    active = [row for row in rows if np.max(np.abs(row['desired'])) > 0.5]
    residuals = np.array([row['residual'] for row in active])
    return {'tag': tag, 'waypoints': len(active),
            'per_joint_rms': np.sqrt(np.mean(residuals ** 2, axis=0)).round(6).tolist(),
            'per_joint_max': np.max(np.abs(residuals), axis=0).round(6).tolist(),
            'overall_rms': round(float(np.sqrt(np.mean(residuals ** 2))), 6),
            'overall_max': round(float(np.max(np.abs(residuals))), 6),
            'windows': rows}


def main():
    if len(sys.argv) < 4:
        print('USAGE: compare_plan_runs.py <original_plan_name> <out.json> <tag> [tag...]')
        return 2
    plan_name, out_path = sys.argv[1], Path(sys.argv[2])
    result = {'schema': 'slaihand_plan_run_comparison/1', 'plan': plan_name, 'runs': {}}
    for tag in sys.argv[3:]:
        try:
            result['runs'][tag] = stats_for(plan_name, tag)
        except (OSError, ValueError, KeyError) as error:
            print('%s: skip (%s)' % (tag, error))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8')
    for tag, stats in result['runs'].items():
        print('%s: waypoints=%d overall RMS %.4f max %.4f deg' % (
            tag, stats['waypoints'], stats['overall_rms'], stats['overall_max']))
    print('wrote', out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
