"""Prepare J1 direction-dependent stiction plans (3 angles, two approach sides).

Each plan visits the same J1 target twice: once from the + side and once from the
- side, so residual differences isolate direction-dependent hysteresis. No driver
is started here; plans are validated by validate_plan.py.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'
OUT = HERE / 'j1_hysteresis_plans_20261111.json'


def write_plan(points, path):
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': [{'degrees': [float(v) for v in row],
                                               'move_s': 6.0, 'dwell_s': 6.0}
                                              for row in points]},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def validate(path):
    result = subprocess.run([sys.executable, str(HERE / 'validate_plan.py'), str(path)],
                            capture_output=True, text=True)
    duration = subprocess.run([sys.executable, str(HERE / 'plan_duration.py'), str(path)],
                              capture_output=True, text=True)
    return {'ok': result.returncode == 0,
            'message': (result.stdout + result.stderr).strip(),
            'recorder_s': int(duration.stdout.strip()) if duration.returncode == 0 else None}


def main():
    zero = np.zeros(5)
    entries = []
    for target in (-8.0, 0.0, 8.0):
        p = zero.copy()
        p[0] = target
        p[1] = 5.0
        delta = zero.copy()
        delta[0] = 2.0
        points = np.vstack([zero, p + delta, p, p - delta, p, zero])
        name = 'J1H_%s' % ('m8' if target < 0 else ('0' if target == 0 else 'p8'))
        path = PLANS / ('%s_20261111.json' % name)
        write_plan(points, path)
        check = validate(path)
        entries.append({'name': name, 'plan': str(path), 'target_deg': target,
                        'waypoints': len(points), **check})
        print('%-8s target J1=%+5.1f wps=%d rec=%ss %s' % (
            name, target, len(points), check['recorder_s'], check['message']))
    report = {'schema': 'slaihand_j1_hysteresis_plans/1', 'entries': entries,
              'all_valid': all(e['ok'] for e in entries)}
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('all_valid', report['all_valid'])
    print('wrote', OUT)


if __name__ == '__main__':
    main()
