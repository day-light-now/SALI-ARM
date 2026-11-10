"""Build an iteratively corrected plan from the measured residuals of a baseline run.

commands = desired - measured residual (per waypoint, all five joints);
first/last waypoints stay at their original (absolute zero) values.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    tag = sys.argv[1]
    plan_name = sys.argv[2]
    out_name = sys.argv[3]
    plan = read(HERE / 'plans' / f'{plan_name}_20261110.json')
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / f'windows_{tag}.json')
    windows = {w['index']: w for w in data['windows'] if 'sag_deg' in w}
    commands = desired.copy()
    corrections = np.zeros_like(desired)
    for index, window in windows.items():
        residual = np.array(window['mean_fb_deg'], dtype=float) - desired[index]
        corrections[index, :] = -residual
        commands[index, :] = desired[index] - residual
    commands[0] = desired[0]
    commands[-1] = desired[-1]
    waypoints = [{'degrees': [float(v) for v in row], 'move_s': 6.0, 'dwell_s': 6.0}
                 for row in commands]
    path = HERE / 'plans' / f'{out_name}_20261110.json'
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote', path)
    for index in range(len(desired)):
        print(' w%d desired=%s residual=%s cmd=%s' % (
            index, np.round(desired[index], 2).tolist(),
            np.round(-corrections[index], 3).tolist(),
            np.round(commands[index], 3).tolist()))


if __name__ == '__main__':
    main()
