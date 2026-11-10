"""Second iterative correction: c2 = c1 + (desired - actual_after_c1).

Usage: make_iter2.py <desired_plan_name> <prev_plan_name> <prev_tag> <out_name>
Plan files are plans/<name>_20261110.json; the measured windows are
windows_<prev_tag>.json.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    desired_name = sys.argv[1] if len(sys.argv) > 1 else 'JZ'
    prev_name = sys.argv[2] if len(sys.argv) > 2 else 'JZc'
    prev_tag = sys.argv[3] if len(sys.argv) > 3 else 'JZc'
    out_name = sys.argv[4] if len(sys.argv) > 4 else 'JZc2'
    desired = np.array([w['degrees'] for w in
                        read(HERE / 'plans' / f'{desired_name}_20261110.json')['waypoints']],
                       dtype=float)
    c1 = np.array([w['degrees'] for w in
                   read(HERE / 'plans' / f'{prev_name}_20261110.json')['waypoints']],
                  dtype=float)
    data = read(HERE / f'windows_{prev_tag}.json')
    c2 = c1.copy()
    for window in data['windows']:
        if 'sag_deg' not in window or window.get('samples', 0) < 150:
            continue
        index = window['index']
        actual = np.array(window['mean_fb_deg'], dtype=float)
        c2[index] = c1[index] + (desired[index] - actual)
    c2[0] = desired[0]
    c2[-1] = desired[-1]
    waypoints = [{'degrees': [float(v) for v in row], 'move_s': 6.0, 'dwell_s': 6.0}
                 for row in c2]
    path = HERE / 'plans' / f'{out_name}_20261110.json'
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote', path)
    for index in range(len(desired)):
        print(' w%d desired=%s c1=%s c2=%s' % (
            index, np.round(desired[index], 2).tolist(),
            np.round(c1[index], 3).tolist(), np.round(c2[index], 3).tolist()))


if __name__ == '__main__':
    main()
