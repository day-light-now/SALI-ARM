"""Estimate stiction/hysteresis from the H4 repeatability run.

H4 visits the same pose three times, approached from different directions.
At the same pose, gravity cancels and a direction-dependent residual difference
is approximately 2 * friction. This script reports per-joint residual spread
(max-min across the same-pose visits) and the implied friction amplitude.
"""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TAG = 'H4_stiction'
PLAN = HERE / 'plans' / f'{TAG}_20261110.json'
WINDOWS = HERE / f'windows_{TAG}.json'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    if not WINDOWS.is_file():
        print('missing', WINDOWS, '- run H4 first')
        return 1
    plan = read(PLAN)
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(WINDOWS)
    same_pose_indices = []
    for window in sorted(data['windows'], key=lambda w: w['index']):
        index = window['index']
        if index in (0, len(desired) - 1):
            continue
        if 'sag_deg' in window and window.get('samples', 0) >= 150:
            if np.max(np.abs(desired[index] - desired[2])) < 0.05:
                same_pose_indices.append(index)
    rows = []
    for window in data['windows']:
        if window['index'] in same_pose_indices:
            rows.append(np.array(window['mean_fb_deg'], dtype=float))
    if len(rows) < 2:
        print('not enough same-pose windows:', same_pose_indices)
        return 1
    matrix = np.array(rows)
    spread = np.max(matrix, axis=0) - np.min(matrix, axis=0)
    friction = spread / 2.0
    report = {'schema': 'slaihand_stiction_estimate/1',
              'tag': TAG, 'indices': same_pose_indices,
              'same_pose_window_count': len(rows),
              'residual_spread_deg': np.round(spread, 4).tolist(),
              'implied_friction_nm_equivalent_deg': np.round(friction, 4).tolist(),
              'note': 'residual spread at one pose; half-spread approximates the '
                      'direction-dependent friction bias (degrees of joint error).'}
    (HERE / 'stiction_H4_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('same-pose windows', len(rows), 'indices', same_pose_indices)
    print('spread deg', report['residual_spread_deg'])
    print('implied friction (deg-equivalent)', report['implied_friction_nm_equivalent_deg'])
    print('wrote stiction_H4_20261110.json')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
