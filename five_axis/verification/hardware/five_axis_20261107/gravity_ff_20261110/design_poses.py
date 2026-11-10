"""Design the next static acquisition poses to improve gravity identifiability.

Uses the 16-parameter rigid-body gravity regressor Y16(q) and the existing
159 windows. Rows live in an 8-dimensional identifiable subspace; we greedily
select new paired poses that maximize logdet information in that subspace,
then emit plans (zero endpoints, <=12 waypoints, 6 s move + 6 s dwell).

Offline only; no hardware motion here.
"""
import json
import sys
from itertools import product
from pathlib import Path

import numpy as np

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
HERE = ROOT / 'verification/hardware/five_axis_20261107/gravity_ff_20261110'
BASE = ROOT / 'verification/hardware/five_axis_20261107'
WINDOWS = BASE / 'refit_gc09_12_20261109/static_windows.json'
PLANS = HERE / 'plans'
sys.path.insert(0, str(BASE / 'five_axis_j5_model_20261109'))
from five_axis_model_j5 import Kinematics  # noqa: E402


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_plan(points, path):
    waypoints = [{'degrees': [float(v) for v in point], 'move_s': 6.0, 'dwell_s': 6.0}
                 for point in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints}, ensure_ascii=False, indent=2) + '\n',
                    encoding='utf-8')


def main():
    kin = Kinematics()
    poses = read(WINDOWS)['poses']
    existing = np.stack([kin.prior_regressor(np.array(p['q_rad'])[:4]) for p in poses])
    flat = existing.reshape(-1, 16)
    u, s, vt = np.linalg.svd(flat, full_matrices=False)
    rank = int(np.sum(s > 1e-9))
    basis = vt[:rank].T  # 16 x rank
    print('existing rows', flat.shape[0], 'rank', rank, 'singular', np.round(s[:rank], 3).tolist())
    print('condition', round(float(s[0] / s[rank - 1]), 2))

    # candidate grid inside joint limits
    grid = list(product([-20, -10, 0, 10, 20], [-5, 0, 10, 20, 30],
                        [-20, -10, 0, 10, 20], [-20, -10, 0, 10, 20]))
    candidates = []
    for j1, j2, j3, j4 in grid:
        q = np.array([j1, j2, j3, j4, 0.0])
        rows = kin.prior_regressor(np.radians(q))
        candidates.append({'degrees': [j1, j2, j3, j4, 0.0],
                           'u': rows @ basis})  # 4 x rank
    print('candidate poses', len(candidates))

    info = np.zeros((rank, rank))
    for row in flat:
        urow = (basis.T @ row).reshape(-1)
        info += np.outer(urow, urow)
    ridge = 1e-3 * np.trace(info) / rank
    info0 = info + ridge * np.eye(rank)
    selected = []
    current = info0.copy()
    for _ in range(10):
        best, best_score = None, -np.inf
        for candidate in candidates:
            if candidate in selected:
                continue
            add = 2.0 * sum(np.outer(v, v) for v in candidate['u'])  # up+down pairing
            score = np.linalg.slogdet(current + add)[1]
            if selected:
                spread = min(max(abs(candidate['degrees'][i] - other['degrees'][i])
                                 for i in range(4)) for other in selected)
            else:
                spread = 50.0
            score += 0.02 * spread
            if score > best_score:
                best, best_score = candidate, score
        selected.append(best)
        add = 2.0 * sum(np.outer(v, v) for v in best['u'])
        current = current + add
    training = [candidate['degrees'] for candidate in selected]

    # validation poses: maximize minimum distance to training and each other
    def distance(a, b):
        return max(abs(a[i] - b[i]) for i in range(4))

    validation = []
    remaining = [c for c in candidates if c['degrees'] not in training]
    while len(validation) < 8 and remaining:
        best, best_score = None, -np.inf
        for candidate in remaining:
            d_train = min(distance(candidate['degrees'], t) for t in training)
            d_self = min([distance(candidate['degrees'], v['degrees']) for v in validation],
                         default=999)
            score = min(d_train, d_self)
            if score > best_score:
                best, best_score = candidate, score
        validation.append(best)
        remaining.remove(best)
    validation_degrees = [v['degrees'] for v in validation]
    print('training poses', training)
    print('validation poses', validation_degrees)

    # finite-sample information for new poses: two visits each
    info_new = info0.copy()
    for candidate in selected + validation:
        for v in candidate['u']:
            info_new += np.outer(v, v)
    # validation-set information from the new validation poses (paired approach)
    info_val = info0.copy()
    for candidate in selected:
        for v in candidate['u']:
            info_val += np.outer(v, v)
    info_val_final = info_val.copy()
    for candidate in validation:
        for v in candidate['u']:
            info_val_final += np.outer(v, v)

    sigma = 0.65

    def pred_sd(info_matrix, pose_rows):
        cov = sigma ** 2 * np.linalg.pinv(info_matrix)
        result = []
        for rows in pose_rows:
            result.append([float(np.sqrt(sigma ** 2 + row @ cov @ row)) for row in rows])
        return result

    before = pred_sd(info0, [v['u'] for v in validation])
    after = pred_sd(info_val_final, [v['u'] for v in validation])
    before_mean = float(np.mean([row for pose in before for row in pose]))
    after_mean = float(np.mean([row for pose in after for row in pose]))
    print('predicted validation RMSE (rough): before %.3f -> after %.3f Nm' % (
        before_mean, after_mean))

    plan_forward = [[0, 0, 0, 0, 0]] + training + [[0, 0, 0, 0, 0]]
    plan_reverse = [[0, 0, 0, 0, 0]] + list(reversed(training)) + [[0, 0, 0, 0, 0]]
    plan_valid = [[0, 0, 0, 0, 0]] + validation_degrees + [[0, 0, 0, 0, 0]]
    for name, points in (('T2_sag_20261110.json', plan_forward),
                         ('T2r_sag_20261110.json', plan_reverse),
                         ('V2_sag_20261110.json', plan_valid)):
        write_plan(points, PLANS / name)
        print('wrote', PLANS / name, len(points), 'waypoints')

    report = {'schema': 'slaihand_pose_design/1',
              'existing_rank': rank,
              'existing_condition': float(s[0] / s[rank - 1]),
              'training_deg': training,
              'validation_deg': validation_degrees,
              'estimated_validation_rmse_before': before_mean,
              'estimated_validation_rmse_after': after_mean,
              'note': 'greedy D-optimal in the identifiable subspace; rough estimate; '
                      'offline design only, no hardware motion.'}
    (HERE / 'pose_design_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote pose_design_20261110.json')


if __name__ == '__main__':
    main()
