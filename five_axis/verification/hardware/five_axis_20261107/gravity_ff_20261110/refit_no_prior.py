"""No-prior rigid-body gravity refit on the 159 accepted static windows (rough pass).

Model:
  effort(w, j) = Y16(q_w)[j] @ beta                      (j = J1..J4 gravity)
               + const_j + dir_j * sign_j(w)             (effort offset + first-order friction)
               + session_offset(session_w, j)            (per-session bias, penalized)

- beta: 16 free physical gravity parameters (4 links x mass + first moment), no old prior.
- J5 gravity term is zero (2026-11-10 coupling test: box COM near J5 axis).
- Validation: leave-one-named-session-out; old fixed prior model shown for reference.
- Offline only; no FF, no deployment.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
HERE = ROOT / 'verification/hardware/five_axis_20261107/gravity_ff_20261110'
BASE = ROOT / 'verification/hardware/five_axis_20261107'
WINDOWS = BASE / 'refit_gc09_12_20261109/static_windows.json'
sys.path.insert(0, str(BASE / 'five_axis_j5_model_20261109'))

from five_axis_model_j5 import Kinematics, prior_parameters  # noqa: E402

NAMES = ['J1', 'J2', 'J3', 'J4', 'J5']
RIDGE = 0.3


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def direction_array(value):
    array = np.zeros(5)
    if value is None:
        return array
    if isinstance(value, (list, tuple)):
        for i, item in enumerate(value[:5]):
            array[i] = float(item)
        return array
    if isinstance(value, dict):
        for key, item in value.items():
            if key in NAMES:
                array[NAMES.index(key)] = float(item)
        return array
    return np.full(5, float(value))


def load_poses():
    rows = []
    for pose in read(WINDOWS)['poses']:
        rows.append({'q': np.array(pose['q_rad'], dtype=float),
                     'y': np.array(pose['effort_mean_nm'], dtype=float),
                     'session': pose.get('session'),
                     'driver': pose.get('driver_session'),
                     'sign': direction_array(pose.get('last_planned_approach_sign'))})
    return rows


def regressor16(rows):
    kin = Kinematics()
    return np.stack([kin.prior_regressor(row['q'][:4]) for row in rows])  # (n,4,16)


def design(rows, sessions, y16):
    session_index = {name: index for index, name in enumerate(sessions)}
    columns = 16 + 5 + 5 + 5 * len(sessions)
    matrix = np.zeros((len(rows), 5, columns))
    for w, row in enumerate(rows):
        for axis in range(5):
            if axis < 4:
                matrix[w, axis, :16] = y16[w, axis]
            matrix[w, axis, 16 + axis] = 1.0
            matrix[w, axis, 21 + axis] = row['sign'][axis]
            if row['driver'] in session_index:
                matrix[w, axis, 26 + 5 * session_index[row['driver']] + axis] = 1.0
    return matrix


def solve(rows, sessions, y16=None):
    if y16 is None:
        y16 = regressor16(rows)
    matrix = design(rows, sessions, y16)
    x = matrix.reshape(-1, matrix.shape[-1])
    y = np.array([row['y'] for row in rows]).reshape(-1)
    penalty = np.full(x.shape[1], RIDGE)
    penalty[:16] = 1e-6
    penalty[16:26] = RIDGE * 0.05
    beta = np.linalg.solve(x.T @ x + np.diag(penalty), x.T @ y)
    return beta, y16


def predict(rows, beta, sessions, y16=None):
    if y16 is None:
        y16 = regressor16(rows)
    session_index = {name: index for index, name in enumerate(sessions)}
    prediction = np.zeros((len(rows), 5))
    for w, row in enumerate(rows):
        for axis in range(5):
            value = beta[16 + axis] + beta[21 + axis] * row['sign'][axis]
            if axis < 4:
                value += y16[w, axis] @ beta[:16]
            if row['driver'] in session_index:
                value += beta[26 + 5 * session_index[row['driver']] + axis]
            prediction[w, axis] = value
    return prediction


def rmse(prediction, target):
    axis = np.sqrt(np.mean((prediction - target) ** 2, axis=0))
    return axis, float(np.sqrt(np.mean((prediction - target) ** 2)))


def main():
    rows = load_poses()
    y = np.array([row['y'] for row in rows])
    sessions = sorted({row['driver'] for row in rows})
    names = sorted({row['session'] for row in rows})
    print('windows %d, named sessions %d, driver sessions %d' % (
        len(rows), len(names), len(sessions)))

    beta, y16 = solve(rows, sessions)
    train_pred = predict(rows, beta, sessions, y16)
    axis_rmse, overall = rmse(train_pred, y)
    print('free fit train RMSE', np.round(axis_rmse, 4), 'overall %.4f' % overall)

    # old fixed prior + the same nuisance terms (fair structural comparison)
    prior = prior_parameters()[0]
    prior_pred = np.zeros_like(y)
    for w, row in enumerate(rows):
        for axis in range(5):
            value = beta[16 + axis] + beta[21 + axis] * row['sign'][axis]
            if axis < 4:
                value += y16[w, axis] @ prior[:16]
            if row['driver'] in sessions:
                value += beta[26 + 5 * sessions.index(row['driver']) + axis]
            prior_pred[w, axis] = value
    prior_axis, prior_overall = rmse(prior_pred, y)
    print('old prior+nusisance RMSE', np.round(prior_axis, 4), 'overall %.4f' % prior_overall)

    fold_overall, fold_axis = [], []
    for held in names:
        train_rows = [row for row in rows if row['session'] != held]
        test_rows = [row for row in rows if row['session'] == held]
        if len(test_rows) < 3:
            continue
        beta_fold, _ = solve(train_rows, sessions)
        prediction = predict(test_rows, beta_fold, sessions)
        target = np.array([row['y'] for row in test_rows])
        axis, over = rmse(prediction, target)
        fold_axis.append(axis)
        fold_overall.append(over)
        print('held %-6s n=%2d overall RMSE %.4f' % (held, len(test_rows), over))
    fold_axis = np.array(fold_axis)
    print('CV mean per-axis RMSE', np.round(fold_axis.mean(axis=0), 4),
          'mean overall %.4f' % float(np.mean(fold_overall)))

    singular = np.linalg.svd(y16.reshape(-1, 16), compute_uv=False)
    print('Y16 singular values', np.round(singular, 5).tolist())
    print('condition %.1f' % (singular[0] / singular[-1]))

    report = {
        'schema': 'slaihand_no_prior_gravity_refit/1',
        'windows': len(rows), 'named_sessions': names, 'driver_sessions': sessions,
        'free_train_rmse_per_axis': axis_rmse.tolist(), 'free_train_overall': overall,
        'old_prior_rmse_per_axis': prior_axis.tolist(), 'old_prior_overall': prior_overall,
        'cv_mean_rmse_per_axis': fold_axis.mean(axis=0).tolist(),
        'cv_mean_overall': float(np.mean(fold_overall)),
        'cv_folds': [{'session': name, 'overall': float(value)}
                     for name, value in zip(names, fold_overall)],
        'beta16': beta[:16].tolist(),
        'const5': beta[16:21].tolist(), 'dir5': beta[21:26].tolist(),
        'session_offsets': {name: beta[26 + 5 * i:31 + 5 * i].tolist()
                            for i, name in enumerate(sessions)},
        'Y16_singular_values': singular.tolist(),
        'condition': float(singular[0] / singular[-1]),
        'note': 'rough pass in effort units; per-axis offset/direction/session terms; '
                'J5 gravity zero; no FF/deployment.',
    }
    (HERE / 'no_prior_refit_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote no_prior_refit_20261110.json')


if __name__ == '__main__':
    main()
