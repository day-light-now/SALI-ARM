"""No-prior rigid-body gravity refit with robust (Huber/IRLS) fitting.

Offline only; no FF, no hardware, no deployment.
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

from five_axis_model_j5 import Kinematics  # noqa: E402

NAMES = ['J1', 'J2', 'J3', 'J4', 'J5']
RIDGE = 0.3
HUBER_DELTA = 0.30


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
    return np.stack([kin.prior_regressor(row['q'][:4]) for row in rows])


def design(rows, sessions, y16):
    index = {name: i for i, name in enumerate(sessions)}
    matrix = np.zeros((len(rows), 5, 16 + 5 + 5 + 5 * len(sessions)))
    for w, row in enumerate(rows):
        for axis in range(5):
            if axis < 4:
                matrix[w, axis, :16] = y16[w, axis]
            matrix[w, axis, 16 + axis] = 1.0
            matrix[w, axis, 21 + axis] = row['sign'][axis]
            if row['driver'] in index:
                matrix[w, axis, 26 + 5 * index[row['driver']] + axis] = 1.0
    return matrix


def penalty_vector(columns):
    penalty = np.full(columns, RIDGE)
    penalty[:16] = 1e-6
    penalty[16:26] = RIDGE * 0.05
    return penalty


def solve_ols(x, y, columns):
    return np.linalg.solve(x.T @ x + np.diag(penalty_vector(columns)), x.T @ y)


def solve_robust(x, y, columns, iterations=6):
    beta = solve_ols(x, y, columns)
    for _ in range(iterations):
        residual = y - x @ beta
        scale = np.maximum(1.0, np.abs(residual) / HUBER_DELTA)
        weights = 1.0 / scale
        wx = x * weights[:, None]
        beta = np.linalg.solve(wx.T @ x + np.diag(penalty_vector(columns)), wx.T @ y)
    return beta


def predict(rows, beta, sessions, y16=None):
    if y16 is None:
        y16 = regressor16(rows)
    index = {name: i for i, name in enumerate(sessions)}
    prediction = np.zeros((len(rows), 5))
    for w, row in enumerate(rows):
        for axis in range(5):
            value = beta[16 + axis] + beta[21 + axis] * row['sign'][axis]
            if axis < 4:
                value += y16[w, axis] @ beta[:16]
            if row['driver'] in index:
                value += beta[26 + 5 * index[row['driver']] + axis]
            prediction[w, axis] = value
    return prediction


def rmse(prediction, target):
    return np.sqrt(np.mean((prediction - target) ** 2, axis=0)), float(
        np.sqrt(np.mean((prediction - target) ** 2)))


def cross_validate(rows, sessions, names, robust):
    folds = []
    for held in names:
        train = [row for row in rows if row['session'] != held]
        test = [row for row in rows if row['session'] == held]
        if len(test) < 3:
            continue
        y16 = regressor16(train)
        matrix = design(train, sessions, y16)
        x = matrix.reshape(-1, matrix.shape[-1])
        y = np.array([row['y'] for row in train]).reshape(-1)
        beta = solve_robust(x, y, x.shape[1]) if robust else solve_ols(x, y, x.shape[1])
        prediction = predict(test, beta, sessions)
        target = np.array([row['y'] for row in test])
        axis, overall = rmse(prediction, target)
        folds.append({'session': held, 'n': len(test),
                      'overall': float(overall), 'axis': axis.tolist()})
    return folds


def main():
    rows = load_poses()
    y = np.array([row['y'] for row in rows])
    sessions = sorted({row['driver'] for row in rows})
    names = sorted({row['session'] for row in rows})
    y16 = regressor16(rows)
    matrix = design(rows, sessions, y16)
    x = matrix.reshape(-1, matrix.shape[-1])
    yflat = y.reshape(-1)

    beta_ols = solve_ols(x, yflat, x.shape[1])
    beta_robust = solve_robust(x, yflat, x.shape[1])
    ols_axis, ols_overall = rmse(predict(rows, beta_ols, sessions, y16), y)
    rob_axis, rob_overall = rmse(predict(rows, beta_robust, sessions, y16), y)
    print('OLS    train', np.round(ols_axis, 4), 'overall %.4f' % ols_overall)
    print('ROBUST train', np.round(rob_axis, 4), 'overall %.4f' % rob_overall)

    ols_folds = cross_validate(rows, sessions, names, robust=False)
    rob_folds = cross_validate(rows, sessions, names, robust=True)
    for label, folds in (('OLS', ols_folds), ('ROBUST', rob_folds)):
        axis = np.array([fold['axis'] for fold in folds])
        overall = float(np.mean([fold['overall'] for fold in folds]))
        print(label, 'CV mean per-axis', np.round(axis.mean(axis=0), 4),
              'mean overall %.4f' % overall)

    singular = np.linalg.svd(y16.reshape(-1, 16), compute_uv=False)
    nonzero = singular[singular > 1e-9]
    print('Y16 rank', len(nonzero), 'condition %.1f' % (nonzero[0] / nonzero[-1]))

    report = {
        'schema': 'slaihand_no_prior_gravity_refit_robust/1',
        'windows': len(rows),
        'ols_train_per_axis': ols_axis.tolist(), 'ols_train_overall': ols_overall,
        'robust_train_per_axis': rob_axis.tolist(), 'robust_train_overall': rob_overall,
        'ols_cv_mean_per_axis': np.array([f['axis'] for f in ols_folds]).mean(axis=0).tolist(),
        'ols_cv_mean_overall': float(np.mean([f['overall'] for f in ols_folds])),
        'robust_cv_mean_per_axis': np.array([f['axis'] for f in rob_folds]).mean(axis=0).tolist(),
        'robust_cv_mean_overall': float(np.mean([f['overall'] for f in rob_folds])),
        'robust_cv_folds': rob_folds,
        'beta16': beta_robust[:16].tolist(),
        'const5': beta_robust[16:21].tolist(),
        'dir5': beta_robust[21:26].tolist(),
        'Y16_rank': int(len(nonzero)),
        'Y16_condition': float(nonzero[0] / nonzero[-1]),
        'note': 'offline rough model; effort units; J5 gravity zero; no FF.',
    }
    (HERE / 'no_prior_refit_robust_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote no_prior_refit_robust_20261110.json')


if __name__ == '__main__':
    main()
