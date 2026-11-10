"""Gravity refit with q-dependent friction columns from the friction sweeps.

Model:
  effort(w,j) = Y16(q)[j] @ beta + const_j + dir_j*sign_j
              + session_offset + scale_j * f_j(q_j) * sign_j

f_j(q) comes from analyze_friction.py (up/down half-difference). The scale_j
coefficients are fitted from data (1.0 = measured friction is directly useful).
Compare CV with and without the friction columns.
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


def load_friction():
    document = read(HERE / 'friction_estimates_20261110.json')['joints']
    curves = {}
    for name, entry in document.items():
        q = np.array([row['q_deg'] for row in entry['bins']])
        f = np.array([row['friction_nm'] for row in entry['bins']])
        order = np.argsort(q)
        curves[name] = (q[order], f[order])
    return curves


def friction_value(curves, axis, q_deg):
    name = 'J%d' % (axis + 1)
    if name not in curves:
        return 0.0
    q, f = curves[name]
    return float(np.interp(q_deg, q, f))


def load_poses(curves):
    rows = []
    for pose in read(WINDOWS)['poses']:
        q = np.array(pose['q_rad'], dtype=float)
        y = np.array(pose['effort_mean_nm'], dtype=float)
        sign = direction_array(pose.get('last_planned_approach_sign'))
        friction = np.zeros(5)
        for axis in range(5):
            if sign[axis] != 0:
                friction[axis] = sign[axis] * friction_value(
                    curves, axis, float(np.degrees(q[axis])))
        rows.append({'q': q, 'y': y, 'sign': sign, 'friction': friction,
                     'session': pose.get('session'), 'driver': pose.get('driver_session')})
    return rows


def design(rows, sessions, with_friction):
    kin = Kinematics()
    y16 = np.stack([kin.prior_regressor(row['q'][:4]) for row in rows])
    index = {name: i for i, name in enumerate(sessions)}
    columns = 16 + 5 + 5 + 5 * len(sessions) + (5 if with_friction else 0)
    matrix = np.zeros((len(rows), 5, columns))
    for w, row in enumerate(rows):
        for axis in range(5):
            if axis < 4:
                matrix[w, axis, :16] = y16[w, axis]
            matrix[w, axis, 16 + axis] = 1.0
            matrix[w, axis, 21 + axis] = row['sign'][axis]
            if row['driver'] in index:
                matrix[w, axis, 26 + 5 * index[row['driver']] + axis] = 1.0
            if with_friction:
                matrix[w, axis, 26 + 5 * len(sessions) + axis] = row['friction'][axis]
    return matrix


def penalty(columns):
    values = np.full(columns, RIDGE)
    values[:16] = 1e-6
    values[16:26] = RIDGE * 0.05
    return values


def solve(x, y, columns, robust=True):
    beta = np.linalg.solve(x.T @ x + np.diag(penalty(columns)), x.T @ y)
    if robust:
        for _ in range(6):
            residual = y - x @ beta
            weights = 1.0 / np.maximum(1.0, np.abs(residual) / HUBER_DELTA)
            wx = x * weights[:, None]
            beta = np.linalg.solve(wx.T @ x + np.diag(penalty(columns)), wx.T @ y)
    return beta


def predict(rows, beta, sessions, with_friction):
    kin = Kinematics()
    y16 = np.stack([kin.prior_regressor(row['q'][:4]) for row in rows])
    index = {name: i for i, name in enumerate(sessions)}
    columns = 26 + 5 * len(sessions) + (5 if with_friction else 0)
    prediction = np.zeros((len(rows), 5))
    for w, row in enumerate(rows):
        for axis in range(5):
            value = beta[16 + axis] + beta[21 + axis] * row['sign'][axis]
            if axis < 4:
                value += y16[w, axis] @ beta[:16]
            if row['driver'] in index:
                value += beta[26 + 5 * index[row['driver']] + axis]
            if with_friction:
                value += beta[26 + 5 * len(sessions) + axis] * row['friction'][axis]
            prediction[w, axis] = value
    return prediction


def rmse(prediction, target):
    return np.sqrt(np.mean((prediction - target) ** 2, axis=0)), float(
        np.sqrt(np.mean((prediction - target) ** 2)))


def cv(rows, sessions, names, with_friction):
    folds = []
    for held in names:
        train = [row for row in rows if row['session'] != held]
        test = [row for row in rows if row['session'] == held]
        if len(test) < 3:
            continue
        matrix = design(train, sessions, with_friction)
        x = matrix.reshape(-1, matrix.shape[-1])
        y = np.array([row['y'] for row in train]).reshape(-1)
        beta = solve(x, y, x.shape[1])
        prediction = predict(test, beta, sessions, with_friction)
        target = np.array([row['y'] for row in test])
        axis, overall = rmse(prediction, target)
        folds.append({'session': held, 'overall': float(overall), 'axis': axis.tolist()})
    return folds


def main():
    curves = load_friction()
    rows = load_poses(curves)
    sessions = sorted({row['driver'] for row in rows})
    names = sorted({row['session'] for row in rows})
    for with_friction in (False, True):
        folds = cv(rows, sessions, names, with_friction)
        axis = np.array([fold['axis'] for fold in folds]).mean(axis=0)
        overall = float(np.mean([fold['overall'] for fold in folds]))
        print('friction_columns=%s CV mean per-axis %s overall %.4f' % (
            with_friction, np.round(axis, 4).tolist(), overall))
    # full fit with friction for coefficient report
    matrix = design(rows, sessions, True)
    x = matrix.reshape(-1, matrix.shape[-1])
    y = np.array([row['y'] for row in rows]).reshape(-1)
    beta = solve(x, y, x.shape[1])
    scales = beta[26 + 5 * len(sessions):].tolist()
    print('fitted friction scales (1.0 = measured):', np.round(scales, 3).tolist())
    report = {'schema': 'slaihand_friction_corrected_gravity_refit/1',
              'windows': len(rows),
              'friction_scales': scales,
              'note': 'offline rough model; q-dependent friction columns from F1-F5 sweeps.'}
    (HERE / 'friction_corrected_refit_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote friction_corrected_refit_20261110.json')


if __name__ == '__main__':
    main()
