"""Compare alternative static-sag feature sets / regularization offline.

Training data: T1a+T1b held static windows (21 poses). Validation: V1 baseline
sag (11 poses, never used for fitting). Metrics: leave-one-pose-out CV on the
training set and V1 prediction error. No model is deployed by this script.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'verification/hardware/five_axis_20261107/five_axis_j5_model_20261109'))
try:
    from five_axis_model_j5 import Kinematics  # noqa: E402
except Exception:
    Kinematics = None

PLANS = HERE / 'plans'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def training_set():
    X, Y = [], []
    for tag in ('T1a', 'T1b'):
        for window in read(HERE / ('windows_%s.json' % tag))['windows']:
            if 'sag_deg' not in window:
                continue
            desired = window.get('desired_deg', window.get('target_deg'))
            if desired is None:
                continue
            X.append([float(v) for v in desired])
            Y.append([float(v) for v in window['sag_deg']])
    return np.array(X), np.array(Y)


def v1_set():
    plan = np.array([w['degrees'] for w in read(PLANS / 'V1_sag_20261110.json')['waypoints']],
                    dtype=float)
    data = read(HERE / 'windows_V1.json')
    rows = [w for w in data['windows'] if 'sag_deg' in w]
    idx = [w['index'] for w in rows]
    sag = np.array([w['sag_deg'] for w in rows], dtype=float)
    return plan[idx], sag


def feat_current(q_deg):
    q = np.asarray(q_deg, dtype=float) / 10.0
    return np.stack([np.ones(len(q)), q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4],
                     q[:, 1] ** 2, q[:, 2] ** 2, q[:, 3] ** 2,
                     q[:, 0] * q[:, 1], q[:, 2] * q[:, 3]], axis=1)


def feat_full_quad(q_deg):
    q = np.asarray(q_deg, dtype=float) / 10.0
    cols = [np.ones(len(q))]
    cols += [q[:, j] for j in range(5)]
    cols += [q[:, j] ** 2 for j in range(5)]
    for i in range(5):
        for j in range(i + 1, 5):
            cols.append(q[:, i] * q[:, j])
    return np.stack(cols, axis=1)


def feat_current_plus(q_deg):
    q = np.asarray(q_deg, dtype=float) / 10.0
    return np.stack([np.ones(len(q)), q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4],
                     q[:, 1] ** 2, q[:, 2] ** 2, q[:, 3] ** 2, q[:, 4] ** 2,
                     q[:, 0] * q[:, 1], q[:, 2] * q[:, 3], q[:, 0] * q[:, 4],
                     q[:, 1] * q[:, 4]], axis=1)


def feat_gravity(q_deg):
    if Kinematics is None:
        return None
    kin = Kinematics()
    rows = []
    for pose in q_deg:
        q = np.radians(np.asarray(pose[:4], dtype=float))
        rows.append(kin.prior_regressor(q).reshape(-1))
    return np.array(rows)


FEATURES = {'current11': feat_current, 'full_quad16': feat_full_quad,
            'current_plus14': feat_current_plus, 'gravity16': feat_gravity}


def ridge_fit(F, Y, alpha):
    A = F.T @ F + alpha * np.eye(F.shape[1])
    return np.linalg.solve(A, F.T @ Y)


def predict(F, coef):
    return F @ coef


def loocv(X, Y, feature_fn, alpha):
    F = feature_fn(X)
    rows = []
    for hold in range(len(X)):
        mask = np.arange(len(X)) != hold
        coef = ridge_fit(F[mask], Y[mask], alpha)
        pred = predict(F[~mask], coef)
        rows.append(pred - Y[~mask])
    residuals = np.vstack(rows)
    return float(np.sqrt(np.mean(residuals ** 2))), np.sqrt(np.mean(residuals ** 2, axis=0))


def main():
    X, Y = training_set()
    v1_x, v1_y = v1_set()
    print('training rows', len(X), 'V1 rows', len(v1_x))
    results = []
    alphas = (1e-4, 1e-3, 1e-2, 0.05, 0.2, 1.0)
    for name, fn in FEATURES.items():
        F = fn(X)
        if F is None:
            continue
        for alpha in alphas:
            cv, cv_axis = loocv(X, Y, fn, alpha)
            coef = ridge_fit(F, Y, alpha)
            pred_v1 = predict(fn(v1_x), coef)
            err_v1 = pred_v1 - v1_y
            v1_rmse = float(np.sqrt(np.mean(err_v1 ** 2)))
            v1_axis = np.sqrt(np.mean(err_v1 ** 2, axis=0))
            results.append({'features': name, 'columns': int(F.shape[1]), 'alpha': alpha,
                            'loocv_rmse_deg': round(cv, 4),
                            'loocv_axis_deg': cv_axis.round(4).tolist(),
                            'v1_rmse_deg': round(v1_rmse, 4),
                            'v1_axis_deg': v1_axis.round(4).tolist(),
                            'v1_max_deg': round(float(np.max(np.abs(err_v1))), 4)})
    results.sort(key=lambda r: (r['v1_rmse_deg'], r['loocv_rmse_deg']))
    current = read(HERE / 'sag_model_20261110.json')['model']
    current_v1_rmse = None
    if 'current11' in FEATURES:
        F = feat_current(v1_x)
        pred = np.stack([F @ np.array(current[str(j)]['coef']) for j in range(5)], axis=1)
        current_v1_rmse = float(np.sqrt(np.mean((pred - v1_y) ** 2)))
    report = {'schema': 'slaihand_sag_model_alternatives/1',
              'training_rows': int(len(X)), 'v1_rows': int(len(v1_x)),
              'current_model_v1_rmse_deg': None if current_v1_rmse is None else round(current_v1_rmse, 4),
              'candidates': results}
    (HERE / 'sag_model_alternatives_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('current model V1 RMSE %.4f deg' % (current_v1_rmse or float('nan')))
    print('%-14s %5s %6s %10s %10s %10s' % ('features', 'cols', 'alpha', 'LOOCV', 'V1_RMSE', 'V1_MAX'))
    for row in results[:20]:
        print('%-14s %5d %6g %10.4f %10.4f %10.4f' % (
            row['features'], row['columns'], row['alpha'],
            row['loocv_rmse_deg'], row['v1_rmse_deg'], row['v1_max_deg']))
    print('wrote sag_model_alternatives_20261110.json')


if __name__ == '__main__':
    main()
