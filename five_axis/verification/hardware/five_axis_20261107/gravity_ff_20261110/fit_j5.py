"""Fit a simple, robust J5 static stiction/gravity model from J5A+J5B."""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def approach_sign(requested_deg):
    signs = [0.0]
    for index in range(1, len(requested_deg)):
        delta = requested_deg[index][4] - requested_deg[index - 1][4]
        signs.append(float(np.sign(delta)))
    return np.array(signs)


def rows_for(tag):
    plan = read(HERE / 'plans' / f'{tag}_20261110.json')
    requested = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    signs = approach_sign(requested)
    data = read(HERE / f'windows_{tag}.json')
    rows = []
    for window in data['windows']:
        if 'sag_deg' not in window or window.get('samples', 0) < 150:
            continue
        index = window['index']
        if index == 0 or index == len(requested) - 1:
            continue
        rows.append((requested[index], signs[index], window['sag_deg'][4]))
    return rows


def features(q_deg, signs):
    q = np.asarray(q_deg, dtype=float)
    return np.stack([np.ones(len(q)),
                     np.asarray(signs, dtype=float),
                     q[:, 1] / 10.0,
                     q[:, 2] / 10.0,
                     q[:, 3] / 10.0,
                     q[:, 4] / 20.0,
                     0.5 * np.asarray(signs, dtype=float) * q[:, 4] / 20.0], axis=1)


def main():
    rows = []
    for tag in ('J5A', 'J5B'):
        rows.extend(rows_for(tag))
    q = np.array([row[0] for row in rows])
    signs = np.array([row[1] for row in rows])
    sag = np.array([row[2] for row in rows])
    F = features(q, signs)
    ridge = 0.1
    coef = np.linalg.solve(F.T @ F + ridge * np.eye(F.shape[1]), F.T @ sag)
    residual = sag - F @ coef
    model = {'schema': 'slaihand_j5_static_sag/2',
             'train_tags': ['J5A', 'J5B'],
             'windows': int(len(rows)),
             'coef': coef.tolist(),
             'feature_names': ['1', 'sign', 'q2/10', 'q3/10', 'q4/10', 'q5/20', 'sign*q5/40'],
             'train_rmse_deg': float(np.sqrt(np.mean(residual ** 2))),
             'train_max_deg': float(np.max(np.abs(residual)))}
    (HERE / 'j5_sag_model_20261110.json').write_text(
        json.dumps(model, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('J5 model windows', len(rows), 'rmse %.4f max %.4f deg' % (
        model['train_rmse_deg'], model['train_max_deg']))
    for index in range(len(rows)):
        print('  q5=%7.2f sign=%+.0f sag=%7.3f pred=%7.3f' % (
            q[index, 4], signs[index], sag[index], F[index] @ coef))


if __name__ == '__main__':
    main()
