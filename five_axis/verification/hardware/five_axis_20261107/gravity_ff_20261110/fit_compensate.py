"""Fit a static gravity-sag model from training poses and emit a compensated plan."""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def features(poses_deg):
    """Basis for per-joint static sag: affine + mild nonlinearity in joint angles."""
    q = np.asarray(poses_deg, dtype=float) / 10.0
    columns = [np.ones(len(q)), q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4],
               q[:, 1] ** 2, q[:, 2] ** 2, q[:, 3] ** 2,
               q[:, 0] * q[:, 1], q[:, 2] * q[:, 3]]
    return np.stack(columns, axis=1)


def fit(tags, ridge=0.02):
    X, Y = [], []
    for tag in tags:
        data = read(HERE / f'windows_{tag}.json')
        rows = [w for w in data['windows']
                if 'sag_deg' in w and w.get('samples', 0) >= 150]
        if not rows:
            continue
        X.append(np.array([w['desired_deg'] for w in rows]))
        Y.append(np.array([w['sag_deg'] for w in rows]))
    X = np.concatenate(X)
    Y = np.concatenate(Y)
    F = features(X)
    model = {}
    for joint in range(5):
        y = Y[:, joint]
        A = F.T @ F + ridge * np.eye(F.shape[1])
        coef = np.linalg.solve(A, F.T @ y)
        residual = y - F @ coef
        model[joint] = {'coef': coef.tolist(),
                        'train_rmse_deg': float(np.sqrt(np.mean(residual ** 2))),
                        'train_max_deg': float(np.max(np.abs(residual)))}
    return model, X, Y


def predict(model, poses_deg):
    F = features(poses_deg)
    return np.stack([F @ np.array(model[j]['coef']) for j in range(5)], axis=1)


def write_plan(points, path, move_s=6.0, dwell_s=6.0):
    waypoints = [{'degrees': [float(v) for v in point], 'move_s': move_s, 'dwell_s': dwell_s}
                 for point in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    train_tags = sys.argv[1].split(',')
    valid_plan = Path(sys.argv[2])
    out_name = sys.argv[3]
    model, X, Y = fit(train_tags)
    plan = read(valid_plan)
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    predicted = predict(model, desired)
    compensated = desired - predicted
    # Safety: acquisition plans must start and end at absolute zero.
    compensated[0] = desired[0]
    compensated[-1] = desired[-1]
    write_plan(compensated, HERE / 'plans' / out_name)
    report = {'schema': 'slaihand_static_sag_compensation/1',
              'train_tags': train_tags,
              'model': {str(k): v for k, v in model.items()},
              'validation_plan': str(valid_plan),
              'validation_desired_deg': desired.tolist(),
              'validation_predicted_sag_deg': np.round(predicted, 4).tolist(),
              'validation_compensated_deg': np.round(compensated, 4).tolist(),
              'compensated_plan': str(HERE / 'plans' / out_name)}
    (HERE / 'sag_model_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('train windows', len(X))
    for joint in range(5):
        print(' J%d train rmse %.4f max %.4f deg' % (
            joint + 1, model[joint]['train_rmse_deg'], model[joint]['train_max_deg']))
    print('wrote', HERE / 'plans' / out_name)
