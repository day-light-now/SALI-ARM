"""Offline analysis: model-based sag compensation vs measured runs.

Uses existing windows files (V1 baseline, V1subc model-compensated) and the
fitted sag model. No hardware access.
"""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MODEL = HERE / 'sag_model_20261110.json'
PLANS = HERE / 'plans'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def features(poses_deg):
    q = np.asarray(poses_deg, dtype=float) / 10.0
    columns = [np.ones(len(q)), q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4],
               q[:, 1] ** 2, q[:, 2] ** 2, q[:, 3] ** 2,
               q[:, 0] * q[:, 1], q[:, 2] * q[:, 3]]
    return np.stack(columns, axis=1)


def plan_degrees(path):
    return np.array([w['degrees'] for w in read(path)['waypoints']], dtype=float)


def load_run(tag):
    data = read(HERE / ('windows_%s.json' % tag))
    windows = {w['index']: w for w in data['windows'] if 'sag_deg' in w}
    indices = sorted(windows)
    desired = np.array([data['desired_deg'][i] if isinstance(data.get('desired_deg'), list)
                        and len(data['desired_deg']) == len(data['windows'])
                        else windows[i]['desired_deg'] for i in indices], dtype=float)
    sag = np.array([windows[i]['sag_deg'] for i in indices], dtype=float)
    return {'tag': tag, 'plan': data.get('plan'), 'indices': indices,
            'desired': desired, 'sag': sag}


def stats(matrix):
    return {'rms_deg': np.sqrt(np.mean(matrix ** 2, axis=0)).round(4).tolist(),
            'max_deg': np.max(np.abs(matrix), axis=0).round(4).tolist(),
            'overall_rms_deg': round(float(np.sqrt(np.mean(matrix ** 2))), 4),
            'overall_max_deg': round(float(np.max(np.abs(matrix))), 4),
            'rows': int(matrix.shape[0])}


def main():
    model = read(MODEL)['model']
    original = plan_degrees(PLANS / 'V1_sag_20261110.json')
    compensated = plan_degrees(PLANS / 'V1c_sag_20261110.json')
    F = features(original)
    predicted = np.stack([F @ np.array(model[str(j)]['coef']) for j in range(5)], axis=1)

    result = {'schema': 'slaihand_model_comp_offline_analysis/1',
              'model': str(MODEL), 'validation_plan': 'plans/V1_sag_20261110.json',
              'predicted_sag_deg': predicted.round(4).tolist()}
    baseline = load_run('V1')
    d_original = float(np.max(np.abs(baseline['desired'] - original[baseline['indices']])))
    print('V1 plan=%s matches original (diff %.5f)' % (baseline['plan'], d_original))
    model_error = predicted[baseline['indices']] - baseline['sag']
    result['model_validation_error'] = stats(model_error)
    result['baseline_sag'] = stats(baseline['sag'])
    print('model validation error vs baseline sag:', json.dumps(result['model_validation_error']))

    compensated_run = load_run('V1subc')
    # Exclude the absolute-zero endpoints (only 0/9 are endpoints for V1sub-like plans).
    interior = compensated_run['indices']
    keep = [k for k in range(len(interior)) if interior[k] not in (0, 9)]
    if keep:
        result['V1subc_residual'] = stats(compensated_run['sag'][keep])
        print('V1subc residual after model compensation:',
              json.dumps(result['V1subc_residual']))
    print('predicted vs compensated plan max diff %.5f deg' % float(
        np.max(np.abs(compensated - (original - predicted)))))

    (HERE / 'offline_model_comp_20261110.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote offline_model_comp_20261110.json')


if __name__ == '__main__':
    main()
