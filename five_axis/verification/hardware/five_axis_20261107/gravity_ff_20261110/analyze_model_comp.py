"""Offline analysis: model-based sag compensation vs measured runs.

Correct residual definition:
  - uncompensated run: residual vs original = mean_fb - original desired;
  - compensated run: residual vs original = mean_fb - ORIGINAL desired pose,
    not mean_fb - compensated command (sag_deg is relative to the command).
Model validation error is predicted sag vs measured sag on the baseline run.
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


def plan_degrees(name):
    return np.array([w['degrees'] for w in read(PLANS / name)['waypoints']], dtype=float)


def load_run(tag):
    data = read(HERE / ('windows_%s.json' % tag))
    rows = [w for w in data['windows'] if 'sag_deg' in w]
    rows.sort(key=lambda w: w['index'])
    indices = np.array([w['index'] for w in rows], dtype=int)
    desired = np.array([w['desired_deg'] for w in rows], dtype=float)
    mean_fb = np.array([w['mean_fb_deg'] for w in rows], dtype=float)
    return {'tag': tag, 'plan': data.get('plan'), 'indices': indices,
            'desired': desired, 'mean_fb': mean_fb}


def stats(matrix):
    return {'rms_deg': np.sqrt(np.mean(matrix ** 2, axis=0)).round(4).tolist(),
            'max_deg': np.max(np.abs(matrix), axis=0).round(4).tolist(),
            'overall_rms_deg': round(float(np.sqrt(np.mean(matrix ** 2))), 4),
            'overall_max_deg': round(float(np.max(np.abs(matrix))), 4),
            'rows': int(matrix.shape[0])}


def main():
    model = read(MODEL)['model']
    v1 = plan_degrees('V1_sag_20261110.json')
    predicted_v1 = np.stack([features(v1) @ np.array(model[str(j)]['coef'])
                             for j in range(5)], axis=1)

    result = {'schema': 'slaihand_model_comp_offline_analysis/2',
              'residual_definition': 'mean_fb_deg - original_desired_deg',
              'model': str(MODEL),
              'validation_plan': 'plans/V1_sag_20261110.json'}

    baseline = load_run('V1')
    sag_baseline = baseline['mean_fb'] - v1[baseline['indices']]
    model_error = predicted_v1[baseline['indices']] - sag_baseline
    result['baseline_sag'] = stats(sag_baseline)
    result['model_validation_error'] = stats(model_error)
    print('V1 baseline sag:', json.dumps(result['baseline_sag']))
    print('model validation error:', json.dumps(result['model_validation_error']))

    sub_original = plan_degrees('V1sub_sag_20261110.json')
    compensated = load_run('V1subc')
    residual = compensated['mean_fb'] - sub_original[compensated['indices']]
    keep = [i for i, index in enumerate(compensated['indices'])
            if index not in (0, len(sub_original) - 1)]
    result['V1subc_residual'] = stats(residual[keep])
    print('V1subc residual vs original:', json.dumps(result['V1subc_residual']))

    (HERE / 'offline_model_comp_20261110.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote offline_model_comp_20261110.json')


if __name__ == '__main__':
    main()
