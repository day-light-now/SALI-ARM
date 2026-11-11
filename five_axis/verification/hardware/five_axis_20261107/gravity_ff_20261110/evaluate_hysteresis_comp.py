"""Offline evaluation of the direction hysteresis compensation.

Applies the fitted delta model to the measured H2/H4/H1 residuals:
    predicted residual after = measured residual - delta * approach_sign
and reports same-pose spread / RMS before and after. This is the evaluation
counterpart of compensate_plan.py --hysteresis (in-sample; hardware validation
with the new J1 plans is still pending).
"""
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'

spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load(name):
    plan = read(PLANS / ('%s_20261110.json' % name))
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % name))
    rows = sorted([w for w in data['windows'] if 'sag_deg' in w and w.get('samples', 0) >= 150],
                  key=lambda w: w['index'])
    indices = np.array([w['index'] for w in rows], dtype=int)
    residual = np.array([w['mean_fb_deg'] for w in rows]) - desired[indices]
    signs = np.zeros_like(desired)
    signs[1:] = np.sign(np.diff(desired, axis=0))
    return {'desired': desired, 'indices': indices, 'residual': residual, 'signs': signs}


def same_pose_indices(name):
    return {'H2_repeat': (2, 4), 'H4_stiction': (2, 4, 6)}.get(name)


def evaluate(name):
    data = load(name)
    model = read(HERE / 'hysteresis_model_20261111.json')
    delta = np.array(model['delta_deg'])
    before = data['residual']
    after = before - delta[None, :] * data['signs'][data['indices']]
    keep = same_pose_indices(name)
    if keep:
        positions = [i for i, index in enumerate(data['indices']) if index in keep]
        before = before[positions]
        after = after[positions]
    return {'name': name,
            'same_pose_only': bool(keep),
            'before_spread_deg': (np.max(before, axis=0) - np.min(before, axis=0)).round(4).tolist(),
            'after_spread_deg': (np.max(after, axis=0) - np.min(after, axis=0)).round(4).tolist(),
            'before_rms_deg': np.sqrt(np.mean(before ** 2, axis=0)).round(4).tolist(),
            'after_rms_deg': np.sqrt(np.mean(after ** 2, axis=0)).round(4).tolist()}


def main():
    report = {'schema': 'slaihand_hysteresis_comp_offline_eval/1',
              'model': 'hysteresis_model_20261111.json',
              'note': 'in-sample; new J1 hysteresis plans are prepared for hardware validation',
              'results': [evaluate(name) for name in ('H2_repeat', 'H4_stiction', 'H1_holdout')]}
    (HERE / 'hysteresis_comp_offline_eval_20261111.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    for item in report['results']:
        print('%-12s same_pose=%s spread %s -> %s' % (
            item['name'], item['same_pose_only'],
            item['before_spread_deg'], item['after_spread_deg']))
    print('wrote hysteresis_comp_offline_eval_20261111.json')


if __name__ == '__main__':
    main()
