"""Fit direction-dependent hysteresis (stiction) offsets from real repeat runs.

Only SAME-POSE visits with opposite approach signs are used, so pose sag cancels:
    residual = mean_fb - desired = a + delta * sign
H2 gives one pair (indices 2/4); H4 gives three visits (2/4/6).
delta_j = (residual(+approach) - residual(-approach)) / 2
Command correction = -delta_j * sign_j.
"""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load(name):
    plan = read(PLANS / ('%s_20261110.json' % name))
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % name))
    rows = sorted([w for w in data['windows'] if 'sag_deg' in w and w.get('samples', 0) >= 150],
                  key=lambda w: w['index'])
    windows = {w['index']: w for w in rows}
    residual = {index: np.array(w['mean_fb_deg']) - desired[index]
                for index, w in windows.items()}
    signs = {}
    for index in windows:
        signs[index] = np.sign(desired[index] - desired[index - 1]) if index > 0 else np.zeros(5)
    return {'desired': desired, 'residual': residual, 'signs': signs}


def delta_from_groups(pairs):
    """pairs: list of (sign, residual vector); returns delta per joint where both signs exist."""
    delta = np.zeros(5)
    detail = []
    for joint in range(5):
        plus = [row[joint] for sign, row in pairs if sign[joint] > 0]
        minus = [row[joint] for sign, row in pairs if sign[joint] < 0]
        if plus and minus:
            delta[joint] = (np.mean(plus) - np.mean(minus)) / 2.0
            detail.append({'joint': joint + 1, 'plus': float(np.mean(plus)),
                           'minus': float(np.mean(minus))})
    return delta, detail


def spread(values):
    return np.max(values, axis=0) - np.min(values, axis=0)


def main():
    h2 = load('H2_repeat')
    h4 = load('H4_stiction')
    h2_pairs = [(h2['signs'][i], h2['residual'][i]) for i in (2, 4)]
    h4_pairs = [(h4['signs'][i], h4['residual'][i]) for i in (2, 4, 6)]
    d2, detail2 = delta_from_groups(h2_pairs)
    d4, detail4 = delta_from_groups(h4_pairs)
    delta = (d2 + d4) / 2.0
    model = {'schema': 'slaihand_direction_hysteresis_model/1',
             'description': 'constant per-joint direction offset fitted from H2/H4 '
                            'same-pose repeat runs; command correction = -delta*sign.',
             'delta_deg': delta.round(6).tolist(),
             'delta_from_H2_deg': d2.round(6).tolist(),
             'delta_from_H4_deg': d4.round(6).tolist(),
             'H2_detail': detail2, 'H4_detail': detail4}
    (HERE / 'hysteresis_model_20261111.json').write_text(
        json.dumps(model, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('delta H2', d2.round(4).tolist())
    print('delta H4', d4.round(4).tolist())
    print('delta avg', delta.round(4).tolist())

    report = {'schema': 'slaihand_direction_hysteresis_analysis/2',
              'model': model,
              'before': {'H2_spread': spread(np.array([h2['residual'][i] for i in (2, 4)])).round(4).tolist(),
                         'H4_spread': spread(np.array([h4['residual'][i] for i in (2, 4, 6)])).round(4).tolist()},
              'cross_validation': []}
    for name, data, indices, fit in (('H2 by H2', h2, (2, 4), d2),
                                     ('H2 by H4', h2, (2, 4), d4),
                                     ('H2 by avg', h2, (2, 4), delta),
                                     ('H4 by H4', h4, (2, 4, 6), d4),
                                     ('H4 by H2', h4, (2, 4, 6), d2),
                                     ('H4 by avg', h4, (2, 4, 6), delta)):
        residual = np.array([data['residual'][i] for i in indices])
        signs = np.array([data['signs'][i] for i in indices])
        after = residual - fit * signs
        report['cross_validation'].append({
            'name': name,
            'before_spread_deg': spread(residual).round(4).tolist(),
            'after_spread_deg': spread(after).round(4).tolist(),
            'before_rms_deg': np.sqrt(np.mean(residual ** 2, axis=0)).round(4).tolist(),
            'after_rms_deg': np.sqrt(np.mean(after ** 2, axis=0)).round(4).tolist()})
        print('%-12s spread before %s -> after %s' % (
            name, report['cross_validation'][-1]['before_spread_deg'],
            report['cross_validation'][-1]['after_spread_deg']))
    (HERE / 'direction_hysteresis_analysis_20261111.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote hysteresis_model_20261111.json and direction_hysteresis_analysis_20261111.json')


if __name__ == '__main__':
    main()
