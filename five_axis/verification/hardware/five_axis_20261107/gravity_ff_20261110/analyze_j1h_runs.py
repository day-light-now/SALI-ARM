"""Analyze the dedicated J1 hysteresis runs (J1H_m8/0/p8).

Each plan visits the same target twice, from + and - sides. Residual =
mean_fb - desired; delta = (residual(+) - residual(-)) / 2 at the same pose.
"""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def analyze(name):
    plan = read(PLANS / ('%s_20261111.json' % name))
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % name))
    rows = {w['index']: w for w in data['windows'] if 'sag_deg' in w}
    residual = {index: np.array(w['mean_fb_deg']) - desired[index]
                for index, w in rows.items()}
    entries = []
    for index in (2, 4):
        if index not in residual:
            continue
        sign = np.sign(desired[index] - desired[index - 1])
        entries.append({'index': index, 'sign': sign.round(3).tolist(),
                        'residual_deg': residual[index].round(4).tolist()})
    if len(entries) == 2:
        first, second = entries
        delta = (np.array(first['residual_deg']) - np.array(second['residual_deg'])) / 2.0
        # Normalize so delta corresponds to the +approach minus -approach.
        if np.all(np.array(first['sign']) > 0):
            delta = (np.array(first['residual_deg']) - np.array(second['residual_deg'])) / 2.0
        else:
            delta = (np.array(second['residual_deg']) - np.array(first['residual_deg'])) / 2.0
    else:
        delta = np.full(5, np.nan)
    return {'name': name, 'target_J1_deg': float(desired[2][0]),
            'visits': entries, 'delta_deg': delta.round(4).tolist(),
            'complete': len(entries) == 2}


def main():
    results = [analyze(name) for name in ('J1H_m8', 'J1H_0', 'J1H_p8')]
    deltas = np.array([row['delta_deg'] for row in results])
    model = read(HERE / 'hysteresis_model_20261111.json')
    report = {'schema': 'slaihand_j1h_analysis/1',
              'runs': results,
              'delta_mean_deg': np.nanmean(deltas, axis=0).round(4).tolist(),
              'model_delta_deg': model['delta_deg'],
              'note': 'delta=(residual(+approach)-residual(-approach))/2 at the same pose'}
    (HERE / 'j1h_analysis_20261112.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    for row in results:
        print('%-8s target J1=%+5.1f delta=%s' % (
            row['name'], row['target_J1_deg'], row['delta_deg']))
    print('mean delta', report['delta_mean_deg'])
    print('model delta', report['model_delta_deg'])
    print('wrote j1h_analysis_20261112.json')


if __name__ == '__main__':
    main()
