"""Compare H2/H4 same-pose spread before vs after hysteresis compensation."""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_run(plan_name, run_name):
    plan = read(PLANS / ('%s_20261110.json' % plan_name))
    original = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % run_name))
    rows = sorted([w for w in data['windows'] if 'sag_deg' in w and w.get('samples', 0) >= 150],
                  key=lambda w: w['index'])
    return {w['index']: np.array(w['mean_fb_deg']) - original[w['index']] for w in rows}


def spread(matrix):
    return np.max(matrix, axis=0) - np.min(matrix, axis=0)


def analyse(label, before_run, after_run, indices):
    before_all = load_run('H2_repeat' if label == 'H2' else 'H4_stiction', before_run)
    after_all = load_run('H2_repeat' if label == 'H2' else 'H4_stiction', after_run)
    before = np.array([before_all[i] for i in indices if i in before_all])
    after = np.array([after_all[i] for i in indices if i in after_all])
    return {'label': label, 'before_indices': [i for i in indices if i in before_all],
            'after_indices': [i for i in indices if i in after_all],
            'before_rms_deg': np.sqrt(np.mean(before ** 2, axis=0)).round(4).tolist(),
            'after_rms_deg': np.sqrt(np.mean(after ** 2, axis=0)).round(4).tolist(),
            'before_spread_deg': spread(before).round(4).tolist(),
            'after_spread_deg': spread(after).round(4).tolist()}


def main():
    results = [analyse('H2', 'H2_repeat', 'H2_hyst2', (2, 4)),
               analyse('H4', 'H4_stiction', 'H4_hyst2', (2, 4, 6))]
    report = {'schema': 'slaihand_hysteresis_retest/1',
              'note': 'residual vs original desired; spread across same-pose visits',
              'results': results}
    (HERE / 'h2h4_hyst_retest_20261112.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    for item in results:
        print('%-3s spread before %s -> after %s' % (
            item['label'], item['before_spread_deg'], item['after_spread_deg']))
        print('    RMS    before %s -> after %s' % (
            item['before_rms_deg'], item['after_rms_deg']))
    print('wrote h2h4_hyst_retest_20261112.json')


if __name__ == '__main__':
    main()
