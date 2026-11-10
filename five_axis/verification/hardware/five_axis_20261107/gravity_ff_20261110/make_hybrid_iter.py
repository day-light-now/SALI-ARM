"""Build the hybrid iter1 plan: one measured correction on top of the model plan.

Unlike make_iter_comp.py (which assumes the run command equals the original
target), the model run already carries a compensation offset. The correct update
uses the ORIGINAL desired pose as target:

    residual = mean_fb - original_desired
    iter1_command = model_command - residual

First/last waypoints stay at the original absolute-zero values.
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def main():
    if len(sys.argv) < 5:
        print('USAGE: make_hybrid_iter.py <model_run_tag> <model_plan_name> '
              '<original_plan_name> <out_name>')
        return 2
    run_tag, model_name, original_name, out_name = sys.argv[1:5]
    model_plan = read(HERE / 'plans' / ('%s_20261110.json' % model_name))
    original_plan = read(HERE / 'plans' / ('%s_20261110.json' % original_name))
    command = np.array([w['degrees'] for w in model_plan['waypoints']], dtype=float)
    original = np.array([w['degrees'] for w in original_plan['waypoints']], dtype=float)
    data = read(HERE / ('windows_%s.json' % run_tag))
    windows = {w['index']: w for w in data['windows'] if 'sag_deg' in w}
    iter1 = command.copy()
    corrections = np.zeros_like(command)
    for index, window in windows.items():
        measured = np.array(window['mean_fb_deg'], dtype=float)
        residual = measured - original[index]
        corrections[index] = -residual
        iter1[index] = command[index] - residual
    iter1[0] = original[0]
    iter1[-1] = original[-1]
    path = HERE / 'plans' / ('%s_20261110.json' % out_name)
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': [{'degrees': [float(v) for v in row],
                                               'move_s': float(w.get('move_s', 6.0)),
                                               'dwell_s': float(w.get('dwell_s', 6.0))}
                                              for row, w in zip(iter1, model_plan['waypoints'])]},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote', path)
    for index in windows:
        print(' w%d measured-original=%s correction=%s command=%s' % (
            index, np.round(np.array(windows[index]['mean_fb_deg']) - original[index], 3).tolist(),
            np.round(corrections[index], 3).tolist(), np.round(iter1[index], 3).tolist()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
