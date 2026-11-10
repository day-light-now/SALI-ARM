"""Compare compensated subset run against the uncompensated V1 baseline."""
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def windows(tag):
    data = read(HERE / f'windows_{tag}.json')
    return {w['index']: w for w in data['windows'] if 'sag_deg' in w}


def main():
    desired_plan = read(HERE / 'plans' / 'V1sub_sag_20261110.json')
    desired = [np.array(w['degrees'], dtype=float) for w in desired_plan['waypoints']]
    uncomp = windows('V1')
    comp = windows('V1subc')
    # Map each subset pose to the V1 window with the same desired pose.
    v1_by_pose = {}
    for index, window in uncomp.items():
        key = tuple(np.round(window['desired_deg'], 4))
        v1_by_pose[key] = np.array(window['mean_fb_deg'], dtype=float)
    rows = []
    for index in range(1, len(desired) - 1):
        want = desired[index]
        key = tuple(np.round(want, 4))
        base = v1_by_pose.get(key)
        window = comp.get(index)
        if base is None or window is None:
            print('skip', index, 'baseline', base is not None, 'comp', window is not None)
            continue
        actual = np.array(window['mean_fb_deg'], dtype=float)
        rows.append({'index': index, 'desired': want.tolist(),
                     'uncompensated_actual': base.tolist(),
                     'compensated_actual': actual.tolist(),
                     'uncompensated_residual': (base - want).tolist(),
                     'compensated_residual': (actual - want).tolist()})
    un = np.array([r['uncompensated_residual'] for r in rows])
    co = np.array([r['compensated_residual'] for r in rows])
    summary = {
        'schema': 'slaihand_static_sag_compensation_result/1',
        'poses': len(rows),
        'uncompensated': {'per_joint_rms': np.sqrt(np.mean(un ** 2, axis=0)).tolist(),
                          'per_joint_max': np.max(np.abs(un), axis=0).tolist(),
                          'overall_rms': float(np.sqrt(np.mean(un ** 2))),
                          'overall_max': float(np.max(np.abs(un)))},
        'compensated': {'per_joint_rms': np.sqrt(np.mean(co ** 2, axis=0)).tolist(),
                        'per_joint_max': np.max(np.abs(co), axis=0).tolist(),
                        'overall_rms': float(np.sqrt(np.mean(co ** 2))),
                        'overall_max': float(np.max(np.abs(co)))},
        'rows': rows,
    }
    print('poses compared:', len(rows))
    print('uncompensated:', json.dumps(summary['uncompensated'], ensure_ascii=False))
    print('compensated:  ', json.dumps(summary['compensated'], ensure_ascii=False))
    print('per-joint |residual| max (deg), compensation off -> on:')
    for joint in range(5):
        before = summary['uncompensated']['per_joint_max'][joint]
        after = summary['compensated']['per_joint_max'][joint]
        reduction = 100.0 * (1.0 - after / before) if before > 1e-9 else float('nan')
        print(' J%d  %.4f -> %.4f  (%.0f%% reduction)' % (joint + 1, before, after, reduction))
    print('overall RMS %.4f -> %.4f deg; max %.4f -> %.4f deg' % (
        summary['uncompensated']['overall_rms'], summary['compensated']['overall_rms'],
        summary['uncompensated']['overall_max'], summary['compensated']['overall_max']))
    (HERE / 'compensation_result_20261110.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote compensation_result_20261110.json')


if __name__ == '__main__':
    main()
