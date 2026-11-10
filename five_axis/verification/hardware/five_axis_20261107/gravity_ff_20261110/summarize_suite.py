"""Summarize the hardware test suite once its runs exist.

Residuals are always measured against the ORIGINAL desired pose:
  - uncompensated run: mean_fb - desired (sag_deg is the same);
  - model-compensated run: mean_fb - original desired (NOT mean_fb - command).
H2/H4 same-pose spread uses sag_deg (feedback minus the identical command).
"""
import json
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / 'test_suite_manifest_20261110.json'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def plan_degrees(name):
    plan = read(HERE / 'plans' / ('%s_20261110.json' % name))
    return np.array([w['degrees'] for w in plan['waypoints']], dtype=float)


def load_windows(name):
    path = HERE / ('windows_%s.json' % name)
    if not path.is_file():
        return None
    data = read(path)
    rows = sorted([w for w in data['windows'] if 'sag_deg' in w], key=lambda w: w['index'])
    return {'indices': np.array([w['index'] for w in rows], dtype=int),
            'desired': np.array([w['desired_deg'] for w in rows], dtype=float),
            'mean_fb': np.array([w['mean_fb_deg'] for w in rows], dtype=float),
            'sag': np.array([w['sag_deg'] for w in rows], dtype=float),
            'count': len(rows)}


def residual_vs_original(run_name, original_plan_name):
    windows = load_windows(run_name)
    if windows is None:
        return None
    original = plan_degrees(original_plan_name)
    residual = windows['mean_fb'] - original[windows['indices']]
    total = len(original)
    keep = [i for i, index in enumerate(windows['indices']) if index not in (0, total - 1)]
    return residual[keep]


def stats(matrix):
    if matrix is None or not len(matrix):
        return None
    return {'rms_deg': np.sqrt(np.mean(matrix ** 2, axis=0)).round(4).tolist(),
            'max_deg': np.max(np.abs(matrix), axis=0).round(4).tolist(),
            'overall_rms_deg': round(float(np.sqrt(np.mean(matrix ** 2))), 4),
            'overall_max_deg': round(float(np.max(np.abs(matrix))), 4),
            'samples': int(matrix.shape[0])}


def main():
    manifest = read(MANIFEST)
    thresholds = manifest['thresholds']
    result = {'schema': 'slaihand_test_suite_result/2',
              'residual_definition': 'mean_fb_deg - original_desired_deg',
              'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
              'thresholds': thresholds, 'entries': {}}
    for entry in manifest['entries']:
        windows = load_windows(entry['name'])
        if windows is None:
            result['entries'][entry['name']] = {'present': False}
            continue
        matrix = residual_vs_original(entry['name'], entry['name'].replace('_comp', ''))
        result['entries'][entry['name']] = {'present': True, 'stats': stats(matrix),
                                            'sag_stats': stats(windows['sag'])}

    checks = []
    baseline = residual_vs_original('H1_holdout', 'H1_holdout')
    compensated = residual_vs_original('H1_holdout_comp', 'H1_holdout')
    if baseline is not None and compensated is not None:
        base_rms = float(np.sqrt(np.mean(baseline ** 2)))
        comp_rms = float(np.sqrt(np.mean(compensated ** 2)))
        checks.append({'name': 'H1 baseline sag RMS >= threshold',
                       'value': round(base_rms, 4),
                       'threshold': thresholds['H1_baseline_sag_rms_deg_min'],
                       'ok': base_rms >= thresholds['H1_baseline_sag_rms_deg_min']})
        checks.append({'name': 'H1 compensated residual RMS <= threshold',
                       'value': round(comp_rms, 4),
                       'threshold': thresholds['H1_comp_residual_rms_deg_max'],
                       'ok': comp_rms <= thresholds['H1_comp_residual_rms_deg_max']})
        result['H1_comparison'] = {'baseline_rms_deg': round(base_rms, 4),
                                   'compensated_rms_deg': round(comp_rms, 4),
                                   'improvement_percent': round(
                                       (1.0 - comp_rms / base_rms) * 100.0, 1)}

    repeat = load_windows('H2_repeat')
    if repeat is not None:
        # H2 waypoints: 0 zero, 1 up, 2 pose, 3 down, 4 pose, 5 zero.
        positions = [i for i, index in enumerate(repeat['indices']) if index in (2, 4)]
        if len(positions) >= 2:
            spread = float(np.max(np.abs(repeat['sag'][positions[0]] - repeat['sag'][positions[1]])))
            checks.append({'name': 'H2 same-pose spread <= threshold',
                           'value': round(spread, 4),
                           'threshold': thresholds['H2_repeat_spread_deg_max'],
                           'ok': spread <= thresholds['H2_repeat_spread_deg_max']})
            result['H2_spread_deg'] = round(spread, 4)

    stiction = load_windows('H4_stiction')
    if stiction is not None:
        positions = [i for i, index in enumerate(stiction['indices']) if index in (2, 4, 6)]
        if len(positions) >= 2:
            matrix = stiction['sag'][positions]
            spread = float(np.max(np.max(matrix, axis=0) - np.min(matrix, axis=0)))
            checks.append({'name': 'H4 stiction same-pose spread <= threshold',
                           'value': round(spread, 4),
                           'threshold': thresholds['H4_stiction_spread_deg_max'],
                           'ok': spread <= thresholds['H4_stiction_spread_deg_max']})
            result['H4_stiction_spread_deg'] = round(spread, 4)

    result['checks'] = checks
    result['all_checks_ok'] = all(check['ok'] for check in checks) if checks else None
    out = HERE / ('test_suite_result_%s.json' % time.strftime('%Y%m%d_%H%M%S'))
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print('wrote', out)


if __name__ == '__main__':
    main()
