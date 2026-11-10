"""Summarize the hardware test suite once its runs exist.

Reads test_suite_manifest_20261110.json and windows_<name>.json; writes a result
JSON with per-joint RMS/max sag, the H1 baseline-vs-compensated comparison and
the H2 repeatability spread; checks the manifest thresholds.
"""
import json
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / 'test_suite_manifest_20261110.json'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_windows(name):
    path = HERE / ('windows_%s.json' % name)
    if not path.is_file():
        return None
    data = read(path)
    rows = sorted(data['windows'], key=lambda w: w['index'])
    valid = [w for w in rows if 'sag_deg' in w]
    indices = [w['index'] for w in valid]
    sag = np.array([w['sag_deg'] for w in valid], dtype=float)
    return {'indices': indices, 'sag': sag, 'count': len(valid)}


def stats(matrix):
    if not len(matrix):
        return None
    return {'rms_deg': np.sqrt(np.mean(matrix ** 2, axis=0)).round(4).tolist(),
            'max_deg': np.max(np.abs(matrix), axis=0).round(4).tolist(),
            'overall_rms_deg': round(float(np.sqrt(np.mean(matrix ** 2))), 4),
            'overall_max_deg': round(float(np.max(np.abs(matrix))), 4),
            'samples': int(matrix.shape[0])}


def interior(sag, indices, total):
    return np.array([row for row, index in zip(sag, indices) if index not in (0, total - 1)])


def main():
    manifest = read(MANIFEST)
    thresholds = manifest['thresholds']
    entries = {entry['name']: entry for entry in manifest['entries']}
    result = {'schema': 'slaihand_test_suite_result/1',
              'created': time.strftime('%Y-%m-%dT%H:%M:%S'),
              'thresholds': thresholds, 'entries': {}}
    data = {}
    for entry in manifest['entries']:
        windows = load_windows(entry['name'])
        if windows is None:
            result['entries'][entry['name']] = {'present': False}
            continue
        matrix = interior(windows['sag'], windows['indices'], entry['waypoints'])
        data[entry['name']] = matrix
        result['entries'][entry['name']] = {'present': True, 'stats': stats(matrix)}

    checks = []
    baseline = data.get('H1_holdout')
    compensated = data.get('H1_holdout_comp')
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

    repeat = data.get('H2_repeat')
    if repeat is not None and len(repeat) >= 2:
        # H2 waypoints: 0 zero, 1 up, 2 pose, 3 down, 4 pose, 5 zero.
        pose_matrix = repeat[[index for index in range(len(repeat))
                              if index in (2, 4)]]
        if len(pose_matrix) == 2:
            spread = np.max(np.abs(pose_matrix[0] - pose_matrix[1]))
            checks.append({'name': 'H2 same-pose spread <= threshold',
                           'value': round(float(spread), 4),
                           'threshold': thresholds['H2_repeat_spread_deg_max'],
                           'ok': float(spread) <= thresholds['H2_repeat_spread_deg_max']})
            result['H2_spread_deg'] = round(float(spread), 4)

    stiction = data.get('H4_stiction')
    if stiction is not None and len(stiction) >= 3:
        # H4 waypoints: 0 zero, 1 up, 2 pose, 3 down, 4 pose, 5 side, 6 pose, 7 zero.
        pose_matrix = stiction[[index for index in range(len(stiction))
                                if index in (2, 4, 6)]]
        if len(pose_matrix) == 3:
            spread = float(np.max(np.max(pose_matrix, axis=0) - np.min(pose_matrix, axis=0)))
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
