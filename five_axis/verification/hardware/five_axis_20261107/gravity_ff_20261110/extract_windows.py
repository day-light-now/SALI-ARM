"""Extract settled static windows (pose + effort) from one acquisition."""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[name] for name in NAMES], dtype=float)


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def extract(tag, plan_path, window_s=2.0):
    client_dir = HERE / 'clients' / tag
    clients = []
    for path in sorted(client_dir.glob('*.json')):
        try:
            result = read(path)
        except ValueError:
            continue
        if result.get('mode') == 'waypoints' and result.get('waypoint_results'):
            clients.append((path, result))
    path, client = clients[-1]
    plan = read(plan_path)
    desired = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    raw = records(HERE / 'raw' / f'{tag}_20261110.jsonl.gz')
    controller = []
    for row in raw:
        if row.get('kind') != 'controller_state':
            continue
        try:
            for field in ('reference_positions', 'feedback_positions', 'reference_velocities'):
                ordered(row['payload'], field)
        except (KeyError, TypeError, ValueError):
            continue
        controller.append(row)
    dynamic = [row for row in raw if row.get('kind') == 'dynamic']
    t = np.array([row['t'] for row in controller])
    dt = np.array([row['t'] for row in dynamic])
    refs = np.array([ordered(row['payload'], 'reference_positions') for row in controller])
    fb = np.array([ordered(row['payload'], 'feedback_positions') for row in controller])
    rv = np.array([ordered(row['payload'], 'reference_velocities') for row in controller])
    windows = []
    cursor = -np.inf
    for index, waypoint in enumerate(client['waypoint_results']):
        target = np.array(waypoint['target_rad'], dtype=float)
        mask = (np.max(np.abs(refs - target), axis=1) < 1e-7) & (np.max(np.abs(rv), axis=1) < 1e-7)
        hits = np.flatnonzero(mask)
        if not len(hits):
            windows.append({'index': index, 'excluded': 'no exact-target dwell'})
            continue
        groups = np.split(hits, np.flatnonzero((np.diff(hits) > 1) | (np.diff(t[hits]) > .04)) + 1)
        groups = [g for g in groups if t[g[-1]] - t[g[0]] >= .6 and t[g[0]] > cursor]
        if not groups:
            windows.append({'index': index, 'excluded': 'no plateau'})
            continue
        group = groups[0]
        cursor = t[group[-1]]
        end = t[group[-1]] - .05
        start = max(end - window_s, t[group[0]] + .1)
        selected = group[(t[group] >= start) & (t[group] <= end)]
        mean_fb = np.degrees(fb[selected].mean(axis=0))
        want = desired[index] if index < len(desired) else np.degrees(target)
        effort_mean, effort_std, fresh = None, None, 0
        if len(dynamic) and len(dt):
            rows = []
            for k in selected:
                j = int(np.searchsorted(dt, t[k]))
                j = min((max(0, j - 1), min(j, len(dt) - 1)),
                        key=lambda n: abs(dt[n] - t[k]))
                if abs(dt[j] - t[k]) > .025:
                    continue
                axes = dynamic[j]['payload']['interfaces']
                ok = True
                for name in NAMES:
                    value = axes[name]
                    if (value.get('motor_state') != 1 or value.get('feedback_valid') != 1
                            or value.get('audit_ok') != 1
                            or value.get('configuration_conflict') != 0
                            or value.get('feedback_age', 1.0) > .06
                            or value.get('timeout_count') != 2000
                            or abs(value.get('velocity', 0.0)) > .035):
                        ok = False
                        break
                if ok:
                    rows.append([axes[name]['effort'] for name in NAMES])
            if len(rows) >= 20:
                effort_mean = np.array(rows).mean(axis=0).tolist()
                effort_std = np.array(rows).std(axis=0).tolist()
                fresh = len(rows)
        windows.append({'index': index,
                        'desired_deg': np.round(want, 4).tolist(),
                        'target_deg': np.round(np.degrees(target), 4).tolist(),
                        'mean_fb_deg': np.round(mean_fb, 4).tolist(),
                        'sag_deg': np.round(mean_fb - want, 4).tolist(),
                        'effort_mean_nm': None if effort_mean is None else np.round(effort_mean, 5).tolist(),
                        'effort_std_nm': None if effort_std is None else np.round(effort_std, 5).tolist(),
                        'effort_samples': fresh,
                        'samples': int(len(selected))})
    out = {'tag': tag, 'plan': str(plan_path), 'client_file': str(path),
           'desired_deg': desired.tolist(), 'windows': windows}
    (HERE / f'windows_{tag}.json').write_text(json.dumps(out, ensure_ascii=False, indent=2) + '\n',
                                              encoding='utf-8')
    return out


if __name__ == '__main__':
    tag, plan = sys.argv[1], sys.argv[2]
    result = extract(tag, plan)
    print(tag, 'windows', len(result['windows']), 'client', Path(result['client_file']).name)
    for window in result['windows']:
        if 'sag_deg' in window:
            print(' ', window['index'], 'desired', window['desired_deg'],
                  'sag', window['sag_deg'], 'effort',
                  window['effort_mean_nm'], 'n', window['samples'])
