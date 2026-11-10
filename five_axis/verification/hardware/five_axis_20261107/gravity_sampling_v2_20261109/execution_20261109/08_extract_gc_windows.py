"""Extract GC05-GC08 static windows and direction-paired friction estimates.

Offline only. For each successful acquisition:
  * locate each plan waypoint plateau from the recorded controller states;
  * keep the last 2 s of each dwell with fresh stationary active dynamic feedback;
  * report per-window mean feedback pose and effort;
  * for the plan's paired J5 targets, report the up/down half-difference, which
    is the first-order stiction bias, and the pair mean, which cancels it.

Writes gc_windows_20261109.json next to this script.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
PLANS = HERE.parent
TAGS = {
    'gc05': 'GC05_J5_ZERO_PAIRED_ZERO.json',
    'gc06': 'GC06_J5_J2_20_PAIRED_ZERO.json',
    'gc07': 'GC07_J5_J3_J4_20_PAIRED_ZERO.json',
    'gc08': 'GC08_J5_MIXED_HOLDOUT_ZERO.json',
}
PAIRS = {
    'gc05': [(1, 3, 10.0), (5, 7, -10.0), (4, 8, 0.0)],
    'gc06': [(2, 4, 10.0), (6, 8, -10.0), (5, 9, 0.0)],
    'gc07': [(2, 4, 10.0), (6, 8, -10.0), (5, 9, 0.0)],
    'gc08': [(2, 4, 10.0), (6, 8, -10.0), (5, 9, 0.0)],
}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[name] for name in NAMES], dtype=float)


def contiguous(times, mask):
    hits = np.flatnonzero(mask)
    if not len(hits):
        return []
    return np.split(hits, np.flatnonzero((np.diff(hits) > 1) | (np.diff(times[hits]) > .04)) + 1)


def pick_client(tag):
    directory = HERE / 'clients' / tag
    for path in sorted(directory.glob('*.json')):
        result = read(path)
        if result.get('mode') == 'waypoints' and result.get('trajectory_error_code') == 0:
            return path, result
    raise ValueError(f'no successful waypoint client result for {tag}')


def extract(tag, window_s=2.0):
    client_path, client = pick_client(tag)
    raw = records(HERE / 'raw' / f'{tag}_20261109.jsonl.gz')
    controller = []
    for row in raw:
        if row['kind'] != 'controller_state':
            continue
        try:
            for field in ('reference_positions', 'feedback_positions', 'reference_velocities'):
                ordered(row['payload'], field)
        except (KeyError, TypeError, ValueError):
            continue
        controller.append(row)
    dynamic = [row for row in raw if row['kind'] == 'dynamic']
    t = np.array([row['t'] for row in controller])
    dt = np.array([row['t'] for row in dynamic])
    refs = np.array([ordered(row['payload'], 'reference_positions') for row in controller])
    fb = np.array([ordered(row['payload'], 'feedback_positions') for row in controller])
    rv = np.array([ordered(row['payload'], 'reference_velocities') for row in controller])

    groups, offsets = [], []
    cursor = -np.inf
    for index, waypoint in enumerate(client['waypoint_results']):
        target = np.array(waypoint['target_rad'])
        mask = (np.max(np.abs(refs - target), axis=1) < 1e-7) & (np.max(np.abs(rv), axis=1) < 1e-7)
        hits = [g for g in contiguous(t, mask) if t[g[-1]] - t[g[0]] >= .6 and t[g[0]] > cursor]
        if not waypoint.get('samples') or not hits:
            groups.append(None)
            continue
        group = hits[0]
        cursor = t[group[-1]]
        groups.append(group)
        if index > 0:
            offsets.append(t[group[0]] - waypoint['arrival_s'])
    plan_offset = float(np.median(offsets))

    windows = []
    for index, waypoint in enumerate(client['waypoint_results']):
        group = groups[index]
        if group is None:
            windows.append({'index': index, 'excluded': 'no exact-target dwell'})
            continue
        end = min(plan_offset + waypoint['end_s'] - .15, t[group[-1]] - .05)
        start = max(end - window_s, plan_offset + waypoint['arrival_s'] + .5, t[group[0]] + .1)
        accepted, dynamic_rows = [], []
        for k in group[(t[group] >= start) & (t[group] <= end)]:
            j = int(np.searchsorted(dt, t[k]))
            j = min((max(0, j - 1), min(j, len(dt) - 1)), key=lambda n: abs(dt[n] - t[k]))
            if abs(dt[j] - t[k]) > .025:
                continue
            axes = dynamic[j]['payload']['interfaces']
            if all(axes[name][field] == expected for name, field, expected in (
                    (name, 'command_id', client['command_id']) for name in NAMES)):
                pass
            ok = True
            for name in NAMES:
                value = axes[name]
                if (value['command_id'] != client['command_id'] or value['motor_state'] != 1
                        or value['control_state'] not in (5, 6) or value['feedback_valid'] != 1
                        or value['audit_ok'] != 1 or value['configuration_conflict'] != 0
                        or value['feedback_age'] > .06 or value['timeout_count'] != 2000
                        or abs(value['velocity']) > .035):
                    ok = False
                    break
            if ok:
                accepted.append(k)
                dynamic_rows.append(axes)
        if len(accepted) < 50 or t[accepted[-1]] - t[accepted[0]] < .6:
            windows.append({'index': index, 'excluded': 'insufficient fresh stationary active samples',
                            'samples': len(accepted)})
            continue
        q = fb[accepted]
        effort = np.array([[axes[name]['effort'] for name in NAMES] for axes in dynamic_rows])
        windows.append({
            'index': index,
            'target_rad': waypoint['target_rad'],
            'q_rad': q.mean(axis=0).tolist(),
            'q_deg': np.degrees(q.mean(axis=0)).tolist(),
            'q_span_deg': np.degrees(np.ptp(q, axis=0)).tolist(),
            'effort_mean_nm': effort.mean(axis=0).tolist(),
            'effort_std_nm': effort.std(axis=0).tolist(),
            'samples': len(accepted),
            'recorder_window_s': [float(t[accepted[0]]), float(t[accepted[-1]])],
        })

    pairing = []
    for up, down, target in PAIRS[tag]:
        a, b = windows[up], windows[down]
        if 'excluded' in a or 'excluded' in b:
            pairing.append({'target_j5_deg': target, 'excluded': True})
            continue
        ea, eb = np.array(a['effort_mean_nm']), np.array(b['effort_mean_nm'])
        qa, qb = np.array(a['q_deg']), np.array(b['q_deg'])
        pairing.append({
            'target_j5_deg': target,
            'up_window': up, 'down_window': down,
            'up_q5_deg': float(qa[4]), 'down_q5_deg': float(qb[4]),
            'up_effort_nm': ea.tolist(), 'down_effort_nm': eb.tolist(),
            'effort_difference_nm': (ea - eb).tolist(),
            'effort_half_difference_nm': ((ea - eb) / 2.0).tolist(),
            'effort_pair_mean_nm': ((ea + eb) / 2.0).tolist(),
            'sag_difference_deg': (qa - qb).tolist(),
        })
    return {
        'tag': tag,
        'client_file': str(client_path),
        'client_command_id': client['command_id'],
        'plan_offset_s': plan_offset,
        'windows': windows,
        'pairing': pairing,
    }


def main():
    result = {'schema': 'slaihand_gc05_08_static_windows/1', 'plans': []}
    for tag in TAGS:
        result['plans'].append(extract(tag))
    out = HERE / 'gc_windows_20261109.json'
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for plan in result['plans']:
        print('=' * 100)
        print(plan['tag'], 'command', plan['client_command_id'], 'plan_offset', round(plan['plan_offset_s'], 2))
        for window in plan['windows']:
            if 'excluded' in window:
                print(f"  w{window['index']}: EXCLUDED {window['excluded']}")
                continue
            print(f"  w{window['index']}: q5={window['q_deg'][4]:+8.3f}  effort " +
                  ' '.join(f'{v:+7.3f}' for v in window['effort_mean_nm']) +
                  f"  n={window['samples']}")
        for pair in plan['pairing']:
            if pair.get('excluded'):
                print(f"  pair {pair['target_j5_deg']}: excluded")
                continue
            print(f"  pair {pair['target_j5_deg']:+5.1f}: half-diff " +
                  ' '.join(f'{v:+7.3f}' for v in pair['effort_half_difference_nm']) +
                  f"  q5 {pair['up_q5_deg']:+.2f}/{pair['down_q5_deg']:+.2f}")
    print('written', out)


if __name__ == '__main__':
    main()
