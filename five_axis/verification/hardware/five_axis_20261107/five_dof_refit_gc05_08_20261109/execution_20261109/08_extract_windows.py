"""Extract GC09+ static windows and upstream direction/repeatability estimates.

Offline only. For each successful acquisition:
  * locate every plan waypoint plateau from the recorded controller states;
  * keep the last 2 s of each dwell with fresh stationary active feedback;
  * report per-window mean feedback pose and effort;
  * for the plan's paired targets report the up/down half-difference (first
    order stiction) and pair mean (cancels it), plus repeatability between the
    two identical pairs.

Writes gc_windows_20261109.json next to this script.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
# tag -> (axis index 0-based, [(up waypoint, down waypoint, target deg), ...])
PAIRS = {
    'gc09': (0, [(1, 3, 10.0), (5, 7, 10.0)]),
    'gc10': (3, [(1, 3, 10.0), (5, 7, 10.0)]),
    'gc11': (1, [(1, 3, 10.0), (5, 7, 10.0)]),
    'gc12': (2, [(1, 3, 10.0), (5, 7, 10.0)]),
    'gc13': (4, [(2, 4, 10.0), (6, 8, -10.0), (5, 9, 0.0)]),
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
    paths = sorted((HERE / 'clients' / tag).glob('*.json'))
    results = [read(path) for path in paths]
    completed = [(path, result) for path, result in zip(paths, results)
                 if result.get('mode') == 'waypoints'
                 and result.get('trajectory_error_code') == 0]
    if completed:
        return completed[-1]
    started = [(path, result) for path, result in zip(paths, results)
               if result.get('mode') == 'waypoints' and result.get('trajectory_started')]
    if started:
        return started[-1]
    raise ValueError(f'no waypoint client result for {tag}')


def extract(tag, window_s=2.0):
    client_path, client = pick_client(tag)
    if not client.get('waypoint_results'):
        return {'tag': tag, 'client_file': str(client_path),
                'client_command_id': client.get('command_id'),
                'client_aborted': True, 'excluded': 'session aborted before trajectory'}
    client = client
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

    axis, pair_spec = PAIRS[tag]
    pairing = []
    for up, down, target in pair_spec:
        a, b = windows[up], windows[down]
        if 'excluded' in a or 'excluded' in b:
            pairing.append({'target_deg': target, 'excluded': True})
            continue
        ea, eb = np.array(a['effort_mean_nm']), np.array(b['effort_mean_nm'])
        qa, qb = np.array(a['q_deg']), np.array(b['q_deg'])
        pairing.append({
            'axis_one_based': axis + 1,
            'target_deg': target,
            'up_window': up, 'down_window': down,
            'up_q_deg': float(qa[axis]), 'down_q_deg': float(qb[axis]),
            'up_effort_nm': ea.tolist(), 'down_effort_nm': eb.tolist(),
            'effort_difference_nm': (ea - eb).tolist(),
            'effort_half_difference_nm': ((ea - eb) / 2.0).tolist(),
            'effort_pair_mean_nm': ((ea + eb) / 2.0).tolist(),
            'sag_difference_deg': (qa - qb).tolist(),
        })
    # repeatability between the two identical pairs, on the moving axis
    repeat = None
    if len(pairing) == 2 and len(pair_spec) == 2 and not any(p.get('excluded') for p in pairing):
        first, second = pairing
        repeat = {
            'axis_one_based': axis + 1,
            'half_difference_first_nm': first['effort_half_difference_nm'][axis],
            'half_difference_second_nm': second['effort_half_difference_nm'][axis],
            'half_difference_repeat_diff_nm': (second['effort_half_difference_nm'][axis]
                                               - first['effort_half_difference_nm'][axis]),
            'pair_mean_first_nm': first['effort_pair_mean_nm'][axis],
            'pair_mean_second_nm': second['effort_pair_mean_nm'][axis],
            'pair_mean_repeat_diff_nm': (second['effort_pair_mean_nm'][axis]
                                         - first['effort_pair_mean_nm'][axis]),
            'up_effort_first_nm': first['up_effort_nm'][axis],
            'up_effort_second_nm': second['up_effort_nm'][axis],
            'up_effort_repeat_diff_nm': (second['up_effort_nm'][axis]
                                         - first['up_effort_nm'][axis]),
            'q_up_first_deg': first['up_q_deg'], 'q_up_second_deg': second['up_q_deg'],
        }
    return {
        'tag': tag, 'client_file': str(client_path),
        'client_command_id': client['command_id'],
        'client_aborted': client.get('trajectory_error_code') != 0,
        'plan_offset_s': plan_offset,
        'moving_axis_zero_based': axis,
        'windows': windows,
        'pairing': pairing,
        'repeatability': repeat,
    }


def main():
    result = {'schema': 'slaihand_gc09_plus_static_windows/1', 'plans': []}
    for tag in sorted(PAIRS):
        if not (HERE / 'raw' / f'{tag}_20261109.jsonl.gz').exists():
            continue
        result['plans'].append(extract(tag))
    out = HERE / 'gc_windows_20261109.json'
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for plan in result['plans']:
        print('=' * 90)
        if plan.get('excluded'):
            print('=' * 90)
            print(plan['tag'], 'EXCLUDED:', plan['excluded'])
            continue
        print(plan['tag'], 'command', plan['client_command_id'],
              'moving axis J%d' % (plan['moving_axis_zero_based'] + 1))
        axis = plan['moving_axis_zero_based']
        for window in plan['windows']:
            if 'excluded' in window:
                print(f"  w{window['index']}: EXCLUDED {window['excluded']}")
                continue
            print(f"  w{window['index']}: q{axis+1}={window['q_deg'][axis]:+8.3f}  "
                  f"effort{axis+1}={window['effort_mean_nm'][axis]:+7.3f}  n={window['samples']}")
        for pair in plan['pairing']:
            if pair.get('excluded'):
                print('  pair excluded')
                continue
            print(f"  pair target {pair['target_deg']:+5.1f}: half-diff(J{axis+1}) "
                  f"{pair['effort_half_difference_nm'][axis]:+7.3f}  pair-mean "
                  f"{pair['effort_pair_mean_nm'][axis]:+7.3f}  "
                  f"q {pair['up_q_deg']:+.3f}/{pair['down_q_deg']:+.3f}")
        repeat = plan['repeatability']
        if repeat:
            print('  repeat: half-diff1 %+.3f  half-diff2 %+.3f  diff %+.3f  '
                  'pair-mean1 %+.3f  pair-mean2 %+.3f  diff %+.3f'
                  % (repeat['half_difference_first_nm'], repeat['half_difference_second_nm'],
                     repeat['half_difference_repeat_diff_nm'],
                     repeat['pair_mean_first_nm'], repeat['pair_mean_second_nm'],
                     repeat['pair_mean_repeat_diff_nm']))
    print('written', out)


if __name__ == '__main__':
    main()
