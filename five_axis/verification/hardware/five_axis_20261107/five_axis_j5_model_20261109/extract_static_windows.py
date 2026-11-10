"""Extract static dwell windows from 2026-11-09 recordings. Offline only.

Uses the same plateau rule as the accepted payload identification: exact
reference target, zero reference velocity, >=0.6 s contiguous dwell; keep the
last 2 s of each plan dwell; require fresh stationary active dynamic feedback.

This module does not judge identifiability; it only reports accepted windows.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / 'verification/hardware/five_axis_20261107'
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[n] for n in NAMES], dtype=float)


def contiguous(times, mask):
    hits = np.flatnonzero(mask)
    if not len(hits):
        return []
    return np.split(hits, np.flatnonzero((np.diff(hits) > 1) | (np.diff(times[hits]) > .04)) + 1)


def extract(state_path, client_path, window_s=2.0):
    client = read(client_path)
    raw = records(state_path)
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

    groups = []
    cursor = -np.inf
    offsets = []
    for i, waypoint in enumerate(client['waypoint_results']):
        target = np.array(waypoint['target_rad'])
        mask = (np.max(np.abs(refs - target), axis=1) < 1e-7) & (np.max(np.abs(rv), axis=1) < 1e-7)
        hits = [g for g in contiguous(t, mask)
                if t[g[-1]] - t[g[0]] >= .6 and t[g[0]] > cursor]
        if not waypoint['samples'] or not hits:
            groups.append(None)
            continue
        group = hits[0]
        cursor = t[group[-1]]
        groups.append(group)
        if i > 0:
            offsets.append(t[group[0]] - waypoint['arrival_s'])
    plan_offset = float(np.median(offsets))

    windows = []
    for i, waypoint in enumerate(client['waypoint_results']):
        group = groups[i]
        if group is None:
            windows.append({'id': i, 'excluded': 'no exact-target dwell'})
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
            windows.append({'id': i, 'excluded': 'insufficient fresh stationary active samples',
                            'samples': len(accepted)})
            continue
        q = fb[accepted]
        effort = np.array([[axes[n]['effort'] for n in NAMES] for axes in dynamic_rows])
        velocity = np.array([[axes[n]['velocity'] for n in NAMES] for axes in dynamic_rows])
        span_deg = np.degrees(np.ptp(q, axis=0))
        windows.append({
            'id': i, 'target_rad': waypoint['target_rad'],
            'q_rad': q.mean(axis=0).tolist(), 'q_deg': np.degrees(q.mean(axis=0)).tolist(),
            'q_std_deg': np.degrees(q.std(axis=0)).tolist(), 'q_span_deg': span_deg.tolist(),
            'effort_mean_nm': effort.mean(axis=0).tolist(),
            'effort_std_nm': effort.std(axis=0).tolist(),
            'velocity_mean_rad_s': velocity.mean(axis=0).tolist(),
            'samples': len(accepted),
            'recorder_window_s': [float(t[accepted[0]]), float(t[accepted[-1]])],
            'trajectory_start_offset_s': plan_offset,
        })
    return {'client_command_id': client['command_id'], 'windows': windows}


SESSIONS = {
    'A': ('acquisition_20261109/raw/ident_a_20261109.jsonl.gz',
          'acquisition_20261109/clients/ident_a_client_20261109_121229.json'),
    'A2': ('acquisition_20261109/raw/ident_a2_20261109.jsonl.gz',
           'acquisition_20261109/clients/ident_a2_client_20261109_121525.json'),
    'B2': ('acquisition_20261109/raw/ident_b2_20261109.jsonl.gz',
           'acquisition_20261109/clients/ident_b2_client_20261109_121946.json'),
    'C': ('acquisition_20261109/raw/ident_c_20261109.jsonl.gz',
          'acquisition_20261109/clients/ident_c_client_20261109_122216.json'),
    'J5': ('acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz',
           'acquisition_20261109/clients/j5_excitation/20261109_141027_321d3798.json'),
}


def main():
    result = {}
    for name, (state, client) in SESSIONS.items():
        result[name] = extract(BASE / state, BASE / client)
    out = BASE / 'five_axis_j5_model_20261109/static_windows_j5.json'
    out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    for session, payload in result.items():
        print('=' * 100)
        print(session, 'command', payload['client_command_id'])
        for window in payload['windows']:
            if 'excluded' in window:
                print(f"  w{window['id']}: EXCLUDED {window['excluded']} samples={window.get('samples')}")
                continue
            q = ' '.join(f'{v:+8.3f}' for v in window['q_deg'])
            e = ' '.join(f'{v:+8.3f}' for v in window['effort_mean_nm'])
            print(f"  w{window['id']}: q[{q}] deg  effort[{e}] Nm  n={window['samples']} "
                  f"span={max(window['q_span_deg']):.2f}deg")
    print('written', out)


if __name__ == '__main__':
    main()
