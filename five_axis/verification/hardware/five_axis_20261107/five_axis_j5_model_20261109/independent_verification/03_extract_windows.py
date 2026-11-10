"""Independent extraction of the six static dwell windows (J5 excitation, 2026-11-09).

READ-ONLY on all data. Offline; no hardware / ROS / driver access.

Rules applied (independently coded):
  * plateau = controller_state rows where reference_positions exactly equal the
    planned waypoint target (all 5 axes) and reference_velocities are all zero,
    contiguous in recorder time (gap <= 40 ms), lasting >= 0.6 s;
  * take the last 2 s of each plan dwell, clipped to the plateau;
  * a recorder sample is "fresh stationary active" only if the nearest dynamic
    row (<= 25 ms) satisfies, on *every* axis: command_id == run command_id,
    motor_state == 1, control_state in {5,6}, feedback_valid == 1, audit_ok == 1,
    configuration_conflict == 0, feedback_age <= 0.06, timeout_count == 2000,
    |velocity| <= 0.035 rad/s.
Writes only into independent_verification/.
"""
import gzip
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
BASE = ROOT / 'verification/hardware/five_axis_20261107'
RAW = BASE / 'acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz'
CLIENT = BASE / 'acquisition_20261109/clients/j5_excitation/20261109_141027_321d3798.json'
OUT = BASE / 'five_axis_j5_model_20261109/independent_verification'

NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_records(path):
    cs, dyn = [], []
    with gzip.open(path, 'rt', encoding='utf-8') as f:
        for line in f:
            row = json.loads(line)
            if row['kind'] == 'controller_state':
                cs.append(row)
            elif row['kind'] == 'dynamic':
                dyn.append(row)
    return cs, dyn


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[n] for n in NAMES], float)


def contiguous(times, mask, gap=0.04):
    hits = np.flatnonzero(mask)
    if not len(hits):
        return []
    cut = np.flatnonzero((np.diff(hits) > 1) | (np.diff(times[hits]) > gap)) + 1
    return np.split(hits, cut)


def main():
    client = read_json(BASE / 'acquisition_20261109/clients/j5_excitation/20261109_141027_321d3798.json')
    command_id = client['command_id']
    targets = [np.array(w['target_rad'], float) for w in client['waypoint_results']]
    cs, dyn = load_records(RAW)
    print(f'loaded controller_state={len(cs)} dynamic={len(dyn)} command_id={command_id}')

    tc = np.array([r['t'] for r in cs], float)
    ref = np.array([ordered(r['payload'], 'reference_positions') for r in cs], float)
    fb = np.array([ordered(r['payload'], 'feedback_positions') for r in cs], float)
    rv = np.array([ordered(r['payload'], 'reference_velocities') for r in cs], float)
    td = np.array([r['t'] for r in dyn], float)

    # ---- exact-target plateaus, in time order ("first match after previous") ----
    groups, cursor = [], -math.inf
    for target in targets:
        mask = (np.max(np.abs(ref - target), axis=1) < 1e-9) & (np.max(np.abs(rv), axis=1) < 1e-9)
        hits = [g for g in contiguous(tc, mask) if tc[g[-1]] - tc[g[0]] >= 0.6 and tc[g[0]] > cursor]
        if not hits:
            groups.append(None)
            continue
        groups.append(hits[0])
        cursor = tc[hits[0][-1]]

    for i, g in enumerate(groups):
        if g is None:
            print(f'waypoint {i}: NO exact-target plateau found')
        else:
            print(f'waypoint {i}: plateau t=[{tc[g[0]]:.3f},{tc[g[-1]]:.3f}] '
                  f'dur={tc[g[-1]]-tc[g[0]]:.3f}s n={len(g)}')

    # ---- plan trajectory start offset from plateau arrival vs arrival_s ----
    arrivals = np.array([tc[g[0]] for g in groups])
    arr_plan = np.array([w['arrival_s'] for w in client['waypoint_results']])
    offsets = arrivals - arr_plan
    plan_offset = float(np.median(offsets))
    print('offset per waypoint (arrival - arrival_s):', np.round(offsets, 4).tolist())
    print('median plan offset:', round(plan_offset, 4))
    # cross-check: first change of reference from its initial value
    t0 = tc[np.argmax(np.any(np.abs(ref - ref[0]) > 1e-12, axis=1))]
    print('first reference change at t=', round(float(t0), 4), ' (plan offset estimate)', round(float(t0), 4))

    # ---- dynamic rows, nearest-match within 25 ms ----
    windows = []
    for i, g in enumerate(groups):
        if g is None:
            windows.append({'id': i, 'excluded': 'no exact-target plateau'})
            continue
        wp = client['waypoint_results'][i]
        dwell_lo = plan_offset + wp['arrival_s']
        dwell_hi = plan_offset + wp['end_s']
        end = min(dwell_hi - 0.15, tc[g[-1]] - 0.05)
        start = max(end - 2.0, dwell_lo + 0.5, tc[g[0]] + 0.1)
        sel = g[(tc[g] >= start) & (tc[g] <= end)]
        accepted, dyn_rows = [], []
        for k in sel:
            j = int(np.searchsorted(td, tc[k]))
            cand = [n for n in (j - 1, j, j + 1) if 0 <= n < len(td)]
            j = min(cand, key=lambda n: abs(td[n] - tc[k]))
            if abs(td[j] - tc[k]) > 0.025:
                continue
            axes = dyn[j]['payload']['interfaces']
            ok = all(
                axes[n]['command_id'] == command_id and axes[n]['motor_state'] == 1
                and axes[n]['control_state'] in (5, 6) and axes[n]['feedback_valid'] == 1
                and axes[n]['audit_ok'] == 1 and axes[n]['configuration_conflict'] == 0
                and axes[n]['feedback_age'] <= 0.06 and axes[n]['timeout_count'] == 2000
                and abs(axes[n]['velocity']) <= 0.035
                for n in NAMES)
            if ok:
                accepted.append(int(k))
                dyn_rows.append(axes)
        if len(accepted) < 50 or tc[accepted[-1]] - tc[accepted[0]] < 0.6:
            windows.append({'id': i, 'excluded': 'insufficient fresh stationary active samples',
                            'n_candidate': len(sel), 'n_accepted': len(accepted)})
            continue
        acc = np.array(accepted)
        q = fb[acc]
        effort = np.array([[axes[n]['effort'] for n in NAMES] for axes in dyn_rows])
        vel = np.array([[axes[n]['velocity'] for n in NAMES] for axes in dyn_rows])
        windows.append({
            'id': i, 'target_rad': wp['target_rad'],
            'q_mean_rad': q.mean(axis=0).tolist(),
            'q_mean_deg': np.degrees(q.mean(axis=0)).tolist(),
            'q_std_deg': np.degrees(q.std(axis=0)).tolist(),
            'q_span_deg': np.degrees(np.ptp(q, axis=0)).tolist(),
            'q_first_deg': np.degrees(q[0]).tolist(),
            'q_last_deg': np.degrees(q[-1]).tolist(),
            'effort_mean_nm': effort.mean(axis=0).tolist(),
            'effort_std_nm': effort.std(axis=0).tolist(),
            'effort_first_nm': effort[0].tolist(),
            'effort_last_nm': effort[-1].tolist(),
            'effort_ptp_nm': np.ptp(effort, axis=0).tolist(),
            'velocity_mean_rad_s': vel.mean(axis=0).tolist(),
            'n_samples': int(len(acc)),
            'recorder_window_s': [float(tc[acc[0]]), float(tc[acc[-1]])],
            'plateau_s': [float(tc[g[0]]), float(tc[g[-1]])],
        })

    result = {
        'source_raw': str(RAW),
        'command_id': command_id,
        'plan_offset_s': plan_offset,
        'plan_offset_per_waypoint_s': offsets.tolist(),
        'windows': windows,
    }
    outfile = OUT / 'windows_extracted.json'
    outfile.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print()
    for w in windows:
        if 'excluded' in w:
            print(f"w{w['id']}: EXCLUDED {w['excluded']} candidates={w.get('n_candidate')} "
                  f"accepted={w.get('n_accepted')}")
            continue
        q = ' '.join(f'{v:+8.3f}' for v in w['q_mean_deg'])
        e = ' '.join(f'{v:+9.4f}' for v in w['effort_mean_nm'])
        es = ' '.join(f'{v:8.4f}' for v in w['effort_std_nm'])
        print(f"w{w['id']}: q[{q}] deg  effort[{e}] Nm  std[{es}] n={w['n_samples']} "
              f"win=[{w['recorder_window_s'][0]:.2f},{w['recorder_window_s'][1]:.2f}]")
    print('written', outfile)


if __name__ == '__main__':
    main()
