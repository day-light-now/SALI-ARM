"""Final quantitative summary for the J5 evidence document (offline only)."""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def main():
    windows = read(HERE / 'static_windows_j5.json')['J5']['windows']
    fit = read(HERE / 'five_axis_payload_fit_result.json')
    probe = read(HERE / 'j5_payload_geometry_probe.json')

    print('--- fit metrics ---')
    for name in ('none', 'fixed', 'j5'):
        f = fit['fits'][name]
        print(name, 'com_mm', np.round(np.array(f['com_m']) * 1000, 1),
              'train', f.get('train', {}).get('per_axis_rmse_nm'),
              'holdout', f.get('holdout', {}).get('per_axis_rmse_nm'),
              'j5_rmse_per_axis', f['j5_windows'].get('per_axis_rmse_nm'))
    print('train_plus_j5 fixed', fit['fits']['fixed']['train_plus_j5_upstream'])
    print('train_plus_j5 j5   ', fit['fits']['j5']['train_plus_j5_upstream'])

    print('\n--- J5 window upstream range: observed vs rotating-payload prediction ---')
    q5 = [round(w['q_deg'][4], 2) for w in windows]
    obs = np.array([w['effort_mean_nm'] for w in windows])
    pred = np.array(fit['fits']['j5']['j5_predictions_nm'])
    for axis in range(4):
        print(f' J{axis+1}: observed range {obs[:,axis].max()-obs[:,axis].min():+.4f} Nm, '
              f'rotating-payload predicted range {pred[:,axis].max()-pred[:,axis].min():+.4f} Nm')

    print('\n--- J5 effort curve ---')
    for w in windows:
        print(f" q5={w['q_deg'][4]:+8.3f} deg  J5 effort {w['effort_mean_nm'][4]:+7.4f} Nm")

    q5_deg = np.array([w['q_deg'][4] for w in windows])
    eff5 = np.array([w['effort_mean_nm'][4] for w in windows])
    basis = np.stack([np.sin(np.radians(q5_deg)), np.cos(np.radians(q5_deg)),
                      np.ones(len(q5_deg))], axis=1)
    coef, *_ = np.linalg.lstsq(basis, eff5, rcond=None)
    residual = basis @ coef - eff5
    print('  best A*sin(q5)+B*cos(q5)+C fit:', np.round(coef, 3),
          'residual rms', round(float(np.sqrt(np.mean(residual ** 2))), 3),
          'max |residual|', round(float(np.max(np.abs(residual))), 3))
    print('  near-zero pair difference (q5=+0.284 vs -0.284):',
          round(float(eff5[0] - eff5[5]), 3), 'Nm')
    j2 = np.array([w['effort_mean_nm'][1] for w in windows])
    print('  J2 effort values', np.round(j2, 5).tolist(),
          ' -> quanta at 200/4096 Nm:', np.round(j2 / (200. / 4096.), 2).tolist())

    print('\n--- audit kp for the run ---')
    audit = records(HERE / 'j5_excitation_cycles_20261109.jsonl.gz')
    session = next(r for r in audit if r['kind'] == 'SESSION')
    for key in ('kp', 'kp_axis', 'kp_axis_source', 'kd', 'kd_axis', 'kd_axis_source',
                'kp_schedule', 'kp_schedule_source', 'kp_schedule_slew_kp_per_s',
                'following_error_deg', 'gravity_ff_alpha', 'reference_speed_rad_s',
                'j5_motion_qualified', 'active_axes'):
        print(' ', key, '=', session.get(key))
    cycles = [r for r in audit if r['kind'] == 'CONTROL_CYCLE'
              and r.get('request_estimate_state') == 'CURRENT']
    print('  current cycles', len(cycles))
    print('  example kp_scaled', cycles[len(cycles) // 2]['kp_scaled'])
    print('  example kp_encoded', cycles[len(cycles) // 2]['kp_encoded'])
    print('  example target', cycles[len(cycles) // 2]['target'])

    # Align audit to recorder via plan_offset + planned arrival.
    at = np.array([r['steady_s'] for r in cycles])
    plan = read(ROOT / 'verification/hardware/five_axis_20261107/acquisition_20261109/plans/J5_EXCITATION_ZERO.json')['waypoints']
    arrivals = np.cumsum([0.] + [w['move_s'] + w['dwell_s'] for w in plan])[:-1] + np.array(plan[0]['move_s'])
    arrivals = np.array([w['arrival_s'] if 'arrival_s' in w else 0 for w in plan])
    # plan waypoint arrival times: arrival = previous end + current move; recompute from client.
    client = read(ROOT / 'verification/hardware/five_axis_20261107/acquisition_20261109/clients/j5_excitation/20261109_141027_321d3798.json')
    arrivals = np.array([w['arrival_s'] for w in client['waypoint_results']])
    plan_offset = float(windows[0]['trajectory_start_offset_s'])
    print('\n  plan trajectory start offset (recorder s)', round(plan_offset, 3))
    print('  recorder predicted arrivals', np.round(plan_offset + arrivals, 2))
    # audit plateau starts as detected previously
    targets = np.array([r['target'] for r in cycles])
    cursor = -np.inf
    audit_starts = []
    for i, waypoint in enumerate(client['waypoint_results']):
        target = np.array(waypoint['target_rad'])
        mask = np.max(np.abs(targets - target), axis=1) < 1e-9
        hits = np.flatnonzero(mask)
        groups = np.split(hits, np.flatnonzero(np.diff(hits) > 1) + 1) if len(hits) else []
        groups = [g for g in groups if len(g) >= 50 and at[g[0]] > cursor]
        if groups:
            audit_starts.append(float(at[groups[0][0]]))
            cursor = at[groups[0][-1]]
        else:
            audit_starts.append(None)
    print('  audit plateau starts', [None if v is None else round(v, 2) for v in audit_starts])
    deltas = []
    for i, a in enumerate(audit_starts):
        if a is None:
            continue
        deltas.append((plan_offset + arrivals[i]) - a)
    print('  recorder-audit delta', [round(d, 2) for d in deltas])
    delta = float(np.median(deltas))
    kp_enc = np.array([r['kp_encoded'] for r in cycles], dtype=float)
    kp_scaled = np.array([r['kp_scaled'] for r in cycles], dtype=float)
    request = np.array([r['request_estimate_nm'] for r in cycles])
    signs = np.array([j['sign'] for j in session['joint_policy']])
    print('\n  per-window audit evidence (align delta', round(delta, 2), 's):')
    for w, arrival in zip(windows, arrivals):
        a0 = plan_offset + w['recorder_window_s'][0] - delta
        a1 = plan_offset + w['recorder_window_s'][1] - delta
        rows = np.flatnonzero((at >= a0) & (at <= a1))
        if not len(rows):
            print('   window', w['id'], 'no audit rows')
            continue
        q = np.array(w['q_rad'])
        target = np.array(w['target_rad'])
        err_model = target - q
        kp_mean = np.round(np.nanmean(kp_enc[rows], axis=0), 2)
        kp_expected = np.round(kp_mean * err_model * signs, 4)
        print(f"   w{w['id']}: kp_encoded={kp_mean.tolist()} kp_scaled="
              f"{np.round(np.nanmean(kp_scaled[rows], axis=0), 2).tolist()}")
        print(f"        effort {np.round(w['effort_mean_nm'], 4).tolist()}")
        print(f"        kp*err(model) {kp_expected.tolist()}   "
              f"audit req(model) {np.round(np.mean(request[rows], axis=0) * signs, 4).tolist()}")


if __name__ == '__main__':
    main()
