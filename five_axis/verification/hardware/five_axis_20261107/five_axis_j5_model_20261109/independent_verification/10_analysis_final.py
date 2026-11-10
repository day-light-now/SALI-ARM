"""Independent analysis of the 2026-11-09 J5 excitation run - all six questions.

READ-ONLY on every data source. This script recomputes from:
  windows_extracted.json          (my own extraction; 100 Hz dynamic + controller_state)
  audit_command1_cycles.jsonl.gz  (my own extraction from the 41 MB audit)
  time_series.json                (my own 0.5 s buckets)
Every number printed here is produced by code below, not copied from any report.
"""
import gzip
import json
import math
from pathlib import Path

import numpy as np

HERE = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
            'hardware/five_axis_20261107/five_axis_j5_model_20261109/independent_verification')
ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
SHORT = [f'J{i}' for i in range(1, 6)]
MOVE_S = [0.5, 8.0, 8.0, 8.0, 8.0, 10.0]
ARRIVAL_S = [0.5, 13.5, 27.5, 41.5, 55.5, 71.5]
PLAN_DEG = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 10], [0, 0, 0, 0, -10],
            [0, 0, 0, 0, 20], [0, 0, 0, 0, -20], [0, 0, 0, 0, 0]]
PLAN = np.radians(np.array(PLAN_DEG, float))

out = {}
windows = json.loads((HERE / 'windows_extracted.json').read_text())
assert all('excluded' not in w for w in windows['windows']), 'window extraction failed'
rec_w = windows['windows']

# ---------------------------------------------------------------- audit load
session, cycles = None, []
with gzip.open(HERE / 'audit_command1_cycles.jsonl.gz', 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if 'backend' in r:
            session = r
        elif 'request_estimate_nm' in r:
            cycles.append(r)
cycles.sort(key=lambda r: r['steady_s'])
at = np.array([r['steady_s'] for r in cycles])
targets = np.array([r['target'] for r in cycles], float)
kp_enc = np.array([r['kp_encoded'] for r in cycles], float)
kp_scale = np.array([r['kp_scaled'] for r in cycles], float)
req_motor = np.array([r['request_estimate_nm'] for r in cycles], float)
req_state = np.array([r['request_estimate_state'] for r in cycles])
kp_state = np.array([r['kp_output_state'] for r in cycles])
signs = np.array([j['sign'] for j in session['joint_policy']], float)
out['session'] = {k: session[k] for k in (
    'backend', 'kp', 'kd', 'kp_axis', 'kd_axis', 'kp_axis_source', 'kd_axis_source',
    'kp_schedule_source', 'kp_schedule', 'gravity_ff_alpha', 'gravity_ff_ramp_s',
    'gravity_model_present', 'following_error_deg', 'physical_torque_off_verified',
    'hardware_motion_accepted', 'driver_current_clamp_verified', 'host_total_torque_clamp',
    'j5_motion_qualified', 'mapping_sha256', 'model_sha256')}
out['signs'] = signs.tolist()

# ---- audit target dwell segments: target within 2e-4 rad (0.01 deg) of the plan ----
seg = []
cursor = -np.inf
for i, tgt in enumerate(PLAN):
    close = np.max(np.abs(targets - tgt), axis=1) < 2e-4
    hits = np.flatnonzero(close & (at > cursor))
    if not len(hits):
        seg.append(None)
        continue
    # contiguous run (audit cycles are 100 Hz, so allow gaps up to 0.04 s)
    brk = np.flatnonzero(np.diff(at[hits]) > 0.04)
    runs = np.split(hits, brk + 1)
    runs = [g for g in runs if at[g[-1]] - at[g[0]] > 0.5]
    if not runs:
        seg.append(None)
        continue
    seg.append(runs[0])
    cursor = at[runs[0][-1]]
print('audit dwell segments (n, steady_s range):')
for i, g in enumerate(seg):
    print(f'  w{i}: ' + ('MISSING' if g is None else
          f'n={len(g)} [{at[g[0]]:.3f},{at[g[-1]]:.3f}] dur={at[g[-1]]-at[g[0]]:.3f}s'))
assert all(g is not None for g in seg), 'audit dwell segment missing'

# ---- clock alignment: compare audit dwell start vs recorder plateau start ----
rec_arrival = np.array([w['plateau_s'][0] for w in rec_w])
aud_arrival = np.array([at[g[0]] for g in seg])
offsets = rec_arrival - aud_arrival
off = float(np.median(offsets))
print('\naudit->recorder clock offsets per window:', np.round(offsets, 4).tolist())
print('median offset', round(off, 4), 's, spread', round(float(np.ptp(offsets)), 4), 's')
out['alignment'] = {'recorder_plateau_start_s': rec_arrival.tolist(),
                    'audit_dwell_start_steady_s': aud_arrival.tolist(),
                    'offset_per_window_s': offsets.tolist(),
                    'median_offset_s': off, 'spread_s': float(np.ptp(offsets))}

# ---------------------------------------------------------------- window table
kp_lsb = {}
print('\n================ Q1: six static windows ================')
table = []
for i, w in enumerate(rec_w):
    q = np.array(w['q_mean_rad'])
    eff = np.array(w['effort_mean_nm'])
    tgt = np.array(w['target_rad'])
    g = seg[i]
    rows = np.flatnonzero((at >= w['recorder_window_s'][0] - off) &
                          (at <= w['recorder_window_s'][1] - off))
    rows = [int(r) for r in rows if (g[0] <= r <= g[-1])]
    ok = [r for r in rows if req_state[r] == 'CURRENT' and kp_state[r] == 'WRITTEN_CURRENT']
    kpe = np.mean(kp_enc[ok], axis=0)
    kps = np.mean(kp_scale[ok], axis=0)
    req = np.mean(req_motor[ok], axis=0) * signs
    item = {
        'id': i, 'target_deg': np.degrees(tgt).tolist(),
        'q_deg': np.degrees(q).tolist(), 'samples': w['n_samples'],
        'recorder_window_s': w['recorder_window_s'],
        'effort_mean_nm': eff.tolist(), 'effort_std_nm': w['effort_std_nm'],
        'effort_first_last_nm': [w['effort_first_nm'], w['effort_last_nm']],
        'kp_encoded_mean': kpe.tolist(), 'kp_scaled_mean': kps.tolist(),
        'kd_axis': session['kd_axis'], 'n_audit_cycles': len(ok),
        'error_deg': np.degrees(tgt - q).tolist(),
        'audit_request_model_nm': req.tolist(),
        'effort_minus_audit_request_nm': (eff - req).tolist(),
        'kp_scaled_times_error_nm': (kps * (tgt - q)).tolist(),
        'effort_minus_kp_times_error_nm': (eff - kps * (tgt - q)).tolist(),
    }
    table.append(item)
    print(f"w{i}: target5={np.degrees(tgt[4]):+7.2f} deg n={w['n_samples']} "
          f"win=[{w['recorder_window_s'][0]:.2f},{w['recorder_window_s'][1]:.2f}]")
    print('   q_deg       ' + ' '.join(f'{v:+9.4f}' for v in np.degrees(q)))
    print('   err_deg     ' + ' '.join(f'{v:+9.4f}' for v in np.degrees(tgt - q)))
    print('   effort_Nm   ' + ' '.join(f'{v:+9.4f}' for v in eff))
    print('   kp_encoded  ' + ' '.join(f'{v:9.3f}' for v in kpe))
    print('   Kp*err_Nm   ' + ' '.join(f'{v:+9.4f}' for v in kps * (tgt - q)))
    print('   eff-Kp*err  ' + ' '.join(f'{v:+9.4f}' for v in eff - kps * (tgt - q)))
    print('   audit_req   ' + ' '.join(f'{v:+9.4f}' for v in req))
    print('   eff-audreq  ' + ' '.join(f'{v:+9.4f}' for v in eff - req))
out['windows'] = table

# ---------------------------------------------------------------- quantization
print('\n================ torque / position / velocity quantization ================')
raw = ROOT / ('verification/hardware/five_axis_20261107/acquisition_20261109/raw/'
              'j5_excitation_20261109.jsonl.gz')
dyn = []
with gzip.open(raw, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'dynamic':
            dyn.append(r)


def dyn_field(field):
    return np.array([[r['payload']['interfaces'][n][field] for n in NAMES] for r in dyn])


td = np.array([r['t'] for r in dyn])
eff_all = dyn_field('effort')
pos_all = dyn_field('position')
vel_all = dyn_field('velocity')
quant = {}
for i, n in enumerate(NAMES):
    vals = np.unique(eff_all[:, i])
    d = np.diff(vals)
    d = d[d > 1e-12]
    torque_lsb = float(d.min()) if len(d) else 0.0
    p = np.unique(pos_all[:, i])
    dp = np.diff(p)
    dp = dp[dp > 1e-12]
    pos_lsb = float(dp.min()) if len(dp) else 0.0
    v = np.unique(vel_all[:, i])
    dv = np.diff(v)
    dv = dv[dv > 1e-12]
    vel_lsb = float(dv.min()) if len(dv) else 0.0
    tmax = torque_lsb * 4095 / 2
    pmax = pos_lsb * 65535 / 2
    quant[n] = {'torque_lsb_nm': torque_lsb, 'implied_motor_tmax_nm': tmax,
                'position_lsb_rad': pos_lsb, 'implied_motor_pmax_rad': pmax,
                'velocity_lsb_rad_s': vel_lsb,
                'n_unique_effort': int(len(vals)),
                'effort_values_nm': vals.tolist()}
    print(f'{n}: torque code={torque_lsb:.8f} Nm (=> motor tMax={tmax:.3f} Nm), '
          f'position code={pos_lsb:.8f} rad (=> pMax={pmax:.3f}), vel code={vel_lsb:.8f}, '
          f'n_unique_eff={len(vals)}')
out['quantization'] = quant

# ---------------------------------------------------------------- Q2: w0-w3 upstream
print('\n================ Q2: J1-J4 effort during w0-w3 (q5 +0.28 -> +19.65 deg) ================')
w4 = np.array([table[i]['effort_mean_nm'] for i in range(4)])
print('per-axis window means w0..w3:')
for j in range(4):
    print(f'  {SHORT[j]}: ' + ' '.join(f'{v:+9.4f}' for v in w4[:, j]) +
          f'   range={np.ptp(w4[:, j]):.6f} Nm  = {np.ptp(w4[:, j])/quant[NAMES[j]]["torque_lsb_nm"]:.3f} torque code')
q2vals = np.unique(eff_all[:, 1][(td > rec_w[0]['recorder_window_s'][0]) &
                                 (td < rec_w[3]['recorder_window_s'][1])])
print('J2 effort unique values inside w0-w3:', q2vals.tolist(),
      '-> counts', {float(v): int(np.sum(eff_all[:, 1] == v)) for v in q2vals})
q2w = [np.unique(eff_all[:, 1][(td >= w['recorder_window_s'][0]) &
                              (td <= w['recorder_window_s'][1])]) for w in rec_w]
print('J2 effort unique per window:', [v.tolist() for v in q2w])
out['q2'] = {'window_means_w0_w3': w4.tolist(),
             'ranges_nm': np.ptp(w4, axis=0).tolist(),
             'ranges_codes': (np.ptp(w4, axis=0) /
                              np.array([quant[n]['torque_lsb_nm'] for n in NAMES])).tolist(),
             'j2_unique_in_w0_w3': q2vals.tolist(),
             'j2_unique_per_window': [v.tolist() for v in q2w]}

# ---------------------------------------------------------------- Q3: Kp check
print('\n================ Q3: audit Kp / kd and effort vs Kp*(target-q) ================')
print('SESSION kp_axis', session['kp_axis'], 'kd_axis', session['kd_axis'],
      'kd', session['kd'], 'kp', session['kp'], 'kp_schedule_source', session['kp_schedule_source'])
print('kp_encoded means per window:')
for i, t in enumerate(table):
    print(f'  w{i}: ' + ' '.join(f'{v:8.3f}' for v in t['kp_encoded_mean']) +
          '   kp_scaled: ' + ' '.join(f'{v:8.3f}' for v in t['kp_scaled_mean']))

# slope check: effort change vs Kp * error change between windows, per axis
print('\nwithin-axis slope d(effort)/d(error) over window pairs with |d err|>0.002 rad:')
slopes = {}
for j in range(5):
    e = np.array([table[i]['error_deg'][j] for i in range(6)]) * math.pi / 180
    f = np.array([table[i]['effort_mean_nm'][j] for i in range(6)])
    kp = np.mean([table[i]['kp_scaled_mean'][j] for i in range(6)])
    ps, ratios = [], []
    for a in range(6):
        for b in range(a + 1, 6):
            de = e[b] - e[a]
            if abs(de) < 0.002:
                continue
            ps.append((f[b] - f[a]) / de)
            ratios.append(((f[b] - f[a]) / de) / kp)
    if ps:
        slopes[SHORT[j]] = {'mean_slope_nm_per_rad': float(np.mean(ps)),
                            'std': float(np.std(ps)), 'n_pairs': len(ps),
                            'mean_ratio_to_kp_scaled': float(np.mean(ratios)),
                            'audit_kp_scaled_mean': float(kp),
                            'audit_kp_encoded_mean': float(np.mean([table[i]['kp_encoded_mean'][j] for i in range(6)]))}
        print(f'  {SHORT[j]}: slope={np.mean(ps):8.2f} Nm/rad (std {np.std(ps):.2f}, n={len(ps)}), '
              f'Kp_scaled={kp:7.2f}, ratio={np.mean(ratios):.4f}')
    else:
        slopes[SHORT[j]] = {'mean_slope_nm_per_rad': None, 'n_pairs': 0,
                            'audit_kp_scaled_mean': float(kp)}
        print(f'  {SHORT[j]}: error constant across windows -> no slope')
out['q3'] = {'kp_axis': session['kp_axis'], 'kd_axis': session['kd_axis'],
             'kp': session['kp'], 'kd': session['kd'],
             'kp_encoded_per_window': [t['kp_encoded_mean'] for t in table],
             'kp_scaled_per_window': [t['kp_scaled_mean'] for t in table],
             'slopes': slopes,
             'window_comparison': [
                 {'id': t['id'], 'error_deg': t['error_deg'],
                  'effort_nm': t['effort_mean_nm'],
                  'kp_scaled_times_error_nm': t['kp_scaled_times_error_nm'],
                  'effort_minus_nm': t['effort_minus_kp_times_error_nm']} for t in table]}

# ---------------------------------------------------------------- Q4: J5 curve
print('\n================ Q4: J5 effort vs q5 ================')
q5 = np.array([table[i]['q_deg'][4] for i in range(6)])
t5 = np.array([table[i]['target_deg'][4] for i in range(6)])
e5 = np.array([table[i]['effort_mean_nm'][4] for i in range(6)])
sag5 = q5 - t5                      # feedback - reference
approach = np.array([-1, +1, -1, +1, -1, +1], float)   # sign of J5 motion into the hold
print('window:  q5_deg    target5   sag5(deg)  effort5_Nm  approach_dir')
for i in range(6):
    print(f'  w{i}: {q5[i]:+9.3f} {t5[i]:+9.3f} {sag5[i]:+10.3f} {e5[i]:+11.4f}   '
          f'{"+" if approach[i]>0 else "-"}')
print(f'\nq5=+0.284 (w0) vs q5=-0.284 (w5): effort {e5[0]:+.4f} vs {e5[5]:+.4f} -> '
      f'difference {e5[0]-e5[5]:+.4f} Nm, mean {(e5[0]+e5[5])/2:+.4f}, half-difference {(e5[0]-e5[5])/2:+.4f}')
# single sinusoid fit: tau = A sin q5 + B cos q5 + C
A = np.column_stack([np.sin(np.radians(q5)), np.cos(np.radians(q5)), np.ones(6)])
coef, *_ = np.linalg.lstsq(A, e5, rcond=None)
pred = A @ coef
print('single sinusoid fit tau = %.4f*sin(q5) + %.4f*cos(q5) + %.4f, rms residual %.4f Nm'
      % (coef[0], coef[1], coef[2], np.sqrt(np.mean((e5 - pred) ** 2))))
print('  residuals:', np.round(e5 - pred, 4).tolist())
# with direction term: tau = a sin q5 + b cos q5 + c + h*approach
A2 = np.column_stack([A, approach])
coef2, *_ = np.linalg.lstsq(A2, e5, rcond=None)
pred2 = A2 @ coef2
print('with stiction-direction term: h=%.4f Nm (approach + vs -), rms residual %.4f Nm'
      % (coef2[3], np.sqrt(np.mean((e5 - pred2) ** 2))))
# direction-pair averages
for (a, b) in ((0, 5), (1, 2), (3, 4)):
    print(f'pair w{a}/w{b}: q5 {q5[a]:+.3f}/{q5[b]:+.3f}  mean_effort {(e5[a]+e5[b])/2:+.4f}  '
          f'half_difference {(e5[a]-e5[b])/2:+.4f} Nm')
out['q4'] = {'q5_deg': q5.tolist(), 'target5_deg': t5.tolist(), 'sag5_deg': sag5.tolist(),
             'effort5_nm': e5.tolist(), 'approach_sign': approach.tolist(),
             'sinusoid_fit': {'A': coef[0], 'B': coef[1], 'C': coef[2],
                              'rms_residual_nm': float(np.sqrt(np.mean((e5 - pred) ** 2))),
                              'residual_nm': (e5 - pred).tolist()},
             'direction_fit': {'a': coef2[0], 'b': coef2[1], 'c': coef2[2], 'h': coef2[3],
                               'rms_residual_nm': float(np.sqrt(np.mean((e5 - pred2) ** 2)))},
             'w0_minus_w5_nm': float(e5[0] - e5[5])}

# ---------------------------------------------------------------- Q5: sag step times
print('\n================ Q5: J1/J4 sag step timing ================')
cs = []
with gzip.open(raw, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'controller_state':
            cs.append(r)


def cs_arr(field):
    return np.array([[dict(zip(r['payload']['joint_names'], r['payload'][field]))[n]
                      for n in NAMES] for r in cs])


tc = np.array([r['t'] for r in cs])
refc = cs_arr('reference_positions')
fbc = cs_arr('feedback_positions')
rvc = cs_arr('reference_velocities')
errc = np.degrees(fbc - refc)


def find_step(j, pre_level, post_level):
    """First persistent departure from pre_level toward post_level."""
    e = errc[:, j]
    start = None
    for k in range(len(tc) - 5):
        if abs(e[k] - pre_level) < 0.02 and abs(e[k] - post_level) > 0.15:
            # scan forward for crossing midpoint
            mid = 0.5 * (pre_level + post_level)
            direction = 1 if post_level > pre_level else -1
            for m in range(k + 1, len(tc)):
                if direction * (e[m] - mid) > 0:
                    return int(k), int(m)
    return None


steps = {}
for j, name in ((0, 'J1'), (3, 'J4')):
    pre = float(np.median(errc[(tc > 25) & (tc < 55), j]))
    post = float(np.median(errc[(tc > 65) & (tc < 86), j]))
    found = find_step(j, pre, post)
    if found is None:
        print(f'{name}: no step found')
        continue
    k, m = found
    # last sample at pre level and first at post level, with the transition interval
    t0, t1 = tc[k], tc[m]
    seg_e = errc[:, j]
    pre_last = tc[np.flatnonzero((seg_e > pre - 0.02) & (tc < t1))[-1]]
    post_first = tc[np.flatnonzero((np.abs(seg_e - post) < 0.02) & (tc > t0))[0]]
    q5_at_t0 = np.degrees(refc[np.argmin(np.abs(tc - t0)), 4])
    q5_at_t1 = np.degrees(refc[np.argmin(np.abs(tc - t1)), 4])
    rv5_t0 = rvc[np.argmin(np.abs(tc - t0)), 4]
    rv5_t1 = rvc[np.argmin(np.abs(tc - t1)), 4]
    # J5 feedback velocity from dynamic (motion direction), nearest sample
    kdyn = int(np.argmin(np.abs(td - t1)))
    fd_vel = dyn[kdyn]['payload']['interfaces']['slaihand_joint5']['velocity']
    steps[name] = {'pre_err_deg': pre, 'post_err_deg': post,
                   'transition_start_s': float(t0), 'transition_end_s': float(t1),
                   'q5_deg_at_start': float(q5_at_t0), 'q5_deg_at_end': float(q5_at_t1),
                   'ref_vel5_at_start': float(rv5_t0), 'ref_vel5_at_end': float(rv5_t1),
                   'fb_vel5_at_end': float(fd_vel),
                   'torque_step_nm': float(np.mean([t['kp_scaled_mean'][j] for t in table]) *
                                           (post - pre) * math.pi / 180)}
    print(f'{name}: pre={pre:+.4f} deg, post={post:+.4f} deg; transition {t0:.3f}..{t1:.3f} s; '
          f'q5 {q5_at_t0:+.3f} -> {q5_at_t1:+.3f} deg; ref_vel5 {rv5_t0:+.4f}->{rv5_t1:+.4f} rad/s; '
          f'effort-equivalent step {steps[name]["torque_step_nm"]:+.4f} Nm')
# demonstrate path dependence: q5 excursion 0->+20 did not move J1/J4
print('J1/J4 error during w0..w3 (q5 +0.28..+19.65):',
      np.round([np.mean(errc[(tc >= w['recorder_window_s'][0]) & (tc <= w['recorder_window_s'][1]), j])
                for w in rec_w[:4] for j in (0, 3)], 4).tolist())
out['q5'] = steps

# compare against the lead's independent extraction (sanity cross-check only)
lead_path = HERE.parent / 'static_windows_j5.json'
if lead_path.exists():
    lead = json.loads(lead_path.read_text())['J5']['windows']
    dq = de = 0.0
    for i, w in enumerate(lead):
        if 'excluded' in w:
            continue
        dq = max(dq, float(np.max(np.abs(np.array(w['q_deg']) - np.array(table[i]['q_deg'])))))
        de = max(de, float(np.max(np.abs(np.array(w['effort_mean_nm']) -
                                          np.array(table[i]['effort_mean_nm'])))))
    print(f'\ncross-check vs lead extraction: max |dq| = {dq:.6f} deg, max |d effort| = {de:.6f} Nm')
    out['cross_check_lead'] = {'max_dq_deg': dq, 'max_deffort_nm': de}

(HERE / 'analysis_final.json').write_text(json.dumps(out, indent=2) + '\n', encoding='utf-8')
print('\nwritten', HERE / 'analysis_final.json')
