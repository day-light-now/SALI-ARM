"""Independent analysis pass 1: audit alignment, gain reconstruction, formula checks.

READ-ONLY on data. Inputs (all in independent_verification/):
  windows_extracted.json      - my own extraction from the recorder
  audit_command1_cycles.jsonl.gz - my own extraction from the 41 MB audit
Outputs: analysis_pass1.json
"""
import gzip
import json
from pathlib import Path

import numpy as np

HERE = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain/verification/'
            'hardware/five_axis_20261107/five_axis_j5_model_20261109/independent_verification')
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
MOVE_S = [0.5, 8.0, 8.0, 8.0, 8.0, 10.0]
ARRIVAL_S = [0.5, 13.5, 27.5, 41.5, 55.5, 71.5]

windows = json.loads((HERE / 'windows_extracted.json').read_text())
session = None
cycles = []
with gzip.open(HERE / 'audit_command1_cycles.jsonl.gz', 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r.get('kind') == 'SESSION' or 'backend' in r:
            session = r
        elif 'request_estimate_nm' in r:
            cycles.append(r)
cycles.sort(key=lambda r: r['steady_s'])
print('audit cycles:', len(cycles), 'session kp', session['kp'], 'kd', session['kd'],
      'kp_axis', session['kp_axis'], 'kd_axis', session['kd_axis'])
print('kp_axis_source', session['kp_axis_source'], 'kp_schedule_source', session['kp_schedule_source'])
print('kp_schedule', session['kp_schedule'])
print('gravity_model_present', session['gravity_model_present'], 'gravity_ff_alpha', session['gravity_ff_alpha'])
signs = np.array([j['sign'] for j in session['joint_policy']], float)
zeros = np.array([j['zero'] for j in session['joint_policy']], float)
print('joint sign', signs.tolist())
print('joint zero', [round(z, 6) for z in zeros])

at = np.array([r['steady_s'] for r in cycles], float)
targets = np.array([r['target'] for r in cycles], float)
kp_enc = np.array([r['kp_encoded'] for r in cycles], float)
kp_scaled = np.array([r['kp_scaled'] for r in cycles], float)
request = np.array([r['request_estimate_nm'] for r in cycles], float)
ctrl_state = np.array([r['control_state'] for r in cycles], float)
req_state = [r['request_estimate_state'] for r in cycles]
kp_out = [r['kp_output_state'] for r in cycles]
print('control_state counts:', dict(zip(*np.unique(ctrl_state, return_counts=True))))
print('request_estimate_state counts:', {s: req_state.count(s) for s in set(req_state)})
print('kp_output_state counts:', {s: kp_out.count(s) for s in set(kp_out)})
print('target values seen:', np.unique(np.round(targets, 6), axis=0).tolist()[:12])
print('kp_encoded range', np.nanmin(kp_enc, axis=0).tolist(), np.nanmax(kp_enc, axis=0).tolist())
print('kp_scaled range', np.nanmin(kp_scaled, axis=0).tolist(), np.nanmax(kp_scaled, axis=0).tolist())

# ---- audit target segments: first cycle of each exact target, in order ----
plan_targets = np.array([
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, np.radians(10.)],
    [0, 0, 0, 0, np.radians(-10.)],
    [0, 0, 0, 0, np.radians(20.)],
    [0, 0, 0, 0, np.radians(-20.)],
    [0, 0, 0, 0, 0],
], float)
seg_first, cursor = [], -np.inf
for tgt in plan_targets:
    mask = (np.max(np.abs(targets - tgt), axis=1) < 1e-9) & (at > cursor)
    hits = np.flatnonzero(mask)
    if not len(hits):
        seg_first.append(None)
        continue
    seg_first.append(int(hits[0]))
    cursor = at[hits[-1]]
print('\naudit segment first cycle steady_s / cycle_id:',
      [(None if i is None else (round(float(at[i]), 4), int(cycles[i]['cycle_id']))) for i in seg_first])

rec_windows = [w for w in windows['windows']]
assert all('excluded' not in w for w in rec_windows), 'some recorder window excluded'
rec_plateau_start = np.array([w['plateau_s'][0] for w in rec_windows])
print('recorder plateau starts:', np.round(rec_plateau_start, 4).tolist())

# align: audit segment start = start of the move; recorder plateau arrival = move start + move_s
offsets = []
for i, seg in enumerate(seg_first):
    if seg is None:
        continue
    offsets.append(float(rec_plateau_start[i] - MOVE_S[i] - at[seg]))
off = float(np.median(offsets))
print('per-window audit->recorder offset estimates:', [round(o, 4) for o in offsets],
      'median', round(off, 4), 'spread', round(float(np.ptp(offsets)), 4))

# ---- per-window audit values aligned by segment (clock-free for gains) ----
report = {'session': {k: session[k] for k in
                      ('backend', 'kp', 'kd', 'kp_axis', 'kd_axis', 'kp_axis_source', 'kd_axis_source',
                       'kp_schedule_source', 'kp_schedule', 'gravity_ff_alpha', 'gravity_ff_ramp_s',
                       'gravity_model_present', 'following_error_deg', 'reference_speed_rad_s',
                       'physical_torque_off_verified', 'hardware_motion_accepted',
                       'driver_current_clamp_verified', 'host_total_torque_clamp')},
          'signs': signs.tolist(), 'zeros': zeros.tolist(),
          'audit_to_recorder_offset_s': off,
          'audit_offset_spread_s': float(np.ptp(offsets)),
          'audit_offset_per_window': offsets,
          'windows': []}

for i, w in enumerate(rec_windows):
    seg = seg_first[i]
    if seg is None:
        report['windows'].append({'id': i, 'excluded': 'no audit segment'})
        continue
    # cycles belonging to this segment (until target changes)
    nxt = seg_first[i + 1] if i + 1 < len(seg_first) and seg_first[i + 1] is not None else len(cycles)
    # restrict to the plateau part: recorder window [start,end] of the dwell = last 2 s
    lo = w['recorder_window_s'][0] - off
    hi = w['recorder_window_s'][1] - off
    rows = np.flatnonzero((at >= lo) & (at <= hi))
    rows = [int(r) for r in rows if seg <= r < nxt]
    rows = [r for r in rows if req_state[r] == 'CURRENT' and kp_out[r] == 'WRITTEN_CURRENT']
    q = np.array(w['q_mean_rad'])
    effort = np.array(w['effort_mean_nm'])
    tgt = np.array(w['target_rad'])
    if not rows:
        report['windows'].append({'id': i, 'excluded': 'no audit rows in recorder window'})
        continue
    kpe = np.mean(kp_enc[rows], axis=0)
    kps = np.mean(kp_scaled[rows], axis=0)
    req_motor = np.mean(request[rows], axis=0)
    req_model = req_motor * signs
    kp_err = kps * (tgt - q)
    item = {
        'id': i, 'n_audit_cycles': len(rows),
        'audit_steady_range': [float(at[rows[0]]), float(at[rows[-1]])],
        'target_rad': tgt.tolist(), 'q_mean_rad': q.tolist(),
        'kp_encoded_mean': kpe.tolist(), 'kp_scaled_mean': kps.tolist(),
        'request_motor_mean_nm': req_motor.tolist(),
        'request_model_mean_nm': req_model.tolist(),
        'effort_dynamic_mean_nm': effort.tolist(),
        'kp_scaled_times_err_nm': kp_err.tolist(),
        'error_deg': np.degrees(tgt - q).tolist(),
        'effort_minus_reqmodel_nm': (effort - req_model).tolist(),
        'effort_minus_kperr_nm': (effort - kp_err).tolist(),
        'kd_axis': session['kd_axis'],
        'velocity_mean_rad_s': w['velocity_mean_rad_s'],
        'q_std_deg': w['q_std_deg'], 'q_span_deg': w['q_span_deg'],
        'n_samples': w['n_samples'],
    }
    report['windows'].append(item)

for item in report['windows']:
    if 'excluded' in item:
        print(f"\nw{item['id']}: EXCLUDED {item['excluded']}")
        continue
    print(f"\nw{item['id']}  n_audit={item['n_audit_cycles']}  recorder n={item['n_samples']}")
    print('  q_deg          ', ' '.join(f'{v:+9.4f}' for v in np.degrees(item['q_mean_rad'])))
    print('  error_deg      ', ' '.join(f'{v:+9.4f}' for v in item['error_deg']))
    print('  kp_encoded     ', ' '.join(f'{v:9.3f}' for v in item['kp_encoded_mean']))
    print('  kp_scaled      ', ' '.join(f'{v:9.3f}' for v in item['kp_scaled_mean']))
    print('  effort[dyn]    ', ' '.join(f'{v:+9.4f}' for v in item['effort_dynamic_mean_nm']))
    print('  req_motor      ', ' '.join(f'{v:+9.4f}' for v in item['request_motor_mean_nm']))
    print('  req_model      ', ' '.join(f'{v:+9.4f}' for v in item['request_model_mean_nm']))
    print('  kp_scaled*err  ', ' '.join(f'{v:+9.4f}' for v in item['kp_scaled_times_err_nm']))
    print('  effort-req_mod ', ' '.join(f'{v:+9.5f}' for v in item['effort_minus_reqmodel_nm']))
    print('  effort-kp*err  ', ' '.join(f'{v:+9.5f}' for v in item['effort_minus_kperr_nm']))

(HERE / 'analysis_pass1.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
print('\nwritten', HERE / 'analysis_pass1.json')
