"""Focused second pass: time-series tail, audit alignment, and effort semantics."""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
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


def main():
    audit = records(HERE / 'j5_excitation_cycles_20261109.jsonl.gz')
    session = next(r for r in audit if r['kind'] == 'SESSION')
    print('SESSION kp_axis', session['kp_axis'], 'kd_axis', session['kd_axis'],
          'kd', session['kd'], 'kp', session['kp'], 'ff_alpha', session['gravity_ff_alpha'],
          'ramp', session['gravity_ff_ramp_s'], 'j5_qualified', session['j5_motion_qualified'],
          'following_error_deg', session['following_error_deg'])
    cycles = [r for r in audit if r['kind'] == 'CONTROL_CYCLE' and r.get('request_estimate_state') == 'CURRENT']
    at = np.array([r['steady_s'] for r in cycles])
    targets = np.array([r['target'] for r in cycles])
    kp = np.array([r['kp_encoded'] for r in cycles])
    kp_scaled = np.array([r['kp_scaled'] for r in cycles])
    request = np.array([r['request_estimate_nm'] for r in cycles])

    # Align the audit clock to the recorder clock using the J5 +10 deg arrival.
    # Recorder: first exact J5=+10deg plateau start (from earlier extraction).
    windows = read(HERE / 'static_windows_j5.json')['J5']['windows']
    print('\nrecorder windows:', [(w['id'], round(w['recorder_window_s'][0], 3),
                                  round(w['recorder_window_s'][1], 3)) for w in windows])
    target_j5 = np.radians([0.0, 10.0, -10.0, 20.0, -20.0, 0.0])
    # Audit plateau arrival = first index of contiguous exact target group.
    audit_plateaus = []
    cursor = -np.inf
    for value in target_j5:
        mask = np.max(np.abs(targets - np.array([0, 0, 0, 0, value])), axis=1) < 1e-9
        hits = np.flatnonzero(mask)
        groups = np.split(hits, np.flatnonzero(np.diff(hits) > 1) + 1) if len(hits) else []
        groups = [g for g in groups if len(g) >= 50 and at[g[0]] > cursor]
        if groups:
            audit_plateaus.append(float(at[groups[0][0]]))
            cursor = at[groups[0][-1]]
        else:
            audit_plateaus.append(None)
    print('\naudit plateau start times:', [None if v is None else round(v, 3) for v in audit_plateaus])
    offsets = []
    for window, audit_start in zip(windows, audit_plateaus):
        if 'excluded' in window or audit_start is None:
            continue
        offsets.append(window['recorder_window_s'][0] - audit_start)
    offset = float(np.median(offsets))
    print('recorder - audit offset estimate s:', round(offset, 3), 'spread', round(float(np.ptp(offsets)), 3))

    print('\nper-window: recorder effort vs audit request (motor coords via sign) vs Kp*error')
    signs = np.array([j['sign'] for j in session['joint_policy']])
    for window, audit_start in zip(windows, audit_plateaus):
        if 'excluded' in window or audit_start is None:
            continue
        r0 = window['recorder_window_s'][0] + offset
        r1 = window['recorder_window_s'][1] + offset
        rows = np.flatnonzero((at >= r0) & (at <= r1))
        if not len(rows):
            print(' no audit rows for window', window['id'])
            continue
        q = np.array(window['q_rad'])
        # model target -> motor request estimate is in motor coords; convert to model
        req_model = np.mean(request[rows], axis=0) * signs
        print(f" w{window['id']}: n={len(rows)} kp_encoded={np.round(np.mean(kp[rows], axis=0), 2).tolist()} "
              f"kp_scaled={np.round(np.mean(kp_scaled[rows], axis=0), 2).tolist()}")
        print(f"      effort(model)  {np.round(np.array(window['effort_mean_nm']), 4).tolist()}")
        print(f"      audit req(model){np.round(req_model, 4).tolist()}")

    # Time series tail.
    raw = gzip.open(BASE / 'acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz', 'rt', encoding='utf-8')
    times, qs, es = [], [], []
    for line in raw:
        row = json.loads(line)
        if row['kind'] != 'dynamic':
            continue
        axes = row['payload']['interfaces']
        qs.append([axes[n]['position'] for n in NAMES])
        es.append([axes[n]['effort'] for n in NAMES])
        times.append(row['t'])
    times = np.array(times)
    qs = np.degrees(np.array(qs))
    es = np.array(es)
    print('\ntime series 40..82 s (0.5 s buckets)')
    for start in np.arange(40.0, times[-1], 0.5):
        sel = (times >= start) & (times < start + .5)
        if not sel.any():
            continue
        q = np.mean(qs[sel], axis=0)
        e = np.mean(es[sel], axis=0)
        print(f'  t={start:5.1f}  q=' + ' '.join(f'{v:+8.3f}' for v in q) +
              '  e=' + ' '.join(f'{v:+7.3f}' for v in e))


if __name__ == '__main__':
    main()
