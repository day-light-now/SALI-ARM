"""Extract and analyse the 2026-11-09 J5-excitation audit/control evidence.

Writes a compact local archive of the audit CONTROL_CYCLE/SESSION records for
command_id 1 (the only command of the J5 excitation run), then reports:
  * per static window: encoded Kp, audit PD request estimate, recorded effort;
  * a 0.5 s time series of q/effort around the run to expose stiction slips.

Offline only: reads local recording and audit files, never a device.
"""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
BASE = ROOT / 'verification/hardware/five_axis_20261107'
AUDIT = Path('/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_qsrQke.jsonl')
ARCHIVE = HERE / 'j5_excitation_cycles_20261109.jsonl.gz'
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def records(path):
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream]


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[n] for n in NAMES], dtype=float)


def extract_audit():
    if ARCHIVE.exists():
        return records(ARCHIVE)
    rows = []
    with AUDIT.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'SESSION' or (row['kind'] == 'CONTROL_CYCLE' and row['command_id'] == 1):
                rows.append(row)
    with gzip.open(ARCHIVE, 'wt', encoding='utf-8') as out:
        for row in rows:
            out.write(json.dumps(row, separators=(',', ':')) + '\n')
    return rows


def main():
    audit = extract_audit()
    print('audit rows', len(audit), 'wrote', ARCHIVE)
    session = next(r for r in audit if r['kind'] == 'SESSION')
    print('SESSION keys', sorted(session.keys()))
    print('SESSION joint_policy signs', [j['sign'] for j in session['joint_policy']])
    cycles = [r for r in audit if r['kind'] == 'CONTROL_CYCLE' and r.get('request_estimate_state') == 'CURRENT']
    print('current cycles', len(cycles), 'first/last steady_s',
          cycles[0]['steady_s'], cycles[-1]['steady_s'])
    print('cycle keys', sorted(cycles[0].keys()))

    windows = read(HERE / 'static_windows_j5.json')['J5']['windows']
    plan = read(BASE / 'acquisition_20261109/plans/J5_EXCITATION_ZERO.json')['waypoints']
    targets = [np.radians(w['degrees']) for w in plan]
    at = np.array([r['steady_s'] for r in cycles])
    targets_audit = np.array([r['target'] for r in cycles])
    kp = np.array([r['kp_encoded'] for r in cycles], dtype=object)
    request = np.array([r['request_estimate_nm'] for r in cycles])

    print('\nper-window audit evidence')
    for w, target in zip(windows, targets):
        if 'excluded' in w:
            print(' window', w['id'], 'EXCLUDED')
            continue
        mask = np.max(np.abs(targets_audit - target), axis=1) < 1e-9
        rows = np.flatnonzero(mask)
        rows = rows[(at[rows] >= w['recorder_window_s'][0]) & (at[rows] <= w['recorder_window_s'][1])]
        kp_axis = np.array([r['kp_encoded'] for r in cycles], dtype=float)[rows]
        print(f" w{w['id']} q5={w['q_deg'][4]:+8.3f} deg  n_audit={len(rows)}  "
              f"kp={np.round(np.nanmean(kp_axis, axis=0), 1).tolist()}")
        print(f"      effort    Nm {np.round(w['effort_mean_nm'], 4).tolist()}")
        print(f"      effort std Nm {np.round(w['effort_std_nm'], 4).tolist()}")
        print(f"      audit PD  Nm {np.round(np.mean(request[rows], axis=0), 4).tolist()}")

    # Time series: 0.5 s means from the recorder dynamic records.
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
    print('\ntime series (0.5 s buckets): t, q1..q5 [deg], effort1..5 [Nm]')
    for start in np.arange(0.0, times[-1], 0.5):
        sel = (times >= start) & (times < start + .5)
        if not sel.any():
            continue
        q = np.mean(qs[sel], axis=0)
        e = np.mean(es[sel], axis=0)
        print(f'  t={start:5.1f}  q=' + ' '.join(f'{v:+8.3f}' for v in q) +
              '  e=' + ' '.join(f'{v:+7.3f}' for v in e))


if __name__ == '__main__':
    main()
