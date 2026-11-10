"""Offline analysis of the three 2026-11-01 afternoon J4 candidate runs.

Reads the recorded trial JSONL and the raw read-only bus log only. It never
opens a port, never writes to the records, and is not imported by production
code. J4 feedback frames are identified by payload[0]&15 because every motor
answers with master id 0.

Run:
  python verification/hardware/commissioning_20261101/openarm_control_review/analyze_runs.py
"""
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

PMAX, VMAX, TMAX = 12.5, 45., 54.
J4_ZERO = -0.40951400015259054
J4_SIGN = -1
RUNS = [
    ('1Nm/0.1rad-s', ROOT / 'tmp/live_trial/20261101_154949_b50554'),
    ('1Nm/0.2rad-s', ROOT / 'tmp/live_trial/20261101_155439_df9c6a'),
    ('2Nm/0.2rad-s', ROOT / 'tmp/live_trial/20261101_155927_e9ad73'),
]


def decode_feedback(d):
    return dict(
        state=d[0] >> 4, joint=d[0] & 15,
        code=(d[1] << 8) | d[2],
        pos=((d[1] << 8) | d[2]) * 2 * PMAX / 65535 - PMAX,
        vel=((d[3] << 4) | (d[4] >> 4)) * 2 * VMAX / 4095 - VMAX,
        tau=(((d[4] & 15) << 8) | d[5]) * 2 * TMAX / 4095 - TMAX,
        t_mos=d[6], t_rotor=d[7])


def tau_of_command(payload):
    d = bytes.fromhex(payload)
    raw = ((d[6] & 15) << 8) | d[7]
    return raw * 2 * TMAX / 4095 - TMAX


def load_run(base):
    rows = [json.loads(line) for line in
            base.with_suffix('.jsonl').read_text(encoding='utf-8').splitlines()]
    result = json.loads(base.with_suffix('.json').read_text(encoding='utf-8'))
    plan = next((r['unix_s'] for r in rows if r['kind'] == 'PLAN_ACCEPTED_NOT_SENT'), None)
    stop = next(r['unix_s'] for r in rows if r['kind'] == 'GLOBAL_STOP_REQUESTED')
    commands = [(r['unix_s'], tau_of_command(r['payload']))
                for r in rows if r['kind'] == 'SERIAL_WRITTEN'
                and r.get('frame_kind') == 'mit' and r.get('joint') == 4]
    return rows, result, plan, stop, commands


def feedback_window(raw_path, lo, hi):
    frames = []
    for line in raw_path.read_text(encoding='utf-8').splitlines():
        r = json.loads(line)
        if r['direction'] != 'rx' or not (lo <= r['time'] <= hi):
            continue
        d = bytes.fromhex(r['payload'])
        if len(d) != 8 or (d[1] == 0 and d[2] == 0x33):
            continue
        f = decode_feedback(d)
        if f['joint'] != 4:
            continue
        f['time'] = r['time']
        f['q'] = J4_SIGN * (f['pos'] - J4_ZERO)
        frames.append(f)
    return frames


def summarize(name, base):
    rows, result, plan, stop, commands = load_run(base)
    raw_path = Path(result['readonly_log'])
    frames = feedback_window(raw_path, stop - 4.0, stop + 2.0)
    print('=' * 100)
    print(name, '| tier', result.get('pd_tier'), '| reason:', result.get('reason'))
    print('plan at', plan, 'stop at', stop, 'plan->stop %.3fs' % (stop - plan))
    print('encoded cap .%.3fNm' % result['encoded_torque_cap_nm'],
          '| peak cmd %.4fNm' % max(abs(t) for _, t in commands))
    print('\n--- feedback around the stop (t relative to stop; q is model deg; vel is raw encoder rad/s) ---')
    print('%8s %5s %6s %10s %10s %10s' % ('t[s]', 'state', 'code', 'q_deg', 'vel', 'tau_fb'))
    selected = [f for f in frames if -1.6 <= f['time'] - stop <= 1.0]
    step = max(1, len(selected) // 40)
    for f in selected[::step]:
        print('%8.3f %5d %6d %10.4f %10.4f %10.4f' % (
            f['time'] - stop, f['state'], f['code'], math.degrees(f['q']), f['vel'], f['tau']))
    post = [f for f in frames if f['time'] > stop and f['state'] == 0]
    pre = [f for f in frames if f['time'] <= stop]
    if post:
        print('post-stop first state0 frames:')
        for f in post[:5]:
            print('   +%.3fs code=%d q=%.4fdeg vel=%.4f tau=%.4f' % (
                f['time'] - stop, f['code'], math.degrees(f['q']), f['vel'], f['tau']))
    if pre:
        print('pre-stop last frame: t=%.3fs state=%d code=%d q=%.4f vel=%.4f' % (
            pre[-1]['time'] - stop, pre[-1]['state'], pre[-1]['code'],
            math.degrees(pre[-1]['q']), pre[-1]['vel']))
        print('code span pre-stop: %d .. %d (%d codes)' % (
            pre[0]['code'], pre[-1]['code'], pre[-1]['code'] - pre[0]['code']))

    # command torque vs measured speed, paired by latest feedback before command
    print('\n--- command torque vs feedback velocity, last 3s before stop ---')
    print('%8s %10s %10s' % ('t[s]', 'cmd_tau', 'vel_before'))
    for t, tau in [c for c in commands if stop - 3.0 <= c[0] <= stop + .5][::max(1, len(commands) // 60)]:
        before = [f for f in frames if f['time'] <= t]
        vel = before[-1]['vel'] if before else float('nan')
        print('%8.3f %10.4f %10.4f' % (t - stop, tau, vel))
    return result


def main():
    for name, base in RUNS:
        summarize(name, base)


if __name__ == '__main__':
    main()
