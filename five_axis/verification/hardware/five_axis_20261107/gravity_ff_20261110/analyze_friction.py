"""Friction identification from single-joint bidirectional slow sweeps.

At the same joint angle q, moving positive vs negative:
    effort(+v) = gravity(q) + friction,  effort(-v) = gravity(q) - friction
=> friction(q) = (effort(+v) - effort(-v)) / 2   (gravity cancels)

The plans F1..F5 sweep each joint between two angles at ~1 deg/s, so each q is
crossed in both directions. We bin by q and report friction per joint, plus a
rough Coulomb + viscous split from the |v| variation inside a move.

Offline only; reads recorded raw files.
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
PLANS = {'F1': 0, 'F2': 1, 'F3': 2, 'F4': 3, 'F5': 4}
TAGS = {name: f'{name}fr' for name in PLANS}
BIN_DEG = 0.25
MIN_VEL = 0.005  # rad/s


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def ordered(payload, field):
    values = dict(zip(payload['joint_names'], payload[field]))
    return np.array([values[name] for name in NAMES], dtype=float)


def load(tag):
    raw = gzip.open(HERE / 'raw' / f'{tag}_20261110.jsonl.gz', 'rt', encoding='utf-8')
    ctrl, dyn = [], []
    for line in raw:
        row = json.loads(line)
        if row.get('kind') == 'controller_state':
            try:
                ordered(row['payload'], 'reference_positions')
                ordered(row['payload'], 'feedback_positions')
                ordered(row['payload'], 'reference_velocities')
            except (KeyError, TypeError, ValueError):
                continue
            ctrl.append(row)
        elif row.get('kind') == 'dynamic':
            dyn.append(row)
    raw.close()
    if not ctrl:
        return None
    t = np.array([row['t'] for row in ctrl])
    refs = np.array([ordered(row['payload'], 'reference_positions') for row in ctrl])
    q = np.array([ordered(row['payload'], 'feedback_positions') for row in ctrl])
    rv = np.array([ordered(row['payload'], 'reference_velocities') for row in ctrl])
    dt = np.array([row['t'] for row in dyn]) if dyn else np.array([])
    effort = np.full((len(ctrl), 5), np.nan)
    for i, time in enumerate(t):
        if not len(dt):
            break
        j = int(np.searchsorted(dt, time))
        j = min((max(0, j - 1), min(j, len(dt) - 1)), key=lambda n: abs(dt[n] - time))
        if abs(dt[j] - time) > 0.025:
            continue
        axes = dyn[j]['payload']['interfaces']
        try:
            effort[i] = [axes[name]['effort'] for name in NAMES]
        except KeyError:
            continue
    return {'t': t, 'refs': refs, 'q': q, 'rv': rv, 'effort': effort}


def analyse_joint(data, joint):
    moving = np.abs(data['rv'][:, joint]) > MIN_VEL
    q_deg = np.degrees(data['q'][:, joint])
    sign = np.sign(data['rv'][:, joint])
    effort = data['effort'][:, joint]
    valid = moving & np.isfinite(effort)
    if valid.sum() < 50:
        return None
    bins_all = np.round(q_deg / BIN_DEG).astype(int)
    rows = []
    for value in np.unique(bins_all[valid]):
        mask = valid & (bins_all == value)
        plus = mask & (sign > 0)
        minus = mask & (sign < 0)
        if plus.sum() < 5 or minus.sum() < 5:
            continue
        f = float((effort[plus].mean() - effort[minus].mean()) / 2.0)
        rows.append({'q_deg': float(q_deg[mask].mean()),
                     'friction_nm': f,
                     'v_plus': float(np.abs(data['rv'][plus, joint]).mean()),
                     'v_minus': float(np.abs(data['rv'][minus, joint]).mean()),
                     'n_plus': int(plus.sum()), 'n_minus': int(minus.sum())})
    if len(rows) < 3:
        return None
    friction = np.array([row['friction_nm'] for row in rows])
    qs = np.array([row['q_deg'] for row in rows])
    return {'joint': joint + 1, 'bins': rows,
            'friction_mean_nm': float(friction.mean()),
            'friction_std_nm': float(friction.std()),
            'friction_min_nm': float(friction.min()),
            'friction_max_nm': float(friction.max()),
            'q_range_deg': [float(qs.min()), float(qs.max())]}


def main():
    result = {'schema': 'slaihand_friction_estimates/1', 'joints': {}}
    for name, joint in PLANS.items():
        tag = TAGS[name]
        path = HERE / 'raw' / f'{tag}_20261110.jsonl.gz'
        if not path.is_file():
            print(tag, 'MISSING raw')
            continue
        data = load(tag)
        if data is None:
            print(tag, 'EMPTY')
            continue
        entry = analyse_joint(data, joint)
        print(name, 'J%d' % (joint + 1))
        if entry is None:
            print('  insufficient bidirectional moving samples')
            continue
        print('  friction mean %.3f std %.3f range [%.3f, %.3f] Nm over q [%.1f, %.1f] deg' % (
            entry['friction_mean_nm'], entry['friction_std_nm'],
            entry['friction_min_nm'], entry['friction_max_nm'],
            entry['q_range_deg'][0], entry['q_range_deg'][1]))
        for row in entry['bins']:
            print('    q=%6.2f f=%7.3f Nm (v %.3f/%.3f rad/s)' % (
                row['q_deg'], row['friction_nm'], row['v_plus'], row['v_minus']))
        result['joints']['J%d' % (joint + 1)] = entry
    (HERE / 'friction_estimates_20261110.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote friction_estimates_20261110.json')


if __name__ == '__main__':
    main()
