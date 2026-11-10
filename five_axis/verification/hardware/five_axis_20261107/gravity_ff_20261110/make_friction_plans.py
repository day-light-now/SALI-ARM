"""Generate single-joint bidirectional slow-sweep plans for friction identification.

At the same angle, effort(up) - effort(down) = 2 * friction (gravity cancels),
so these plans isolate stiction/viscous friction per joint without weights or a
gravity model. 5 waypoints each, <=178 s client budget, zero endpoints.
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'
PLANS.mkdir(exist_ok=True)

SWEEPS = {
    'F1': (0, (-15, 15)),
    'F2': (1, (-5, 15)),
    'F3': (2, (-15, 15)),
    'F4': (3, (-10, 10)),
    'F5': (4, (-20, 20)),
}


def plan(joint, low, high, path):
    zero = [0.0] * 5
    waypoints = []
    for target, move in ((zero, 8.0),
                         (zero[:joint] + [float(high)] + zero[joint + 1:], 15.0),
                         (zero[:joint] + [float(low)] + zero[joint + 1:], 30.0),
                         (zero[:joint] + [float(high)] + zero[joint + 1:], 30.0),
                         (zero, 15.0)):
        waypoints.append({'degrees': target, 'move_s': move, 'dwell_s': 5.0})
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    total = sum(w['move_s'] + w['dwell_s'] for w in waypoints)
    print('wrote', path, 'J%d' % (joint + 1), 'waypoints', len(waypoints),
          'plan_time %.0f s' % total)


for name, (joint, (low, high)) in SWEEPS.items():
    plan(joint, low, high, PLANS / f'{name}_friction_20261110.json')
