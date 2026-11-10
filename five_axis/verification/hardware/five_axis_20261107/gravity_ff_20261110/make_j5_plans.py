"""J5-focused static plans: fixed base pose, bidirectional J5 sweep, absolute-zero ends."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'
PLANS.mkdir(exist_ok=True)


def plan(points, path):
    waypoints = [{'degrees': [float(v) for v in point], 'move_s': 6.0, 'dwell_s': 6.0}
                 for point in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path


J5A = [
    [0, 0, 0, 0, 0],
    [0, 15, 0, 0, 0],
    [0, 15, 0, 0, -30],
    [0, 15, 0, 0, -15],
    [0, 15, 0, 0, 0],
    [0, 15, 0, 0, 15],
    [0, 15, 0, 0, 30],
    [0, 15, 0, 0, 15],
    [0, 15, 0, 0, 0],
    [0, 15, 0, 0, -15],
    [0, 15, 0, 0, -30],
    [0, 0, 0, 0, 0],
]
J5B = [
    [0, 0, 0, 0, 0],
    [0, 0, 15, 10, 0],
    [0, 0, 15, 10, -30],
    [0, 0, 15, 10, -15],
    [0, 0, 15, 10, 0],
    [0, 0, 15, 10, 15],
    [0, 0, 15, 10, 30],
    [0, 0, 15, 10, 15],
    [0, 0, 15, 10, 0],
    [0, 0, 15, 10, -15],
    [0, 0, 15, 10, -30],
    [0, 0, 0, 0, 0],
]
# Novel base pose for validation; J5 sweep where the direction reverses twice.
JV = [
    [0, 0, 0, 0, 0],
    [8, 10, 8, 5, 0],
    [8, 10, 8, 5, -25],
    [8, 10, 8, 5, -10],
    [8, 10, 8, 5, 5],
    [8, 10, 8, 5, 20],
    [8, 10, 8, 5, 5],
    [8, 10, 8, 5, -10],
    [8, 10, 8, 5, -25],
    [8, 10, 8, 5, 0],
    [0, 0, 0, 0, 0],
]

for name, points in (('J5A', J5A), ('J5B', J5B), ('JV', JV)):
    print('wrote', plan(points, PLANS / f'{name}_20261110.json'), 'waypoints', len(points))
