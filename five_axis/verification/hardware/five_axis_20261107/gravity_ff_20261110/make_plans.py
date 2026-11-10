"""Generate training/validation plans (<=12 waypoints) for static sag compensation."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'
PLANS.mkdir(exist_ok=True)

TRAIN_A = [
    [0, 0, 0, 0, 0],
    [10, 0, 0, 0, 0],
    [-10, 0, 0, 0, 0],
    [0, 10, 0, 0, 0],
    [0, 20, 0, 0, 0],
    [0, 10, 5, 0, 0],
    [0, 0, 10, 10, 0],
    [8, 10, 0, 8, 0],
    [0, 10, 0, 0, -15],
    [0, 0, 0, 0, 0],
    [5, 0, 0, 10, 0],
    [0, 0, 0, 0, 0],
]
TRAIN_B = [
    [0, 0, 0, 0, 0],
    [12, 5, 5, 5, 0],
    [0, 15, 10, 0, 0],
    [10, 10, 0, 0, 0],
    [0, 5, 0, 5, 0],
    [8, 0, 10, 0, 0],
    [0, 20, 0, 0, 0],
    [0, 0, 0, 10, 10],
    [5, 10, 0, 0, 0],
    [0, 0, 5, 5, 0],
    [10, 0, 0, 0, 15],
    [0, 0, 0, 0, 0],
]
VALID = [
    [0, 0, 0, 0, 0],
    [5, 8, 0, 0, 0],
    [0, 12, 5, 0, 0],
    [-5, 0, 0, 10, 0],
    [0, 20, 0, 0, -15],
    [12, 5, 5, 5, 0],
    [0, 8, 0, 5, 0],
    [8, 0, 5, 0, 10],
    [0, 15, 5, 0, 0],
    [0, 0, 0, 8, 0],
    [0, 0, 0, 0, 0],
]


def plan(points, path):
    waypoints = [{'degrees': [float(v) for v in point], 'move_s': 6.0, 'dwell_s': 6.0}
                 for point in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path


for name, points in (('T1a', TRAIN_A), ('T1b', TRAIN_B), ('V1', VALID)):
    print('wrote', plan(points, PLANS / f'{name}_sag_20261110.json'), 'waypoints', len(points))
