"""Write a short validation plan (subset of V1 poses) for the compensated rerun."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SUBSET = [
    [0, 0, 0, 0, 0],
    [5, 8, 0, 0, 0],
    [0, 12, 5, 0, 0],
    [-5, 0, 0, 10, 0],
    [0, 20, 0, 0, -15],
    [12, 5, 5, 5, 0],
    [0, 8, 0, 5, 0],
    [8, 0, 5, 0, 10],
    [0, 15, 5, 0, 0],
    [0, 0, 0, 0, 0],
]
waypoints = [{'degrees': [float(v) for v in point], 'move_s': 6.0, 'dwell_s': 6.0}
             for point in SUBSET]
path = HERE / 'plans' / 'V1sub_sag_20261110.json'
path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                            'coordinates': 'absolute_deg',
                            'waypoints': waypoints},
                           ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('wrote', path, len(waypoints))
