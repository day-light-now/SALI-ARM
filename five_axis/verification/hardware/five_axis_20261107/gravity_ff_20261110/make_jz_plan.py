"""New-pose validation plan for iterative static compensation (J3 bias + J5 reversal)."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
JZ = [
    [0, 0, 0, 0, 0],
    [6, 9, 6, 4, 0],
    [6, 9, 6, 4, -20],
    [6, 9, 6, 4, -5],
    [6, 9, 6, 4, 10],
    [6, 9, 6, 4, 25],
    [6, 9, 6, 4, 10],
    [6, 9, 6, 4, -5],
    [6, 9, 6, 4, -20],
    [6, 9, 6, 4, 0],
    [0, 0, 0, 0, 0],
]
waypoints = [{'degrees': [float(v) for v in point], 'move_s': 6.0, 'dwell_s': 6.0}
             for point in JZ]
path = HERE / 'plans' / 'JZ_20261110.json'
path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                            'coordinates': 'absolute_deg',
                            'waypoints': waypoints},
                           ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('wrote', path, len(waypoints))
