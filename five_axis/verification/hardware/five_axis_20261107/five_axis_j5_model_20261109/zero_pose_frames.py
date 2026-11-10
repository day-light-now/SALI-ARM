"""Print zero-pose frame geometry of the canonical four-axis URDF. Offline only.

The identified gravity prior uses the user-confirmed fixed J3 placement offset
pi. Frames here follow the same convention as identify_payload.Geometry.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'


def load():
    joints = []
    for joint in ET.parse(URDF).getroot().findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz'), sep=' ')
        rpy = np.fromstring(origin.get('rpy'), sep=' ')
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        axis /= np.linalg.norm(axis)
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        t[:3, 3] = xyz
        joints.append({'name': joint.get('name'), 't': t, 'axis': axis})
    return joints


def frames(q, joints):
    world = np.eye(4)
    out = []
    for value, joint, offset in zip(q, joints, (0., 0., math.pi, 0.)):
        world = world @ joint['t']
        out.append({'name': joint['name'], 'pivot': world[:3, 3].copy(),
                    'axis': world[:3, :3] @ joint['axis']})
        motion = np.eye(4)
        motion[:3, :3] = Rotation.from_rotvec(joint['axis'] * (value + offset)).as_matrix()
        world = world @ motion
    out.append({'name': 'link4_frame', 'pivot': world[:3, 3].copy(),
                'axis': world[:3, :3] @ np.array([0., 0., 1.])})
    return out


def main():
    joints = load()
    for q_deg in ([0, 0, 0, 0], None):
        q = np.zeros(4) if q_deg is None else np.radians(q_deg)
        rows = frames(q, joints)
        print('=' * 90)
        print('q_deg', q_deg)
        for row in rows:
            print(f"  {row['name']:18s} pivot={np.round(row['pivot'], 5)} axis={np.round(row['axis'], 5)}")
    # Dedicated report for the J5 excitation J1-J4 pose.
    q = np.radians([0.3279, 0.1311, 0.0, 0.4809])
    rows = frames(q, joints)
    link4 = rows[-1]
    print('=' * 90)
    print('J5 excitation J1-J4 pose (deg 0.33,0.13,0,0.48):')
    print('  link4 origin', np.round(link4['pivot'], 5))
    print('  link4 +Z (five-axis URDF J5 axis candidate)', np.round(link4['axis'], 5))
    print('  J4 axis', np.round(rows[3]['axis'], 5))
    print('  dot(J4 axis, base +Y gravity) =', float(rows[3]['axis'] @ np.array([0., 1., 0.])))
    print('  dot(J5 +Z, base +Y) =', float(link4['axis'] @ np.array([0., 1., 0.])))

    # Static eight-pose range of link4 orientation/axis for context.
    print('=' * 90)
    combos = [(0, 0, 0, 0), (30, 0, 0, 0), (-20, 0, 0, 0), (0, 40, 0, 0),
              (0, 0, 40, 0), (0, 0, -40, 0), (0, 0, 0, 40), (0, 0, 0, -40)]
    for c in combos:
        rows = frames(np.radians(c), joints)
        print(f"  q_deg={c}  J5axis={np.round(rows[-1]['axis'], 4)}  dot_gravity={rows[-1]['axis'] @ np.array([0., 1., 0.]):+.4f}")

    out = ROOT / 'verification/hardware/five_axis_20261107/five_axis_j5_model_20261109/zero_pose_frames.json'
    payload = {'urdf': str(URDF.relative_to(ROOT)), 'j3_offset_rad': math.pi,
               'q_deg': [0, 0, 0, 0],
               'frames': [{'name': r['name'], 'pivot_m': r['pivot'].tolist(),
                           'axis_base': r['axis'].tolist()} for r in frames(np.zeros(4), joints)]}
    out.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')
    print('written', out)


if __name__ == '__main__':
    main()
