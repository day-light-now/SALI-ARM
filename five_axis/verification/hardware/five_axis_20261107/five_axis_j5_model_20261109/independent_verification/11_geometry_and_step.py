"""Q5 precise sag-step timing + independent geometry sensitivity for an off-axis
J5-output payload (3.5 kg), using the canonical four-axis URDF + J5 placeholder.

READ-ONLY. No hardware, no ROS, no driver.
"""
import gzip
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')
HERE = ROOT / ('verification/hardware/five_axis_20261107/five_axis_j5_model_20261109/'
               'independent_verification')
RAW = ROOT / 'verification/hardware/five_axis_20261107/acquisition_20261109/raw/j5_excitation_20261109.jsonl.gz'
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
G = np.array([0., 9.80665, 0.])
MASS = 3.5

# ---------- precise release time from 100 Hz controller_state ----------
cs = []
with gzip.open(RAW, 'rt', encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'controller_state':
            cs.append(r)
tc = np.array([r['t'] for r in cs])


def col(field):
    return np.array([[dict(zip(r['payload']['joint_names'], r['payload'][field]))[n]
                      for n in NAMES] for r in cs])


ref = col('reference_positions')
fb = col('feedback_positions')
rv = col('reference_velocities')
err = np.degrees(fb - ref)
step = {}
for j, name in ((0, 'J1'), (3, 'J4')):
    e = err[:, j]
    sel = (tc > 55.0)
    ix = np.flatnonzero(sel)
    hi = [i for i in ix if e[i] > 0.25]
    lo = [i for i in ix if e[i] < 0.15]
    last_hi = hi[-1]
    first_lo = min(i for i in lo if i > last_hi)
    step[name] = {
        'last_pre_value_deg': float(e[last_hi]), 'last_pre_t': float(tc[last_hi]),
        'first_post_value_deg': float(e[first_lo]), 'first_post_t': float(tc[first_lo]),
        'release_interval_s': [float(tc[last_hi]), float(tc[first_lo])],
        'q5_deg_at_last_pre': float(np.degrees(ref[last_hi, 4])),
        'q5_deg_at_first_post': float(np.degrees(ref[first_lo, 4])),
        'ref_vel5_at_last_pre_rad_s': float(rv[last_hi, 4]),
        'ref_vel5_at_first_post_rad_s': float(rv[first_lo, 4]),
    }
    print(f'{name}: pre {step[name]["last_pre_value_deg"]:+.4f} deg @ {tc[last_hi]:.3f}s '
          f'(q5={step[name]["q5_deg_at_last_pre"]:+.3f} deg, ref vel5={rv[last_hi,4]:+.4f}) '
          f'-> post {step[name]["first_post_value_deg"]:+.4f} deg @ {tc[first_lo]:.3f}s '
          f'(q5={step[name]["q5_deg_at_first_post"]:+.3f} deg, ref vel5={rv[first_lo,4]:+.4f})')

# ---------- geometry: canonical URDF, J3 offset pi, J5 at link4 origin +Z ----------
joints = []
for jt in ET.parse(URDF).getroot().findall('joint'):
    o = jt.find('origin')
    t = np.eye(4)
    t[:3, 3] = np.fromstring(o.get('xyz'), sep=' ')
    t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy'), sep=' ')).as_matrix()
    ax = np.fromstring(jt.find('axis').get('xyz'), sep=' ')
    ax = ax / np.linalg.norm(ax)
    joints.append((jt.get('name'), t, ax))
axes5 = np.array([0., 0., 1.])


def frames(q):
    world, piv, axes, after = np.eye(4), [], [], []
    for (name, t, ax), val in zip(joints, q[:4]):
        world = world @ t
        piv.append(world[:3, 3].copy())
        axes.append(world[:3, :3] @ ax)
        off = math.pi if name == NAMES[2] else 0.0
        m = np.eye(4)
        m[:3, :3] = Rotation.from_rotvec(ax * (val + off)).as_matrix()
        world = world @ m
        after.append(world.copy())
    # J5: identity origin on link4, axis +Z of link4 frame
    piv.append(world[:3, 3].copy())
    axes.append(world[:3, :3] @ axes5)
    m5 = np.eye(4)
    m5[:3, :3] = Rotation.from_rotvec(axes5 * q[4]).as_matrix()
    world = world @ m5
    after.append(world.copy())
    return piv, axes, after


def payload_torque(q, com_m):
    """holding torque on J1..J5 from a point mass MASS at COM (link5 frame)."""
    piv, axes, after = frames(np.asarray(q, float))
    p5, R5 = after[4][:3, 3], after[4][:3, :3]
    p = p5 + R5 @ np.asarray(com_m, float)
    tau = np.zeros(5)
    for j in range(5):
        tau[j] = -axes[j] @ np.cross(p - piv[j], G) * MASS
    return tau


q0 = np.zeros(5)
coms = {
    'codex_candidate_110_9_-47_mm': np.array([0.110, 0.009, -0.047]),
    'off_axis_50mm_x': np.array([0.050, 0.0, 0.0]),
    'off_axis_10mm_x': np.array([0.010, 0.0, 0.0]),
}
print('\n=== predicted payload gravity holding torque change, q5 +20 -> -20 deg '
      '(J1..J4 = 0), 3.5 kg ===')
q_plus, q_minus = q0.copy(), q0.copy()
q_plus[4] = math.radians(20.0)
q_minus[4] = math.radians(-20.0)
geom = {'mass_kg': MASS, 'gravity': G.tolist(),
        'convention': 'slaihand_four_axis.urdf + J3 offset pi + J5 at link4 origin along +Z',
        'cases': {}}
for name, com in coms.items():
    tp, tm = payload_torque(q_plus, com), payload_torque(q_minus, com)
    d = tp - tm
    geom['cases'][name] = {'com_m': com.tolist(),
                           'tau_at_q5_plus20_nm': tp.tolist(),
                           'tau_at_q5_minus20_nm': tm.tolist(),
                           'delta_tau_nm': d.tolist()}
    print(f'{name}: COM={np.round(com*1000,1).tolist()} mm  d_tau J1..J5 = '
          + ' '.join(f'{v:+.4f}' for v in d))
# sensitivity per mm: d_tau for 1 mm COM offset along link5 x/y (q5 +20 vs -20)
sens = {}
for k, lab in ((0, 'x'), (1, 'y')):
    unit = np.zeros(3)
    unit[k] = 0.001
    d = payload_torque(q_plus, unit) - payload_torque(q_minus, unit)
    sens[lab] = d.tolist()
    print(f'sensitivity per 1 mm COM offset along link5 {lab}: d_tau = '
          + ' '.join(f'{v:+.5f}' for v in d))
geom['sensitivity_per_mm'] = sens
# bound on rotating-payload lateral COM from the measured J2 null
j2_bound_nm = 0.049  # half of the 0.09768 Nm J1/J2 torque code
s2 = max(abs(sens['x'][1]), abs(sens['y'][1]))
geom['j2_null'] = {'measured_delta_tau_j2_nm': 0.0,
                   'measurement_bound_nm': j2_bound_nm,
                   'implied_max_com_offset_mm': j2_bound_nm / s2 if s2 else None,
                   'sensitivity_nm_per_mm_j2': s2}
print(f'J2 null: measured d_tau = 0.000 Nm over q5 +20->-20; '
      f'bound +/-{j2_bound_nm:.4f} Nm (half torque code) -> rotating-payload lateral COM '
      f'<= {j2_bound_nm/s2:.2f} mm')

(HERE / 'geometry_probe.json').write_text(json.dumps({'sag_step': step, 'geometry': geom},
                                                     indent=2) + '\n', encoding='utf-8')
print('\nwritten', HERE / 'geometry_probe.json')
