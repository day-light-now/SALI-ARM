"""Test the hypothesis that the 3.5 kg payload rotates with the J5 output.

Offline geometry/regressor arithmetic only. The J5 joint is placed at the
canonical link4 origin along link4 +Z (five-axis placeholder convention). The
payload is a point mass on link5 (the J5 output frame).

Observables tested:
  * the abrupt J1/J4 static-sag change while J5 moved from +20 deg to -20 deg
    (a stiction slip), converted to holding-torque change with the recorded Kp;
  * the J5 effort curve versus q5.
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
G = np.array([0., 9.80665, 0.])
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def urdf_joints():
    out = []
    for joint in ET.parse(URDF).getroot().findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz'), sep=' ')
        rpy = np.fromstring(origin.get('rpy'), sep=' ')
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        axis /= np.linalg.norm(axis)
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        t[:3, 3] = xyz
        out.append((joint.get('name'), t, axis))
    return out


class FiveAxis:
    """Canonical J1-J4 URDF plus an added J5 revolute at the link4 origin."""

    def __init__(self, j3_offset=math.pi, j5_axis=(0., 0., 1.)):
        self.joints = []
        for name, t, axis in urdf_joints():
            offset = j3_offset if name == NAMES[2] else 0.
            self.joints.append((t, axis, offset))
        axis5 = np.asarray(j5_axis, dtype=float)
        self.joints.append((np.eye(4), axis5 / np.linalg.norm(axis5), 0.))

    def frames(self, q):
        """Per-joint pivot/axis (in the joint frame) and the frame after each joint."""
        world = np.eye(4)
        pivots, axes, after = [], [], []
        for value, (origin, axis, offset) in zip(q, self.joints):
            world = world @ origin
            pivots.append(world[:3, 3].copy())
            axes.append(world[:3, :3] @ axis)
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(axis * (value + offset)).as_matrix()
            world = world @ motion
            after.append(world.copy())
        return pivots, axes, after

    def payload_terms(self, q, mass):
        """(offset (5,), A (5,3)) so that payload holding torque = mass*(offset + A c)."""
        pivots, axes, after = self.frames(q)
        payload = after[4]          # link5 frame, rotated by q5
        p5, R5 = payload[:3, 3], payload[:3, :3]
        offset = np.zeros(5)
        A = np.zeros((5, 3))
        for joint in range(5):
            axis = axes[joint]
            offset[joint] = -axis @ np.cross(p5 - pivots[joint], G)
            for k in range(3):
                A[joint, k] = -axis @ np.cross(R5[:, k], G)
        return mass * offset, mass * A

    def payload_torque(self, q, mass, com):
        offset, A = self.payload_terms(q, mass)
        return offset + A @ com


def main():
    model = FiveAxis()
    kp = np.array([160.81, 160.44, 159.95, 160.81, 159.95])  # recorded kp_encoded at the near-zero pose
    q_pre = np.radians([0.306, 0.131, 0.0, 0.437])
    q_post = np.radians([0.040, 0.131, 0.0, 0.130])
    q5_pre, q5_post = math.radians(20.0), math.radians(-20.0)
    pre = np.concatenate([q_pre, [q5_pre]])
    post = np.concatenate([q_post, [q5_post]])

    y_pre = -kp[:4] * q_pre
    y_post = -kp[:4] * q_post
    delta_tau = np.concatenate([y_post - y_pre, [0.]])
    print('observed sag-change torque estimate J1..J4 Nm:', np.round(delta_tau[:4], 4))

    o_pre, A_pre = model.payload_terms(pre, 3.5)
    o_post, A_post = model.payload_terms(post, 3.5)
    do, dA = o_post - o_pre, A_post - A_pre

    # Solve the 4 J1..J4 equations for c.
    c_fit, *_ = np.linalg.lstsq(dA[:4], delta_tau[:4] - do[:4], rcond=None)
    print('fitted COM link5 (mm) from the slip:', np.round(c_fit * 1000, 1))
    print('residual J1..J4 Nm:', np.round(dA[:4] @ c_fit + do[:4] - delta_tau[:4], 4))
    print('predicted J5 d_tau Nm for that COM:', round(float(dA[4] @ c_fit + do[4]), 4))

    com_codex_link4 = np.array([0.110, 0.009, -0.047])
    q5_ref = math.radians(2.2294)
    com_codex = Rotation.from_rotvec([0, 0, -q5_ref]).apply(com_codex_link4)
    print('\nCodex COM in link5 frame (mm):', np.round(com_codex * 1000, 1))
    for name, com in (('fitted-from-slip', c_fit), ('Codex', com_codex)):
        pred = dA @ com + do
        print(f'predicted d_tau [J1..J5] for {name}:', np.round(pred, 4))
        print(f'   mismatch vs observed slip [J1..J4]:',
              np.round(pred[:4] - delta_tau[:4], 4))

    # Predicted payload torque curve vs q5, compared with the recorded J5 effort.
    measured = {0.284: -0.8773, 9.748: 0.6118, -9.683: -0.9882, 19.649: 0.8959,
                -19.693: -0.9515, -0.284: 0.7271}
    print('\npayload model torque (Nm) vs q5 for Codex COM, observed J5 effort:')
    q5_order = np.array([0.284, 9.748, 19.649, -9.683, -19.693, -0.284])
    pred5 = []
    for q5_deg in q5_order:
        q = np.concatenate([q_pre, [math.radians(q5_deg)]])
        tau = model.payload_torque(q, 3.5, com_codex)
        pred5.append(tau[4])
        print(f'  q5={q5_deg:+8.3f}  payload [J1..J5] ' +
              ' '.join(f'{v:+7.3f}' for v in tau) +
              f'  observed J5 effort {measured[q5_deg]:+7.3f}')
    pred5 = np.array(pred5)
    eff = np.array([measured[v] for v in q5_order])
    offset = float(np.mean(eff - pred5))
    residual = eff - pred5 - offset
    print('\nJ5 effort curve vs Codex payload curve with one constant offset:')
    print('  offset', round(offset, 3), 'rmse', round(float(np.sqrt(np.mean(residual ** 2))), 3))
    print('  residuals', np.round(residual, 3))

    out = {
        'schema': 'slaihand_j5_payload_geometry_probe/2',
        'status': 'OFFLINE_GEOMETRY_PROBE_NOT_A_DEPLOYABLE_MODEL',
        'convention': {'urdf': str(URDF.relative_to(ROOT)), 'j3_offset_rad': math.pi,
                       'j5_origin': 'link4 origin (five-axis placeholder)',
                       'j5_axis_link4': [0., 0., 1.], 'gravity_base_m_s2': G.tolist()},
        'kp_axis_nm_per_rad': kp.tolist(),
        'observed_pre_slip_sag_deg': np.degrees(q_pre).tolist(),
        'observed_post_slip_sag_deg': np.degrees(q_post).tolist(),
        'observed_sag_change_torque_nm_J1_J4': delta_tau[:4].tolist(),
        'fitted_com_link5_mm': (c_fit * 1000).tolist(),
        'slip_fit_residual_nm_J1_J4': (dA[:4] @ c_fit + do[:4] - delta_tau[:4]).tolist(),
        'slip_fit_predicted_d_tau5_nm': float(dA[4] @ c_fit + do[4]),
        'codex_com_link4_mm': (com_codex_link4 * 1000).tolist(),
        'codex_com_link5_mm': (com_codex * 1000).tolist(),
        'codex_predicted_d_tau_nm': (dA @ com_codex + do).tolist(),
        'j5_effort_curve': {'q5_deg': q5_order.tolist(), 'observed_nm': eff.tolist(),
                            'codex_payload_predicted_nm': pred5.tolist(),
                            'constant_offset_nm': offset, 'residual_nm': residual.tolist(),
                            'rmse_nm': float(np.sqrt(np.mean(residual ** 2)))},
        'caveat': ('Sag-change torques assume the post-slip state is a gravity equilibrium; '
                   'residual friction bias after the slip is not independently measured, so '
                   'this is a conditional test, not a deployed model.'),
    }
    (HERE / 'j5_payload_geometry_probe.json').write_text(
        json.dumps(out, indent=2) + '\n', encoding='utf-8')
    print('\nwritten', HERE / 'j5_payload_geometry_probe.json')


if __name__ == '__main__':
    main()
