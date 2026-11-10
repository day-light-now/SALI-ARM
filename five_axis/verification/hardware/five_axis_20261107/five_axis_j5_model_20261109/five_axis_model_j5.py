"""Conditional five-axis gravity fit with a J5-output payload (offline only).

Compares three hypotheses against the accepted 30-pose static set and the
2026-11-09 J5-excitation static set:

  H0  no added payload
  HF  the 3.5 kg box is rigidly fixed to link4 (Codex candidate structure)
  HR  the 3.5 kg box is carried by the J5 output and rotates with q5

The original four-axis identified gravity prior stays fixed; only the payload
centre of mass is fitted. Nothing here is deployable and no FF is enabled.

Reproducible inputs are the committed static window tables. No hardware/ROS.
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
BASE = ROOT / 'verification/hardware/five_axis_20261107'
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_four_axis.urdf'
PRIOR = ROOT / 'src/slaihand_bringup/config/gravity_identified_candidate.json'
G = np.array([0., 9.80665, 0.])
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
MASS_BOX = 3.5


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Kinematics:
    """Canonical J1-J4 URDF (fixed J3 = pi placement) plus a J5 tool-roll joint.

    The J5 joint is the five-axis placeholder: origin identity on link4 and axis
    +Z of the link4/joint5 frame. The payload sits on link5, so it rotates with
    q5. The URDF position/effort limits are not used here.
    """

    def __init__(self):
        self.joints = []
        for joint in ET.parse(URDF).getroot().findall('joint'):
            origin = joint.find('origin')
            t = np.eye(4)
            t[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
            t[:3, :3] = Rotation.from_euler(
                'xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            offset = math.pi if joint.get('name') == NAMES[2] else 0.
            self.joints.append({'name': joint.get('name'), 't': t, 'axis': axis,
                                'offset': offset})

    def frames(self, q):
        world = np.eye(4)
        pivots, axes, after = [], [], []
        for value, joint in zip(q, self.joints):
            world = world @ joint['t']
            pivots.append(world[:3, 3].copy())
            axes.append(world[:3, :3] @ joint['axis'])
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(
                joint['axis'] * (value + joint['offset'])).as_matrix()
            world = world @ motion
            after.append(world.copy())
        return pivots, axes, after

    def prior_regressor(self, q):
        """(4,16) regressor: columns link_i mass, then link_i first moment (i=1..4)."""
        pivots, axes, after = self.frames(q)
        result = np.zeros((4, 16))
        for link, frame in enumerate(after[:4]):
            p, r = frame[:3, 3], frame[:3, :3]
            for joint in range(link + 1):
                result[joint, link * 4] = -axes[joint] @ np.cross(p - pivots[joint], G)
                for k in range(3):
                    result[joint, link * 4 + k + 1] = -axes[joint] @ np.cross(r[:, k], G)
        return result

    def payload_rows(self, q, rotating):
        """(5,3) payload first-moment rows and the mass-column rows.

        Returns (mass_rows (4,), H (5,3)) where the payload holding torque is
        mass * mass_rows_at_origin + H @ (mass * com). For the rotating payload
        the frame is link5; for the fixed payload it is link4.
        """
        pivots, axes, after = self.frames(q)
        if rotating:
            q5 = float(q[4])
            origin = after[3][:3, 3]
            rot = after[3][:3, :3] @ Rotation.from_rotvec([0., 0., q5]).as_matrix()
            frame = np.eye(4)
            frame[:3, 3] = origin
            frame[:3, :3] = rot
            joint_axes = axes + [after[3][:3, :3] @ np.array([0., 0., 1.])]
            frame_pivot = origin
            n_joint = 5
        else:
            frame = after[3]
            joint_axes = axes
            frame_pivot = after[3][:3, 3]
            n_joint = 4
        mass_rows = np.zeros(4)
        H = np.zeros((5, 3))
        for joint in range(n_joint):
            if joint < 4:
                mass_rows[joint] = -joint_axes[joint] @ np.cross(
                    frame_pivot - pivots[joint], G)
            for k in range(3):
                H[joint, k] = -joint_axes[joint] @ np.cross(frame[:3, :3][:, k], G)
        return mass_rows, H


def prior_parameters():
    profile = read(PRIOR)
    expected = [f'link{i}:{f}' for i in range(1, 5) for f in ('mass', 'hx', 'hy', 'hz')]
    if profile['parameter_slots'] != expected:
        raise ValueError('unexpected prior parameter order')
    if not np.allclose(profile['identity']['gravity_base_m_s2'], G, atol=1e-12):
        raise ValueError('prior gravity convention changed')
    if abs(profile['identity']['j3_offset_rad'] - math.pi) > 1e-12:
        raise ValueError('prior J3 offset changed')
    return (np.array(profile['effective_coefficients']),
            {'prior_sha256': sha(PRIOR), 'prior_holdout_rmse_nm':
             profile['holdout']['rmse_nm']})


def metrics(prediction, target):
    residual = np.asarray(prediction) - np.asarray(target)
    return {'rmse_nm': float(np.sqrt(np.mean(residual ** 2))),
            'per_axis_rmse_nm': np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
            'per_axis_max_abs_nm': np.max(np.abs(residual), axis=0).tolist()}


class Dataset:
    def __init__(self):
        poses = read(BASE / 'identification_codex_20261109/static_windows.json')['poses']
        windows = read(HERE / 'static_windows_j5.json')['J5']['windows']
        self.poses = [p for p in poses if 'excluded' not in p]
        self.j5 = [w for w in windows if 'excluded' not in w]
        if len(self.poses) != 30 or len(self.j5) != 6:
            raise ValueError(f'expected 30 + 6 accepted windows, got '
                             f'{len(self.poses)} + {len(self.j5)}')
        self.train = np.array([p['session'] != 'C' for p in self.poses])
        self.hold = ~self.train

    def q(self):
        return np.array([p['q_rad'] for p in self.poses], dtype=float)

    def y(self):
        return np.array([p['effort_mean_nm'] for p in self.poses], dtype=float)

    def q_j5(self):
        return np.array([w['q_rad'] for w in self.j5], dtype=float)

    def y_j5(self):
        return np.array([w['effort_mean_nm'] for w in self.j5], dtype=float)


def build(dataset, kin, prior, hypothesis):
    """Return (A_arm (N,4,16), base (N,4,5), design (N,5,3)) for each hypothesis."""
    q = dataset.q()
    base = np.zeros((len(q), 5))
    design = np.zeros((len(q), 5, 3))
    for i, qi in enumerate(q):
        y4 = kin.prior_regressor(qi[:4]) @ prior
        base[i, :4] = y4
        if hypothesis == 'none':
            continue
        mass_rows, H = kin.payload_rows(qi, rotating=(hypothesis == 'j5'))
        base[i, :4] += MASS_BOX * mass_rows
        design[i] = H
    return base, design


def fit(base, design, y, rows, axes=4):
    """Least-squares payload first moment on the given pose rows and axis set."""
    target = y[:, :axes][rows]
    residual = target - base[:, :axes][rows]
    d = design[:, :axes, :][rows].reshape(-1, 3)
    h, *_ = np.linalg.lstsq(d, residual.ravel(), rcond=None)
    return h


def evaluate(base, design, y, rows, h, axes=4):
    prediction = base[:, :axes] + design[:, :axes, :] @ h
    return metrics(prediction[rows], y[rows, :axes])


def main():
    prior, provenance = prior_parameters()
    dataset = Dataset()
    kin = Kinematics()
    y = dataset.y()
    y5 = dataset.y_j5()
    q5 = dataset.q_j5()
    print(f"prior holdout RMSE {provenance['prior_holdout_rmse_nm']:.3f} Nm; "
          f"30 poses + 6 J5 windows")

    result = {'schema': 'slaihand_five_axis_payload_fit/1',
              'status': 'OFFLINE_CONDITIONAL_FIT_NOT_DEPLOYABLE',
              'deployable': False, 'gravity_ff_alpha_changed': False,
              'hypotheses': ['no_added_payload', 'fixed_to_link4', 'rotating_with_j5'],
              'prior': provenance, 'box_mass_kg': MASS_BOX,
              'mass_source': 'operator estimate, not measured',
              'j3_offset_rad': math.pi, 'gravity_base_m_s2': G.tolist(),
              'fit_target': 'DAMIAO effort estimate (not current/torque measurement)'}

    fits = {}
    for hypothesis in ('none', 'fixed', 'j5'):
        base, design = build(dataset, kin, prior, hypothesis)
        if hypothesis == 'none':
            fits[hypothesis] = {'com_m': [0., 0., 0.], 'train': {}, 'holdout': {},
                                'j5_windows': {}}
            h = np.zeros(3)
        else:
            h = fit(base, design, y, dataset.train)
            fits[hypothesis] = {'com_m': (h / MASS_BOX).tolist(),
                                'first_moment_kg_m': h.tolist(),
                                'train': evaluate(base, design, y, dataset.train, h),
                                'holdout': evaluate(base, design, y, dataset.hold, h),
                                'j5_windows': {}}
        # Predict the six J5-excitation windows (J1..J5).
        qstack = q5
        base5 = []
        design5 = []
        for qi in qstack:
            y4 = kin.prior_regressor(qi[:4]) @ prior
            row_base = np.zeros(5)
            row_base[:4] = y4
            row_design = np.zeros((5, 3))
            if hypothesis != 'none':
                mass_rows, H = kin.payload_rows(qi, rotating=(hypothesis == 'j5'))
                row_base[:4] += MASS_BOX * mass_rows
                row_design = H
            base5.append(row_base)
            design5.append(row_design)
        base5 = np.array(base5)
        design5 = np.array(design5)
        pred5 = base5 + design5 @ h
        fits[hypothesis]['j5_windows'] = metrics(pred5, y5)
        fits[hypothesis]['j5_predictions_nm'] = pred5.tolist()
        fits[hypothesis]['j5_observed_nm'] = y5.tolist()

    # Fit with the J5 excitation windows added (upstream rows only: the J5 row is
    # not used because mechanism-level stiction/harness torque is not gravity).
    base_f, design_f = build(dataset, kin, prior, 'fixed')
    base_r, design_r = build(dataset, kin, prior, 'j5')
    q_all = dataset.q()
    extra_cache = {}
    for hypothesis in ('fixed', 'j5'):
        base_extra = np.zeros((6, 5))
        design_extra = np.zeros((6, 5, 3))
        for i, qi in enumerate(q5):
            mass_rows, H = kin.payload_rows(qi, rotating=(hypothesis == 'j5'))
            base_extra[i, :4] = kin.prior_regressor(qi[:4]) @ prior \
                + MASS_BOX * mass_rows
            design_extra[i] = H
        extra_cache[hypothesis] = (base_extra, design_extra)
    for name, base, design in (('fixed', base_f, design_f), ('j5', base_r, design_r)):
        base_j5_extra, design_j5_extra = extra_cache[name]
        stacked_base = np.concatenate([base, base_j5_extra])
        stacked_design = np.concatenate([design, design_j5_extra])
        stacked_y = np.concatenate([y, y5])
        rows = np.concatenate([dataset.train, np.ones(6, dtype=bool)])
        h = fit(stacked_base, stacked_design, stacked_y, rows, axes=4)
        fits[name]['train_plus_j5_upstream'] = {
            'com_m': (h / MASS_BOX).tolist(),
            'train_metrics': evaluate(stacked_base, stacked_design, stacked_y, rows, h),
            'holdout_metrics': evaluate(base, design, y, dataset.hold, h)}

    # Slip cross-check: J1..J4 holding-torque change between J5=+20 and -20 deg.
    q_pre = np.radians([0.306, 0.131, 0.0, 0.437, 20.0])
    q_post = np.radians([0.040, 0.131, 0.0, 0.130, -20.0])
    kp = np.array([160.81, 160.44, 159.95, 160.81, 159.95])  # recorded kp_encoded at the near-zero pose
    observed = np.concatenate([-(kp[:4] * q_post[:4]) + kp[:4] * q_pre[:4], [0.]])
    slip = {'observed_sag_change_nm_J1_J4': observed[:4].tolist()}
    for hypothesis in ('fixed', 'j5'):
        base_pre, design_pre = build_single(kin, prior, q_pre, hypothesis)
        base_post, design_post = build_single(kin, prior, q_post, hypothesis)
        h = np.array(fits[hypothesis]['first_moment_kg_m'])
        d = (base_post - base_pre) + (design_post - design_pre) @ h
        slip[hypothesis] = {'predicted_d_tau_nm': d.tolist(),
                            'mismatch_J1_J4_nm': (d[:4] - observed[:4]).tolist()}

    result['fits'] = fits
    result['stiction_slip_check'] = slip
    result['identifiability_note'] = (
        'The 30-pose set holds q5 nearly constant (2.21-2.23 deg, one pose 0.31 deg), '
        'so it cannot separate a payload fixed to link4 from a payload carried by J5. '
        'The six J5 windows vary q5 while J1-J4 stay near absolute zero, so they add '
        'little independent pose information but do constrain the q5 dependence.')

    out = HERE / 'five_axis_payload_fit_result.json'
    out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')

    print('\nhypothesis                 COM link frame [mm]            train RMSE  holdout RMSE (C)  J5-window RMSE')
    for hypothesis in ('none', 'fixed', 'j5'):
        f = fits[hypothesis]
        com = np.round(np.array(f['com_m']) * 1000, 1)
        train = f['train'].get('rmse_nm')
        hold = f['holdout'].get('rmse_nm')
        j5m = f['j5_windows']['rmse_nm']
        print(f'{hypothesis:26s} {str(com):32s} {train!s:>11} {hold!s:>18} {j5m:>16.3f}')
    print('\nJ5 windows observed vs predicted (J1..J5, Nm)')
    for i, row in enumerate(y5):
        print(f'  q5={np.degrees(q5[i, 4]):+7.2f}  obs ' +
              ' '.join(f'{v:+6.2f}' for v in row) +
              '   none ' + ' '.join(f'{v:+6.2f}' for v in fits['none']['j5_predictions_nm'][i]))
        print('                     fixed ' +
              ' '.join(f'{v:+6.2f}' for v in fits['fixed']['j5_predictions_nm'][i]) +
              '   j5 ' + ' '.join(f'{v:+6.2f}' for v in fits['j5']['j5_predictions_nm'][i]))
    print('\nstiction slip check (J1..J5): observed J1..J4', np.round(observed[:4], 3),
          ' fixed', np.round(slip['fixed']['predicted_d_tau_nm'], 3),
          ' j5', np.round(slip['j5']['predicted_d_tau_nm'], 3))
    print('\nwritten', out)


def build_single(kin, prior, q, hypothesis):
    base = np.zeros(5)
    base[:4] = kin.prior_regressor(q[:4]) @ prior
    design = np.zeros((5, 3))
    if hypothesis != 'none':
        mass_rows, H = kin.payload_rows(q, rotating=(hypothesis == 'j5'))
        base[:4] += MASS_BOX * mass_rows
        design = H
    return base, design


if __name__ == '__main__':
    main()
