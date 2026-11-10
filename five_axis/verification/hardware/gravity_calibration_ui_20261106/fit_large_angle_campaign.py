"""Offline paired fit for the 2026-11-06 large-angle campaign (L01-L04 / V02).

Runs against the deployment workspace's patched calibration modules, which accept the
identified gravity profile status used on the real machine.
"""
import hashlib
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws')
DEPLOY = Path('/home/xuyu/slaihand_control_panel_ws_gravitycal_deploy_20261106')
SCRIPTS = DEPLOY / 'install/slaihand_bringup/lib/slaihand_bringup'
sys.path.insert(0, str(SCRIPTS))
import calibration_core as core  # noqa: E402
import calibration_capture as capture  # noqa: E402
from control_panel_gravity import FourAxisGravityModel  # noqa: E402

OPERATIONS = {
    'L01': '180d6b74b7d24dc0',
    'L02': '9830153ec4964beb',
    'L03': '69902a2c700546cc',
    'L04': 'ff6b9c57cdb145b0',
    'V02': '4ce01f5f9ef844c6',
}
HOLDOUT_GROUPS = {'V02'}
EXPECTED_ROWS = {'L01': 12, 'L02': 12, 'L03': 12, 'L04': 12, 'V02': 8}
OUT_DIR = Path('/tmp/gravity_large_angle_fit')
OLD_CANDIDATE = DEPLOY / 'install/slaihand_bringup/share/slaihand_bringup/config/gravity_identified_candidate.json'


def load_campaign():
    pairs = []
    sources = []
    expected_identity = None
    for group, operation in OPERATIONS.items():
        directory = Path.home() / '.ros/slaihand/control_panel/operations' / operation / 'client'
        files = sorted(directory.glob('*.json'))
        if not files:
            raise SystemExit('missing result for %s (%s)' % (group, directory))
        result = json.loads(files[-1].read_text(encoding='utf-8'))
        summary = result.get('calibration') or {}
        assert summary.get('group') == group, (group, summary.get('group'))
        assert summary.get('campaign_complete') is True, (group, summary)
        assert result.get('trajectory_error_code') == 0, (group, result.get('trajectory_error_code'))
        assert result.get('stop_observations_confirmed') is True, group
        raw = Path(summary['raw_file'])
        payload = core.load_raw(raw)
        rows = payload['samples']
        assert len(rows) == EXPECTED_ROWS[group], (group, len(rows))
        role = 'holdout' if group in HOLDOUT_GROUPS else 'training'
        for row in rows:
            ok, reasons = core.validate_sample(row)
            assert ok, (group, reasons)
            exclusion = core.exclusion_reason(row)
            assert exclusion is None, (group, exclusion)
            assert row.get('role') == role, (group, row.get('role'), role)
            identity = core._identity(row)
            if expected_identity is None:
                expected_identity = identity
            assert identity == expected_identity, 'mixed configuration identity'
        for index in range(0, len(rows), 4):
            positive, negative = rows[index + 1], rows[index + 3]
            assert positive['pair_id'] == negative['pair_id']
            assert positive['approach'] == 'positive' and negative['approach'] == 'negative'
            assert positive['target_rad'] == negative['target_rad']
            pairs.append({
                'pair_id': positive['pair_id'], 'role': role, 'group': group,
                'positive_q_rad': positive['positions_rad'],
                'negative_q_rad': negative['positions_rad'],
                'mean_estimate_nm': ((np.array(positive['effort_estimate_nm'])
                                      + np.array(negative['effort_estimate_nm'])) / 2).tolist(),
                'half_difference_nm': ((np.array(positive['effort_estimate_nm'])
                                        - np.array(negative['effort_estimate_nm'])) / 2).tolist(),
                'pose_difference_deg': np.degrees(np.array(positive['positions_rad'])
                                                  - np.array(negative['positions_rad'])).tolist(),
            })
        sources.append({'group': group, 'operation_id': operation, 'raw_file': str(raw),
                        'sha256': hashlib.sha256(raw.read_bytes()).hexdigest(),
                        'qualified_points': len(rows)})
    current = capture._expected_identity(capture.provenance('no_end_load'))
    assert core.identity_matches(expected_identity, current), 'analysis resources differ from capture identity'
    return pairs, sources, expected_identity, current


def geometry_regressor(model, q):
    columns = []
    original = model.masses
    try:
        for link in list(original):
            if link == model.root:
                continue
            model.masses = {link: (1., [0., 0., 0.])}
            origin = np.array(model.compute(q)['tau_model_nm'])
            columns.append(origin)
            for axis in range(3):
                center = [0., 0., 0.]
                center[axis] = 1.
                model.masses = {link: (1., center)}
                columns.append(np.array(model.compute(q)['tau_model_nm']) - origin)
    finally:
        model.masses = original
    return np.array(columns).T


def metrics(prediction, measured):
    error = prediction - measured
    return {'rmse_nm': float(np.sqrt(np.mean(error ** 2))),
            'per_axis_rmse_nm': np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
            'per_axis_max_abs_error_nm': np.max(np.abs(error), axis=0).tolist(),
            'prediction_nm': prediction.tolist(),
            'measured_nm': measured.tolist(),
            'error_nm': error.tolist()}


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pairs, sources, identity, current_identity = load_campaign()
    policy = capture._policy_values()
    # geometry regressor requires the mass/com parameterization; use the provisional
    # model for kinematics while identity checks use the deployed identified profile
    profile = capture.PROFILE
    model = FourAxisGravityModel(capture.URDF, profile,
                                 gravity_base=[float(policy['gravity_base_' + axis]) for axis in 'xyz'],
                                 fixed_joint_offsets={'slaihand_joint3': float(policy['gravity_j3_offset_rad'])})
    train = [pair for pair in pairs if pair['role'] == 'training']
    holdout = [pair for pair in pairs if pair['role'] == 'holdout']
    assert len(train) == 12 and len(holdout) == 2, (len(train), len(holdout))

    def matrices(rows):
        return np.stack([(geometry_regressor(model, p['positive_q_rad'])
                          + geometry_regressor(model, p['negative_q_rad'])) / 2 for p in rows])

    xtrain = matrices(train)
    ytrain = np.array([p['mean_estimate_nm'] for p in train])
    design = xtrain.reshape(-1, xtrain.shape[-1])
    coefficients, _, rank, singular = np.linalg.lstsq(design, ytrain.ravel(), rcond=1e-6)
    _, _, right_vectors = np.linalg.svd(design, full_matrices=False)
    provisional_train = np.array([(np.array(model.compute(p['positive_q_rad'])['tau_model_nm'])
                                   + model.compute(p['negative_q_rad'])['tau_model_nm']) / 2
                                  for p in train])
    scale = float(np.sum(provisional_train * ytrain) / np.sum(provisional_train ** 2))
    parameters = []
    names = []
    for link, (mass, com) in model.masses.items():
        if link == model.root:
            continue
        parameters.extend([mass, *[mass * x for x in com]])
        names.extend([link + ':mass', link + ':hx', link + ':hy', link + ':hz'])
    algebra_error = float(np.max(np.abs(xtrain @ np.array(parameters) - provisional_train)))
    assert algebra_error < 1e-10, algebra_error

    xhold = matrices(holdout)
    yhold = np.array([p['mean_estimate_nm'] for p in holdout])
    provisional_hold = np.array([(np.array(model.compute(p['positive_q_rad'])['tau_model_nm'])
                                  + model.compute(p['negative_q_rad'])['tau_model_nm']) / 2
                                 for p in holdout])
    physical = {'kind': 'URDF_linear_gravity_base_parameters', 'rank': int(rank),
                'columns': len(names), 'rcond': 1e-6,
                'singular_values': singular.tolist(),
                'minimum_norm_parameters': dict(zip(names, coefficients.tolist())),
                'train': metrics(xtrain @ coefficients, ytrain),
                'holdout': metrics(xhold @ coefficients, yhold)}
    baseline = {'kind': 'provisional_model_global_scale', 'scale': scale,
                'train': metrics(scale * provisional_train, ytrain),
                'holdout': metrics(scale * provisional_hold, yhold)}
    old = json.loads(OLD_CANDIDATE.read_text(encoding='utf-8'))
    old_holdout = old.get('holdout') or {}
    # Fair gate: evaluate the deployed candidate on the SAME V02 holdout, not its
    # original V01 holdout. Different holdout sets cannot be compared by RMSE directly.
    old_model = FourAxisGravityModel(capture.URDF, OLD_CANDIDATE,
                                     gravity_base=[float(policy['gravity_base_' + axis]) for axis in 'xyz'],
                                     fixed_joint_offsets={'slaihand_joint3': float(policy['gravity_j3_offset_rad'])})
    old_hold_pred = np.array([
        (np.array(old_model.compute(p['positive_q_rad'])['tau_model_nm'])
         + np.array(old_model.compute(p['negative_q_rad'])['tau_model_nm'])) / 2
        for p in holdout])
    old_hold_metrics = metrics(old_hold_pred, yhold)
    old_train_pred = np.array([
        (np.array(old_model.compute(p['positive_q_rad'])['tau_model_nm'])
         + np.array(old_model.compute(p['negative_q_rad'])['tau_model_nm'])) / 2
        for p in train])
    old_train_metrics = metrics(old_train_pred, ytrain)
    gate = {
        'new_holdout_rmse_nm': physical['holdout']['rmse_nm'],
        'old_candidate_same_holdout_rmse_nm': old_hold_metrics['rmse_nm'],
        'new_per_axis_max_abs_nm': physical['holdout']['per_axis_max_abs_error_nm'],
        'old_candidate_same_holdout_per_axis_max_abs_nm': old_hold_metrics['per_axis_max_abs_error_nm'],
        'rmse_not_worse': physical['holdout']['rmse_nm'] <= old_hold_metrics['rmse_nm'] + 1e-9,
        'per_axis_not_worse': all(a <= b + 1e-9 for a, b in zip(
            physical['holdout']['per_axis_max_abs_error_nm'],
            old_hold_metrics['per_axis_max_abs_error_nm'])),
        'old_candidate_reported_v01_holdout_rmse_nm': old_holdout.get('rmse_nm'),
    }
    gate['accepted'] = bool(gate['rmse_not_worse'] and gate['per_axis_not_worse'])
    report_old = {'old_candidate_same_holdout': old_hold_metrics,
                  'old_candidate_training': old_train_metrics}

    report = {'schema': 'paired_gravity_offline_review/1', 'campaign': 'large_angle_20261106',
              'sources': sources, 'identity': identity, 'current_identity': current_identity,
              'training_pairs': len(train), 'independent_validation_pairs': len(holdout),
              'pairs': pairs, 'regressor_algebra_error_nm': algebra_error,
              'baseline': baseline, 'geometry_fit': physical, 'gate': gate,
              'old_candidate_evaluation': report_old,
              'acceptance_rmse_nm': 0.1576, 'parameters_applied': False,
              'mass_profile_written': False, 'alpha_changed': False}
    candidate = {
        'schema': 'slaihand_identified_gravity_base/1',
        'status': 'LOCAL_OFFLINE_CANDIDATE',
        'identity': identity,
        'sources': sources,
        'training_pair_ids': [pair['pair_id'] for pair in train],
        'validation_pair_ids': [pair['pair_id'] for pair in holdout],
        'parameter_slots': names,
        'effective_coefficients': coefficients.tolist(),
        'identifiable_rank': int(rank),
        'identifiable_basis': right_vectors[:rank].tolist(),
        'base_coefficients': (right_vectors[:rank] @ coefficients).tolist(),
        'observed_training_bounds_rad': np.stack([
            np.min([q for pair in train for q in (pair['positive_q_rad'], pair['negative_q_rad'])], axis=0),
            np.max([q for pair in train for q in (pair['positive_q_rad'], pair['negative_q_rad'])], axis=0),
        ]).T.tolist(),
        'prior_mass_values_used_in_fit': False,
        'individual_link_masses_identified': False,
        'holdout': physical['holdout'],
        'parameters_applied': False,
    }
    gains = set()
    for source in sources:
        header = core.load_raw(source['raw_file'])['header']
        runtime = header['runtime']['identity']
        gains.add((runtime['kp'], runtime['kd']))
    assert len(gains) == 1, 'capture gains changed within campaign'
    kp, kd = next(iter(gains))
    candidate['training_gains'] = {'kp': kp, 'kd': kd}
    candidate_path = OUT_DIR / 'gravity_large_angle_candidate.json'
    candidate_path.write_text(json.dumps(candidate, ensure_ascii=False, indent=2), encoding='utf-8')
    report['candidate_file'] = str(candidate_path)
    (OUT_DIR / 'gravity_large_angle_fit_report.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({
        'sources': sources, 'training_pairs': len(train), 'holdout_pairs': len(holdout),
        'rank': int(rank), 'columns': len(names),
        'baseline_train_rmse': baseline['train']['rmse_nm'],
        'baseline_holdout_rmse': baseline['holdout']['rmse_nm'],
        'new_train_rmse': physical['train']['rmse_nm'],
        'new_holdout_rmse': physical['holdout']['rmse_nm'],
        'per_axis_holdout_max_abs': physical['holdout']['per_axis_max_abs_error_nm'],
        'old_candidate_same_holdout_rmse': old_hold_metrics['rmse_nm'],
        'old_candidate_same_holdout_per_axis_max_abs': old_hold_metrics['per_axis_max_abs_error_nm'],
        'gate': gate, 'candidate': str(candidate_path),
    }, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
