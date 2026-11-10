"""Offline refit; immutable sources, predefined splits, no controller imports.

GC08 is used only for final reporting. Direction terms describe approach history,
not a deployable friction law. Every prediction uses the actual feedback pose.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
OLD = BASE / 'five_dof_identification_20261109'
NEW = BASE / 'gravity_sampling_v2_20261109/execution_20261109'
sys.path.insert(0, str(OLD))
from fit_five_dof import FiveDofPotential, rigid_parameters, legacy

HISTORICAL_HOLDOUT = ('C', 'GC03', 'GC04')
PRIMARY = 'rigid_J5_direction'
VARIANTS = ('fixed_J5_direction', 'free_orbit_J5_direction', 'rigid_gravity_only',
            PRIMARY, 'rigid_all_directions')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def save(name, value):
    (HERE / name).write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                      allow_nan=False) + '\n', encoding='utf-8')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local(path):
    """Map archived WSL workspace paths without consulting live ROS/device state."""
    tail = path.split('/verification/hardware/five_axis_20261107/', 1)[1]
    return BASE / tail


def prepare():
    manifest = read(OLD / 'SOURCE_MANIFEST.json')
    # Bind the previous extraction's client/plan hashes as well as raw data.
    old_windows = read(OLD / 'static_windows.json')
    for path, expected in old_windows['source_hashes'].items():
        if sha(BASE / path) != expected:
            raise ValueError('old source hash changed: ' + path)
    execution = read(NEW / 'EXECUTION_MANIFEST_20261109.json')
    for field in ('artifact', 'evidence'):
        if sha(local(execution['admission'][field])) != execution['admission'][field + '_sha256']:
            raise ValueError('new admission hash mismatch')
    manifest['additional_admission'] = execution['admission']
    manifest['upstream_manifests_sha256'] = {
        'old': sha(OLD / 'SOURCE_MANIFEST.json'),
        'new': sha(NEW / 'EXECUTION_MANIFEST_20261109.json')}
    manifest['source_commit'] = 'e48f2c6'
    for plan in execution['plans']:
        client = plan['primary_client']
        for path, expected in ((plan['raw_file'], plan['raw_sha256']),
                               (plan['plan_file'], plan['plan_sha256']),
                               (client['file'], client['sha256'])):
            if sha(local(path)) != expected:
                raise ValueError('new source hash mismatch: ' + path)
        if client['trajectory_error_code'] != 0 or client['mode'] != 'waypoints':
            raise ValueError('refused/non-executed client cannot enter extraction')
        manifest['sessions'].append({
            'name': plan['tag'].upper(),
            'state': local(plan['raw_file']).relative_to(BASE).as_posix(),
            'state_sha256': plan['raw_sha256'],
            'plan': local(plan['plan_file']).relative_to(BASE).as_posix(),
            'client': local(client['file']).relative_to(BASE).as_posix()})
    audits = read(OLD / 'audit_sources.json')
    for entry in audits:
        if sha(OLD / entry['archive']) != entry['archive_sha256']:
            raise ValueError('old audit archive mismatch')
        shutil.copyfile(OLD / entry['archive'], HERE / entry['archive'])
    for source in execution['audit']:
        entry = dict(source)
        source_path = local(entry['archive'])
        if sha(source_path) != entry['archive_sha256']:
            raise ValueError('new audit archive mismatch')
        entry['archive'] = source_path.name
        shutil.copyfile(source_path, HERE / entry['archive'])
        audits.append(entry)
    save('SOURCE_MANIFEST.json', manifest)
    save('audit_sources.json', audits)
    extracted = legacy.extract(data_dir=BASE, output_dir=HERE,
                               manifest_path=HERE / 'SOURCE_MANIFEST.json')
    # Independent extractor has no audit join; differences are recorded, not hidden.
    checks = []
    independent = read(NEW / 'gc_windows_20261109.json')
    for plan in independent['plans']:
        for w in plan['windows']:
            if 'excluded' in w:
                continue
            identifier = f"{plan['tag'].upper()}_{w['index']}"
            ours = next(p for p in extracted['poses'] if p['id'] == identifier)
            checks.append({'id': identifier,
                'max_q_difference_deg': float(np.max(np.abs(np.degrees(ours['q_rad']) - w['q_deg']))),
                'max_effort_difference_nm': float(np.max(np.abs(np.array(ours['effort_mean_nm']) - w['effort_mean_nm']))),
                'our_samples': ours['samples'], 'independent_samples': w['samples']})
    save('extraction_crosscheck.json', checks)
    return extracted


def attach_directions(poses, manifest):
    clients = {s['name']: read(BASE / s['client']) for s in manifest['sessions']}
    for p in poses:
        client = clients[p['session']]
        last = np.zeros(5)
        for i in range(p['waypoint'] + 1):
            previous = client['initial_model_rad'] if i == 0 else client['waypoint_results'][i - 1]['target_rad']
            delta = np.array(client['waypoint_results'][i]['target_rad']) - previous
            # Sub-degree near-zero entry corrections do not establish approach history.
            signs = np.where(np.abs(delta) > math.radians(.5), np.sign(delta), 0)
            last = np.where(signs != 0, signs, last)
        p['last_planned_approach_sign'] = last.astype(int).tolist()


def splits(poses):
    historical = np.array([p['session'] in HISTORICAL_HOLDOUT or
                           p['id'] in ('J5_3', 'J5_4') for p in poses])
    final = np.array([p['session'] == 'GC08' for p in poses])
    train = ~(historical | final)
    return train, historical, final


def observations(poses, directions):
    mask = np.ones((len(poses), 5), dtype=bool)
    # Old constant-q5 sessions and unknown initial approach do not identify J5 gravity.
    mask[:, 4] = [p['session'] in ('J5', 'GC05', 'GC06', 'GC07', 'GC08')
                   and directions[i, 4] != 0 for i, p in enumerate(poses)]
    return mask


def direction_design(directions, n):
    out = np.zeros((len(directions), 5, n))
    if n == 1:
        out[:, 4, 0] = directions[:, 4]
    elif n == 5:
        for axis in range(5):
            out[:, axis, axis] = directions[:, axis]
    return out


def fit(kind, x, base, y, directions, rows):
    n = 0 if kind == 'rigid_gravity_only' else 5 if kind == 'rigid_all_directions' else 1
    d = direction_design(directions, n)
    if kind.startswith('fixed') or kind.startswith('free'):
        k = 3 if kind.startswith('fixed') else 9
        design = np.concatenate([x[:, :, :k], d], axis=2)
        par, _, rank, sig = np.linalg.lstsq(design[rows], (y - base)[rows], rcond=1e-8)
        h = np.r_[par[:k], np.zeros(9 - k)]
        gravity = base + x @ h
        total = gravity + d @ par[k:]
        return {'kind': kind, 'h_kg_m': h.tolist(), 'axis_unverified': None,
                'nuisance_nm': par[k:].tolist(), 'training_cost': float(np.sum((total-y)[rows]**2)/2),
                'rank': int(rank), 'condition_number': float(sig[0]/sig[-1]),
                'optimizer_success': True}, gravity, total
    def predict(z):
        return base + x @ rigid_parameters(z)[0] + d @ z[7:]
    solutions = []
    for theta in np.linspace(-math.pi, math.pi, 6, endpoint=False):
        for phi in (-.8, 0., .8):
            initial = [.1, .01, -.05, .03, .01, theta, phi] + [.8] * n
            solutions.append(least_squares(lambda z: (predict(z)-y)[rows], initial,
                max_nfev=700, ftol=1e-9, xtol=1e-9, gtol=1e-9))
    best = min(solutions, key=lambda r: r.cost)
    h, axis = rigid_parameters(best.x)
    gravity = base + x @ h
    return {'kind': kind, 'h_kg_m': h.tolist(), 'axis_unverified': axis.tolist(),
            'nuisance_nm': best.x[7:].tolist(), 'training_cost': float(best.cost),
            'rank': int(np.linalg.matrix_rank(best.jac)),
            'condition_number': float(np.linalg.cond(best.jac)),
            'optimizer_success': bool(best.success),
            'multistart_cost_range': [float(min(s.cost for s in solutions)), float(max(s.cost for s in solutions))]}, gravity, predict(best.x)


def score(prediction, y, selection, observed):
    result = legacy.metrics(prediction[selection, :4], y[selection, :4])
    mask = selection & observed[:, 4]
    result['J5_known_direction_window_count'] = int(sum(mask))
    result['J5_rmse_nm'] = float(np.sqrt(np.mean((prediction[mask, 4] - y[mask, 4])**2))) if mask.any() else None
    result['J5_max_abs_nm'] = float(np.max(np.abs(prediction[mask, 4] - y[mask, 4]))) if mask.any() else None
    return result


def paired(poses, gravity, total):
    output = []
    for session in ('GC05', 'GC06', 'GC07', 'GC08'):
        offset = 0 if session == 'GC05' else 1
        for a, b in ((1, 3), (5, 7), (4, 8)):
            ia = next(i for i, p in enumerate(poses) if p['id'] == f'{session}_{a+offset}')
            ib = next(i for i, p in enumerate(poses) if p['id'] == f'{session}_{b+offset}')
            ya, yb = (np.array(poses[i]['effort_mean_nm']) for i in (ia, ib))
            actual_mean = (ya + yb) / 2
            # Average the predictions at both ACTUAL poses. Nuisance cancels only
            # for exactly opposing direction terms; never evaluate at mean target.
            gmean = (gravity[ia] + gravity[ib]) / 2
            tmean = (total[ia] + total[ib]) / 2
            output.append({'session': session, 'ids': [poses[ia]['id'], poses[ib]['id']],
                'actual_mean_effort_nm': actual_mean.tolist(),
                'observed_half_difference_nm': ((ya-yb)/2).tolist(),
                'gravity_pair_mean_prediction_nm': gmean.tolist(),
                'gravity_pair_mean_residual_nm': (gmean-actual_mean).tolist(),
                'total_pair_mean_residual_nm': (tmean-actual_mean).tolist(),
                'pose_separation_deg': np.degrees(np.array(poses[ia]['q_rad'])-poses[ib]['q_rad']).tolist()})
    return output


def run():
    data = read(HERE / 'static_windows.json')
    poses = data['poses']
    attach_directions(poses, read(HERE / 'SOURCE_MANIFEST.json'))
    save('static_windows.json', data)
    if len(poses) != 115:
        raise ValueError(f'expected 115 accepted windows, got {len(poses)}')
    model = FiveDofPotential()
    x = np.stack([model.orbit_regressor(p['q_rad']) for p in poses])
    base = np.stack([model.base(p['q_rad']) for p in poses])
    y = np.array([p['effort_mean_nm'] for p in poses])
    directions = np.array([p['last_planned_approach_sign'] for p in poses])
    train, historical, final = splits(poses)
    observed = observations(poses, directions)
    # No final/historical effort values are read by an optimizer.
    results, predictions = [], {}
    for kind in VARIANTS:
        result, gravity, total = fit(kind, x, base, y, directions, observed & train[:, None])
        for name, selection in (('training', train), ('historical_validation', historical), ('GC08_final', final)):
            result[name] = {'gravity_vs_effort': score(gravity, y, selection, observed),
                            'including_nuisance': score(total, y, selection, observed)}
        result['pairs'] = paired(poses, gravity, total)
        h = np.array(result['h_kg_m']).reshape(3, 3)
        result['COM_q5_zero_link4_mm'] = ((h[0]+h[1])/3.5*1000).tolist()
        result['orbit_radii_mm'] = (np.linalg.norm(h[1:], axis=1)/3.5*1000).tolist()
        result['u_dot_v_kg2_m2'] = float(h[1]@h[2])
        result['deployable'] = False
        results.append(result)
        predictions[kind] = {'gravity_nm': gravity.tolist(), 'including_nuisance_nm': total.tolist()}
        print(kind, 'cost', round(result['training_cost'], 3),
              'GC08 upstream/J5', result['GC08_final']['including_nuisance']['rmse_nm'],
              result['GC08_final']['including_nuisance']['J5_rmse_nm'], flush=True)
    # Sensitivity uses training sessions only. GC08 is never used to choose a model.
    sensitivity = []
    for excluded in ('GC05', 'GC06', 'GC07'):
        subset = train & np.array([p['session'] != excluded for p in poses])
        r, _, _ = fit(PRIMARY, x, base, y, directions, observed & subset[:, None])
        h = np.array(r['h_kg_m']).reshape(3, 3)
        sensitivity.append({'excluded_training_session': excluded,
            'COM_q5_zero_link4_mm': ((h[0]+h[1])/3.5*1000).tolist(),
            'axis_unverified': r['axis_unverified'], 'nuisance_nm': r['nuisance_nm']})
        print('sensitivity', excluded, flush=True)
    primary_pairs = next(r for r in results if r['kind'] == PRIMARY)['pairs']
    pair_bias = float(np.mean([p['actual_mean_effort_nm'][4] for p in primary_pairs if p['session'] != 'GC08']))
    final_pair_truth = np.array([p['actual_mean_effort_nm'][4] for p in primary_pairs if p['session'] == 'GC08'])
    report = {'schema': 'slaihand_five_dof_direction_refit/1', 'source_commit': 'e48f2c6',
        'deployable': False, 'selected_for_deployment': None, 'parameters_applied': False,
        'no_hardware_actions': True, 'primary_model_predeclared': PRIMARY,
        'model_selection_on_GC08': False,
        'session_counts': {s: sum(p['session'] == s for p in poses) for s in dict.fromkeys(p['session'] for p in poses)},
        'train_ids': [p['id'] for p, k in zip(poses, train) if k],
        'historical_validation_ids': [p['id'] for p, k in zip(poses, historical) if k],
        'GC08_final_ids': [p['id'] for p, k in zip(poses, final) if k],
        'excluded_windows': data['excluded'],
        'convention': {'gravity_base_m_s2': legacy.G.tolist(), 'J3_offset_rad': math.pi,
            'box_mass_kg': 3.5, 'box_mass_excludes_motor': True,
            'J5_axis_measured': False, 'J5_pivot_measured': False,
            'motor_mass_added_again': False},
        'nuisance': 'last planned approach sign, carried within session; initial sub-0.5deg entry ignored; not a validated friction law',
        'J5_observation_policy': 'J5-excitation sessions only, known approach history; all upstream observations retained regardless of client accuracy criterion',
        'weighting': 'one equally weighted row per axis/window, no overweighting 100Hz samples',
        'models': results, 'training_session_sensitivity': sensitivity,
        'exploratory_constant_J5_pair_mean_control': {
            'training_sessions': ['GC05', 'GC06', 'GC07'], 'training_pair_count': 9,
            'constant_nm': pair_bias, 'GC08_pair_count': 3,
            'GC08_pair_mean_rmse_nm': float(np.sqrt(np.mean((final_pair_truth-pair_bias)**2))),
            'scope': 'post-report sanity control, trained without GC08; not an offset calibration or selected gravity model'},
        'predictions': predictions,
        'source_hashes': {**data['source_hashes'], 'prior_and_geometry': legacy.prior_parameters()[2]}}
    save('refit_result.json', report)
    save('offline_candidates.json', {'deployable': False, 'selected_for_deployment': None,
        'source_result_sha256': sha(HERE/'refit_result.json'),
        'reason': 'Unmeasured spatial geometry; residuals and direction-law sensitivity require independent physical validation.'})
    make_plot(report, poses, y, final)


def make_plot(report, poses, y, final):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    pred = report['predictions'][PRIMARY]
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout='constrained')
    ix = np.flatnonzero(final)
    for name, label in (('gravity_nm', 'Gravity component'), ('including_nuisance_nm', 'Gravity + approach nuisance')):
        axes[0].plot(range(len(ix)), np.array(pred[name])[ix, 4], 'o-', label=label)
    known = np.array([poses[i]['last_planned_approach_sign'][4] != 0 for i in ix])
    axes[0].plot(np.flatnonzero(known), y[ix[known], 4], 'ks--', label='Recorded (scored windows)')
    axes[0].plot(np.flatnonzero(~known), y[ix[~known], 4], 'kx', markersize=10,
                 label='Unknown initial approach (unscored)')
    axes[0].set(xlabel='GC08 window', ylabel='J5 torque [Nm]', title='Independent GC08: gravity and approach separated')
    axes[0].legend(fontsize=8)
    for result in report['models']:
        if result['kind'] == PRIMARY:
            for session, marker in (('GC05', 'o'), ('GC06', 's'), ('GC07', '^'), ('GC08', 'D')):
                pairs = [p for p in result['pairs'] if p['session'] == session]
                axes[1].plot([p['actual_mean_effort_nm'][4] for p in pairs],
                             [p['gravity_pair_mean_prediction_nm'][4] for p in pairs], marker,
                             label=session)
    axes[1].axline((0, 0), slope=1, color='gray', linewidth=1)
    axes[1].set(xlabel='Observed paired J5 mean [Nm]', ylabel='Predicted gravity paired mean [Nm]', title='Each pair evaluated at its actual poses')
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle('115-window offline identification; not deployable')
    fig.savefig(HERE/'refit_diagnostics.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extract', action='store_true', help='verify archived sources and re-extract all windows')
    args = parser.parse_args()
    if args.extract:
        prepare()
    run()
