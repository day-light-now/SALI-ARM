"""Convert any joint plan into a static-gravity-compensated plan.

Uses the already fitted sag model (sag_model_20261110.json): command = desired -
predicted sag. First/last waypoints stay at their absolute-zero values. All five
joints are compensated; per-waypoint move/dwell times are preserved.

Safety guards (refuse to extrapolate):
  * every waypoint must stay inside the training pose domain (T1a/T1b) plus a
    small margin;
  * the predicted sag magnitude must not exceed the training sag envelope.
A plan that fails these checks is rejected instead of emitting unsafe commands.
"""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODEL = HERE / 'sag_model_20261110.json'
HYSTERESIS_MODEL = HERE / 'hysteresis_model_20261111.json'
URDF = ROOT / 'src/slaihand_description/urdf/slaihand_five_axis.urdf'
TRAIN_TAGS = ('T1a', 'T1b')
NAMES = ['slaihand_joint%d' % i for i in range(1, 6)]
DOMAIN_MARGIN_DEG = 2.0
SAG_LIMIT_DEG = 3.0


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def features(poses_deg):
    q = np.asarray(poses_deg, dtype=float) / 10.0
    columns = [np.ones(len(q)), q[:, 0], q[:, 1], q[:, 2], q[:, 3], q[:, 4],
               q[:, 1] ** 2, q[:, 2] ** 2, q[:, 3] ** 2,
               q[:, 0] * q[:, 1], q[:, 2] * q[:, 3]]
    return np.stack(columns, axis=1)


def limits_deg():
    root = ET.parse(URDF).getroot()
    values = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        limit = joint.find('limit')
        if name in NAMES and limit is not None:
            try:
                low = float(limit.get('lower'))
                high = float(limit.get('upper'))
            except (TypeError, ValueError):
                continue
            values[name] = (float(np.degrees(low)), float(np.degrees(high)))
    return values


def training_domain():
    rows, sags = [], []
    for tag in TRAIN_TAGS:
        path = HERE / ('windows_%s.json' % tag)
        if not path.is_file():
            continue
        data = read(path)
        for window in data['windows']:
            if 'sag_deg' not in window:
                continue
            desired = window.get('desired_deg', window.get('target_deg'))
            if desired is None:
                continue
            rows.append([float(v) for v in desired])
            sags.append([float(v) for v in window['sag_deg']])
    if not rows:
        raise RuntimeError('training windows missing; cannot build domain guard')
    q = np.array(rows)
    sag = np.array(sags)
    return {'low_deg': (q.min(axis=0) - DOMAIN_MARGIN_DEG).tolist(),
            'high_deg': (q.max(axis=0) + DOMAIN_MARGIN_DEG).tolist(),
            'train_sag_max_deg': float(np.max(np.abs(sag)))}


def approach_signs(desired_deg):
    """Sign of the planned motion into each waypoint (per joint)."""
    desired = np.asarray(desired_deg, dtype=float)
    signs = np.zeros_like(desired)
    signs[1:] = np.sign(np.diff(desired, axis=0))
    return signs


def hysteresis_correction(desired_deg, model=None):
    """Command correction -delta*sign to cancel the measured direction bias."""
    model = model or read(HYSTERESIS_MODEL)
    delta = np.asarray(model['delta_deg'], dtype=float)
    signs = approach_signs(desired_deg)
    correction = delta[None, :] * signs
    return correction, model, signs


def compute(desired_deg, model=None, hysteresis=False):
    model = model or read(MODEL)['model']
    desired = np.asarray(desired_deg, dtype=float)
    F = features(desired)
    predicted = np.stack([F @ np.array(model[str(j)]['coef']) for j in range(5)], axis=1)
    compensated = desired - predicted
    hcorr = np.zeros_like(desired)
    hmodel = None
    if hysteresis:
        hcorr, hmodel, _ = hysteresis_correction(desired_deg)
        compensated = compensated - hcorr
    # Keep absolute-zero endpoints (acquisition plan safety invariant).
    compensated[0] = desired[0]
    compensated[-1] = desired[-1]
    domain = training_domain()
    problems = []
    for index, row in enumerate(desired):
        for joint, value in enumerate(row):
            if value < domain['low_deg'][joint] - 1e-9 or value > domain['high_deg'][joint] + 1e-9:
                problems.append('waypoint %d %s=%.2f outside training domain [%.2f, %.2f]' % (
                    index, NAMES[joint], value,
                    domain['low_deg'][joint], domain['high_deg'][joint]))
    sag_limit = max(SAG_LIMIT_DEG, 1.25 * domain['train_sag_max_deg'])
    if float(np.max(np.abs(predicted))) > sag_limit:
        problems.append('predicted sag %.2f deg exceeds limit %.2f deg (extrapolation)' % (
            float(np.max(np.abs(predicted))), sag_limit))
    return {'desired': desired, 'predicted': predicted,
            'compensated': compensated, 'domain': domain,
            'sag_limit_deg': sag_limit, 'problems': problems,
            'hysteresis_correction': hcorr, 'hysteresis_model': hmodel}


def main():
    argv = [arg for arg in sys.argv[1:] if arg != '--hysteresis']
    use_hysteresis = '--hysteresis' in sys.argv[1:]
    if not argv:
        print('USAGE: compensate_plan.py <plan.json> [output_name] [--hysteresis]')
        return 2
    plan_path = Path(argv[0])
    out_name = argv[1] if len(argv) > 1 else plan_path.stem + '_comp'
    plan = read(plan_path)
    waypoints = plan['waypoints']
    desired = np.array([w['degrees'] for w in waypoints], dtype=float)
    result = compute(desired, hysteresis=use_hysteresis)
    if result['problems']:
        print('REJECTED: plan is outside the validated static-sag domain')
        for problem in result['problems'][:8]:
            print('  -', problem)
        return 1
    compensated = result['compensated']
    limits = limits_deg()
    violations = []
    for index, row in enumerate(compensated):
        for joint, value in enumerate(row):
            name = NAMES[joint]
            if name in limits:
                low, high = limits[name]
                if value < low - 1e-9 or value > high + 1e-9:
                    violations.append({'waypoint': index, 'joint': name,
                                       'command_deg': float(value),
                                       'limits_deg': [low, high]})
    if violations:
        print('REJECTED: compensated commands exceed URDF limits: %s'
              % json.dumps(violations[:5]))
        return 1

    output = {'schema': 'slaihand_joint_plan/2', 'coordinates': 'absolute_deg',
              'waypoints': [{'degrees': [float(v) for v in row],
                             'move_s': float(w.get('move_s', 6.0)),
                             'dwell_s': float(w.get('dwell_s', 6.0))}
                            for row, w in zip(compensated, waypoints)]}
    out_path = HERE / 'plans' / ('%s_20261110.json' % out_name)
    out_path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8')
    report = {'schema': 'slaihand_compensated_plan/1',
              'source_plan': str(plan_path),
              'model': str(MODEL),
              'output_plan': str(out_path),
              'training_domain_deg': result['domain'],
              'predicted_sag_limit_deg': result['sag_limit_deg'],
              'desired_deg': np.round(result['desired'], 4).tolist(),
              'predicted_sag_deg': np.round(result['predicted'], 4).tolist(),
              'compensated_deg': np.round(compensated, 4).tolist(),
              'hysteresis_enabled': bool(use_hysteresis),
              'hysteresis_delta_deg': (None if result['hysteresis_model'] is None else
                                        result['hysteresis_model']['delta_deg']),
              'hysteresis_correction_deg': np.round(
                  result['hysteresis_correction'], 4).tolist(),
              'limits_checked': True}
    (HERE / ('comp_plan_%s_20261110.json' % out_name)).write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('wrote', out_path)
    for index in range(len(desired)):
        print(' w%d desired=%s sag=%s cmd=%s' % (
            index, np.round(result['desired'][index], 2).tolist(),
            np.round(result['predicted'][index], 3).tolist(),
            np.round(compensated[index], 3).tolist()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
