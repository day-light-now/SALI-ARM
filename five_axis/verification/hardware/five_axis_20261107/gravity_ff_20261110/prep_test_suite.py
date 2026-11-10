"""Prepare the hardware test suite (offline; no driver, no motion).

Generates in-domain held-out plans (uncompensated + model-compensated), a
repeatability plan and a J5-focused plan; validates each plan with the project
validator, computes recorder durations and expected sag; writes a manifest and
a ready-to-run test list for the next hardware session.
"""
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PLANS = HERE / 'plans'
MANIFEST = HERE / 'test_suite_manifest_20261110.json'

spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_plan(points, path):
    waypoints = [{'degrees': [float(v) for v in row], 'move_s': 6.0, 'dwell_s': 6.0}
                 for row in points]
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': waypoints},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def training_poses():
    rows = []
    for tag in cp.TRAIN_TAGS:
        path = HERE / ('windows_%s.json' % tag)
        if not path.is_file():
            continue
        for window in read(path)['windows']:
            desired = window.get('desired_deg', window.get('target_deg'))
            if desired is not None and 'sag_deg' in window:
                rows.append([float(v) for v in desired])
    return np.array(rows)


def choose_holdout(train, domain):
    low = np.array(domain['low_deg']) + 0.5
    high = np.array(domain['high_deg']) - 0.5
    rng = np.random.default_rng(20261110)
    chosen = []
    tries = 0
    while len(chosen) < 8 and tries < 20000:
        tries += 1
        pose = np.round(rng.uniform(low, high) * 2.0) / 2.0
        if np.any(pose < low) or np.any(pose > high):
            continue
        if np.min(np.max(np.abs(train - pose), axis=1)) < 2.0:
            continue
        if chosen and np.min(np.max(np.abs(np.array(chosen) - pose), axis=1)) < 3.0:
            continue
        predicted = cp.compute(np.array([pose]))['predicted'][0]
        if float(np.max(np.abs(predicted))) > 2.0:
            continue
        chosen.append(pose)
    return np.array(chosen)


def validate(path):
    result = subprocess.run([sys.executable, str(HERE / 'validate_plan.py'), str(path)],
                            capture_output=True, text=True)
    duration = subprocess.run([sys.executable, str(HERE / 'plan_duration.py'), str(path)],
                              capture_output=True, text=True)
    return {'ok': result.returncode == 0, 'message': (result.stdout + result.stderr).strip(),
            'recorder_s': int(duration.stdout.strip()) if duration.returncode == 0 else None}


def build_entry(role, points, name, compensated=False):
    path = PLANS / ('%s_20261110.json' % name)
    write_plan(points, path)
    check = validate(path)
    desired = np.array(points, dtype=float)
    predicted = cp.compute(desired)['predicted']
    entry = {'role': role, 'plan': str(path), 'name': name,
             'waypoints': len(points), 'compensated': compensated,
             'predicted_sag_max_deg': round(float(np.max(np.abs(predicted))), 4),
             'recorder_s': check['recorder_s'], 'valid': check['ok'],
             'validator': check['message']}
    return entry


def main():
    domain = cp.training_domain()
    train = training_poses()
    print('training poses', len(train))
    print('domain low ', np.round(domain['low_deg'], 2).tolist())
    print('domain high', np.round(domain['high_deg'], 2).tolist())
    holdout = choose_holdout(train, domain)
    if len(holdout) < 8:
        print('could not find 8 holdout poses; got', len(holdout))
    print('holdout poses:')
    for pose in holdout:
        print(' ', pose.tolist())

    zero = np.zeros(5)
    h1 = np.vstack([zero, holdout, zero])
    entries = []
    entries.append(build_entry('H1 baseline (uncompensated)', h1, 'H1_holdout'))

    comp = cp.compute(h1)
    if not comp['problems']:
        entries.append(build_entry('H1 model-compensated', comp['compensated'], 'H1_holdout_comp', True))
    else:
        print('H1 compensation rejected:', comp['problems'][:3])

    # Repeatability: approach one holdout pose from two directions.
    p = holdout[0]
    delta = np.zeros(5)
    delta[0] = 2.0
    delta[1] = 2.0
    up = p + delta
    down = p - delta
    if np.any(up > np.array(domain['high_deg'])) or np.any(down < np.array(domain['low_deg'])):
        delta /= 2.0
        up = p + delta
        down = p - delta
    h2 = np.vstack([zero, up, p, down, p, zero])
    entries.append(build_entry('H2 repeatability (same pose, two approach sides)', h2, 'H2_repeat'))

    # J5-focused in-domain plan (box present; J5 gravity is small, covers sign change).
    j5_pose = np.array([0.0, 8.0, 0.0, 0.0, 0.0])
    j5_a = j5_pose.copy(); j5_a[4] = -8.0
    j5_b = j5_pose.copy(); j5_b[4] = 8.0
    h3 = np.vstack([zero, j5_a, j5_pose, j5_b, zero])
    entries.append(build_entry('H3 J5 sign sweep (in domain)', h3, 'H3_j5'))
    comp3 = cp.compute(h3)
    if not comp3['problems']:
        entries.append(build_entry('H3 J5 sign sweep compensated', comp3['compensated'], 'H3_j5_comp', True))

    # H4: same pose approached from three directions to expose stiction/hysteresis.
    p4 = holdout[1]
    delta_a = np.zeros(5); delta_a[0] = 2.0; delta_a[1] = 2.0
    delta_b = np.zeros(5); delta_b[0] = -2.0; delta_b[1] = 2.0
    for delta in (delta_a, delta_b):
        for scale in (1.0, 0.5):
            candidate = p4 + delta * scale
            if np.all(candidate <= np.array(domain['high_deg'])) and \
               np.all(candidate >= np.array(domain['low_deg'])):
                delta *= scale
                break
    h4 = np.vstack([zero, p4 + delta_a, p4, p4 - delta_a, p4, p4 + delta_b, p4, zero])
    entries.append(build_entry('H4 stiction repeatability (3 approaches)', h4, 'H4_stiction'))

    manifest = {'schema': 'slaihand_hardware_test_suite/1',
                'model': str(cp.MODEL),
                'training_domain_deg': domain,
                'thresholds': {
                    'following_error_deg': 4.0,
                    'H1_baseline_sag_rms_deg_min': 0.3,
                    'H1_comp_residual_rms_deg_max': 0.6,
                    'H2_repeat_spread_deg_max': 0.25,
                    'H4_stiction_spread_deg_max': 0.4,
                    'abort_on_any_following_error': True,
                },
                'entries': entries}
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
                        encoding='utf-8')
    print('=== suite ===')
    for entry in entries:
        print('%-48s wps=%2d rec=%ss sag=%.2f %s' % (
            entry['role'], entry['waypoints'], entry['recorder_s'],
            entry['predicted_sag_max_deg'], 'OK' if entry['valid'] else 'INVALID'))
    print('all_valid', all(entry['valid'] for entry in entries))
    print('wrote', MANIFEST)


if __name__ == '__main__':
    main()
