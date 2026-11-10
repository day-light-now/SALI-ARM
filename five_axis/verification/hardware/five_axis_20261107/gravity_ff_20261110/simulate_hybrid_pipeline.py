"""Simulate the one-click hybrid path end-to-end (offline).

model conversion -> simulated model run -> measured-correction plan (iter1) ->
simulated iter1 run -> compare_plan_runs/summarize_model_comp. This exercises the
same analysis/report scripts as the real hybrid pipeline, without a driver.
"""
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TAG = 'SIMHYB'
PLANS = HERE / 'plans'
RUNS = HERE / 'runs' / TAG
SIM = HERE / 'offline_sim' / 'hybrid'

spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_plan(points, path):
    path.write_text(json.dumps({'schema': 'slaihand_joint_plan/2',
                                'coordinates': 'absolute_deg',
                                'waypoints': [{'degrees': [float(v) for v in row],
                                               'move_s': 6.0, 'dwell_s': 6.0}
                                              for row in points]},
                               ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def simulate_run(plan_name, run_name, error_deg=0.1, noise_deg=0.04):
    plan = read(PLANS / ('%s_20261110.json' % plan_name))
    command = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    sag = cp.compute(command)['predicted']
    error = error_deg * np.sin(np.arange(5) + np.arange(len(command))[:, None] * 0.6)
    noise = noise_deg * np.sin(np.arange(5) * 1.3 + np.arange(len(command))[:, None])
    measured = command + sag + error + noise
    windows = []
    for index in range(len(command)):
        windows.append({'index': index,
                        'desired_deg': np.round(command[index], 4).tolist(),
                        'target_deg': np.round(command[index], 4).tolist(),
                        'mean_fb_deg': np.round(measured[index], 4).tolist(),
                        'sag_deg': np.round(measured[index] - command[index], 4).tolist(),
                        'samples': 200})
    data = {'tag': run_name, 'plan': str(PLANS / ('%s_20261110.json' % plan_name)),
            'client_file': 'synthetic', 'desired_deg': command.tolist(),
            'windows': windows}
    (HERE / ('windows_%s.json' % run_name)).write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return command, sag, np.array([w['sag_deg'] for w in windows])


def run(script, *args):
    result = subprocess.run([sys.executable, str(HERE / script), *args],
                            capture_output=True, text=True)
    print('--- %s %s exit=%d ---' % (script, ' '.join(args), result.returncode))
    print((result.stdout + result.stderr).strip()[:1600])
    return result


def main():
    RUNS.mkdir(parents=True, exist_ok=True)
    SIM.mkdir(parents=True, exist_ok=True)
    original = read(PLANS / 'V1sub_sag_20261110.json')
    original_points = [w['degrees'] for w in original['waypoints']]

    # 1. model-compensated command plan
    computed = cp.compute(original_points)
    write_plan(computed['compensated'].tolist(), PLANS / ('%s_modelcomp_20261110.json' % TAG))
    model_command, model_sag, model_measured_sag = simulate_run(
        '%s_modelcomp' % TAG, '%s_model' % TAG)

    # 2. measure-residual correction against the ORIGINAL target (hybrid rule)
    model_measured = model_command + model_measured_sag
    iter1 = model_command - (model_measured - np.array(original_points))
    iter1[0] = original_points[0]
    iter1[-1] = original_points[-1]
    write_plan(iter1.tolist(), PLANS / ('%s_iter1_20261110.json' % TAG))
    simulate_run('%s_iter1' % TAG, '%s_iter1' % TAG)

    # 3. real comparison/report scripts
    run('compare_plan_runs.py', 'V1sub_sag',
        str(RUNS / ('%s_model_compare.json' % TAG)), '%s_model' % TAG)
    run('compare_plan_runs.py', 'V1sub_sag',
        str(RUNS / ('%s_iter1_compare.json' % TAG)), '%s_iter1' % TAG)
    run('summarize_model_comp.py', TAG)

    model_cmp = read(RUNS / ('%s_model_compare.json' % TAG))
    iter_cmp = read(RUNS / ('%s_iter1_compare.json' % TAG))
    model_stats = next(iter(model_cmp['runs'].values()))
    iter_stats = next(iter(iter_cmp['runs'].values()))
    report = {'schema': 'slaihand_hybrid_pipeline_simulation/1',
              'model_rms_deg': model_stats['overall_rms'],
              'iter1_rms_deg': iter_stats['overall_rms'],
              'model_max_deg': model_stats['overall_max'],
              'iter1_max_deg': iter_stats['overall_max'],
              'improved': iter_stats['overall_rms'] < model_stats['overall_rms'],
              'report_exists': (RUNS / ('%s_REPORT.md' % TAG)).is_file()}
    (SIM / 'hybrid_simulation_20261110.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('HYBRID_SIMULATION', json.dumps(report, ensure_ascii=False))

    # Keep synthetic artifacts separated from real data.
    for path in list(HERE.glob('windows_%s_*.json' % TAG)) + list(PLANS.glob('%s_*_20261110.json' % TAG)):
        target = SIM / 'artifacts'
        target.mkdir(exist_ok=True)
        shutil.move(str(path), str(target / path.name))
    if RUNS.exists():
        target = SIM / 'runs'
        if target.exists():
            shutil.rmtree(target)
        shutil.move(str(RUNS), str(target))
    return 0 if report['improved'] and report['report_exists'] else 1


if __name__ == '__main__':
    sys.exit(main())
