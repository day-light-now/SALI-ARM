"""Offline end-to-end simulation of the analysis pipeline.

Generates realistic synthetic `windows_*` files (command + predicted sag, or
compensated command + model error), runs the real summarizers/analyzers, then
moves the synthetic files to `offline_sim/` so they never mix with hardware data.
No driver, no motion.
"""
import importlib.util
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE / 'offline_sim'
MANIFEST = HERE / 'test_suite_manifest_20261110.json'

spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def plan_degrees(name):
    plan = read(HERE / 'plans' / ('%s_20261110.json' % name))
    return np.array([w['degrees'] for w in plan['waypoints']], dtype=float)


def synthetic_windows(entry, noise_deg=0.06):
    plan = read(Path(entry['plan']))
    command = np.array([w['degrees'] for w in plan['waypoints']], dtype=float)
    if entry['compensated']:
        original = plan_degrees(entry['name'].replace('_comp', ''))
        # True residual vs original is a small model error; sag w.r.t. command
        # is predicted sag + that error.
        error = 0.12 * np.sin(np.arange(5) + np.arange(len(command))[:, None] * 0.7)
        measured = original + error
    else:
        predicted = cp.compute(command)['predicted']
        measured = command + predicted
        error = np.zeros_like(command)
    rows = []
    for index in range(len(command)):
        offset = np.zeros(5)
        if entry['name'] == 'H2_repeat' and index in (1, 2, 3, 4):
            offset[1] = 0.10 if index in (1, 2) else -0.10
        if entry['name'] == 'H4_stiction' and index in (1, 2, 3, 4, 5, 6):
            offset[1] = [0.15, 0.0, -0.15][(index - 1) % 3]
        noise = noise_deg * np.sin(np.arange(5) + index * 0.7)
        mean_fb = measured[index] + offset + noise
        rows.append({'index': index,
                     'desired_deg': np.round(command[index], 4).tolist(),
                     'target_deg': np.round(command[index], 4).tolist(),
                     'mean_fb_deg': np.round(mean_fb, 4).tolist(),
                     'sag_deg': np.round(mean_fb - command[index], 4).tolist(),
                     'samples': 200})
    data = {'tag': entry['name'], 'plan': entry['plan'],
            'client_file': 'synthetic', 'desired_deg': command.tolist(),
            'windows': rows}
    path = HERE / ('windows_%s.json' % entry['name'])
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path


def run(script, *args):
    result = subprocess.run([sys.executable, str(HERE / script), *args],
                            capture_output=True, text=True)
    print('--- %s %s exit=%d ---' % (script, ' '.join(args), result.returncode))
    print((result.stdout + result.stderr).strip()[:1800])
    return result.returncode


def main():
    manifest = read(MANIFEST)
    created = []
    SIM_DIR.mkdir(parents=True, exist_ok=True)
    backup_dir = SIM_DIR / ('backup_%s' % time.strftime('%Y%m%d_%H%M%S'))
    saved = []

    def protect(path):
        if path.is_file():
            backup_dir.mkdir(parents=True, exist_ok=True)
            target = backup_dir / path.name
            shutil.move(str(path), str(target))
            saved.append((target, path))

    for entry in manifest['entries']:
        protect(HERE / ('windows_%s.json' % entry['name']))
    for pattern in ('stiction_H4_20261110.json', 'test_suite_result_*.json'):
        for path in HERE.glob(pattern):
            protect(path)
    try:
        for entry in manifest['entries']:
            created.append(synthetic_windows(entry))
        rc_summary = run('summarize_suite.py')
        rc_stiction = run('analyze_stiction.py')
        rc_compare = run('compare_plan_runs.py', 'H1_holdout',
                         str(HERE / 'offline_sim' / 'h1_compare.json'), 'H1_holdout_comp')
        report = {'schema': 'slaihand_offline_pipeline_simulation/2',
                  'entries': [entry['name'] for entry in manifest['entries']],
                  'residual_definition': 'mean_fb - original desired',
                  'summary_exit': rc_summary,
                  'stiction_exit': rc_stiction,
                  'compare_exit': rc_compare,
                  'ok': rc_summary == 0 and rc_stiction == 0 and rc_compare == 0}
        SIM_DIR.mkdir(exist_ok=True)
        (SIM_DIR / 'offline_simulation_20261110.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        print('SIMULATION_OK' if report['ok'] else 'SIMULATION_FAILED')
        return 0 if report['ok'] else 1
    finally:
        SIM_DIR.mkdir(exist_ok=True)
        for path in created:
            if path.is_file():
                shutil.move(str(path), str(SIM_DIR / path.name))
        # Analysis outputs produced from synthetic windows must not look like real data.
        for pattern in ('stiction_H4_20261110.json', 'test_suite_result_*.json'):
            for path in HERE.glob(pattern):
                if path.is_file():
                    shutil.move(str(path), str(SIM_DIR / path.name))
        # Restore any real hardware files that were set aside before the simulation.
        for source, destination in saved:
            shutil.move(str(source), str(destination))
        if backup_dir.is_dir() and not any(backup_dir.iterdir()):
            backup_dir.rmdir()


if __name__ == '__main__':
    sys.exit(main())
