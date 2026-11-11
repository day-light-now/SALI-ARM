"""Hardware test-suite runner (prepared offline).

--dry : validate every plan in the manifest; no driver, no motion.
--run : assume the admitted 5-axis driver is already running; execute the suite
        sequentially, extract windows and summarize. Operator must be present.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
MANIFEST = HERE / 'test_suite_manifest_20261110.json'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def driver_count():
    result = subprocess.run(['pgrep', '-f', 'ros2_control_node --ros-args'],
                            capture_output=True, text=True)
    return len([line for line in result.stdout.splitlines() if line.strip()])


def call(args, stream=False):
    return subprocess.run(args, capture_output=not stream, text=True)


def dry_run(manifest):
    ok = True
    print('=== dry validation (no driver, no motion) ===')
    for entry in manifest['entries']:
        plan = entry['plan']
        validated = call([sys.executable, str(HERE / 'validate_plan.py'), plan])
        duration = call([sys.executable, str(HERE / 'plan_duration.py'), plan])
        message = (validated.stdout + validated.stderr).strip()
        actual = duration.stdout.strip() if duration.returncode == 0 else '?'
        status = 'OK'
        if not message.startswith('PLAN_OK'):
            status = 'FAIL(%s)' % message
        elif actual != str(entry['recorder_s']):
            status = 'RECORDER_MISMATCH(%s!=%s)' % (actual, entry['recorder_s'])
        print('%-46s wps=%2d rec=%-4s %s' % (
            entry['name'], entry['waypoints'], entry['recorder_s'], status))
        ok = ok and (status == 'OK')
    print('DRY_RUN_OK: all suite plans valid' if ok else 'DRY_RUN_FAILED')
    return 0 if ok else 1


def hardware_run(manifest, stop_driver):
    if driver_count() != 1:
        print('ABORT: exactly one ros2_control_node is required (5-axis driver)')
        return 1
    if not os.path.exists('/dev/ttyACM0'):
        print('ABORT: /dev/ttyACM0 missing; attach the USB-CAN adapter first')
        return 1
    for entry in manifest['entries']:
        name = entry['name']
        print('=== run %s (%s), recorder=%ss ===' % (name, entry['role'], entry['recorder_s']))
        stage = call([sys.executable, str(HERE / 'panel_stage.py')])
        if stage.returncode == 0 and stage.stdout.strip() == '0':
            print('already DISARMED (stage=0); skipping software reset')
        else:
            reset = call([sys.executable, str(HERE / 'panel_reset.py')])
            if reset.returncode != 0:
                print('ABORT: panel reset failed for %s; do a manual reset first' % name)
                return 1
            time.sleep(1.0)
        executed = call(['bash', str(HERE / 'run_plan.sh'), entry['plan'], name,
                         str(entry['recorder_s'])], stream=True)
        if executed.returncode != 0:
            print('WARN: client_exit=%d for %s' % (executed.returncode, name))
        if driver_count() != 1:
            print('ABORT: driver died after %s' % name)
            return 1
        call([sys.executable, str(HERE / 'extract_windows.py'), name, entry['plan']])
    call([sys.executable, str(HERE / 'summarize_suite.py')], stream=True)
    if stop_driver:
        call(['bash', str(HERE / 'stop_driver.sh')])
    print('TEST_SUITE_DONE')
    return 0


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--dry', action='store_true', help='validate plans only')
    group.add_argument('--run', action='store_true', help='execute on hardware')
    parser.add_argument('--stop-driver', action='store_true')
    args = parser.parse_args()
    manifest = read(MANIFEST)
    if args.run:
        return hardware_run(manifest, args.stop_driver)
    return dry_run(manifest)


if __name__ == '__main__':
    sys.exit(main())
