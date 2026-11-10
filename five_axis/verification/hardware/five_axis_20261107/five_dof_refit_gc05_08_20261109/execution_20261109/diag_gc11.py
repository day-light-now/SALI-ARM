"""Diagnose the GC11 global STOP/UNKNOWN. Offline read-only."""
import glob
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]
AUDIT = Path('/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_8TXfxe.jsonl')

files = sorted((HERE / 'clients' / 'gc11').glob('*.json'))
result = json.load(open(files[-1], encoding='utf-8'))
print('client', files[-1].name)
for key in ('reason', 'phase', 'mode', 'stop_observations_confirmed', 'unknown_observed',
            'trajectory_started', 'trajectory_error_code', 'stop_result', 'stop_requested',
            'prearm_reasons'):
    print(' ', key, '=', result.get(key))
print('  waypoints:')
for index, waypoint in enumerate(result.get('waypoint_results', [])):
    print('   w%d passed=%s maxerr=%s' % (
        index, waypoint.get('passed'),
        [round(v, 2) for v in (waypoint.get('max_error_deg') or [])]))
print('  last_observed_status:')
for name, value in (result.get('last_observed_status') or {}).items():
    print('   ', name, 'motor_state', value.get('motor_state'),
          'control_state', value.get('control_state'), 'effort', value.get('effort'),
          'age', value.get('feedback_age'), 'disabled_observed', value.get('disabled_observed'))

print('\ndriver log warnings:')
log = HERE / 'logs' / 'driver_launch.log'
for line in log.read_text(encoding='utf-8', errors='replace').splitlines():
    low = line.lower()
    if any(token in low for token in ('warn', 'error', 'stop', 'unknown', 'following', 'guard', 'fault')):
        print(' ', line[-260:])

print('\naudit first-fault records:')
if not AUDIT.is_file():
    print('  AUDIT MISSING', AUDIT)
else:
    commands = {}
    last_cycles = {}
    stages = []
    with AUDIT.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            kind = row.get('kind')
            if kind == 'SESSION':
                print('  SESSION joint_policy signs', [j['sign'] for j in row.get('joint_policy', [])],
                      'ff_alpha', row.get('gravity_ff_alpha'),
                      'following_error_deg', row.get('following_error_deg'))
            elif kind == 'STAGE':
                stages.append(row)
            elif kind == 'CONTROL_CYCLE':
                commands[row.get('command_id')] = commands.get(row.get('command_id'), 0) + 1
                last_cycles[row.get('command_id')] = row
    print('  cycles per command', commands)
    for row in stages[-6:]:
        print('  STAGE steady=%.3f command=%s state=%s reason=%s first_fault=%s' % (
            row.get('steady_s', -1), row.get('command_id'), row.get('control_state'),
            row.get('reason'), json.dumps(row.get('first_fault'), ensure_ascii=False)[:600]))
    for command, row in sorted(last_cycles.items()):
        print('  last cycle command', command, 'state', row.get('control_state'),
              'target', [round(v, 3) for v in row.get('target', [])])
