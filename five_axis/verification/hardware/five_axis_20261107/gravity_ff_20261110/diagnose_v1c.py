"""Diagnose why the V1c compensated run was globally stopped."""
import glob
import gzip
import json
from pathlib import Path

base = Path(__file__).resolve().parent
audit_dir = Path('/home/xuyu/.ros/slaihand/canusb_five_axis_candidate')

paths = sorted((base / 'clients' / 'V1c').glob('*.json'))
data = json.loads(paths[-1].read_text(encoding='utf-8'))
print('client', paths[-1].name)
for key in ('reason', 'stop_requested', 'stop_observations_confirmed', 'unknown_observed',
            'stop_result', 'control_observed', 'phase'):
    value = data.get(key)
    if isinstance(value, str) and len(value) > 500:
        value = value[:500] + '...'
    print(' ', key, '=', value)
print(' waypoint results:')
for index, waypoint in enumerate(data.get('waypoint_results') or []):
    print('  w%d' % index, json.dumps({k: waypoint.get(k) for k in
          ('arrival_s', 'end_s', 'samples', 'passed', 'max_error_deg', 'feedback_error_deg',
           'warnings', 'errors')}, ensure_ascii=False)[:400])
print(' last_observed:')
for axis, status in (data.get('last_observed_status') or {}).items():
    print('  ', axis, {k: status.get(k) for k in
          ('position', 'velocity', 'motor_state', 'control_state', 'hold_ready',
           'feedback_valid', 'feedback_age', 'disabled_observed')})

print('\ndriver log fault lines:')
for log in sorted((base / 'logs').glob('driver_launch_*.log')):
    lines = log.read_text(encoding='utf-8', errors='replace').splitlines()
    hits = [line for line in lines
            if any(term in line for term in ('STOP', 'UNKNOWN', 'first_fault', 'following',
                                             'guard', 'DISABLE_AFTER', 'fault', 'Fault',
                                             'disabled', 'Disable'))]
    print(log.name, 'lines', len(lines), 'hits', len(hits))
    for line in hits[-18:]:
        print('  ', line[:260])

print('\naudit STAGE records (newest audit):')
audits = sorted(audit_dir.glob('canusb_motion_*.jsonl'), key=lambda p: p.stat().st_mtime)
if audits:
    newest = audits[-1]
    print('audit', newest.name, 'size', newest.stat().st_size)
    stages = []
    with newest.open(encoding='utf-8') as stream:
        for line in stream:
            if '"kind":"STAGE"' in line:
                try:
                    stages.append(json.loads(line))
                except ValueError:
                    pass
    for stage in stages[-8:]:
        print('  ', json.dumps({k: stage.get(k) for k in
              ('steady_s', 'command_id', 'control_state', 'reason', 'first_fault')},
              ensure_ascii=False)[:400])
