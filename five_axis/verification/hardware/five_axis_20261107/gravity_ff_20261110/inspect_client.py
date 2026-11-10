"""Inspect the newest client result in clients/<tag>/ for failure diagnosis."""
import glob
import json
import sys
from pathlib import Path

tag = sys.argv[1]
here = Path(__file__).resolve().parent
paths = sorted((here / 'clients' / tag).glob('*.json'))
print('files:', len(paths))
for path in paths:
    print(' ', path.name, path.stat().st_size)
if not paths:
    sys.exit(0)
path = paths[-1]
data = json.loads(path.read_text(encoding='utf-8'))
print('newest:', path)
for key in ('schema', 'mode', 'phase', 'reason', 'trajectory_error_code',
            'trajectory_requested', 'trajectory_started', 'stop_requested',
            'stop_observations_confirmed', 'unknown_observed', 'prearm_timeout',
            'prearm_reasons', 'stop_result', 'entry_state'):
    value = data.get(key)
    if isinstance(value, str) and len(value) > 400:
        value = value[:400] + '...'
    print(' ', key, '=', json.dumps(value, ensure_ascii=False)[:500]
          if not isinstance(value, str) else value)
print(' waypoint_results =', len(data.get('waypoint_results') or []))
print(' last_observed axes =', list((data.get('last_observed_status') or {}).keys()))
