"""Inspect one acquisition: client result, extracted windows, raw size, log tail."""
import json
import sys
from pathlib import Path

base = Path(__file__).resolve().parent
tag = sys.argv[1]
paths = sorted((base / 'clients' / tag).glob('*.json'))
print('client files', len(paths))
if paths:
    data = json.loads(paths[-1].read_text(encoding='utf-8'))
    print('newest', paths[-1].name)
    for key in ('mode', 'reason', 'trajectory_error_code', 'trajectory_requested',
                'trajectory_started', 'stop_result', 'waypoint_count', 'waypoints_passed',
                'prearm_reasons'):
        value = data.get(key)
        if isinstance(value, str) and len(value) > 300:
            value = value[:300] + '...'
        print(' ', key, '=', value)
    print(' waypoint_results', len(data.get('waypoint_results') or []))
windows_path = base / f'windows_{tag}.json'
if windows_path.exists():
    document = json.loads(windows_path.read_text(encoding='utf-8'))
    print(' windows:', [(w['index'], w.get('samples'), w.get('excluded'))
                        for w in document['windows']])
raw = base / 'raw' / f'{tag}_20261110.jsonl.gz'
print(' raw size', raw.stat().st_size if raw.exists() else None)
log = base / 'logs' / f'{tag}_20261110.log'
if log.exists():
    lines = log.read_text(encoding='utf-8', errors='replace').splitlines()
    print(' log tail:')
    for line in lines[-6:]:
        print('  ', line[:240])
