"""Post-incident diagnostics for the GC12 driver crash. Offline read-only."""
import json
from collections import Counter
from pathlib import Path

AUDIT = Path('/home/xuyu/.ros/slaihand/canusb_five_axis_candidate/canusb_motion_waNPuU.jsonl')
HERE = Path(__file__).resolve().parent
ROOT = Path('/mnt/d/CodexData/Projects/机械臂/slaihand_five_axis_ws_gain')

print('audit exists', AUDIT.is_file(), 'size', AUDIT.stat().st_size if AUDIT.is_file() else None)
kinds = Counter()
last_stage = []
last_cycles = []
session = None
with AUDIT.open(encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        kind = row.get('kind')
        kinds[kind] += 1
        if kind == 'SESSION':
            session = row
        elif kind == 'STAGE':
            last_stage.append(row)
            last_stage = last_stage[-8:]
        elif kind == 'CONTROL_CYCLE':
            last_cycles.append(row)
            last_cycles = last_cycles[-3:]
print('kinds', dict(kinds))
if session:
    print('session joint_policy signs', [j['sign'] for j in session.get('joint_policy', [])],
          'ff_alpha', session.get('gravity_ff_alpha'),
          'following_error_deg', session.get('following_error_deg'),
          'j5_qualified', session.get('j5_motion_qualified'))
print('last stage records:')
for row in last_stage:
    print(' ', json.dumps({k: row.get(k) for k in
                           ('steady_s', 'cycle_id', 'command_id', 'control_state',
                            'reason', 'first_fault', 'request_estimate_state')},
                          ensure_ascii=False)[:500])
print('last control cycles:')
for row in last_cycles:
    print(' ', json.dumps({k: row.get(k) for k in
                           ('steady_s', 'command_id', 'control_state', 'target',
                            'request_estimate_nm')}, ensure_ascii=False)[:400])

print('\ncore dumps / crash artefacts:')
patterns = ['core', 'core.*', '*.core']
found = 0
for base in (ROOT, Path.home(), Path('/tmp')):
    for pattern in patterns:
        for path in base.glob(pattern):
            print(' ', path, path.stat().st_size)
            found += 1
if not found:
    print('  none in workspace/home/tmp (system may keep them elsewhere or disabled)')

print('\npanel crash-relevant events (latest archive):')
archive = Path.home() / '.ros/slaihand/canusb_five_axis_candidate'
event_files = sorted((Path.home() / '.ros/slaihand').rglob('events*.jsonl'),
                     key=lambda p: p.stat().st_mtime, reverse=True)[:2]
for path in event_files:
    print(' file', path, path.stat().st_size)
    lines = path.read_text(encoding='utf-8', errors='replace').splitlines()[-15:]
    for line in lines:
        print('  ', line[:300])
if not event_files:
    print('  no panel event files found')
