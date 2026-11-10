"""Verify the extracted audit archive for GC05-GC08. Offline, read-only."""
import gzip
import json
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
archive = HERE / 'audit' / 'canusb_motion_vPEeFe_cycles_20261109.jsonl.gz'
kinds = Counter()
commands = defaultdict(int)
session = None
with gzip.open(archive, 'rt', encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        kinds[row.get('kind')] += 1
        if row.get('kind') == 'SESSION':
            session = row
        if row.get('kind') == 'CONTROL_CYCLE':
            commands[row.get('command_id')] += 1
print('kinds', dict(kinds))
print('commands', dict(sorted(commands.items())))
if session:
    print('session kp_axis', session.get('kp_axis'), 'kd_axis', session.get('kd_axis'))
    print('session ff_alpha', session.get('gravity_ff_alpha'),
          'following_error_deg', session.get('following_error_deg'),
          'j5_qualified', session.get('j5_motion_qualified'),
          'active_axes', session.get('active_axes'))
    print('session kp_schedule', session.get('kp_schedule'))
