"""Classify a j5_timeout_setup evidence run: ok / write / fail."""
import json
import sys

try:
    document = json.load(open(sys.argv[1], encoding='utf-8'))
except (OSError, ValueError):
    print('fail')
    raise SystemExit(0)
if document.get('ok'):
    print('ok')
elif document.get('timeout_before') == 0:
    print('write')
else:
    print('fail')
