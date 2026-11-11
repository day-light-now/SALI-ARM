"""Print the panel driver stage (0=DISARMED) for the one-click pipeline."""
import json
import os
import sys
import urllib.request

TOKEN = os.environ.get('PANEL_TOKEN', 'RevgJa5FqG3P7xbk3OQVSxGlJ1BxfApo')
BASE = os.environ.get('PANEL_BASE', 'http://127.0.0.1:42664')
try:
    request = urllib.request.Request(BASE + '/api/status',
                                     headers={'X-Panel-Token': TOKEN})
    with urllib.request.urlopen(request, timeout=2.0) as response:
        document = json.loads(response.read().decode('utf-8'))
    print(document.get('ros', {}).get('stage'))
except Exception:
    raise SystemExit(1)
