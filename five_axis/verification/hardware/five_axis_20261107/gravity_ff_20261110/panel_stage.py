"""Print the panel driver stage (0=DISARMED) for the one-click pipeline."""
import json
import sys
import urllib.request

TOKEN = '<redacted-panel-token>'
try:
    request = urllib.request.Request('http://127.0.0.1:42664/api/status',
                                     headers={'X-Panel-Token': TOKEN})
    with urllib.request.urlopen(request, timeout=2.0) as response:
        document = json.loads(response.read().decode('utf-8'))
    print(document.get('ros', {}).get('stage'))
except Exception:
    raise SystemExit(1)
