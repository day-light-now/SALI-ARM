"""Request a panel software reset and verify the driver is back to DISARMED."""
from __future__ import annotations

import json
import sys
import time
import urllib.request

BASE = 'http://127.0.0.1:42664'
TOKEN = 'RevgJa5FqG3P7xbk3OQVSxGlJ1BxfApo'


def request(method, path, body=None):
    data = None if body is None else json.dumps(body).encode('utf-8')
    req = urllib.request.Request(BASE + path, data=data, method=method)
    req.add_header('X-Panel-Token', TOKEN)
    req.add_header('Origin', BASE)
    if data is not None:
        req.add_header('Content-Type', 'application/json; charset=utf-8')
    with urllib.request.urlopen(req, timeout=10) as response:
        return json.loads(response.read().decode('utf-8'))


result = request('POST', '/api/reset', {})
print('reset', json.dumps(result, ensure_ascii=False, sort_keys=True))
for attempt in range(20):
    time.sleep(0.5)
    status = request('GET', '/api/status')
    if status['ros']['stage'] == 0 and status['ros']['stage_fresh'] \
            and status['ros']['feedback_valid_all']:
        print('stage=0 DISARMED fresh validAll=True after %.1fs' % (0.5 * (attempt + 1)))
        sys.exit(0)
status = request('GET', '/api/status')
print('RESET_NOT_CONFIRMED stage=%s fresh=%s validAll=%s'
      % (status['ros']['stage'], status['ros']['stage_fresh'],
         status['ros']['feedback_valid_all']))
sys.exit(1)
