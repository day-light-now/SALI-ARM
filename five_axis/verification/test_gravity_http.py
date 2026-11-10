"""Read-only and rejected-command checks against the running localhost service."""
import json
import re
import unittest
import urllib.error
import urllib.request

BASE = 'http://127.0.0.1:8767'
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))

class HttpSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with OPENER.open(BASE, timeout=3) as response:
            cls.token = re.search(r"const token='([a-f0-9]+)'", response.read().decode()).group(1)

    def post(self, path, payload, token=None):
        headers = {'Content-Type': 'application/json'}
        if token is not None:
            headers['X-Gravity-Token'] = token
        request = urllib.request.Request(BASE + path, data=json.dumps(payload).encode(), headers=headers)
        try:
            with OPENER.open(request, timeout=3) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            return error.code, json.load(error)

    def test_no_token_rejected(self):
        self.assertEqual(self.post('/sim-command', {'command': 'move', 'degrees': [0]*4})[0], 403)

    def test_bad_token_rejected(self):
        self.assertEqual(self.post('/sim-command', {'command': 'move', 'degrees': [0]*4}, 'invalid')[0], 403)

    def test_out_of_limit_rejected(self):
        self.assertEqual(self.post('/sim-command', {'command': 'move', 'degrees': [999,0,0,0]}, self.token)[0], 400)

    def test_unknown_command_rejected(self):
        self.assertEqual(self.post('/sim-command', {'command': 'enable'}, self.token)[0], 400)

    def test_unconfirmed_gravity_rejected(self):
        self.assertEqual(self.post('/calculate', {'degrees': [0]*4, 'gravity': ''})[0], 400)

    def test_default_shoulder_gravity(self):
        code, data = self.post('/calculate', {'degrees': [0]*4})
        self.assertEqual(code, 200)
        self.assertGreater(abs(data['torque'][0]), .1)

if __name__ == '__main__':
    unittest.main()
