"""Production-callback counterexamples for the stop epoch gate.

Runs the real PanelBackend and the real HTTP handler (no ROS spin, no services,
no serial, Popen mocked) exactly like Codex's stop-race repro.  Requires rclpy
to import the production module, so it runs under the sourced ROS environment.

Covers: stop immediately followed by hold/execute/preview, stop during client
spawn, fresh DISARMED without a manual reset, reset reply alone, stale or
inconsistent observations, and re-locking after a later stop.
"""
import http.client
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))

from control_panel import PanelBackend  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402

NAMES = ["slaihand_joint%d" % i for i in range(1, 5)]
TOKEN = "callback-token"
CONFIRMATIONS = {"supported": True, "clearance_and_local_power_cut": True,
                 "native_output_not_hardware_clamp": True}
PLAN = {"coordinates": "relative_start_deg",
        "waypoints": [{"degrees": [0.0, 0.0, 0.0, 0.0], "move_s": 2.0, "dwell_s": 1.0}]}


class FakeNode:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return None


def axes(stage=0, valid=1, disabled=0, owner=0, position=0.05):
    return {name: {"control_state": stage, "feedback_valid": valid, "position": position,
                   "velocity": 0.0, "effort": 0.0, "driver_temperature": 30.0,
                   "motor_temperature": 25.0, "motor_state": 0,
                   "disabled_observed": disabled, "command_owner": owner, "audit_ok": 1}
            for name in NAMES}


class CallbackCase(unittest.TestCase):
    def setUp(self):
        self._directory = tempfile.TemporaryDirectory(prefix="panel-cb-")
        self.backend = PanelBackend(FakeNode(), self._directory.name, 60, 3, TOKEN)
        self.store(stage=0)
        self.backend.heartbeat()
        self.server = PanelServer(self.backend, TOKEN, self.backend.static_dir, 0)
        self.server.start()
        self.port = self.server.port
        self.origin = "http://127.0.0.1:%d" % self.port
        self.spawned = []
        self.patcher = patch("control_panel.subprocess.Popen", side_effect=self._spawn)
        self.patcher.start()

    def tearDown(self):
        self.patcher.stop()
        self.server.stop()
        self._directory.cleanup()

    def _spawn(self, *args, **kwargs):
        proc = SimpleNamespace(pid=999999999, stdout=iter(()), poll=lambda: None,
                               wait=lambda timeout=None: 0)
        self.spawned.append((args, kwargs))
        return proc

    # -- helpers ---------------------------------------------------------
    def store(self, stage, at=None, valid=1, disabled=0, owner=0, position=0.05):
        now = time.monotonic() if at is None else at
        self.backend._store("dynamic", {"at": now, "axes": axes(stage, valid, disabled, owner, position)})
        self.backend._store("joints", {"at": now, "positions": [position] * 4})

    def call(self, method, path, body=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=8)
        payload = json.dumps(body).encode() if body is not None else None
        headers = {"Host": "127.0.0.1:%d" % self.port, "X-Panel-Token": TOKEN, "Origin": self.origin}
        if payload is not None:
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(payload))
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            return response.status, json.loads(response.read().decode() or "{}")
        finally:
            connection.close()

    def preview(self):
        code, body = self.call("POST", "/api/preview", {"document": PLAN})
        self.assertEqual(code, 200, body)
        return body["ticket"]["id"]

    def execute(self, ticket):
        return self.call("POST", "/api/execute",
                         {"ticket": ticket, "document": PLAN, "confirmations": CONFIRMATIONS})

    def hold(self):
        return self.call("POST", "/api/hold", {"confirmations": CONFIRMATIONS})

    def stop(self):
        return self.call("POST", "/api/stop", {"reason": "callback test"})

    def gate(self):
        return self.backend.status()["stop_gate"]

    # -- counterexamples --------------------------------------------------
    def test_stop_then_hold_is_locked_and_never_spawns(self):
        ticket = self.preview()
        self.assertIsNotNone(ticket)
        code, body = self.stop()
        self.assertEqual(code, 202)
        self.assertTrue(self.gate()["locked"])
        code, body = self.hold()
        self.assertEqual((code, body.get("code")), (409, "STOP_LOCKED"), body)
        self.assertEqual(len(self.spawned), 0)

    def test_stop_then_execute_is_locked_even_with_valid_ticket(self):
        ticket = self.preview()
        self.assertEqual(self.stop()[0], 202)
        code, body = self.execute(ticket)
        self.assertEqual((code, body.get("code")), (409, "STOP_LOCKED"), body)
        self.assertEqual(len(self.spawned), 0)

    def test_stop_then_preview_is_locked(self):
        self.assertEqual(self.stop()[0], 202)
        code, body = self.call("POST", "/api/preview", {"document": PLAN})
        self.assertEqual((code, body.get("code")), (409, "STOP_LOCKED"), body)

    def test_fresh_disarmed_without_manual_reset_does_not_unlock(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=0)  # fresh, consistent, but no reset was requested
        self.backend._check_stop_gate(time.monotonic())
        self.assertTrue(self.gate()["locked"])
        code, body = self.call("POST", "/api/preview", {"document": PLAN})
        self.assertEqual((code, body.get("code")), (409, "STOP_LOCKED"), body)

    def test_reset_reply_alone_does_not_unlock(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=8, disabled=1)  # driver is disabled; reset is allowed
        code, body = self.call("POST", "/api/reset", {})
        self.assertEqual(code, 202, body)
        self.assertIsNotNone(self.gate()["reset_requested_at"])
        # No new dynamic observation has arrived yet.
        self.assertTrue(self.gate()["locked"])
        code, body = self.call("POST", "/api/preview", {"document": PLAN})
        self.assertEqual((code, body.get("code")), (409, "STOP_LOCKED"), body)

    def test_stale_disarmed_after_reset_does_not_unlock(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=8, disabled=1)
        self.assertEqual(self.call("POST", "/api/reset", {})[0], 202)
        reset_at = self.backend._reset_after_stop_at
        self.store(stage=0, at=reset_at - 1.0)  # published before the reset
        self.backend._check_stop_gate(time.monotonic())
        self.assertTrue(self.gate()["locked"])
        self.assertEqual(self.call("POST", "/api/preview", {"document": PLAN})[0], 409)

    def test_inconsistent_four_axis_observation_does_not_unlock(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=8, disabled=1)
        self.assertEqual(self.call("POST", "/api/reset", {})[0], 202)
        mixed = axes(0)
        mixed[NAMES[2]]["control_state"] = 8
        self.backend._store("dynamic", {"at": time.monotonic(), "axes": mixed})
        self.backend._check_stop_gate(time.monotonic())
        self.assertTrue(self.gate()["locked"])
        invalid = axes(0)
        invalid[NAMES[1]]["feedback_valid"] = 0
        self.backend._store("dynamic", {"at": time.monotonic(), "axes": invalid})
        self.backend._check_stop_gate(time.monotonic())
        self.assertTrue(self.gate()["locked"])
        self.store(stage=0)
        self.backend._check_stop_gate(time.monotonic())
        self.assertFalse(self.gate()["locked"])

    def test_fresh_disarmed_after_reset_unlocks_and_preview_recovers(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=8, disabled=1)
        self.assertEqual(self.call("POST", "/api/reset", {})[0], 202)
        self.store(stage=0)
        self.backend._check_stop_gate(time.monotonic())
        self.assertFalse(self.gate()["locked"])
        code, body = self.call("POST", "/api/preview", {"document": PLAN})
        self.assertEqual(code, 200, body)

    def test_unknown_locks_and_cannot_be_reset(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=9, disabled=0)
        self.backend._check_stop_gate(time.monotonic())
        self.assertTrue(self.gate()["locked"])
        code, body = self.call("POST", "/api/reset", {})
        self.assertEqual((code, body.get("code")), (409, "NOT_DISABLED"), body)
        self.assertEqual(self.call("POST", "/api/preview", {"document": PLAN})[0], 409)
        self.assertEqual(self.hold()[0], 409)

    def test_stop_during_client_spawn_is_cancelled(self):
        ticket = self.preview()

        def spawn_then_stop(*args, **kwargs):
            self.backend.stop("stop during spawn")
            return self._spawn(*args, **kwargs)

        with patch("control_panel.subprocess.Popen", side_effect=spawn_then_stop):
            code, body = self.execute(ticket)
        self.assertEqual(code, 202, body)
        self.assertEqual(len(self.spawned), 1)
        operation = self.backend.status()["operation"]
        self.assertTrue(operation["cancel_requested"])
        self.assertTrue(self.gate()["locked"])
        self.assertEqual(self.call("POST", "/api/preview", {"document": PLAN})[0], 409)
        self.assertEqual(self.hold()[0], 409)

    def test_second_stop_after_unlock_relocks(self):
        self.assertEqual(self.stop()[0], 202)
        self.store(stage=8, disabled=1)
        self.assertEqual(self.call("POST", "/api/reset", {})[0], 202)
        self.store(stage=0)
        self.backend._check_stop_gate(time.monotonic())
        self.assertFalse(self.gate()["locked"])
        first, second = self.preview(), self.preview()  # previews work again
        self.assertNotEqual(first, second)
        self.assertEqual(self.stop()[0], 202)
        self.assertTrue(self.gate()["locked"])
        self.assertEqual(self.call("POST", "/api/preview", {"document": PLAN})[0], 409)


if __name__ == "__main__":
    unittest.main(verbosity=2)
