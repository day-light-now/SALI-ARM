"""Offline tests for the control panel's production HTTP handler and plan model.

No ROS and no serial device: the handler is driven over a real loopback socket.
Backend behaviour that needs ROS/JTC is covered by verify_control_panel.py in an
isolated domain; this file pins transport, authorisation, tickets and semantics.
"""
import http.client
import json
import math
import socket
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
STATIC = ROOT / "src" / "slaihand_bringup" / "web" / "control_panel"
sys.path.insert(0, str(SCRIPTS))

import control_panel_model as model
from control_panel_http import PanelServer
from joint_waypoint_plan import compile_plan

BOUNDS = [tuple(map(math.radians, pair)) for pair in [(-80, 200), (-10, 190), (-90, 90), (-90, 50)]]
TOKEN = "<redacted-panel-token>"


class FakeBackend:
    """Transport-level stand-in; business logic lives in the real backend."""

    def __init__(self):
        self.calls = []

    def record_event(self, message, level="info"):
        self.calls.append((level, message))

    def status(self):
        return {"ok": True, "server": {"ros_domain_id": "176"}, "ros": {}, "axes": [],
                "operation": None, "preview": None, "stop": {"state": "not_requested"},
                "lease": {"fresh": True}, "services": {}, "events": [], "capabilities": {}}

    def _accept(self, phase):
        return {"ok": True, "accepted": True, "completed": False, "phase": phase}

    def preview(self, document):
        self.calls.append(("preview", document))
        return {"ok": True, "accepted": True, "completed": True, "ticket": {"id": "a" * 32},
                "preview": {"waypoints": [], "total_s": 1.0}}

    def execute(self, ticket, document, confirmations):
        self.calls.append(("execute", ticket, document, confirmations))
        return self._accept("accepted")

    def hold(self, confirmations):
        self.calls.append(("hold", confirmations))
        return self._accept("accepted")

    def stop(self, reason):
        self.calls.append(("stop", reason))
        return self._accept("stop_requested")

    def reset(self):
        self.calls.append(("reset",))
        return self._accept("reset_requested")

    def heartbeat(self):
        return {"ok": True, "lease": {"last_heartbeat_age_s": 0.0}}

    def abandon_lease(self):
        return {"ok": True, "accepted": False, "completed": True}

    def save_plan(self, name, document):
        return {"ok": True, "plan": {"id": "b" * 32, "name": name or "x"}}

    def list_plans(self):
        return []

    def load_plan(self, plan_id):
        return {"ok": True, "document": {"coordinates": "absolute_deg", "waypoints": []}}


class ServerCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend = FakeBackend()
        cls.server = PanelServer(cls.backend, TOKEN, STATIC, 0)
        cls.server.start()
        cls.port = cls.server.port
        cls.origin = "http://127.0.0.1:%d" % cls.port
        cls.host = "127.0.0.1:%d" % cls.port

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()

    def call(self, method, path, body=None, headers=None, host=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        supplied = {"Host": host or self.host}
        supplied.update(headers or {})
        payload = None
        if body is not None:
            payload = json.dumps(body).encode("utf-8")
            supplied.setdefault("Content-Type", "application/json")
            supplied["Content-Length"] = str(len(payload))
        try:
            connection.request(method, path, body=payload, headers=supplied)
            response = connection.getresponse()
            data = response.read()
            return response.status, {key.lower(): value for key, value in response.getheaders()}, data
        finally:
            connection.close()

    def post(self, path, body=None, token=True, origin=True, **kwargs):
        headers = kwargs.pop("headers", {})
        if token:
            headers["X-Panel-Token"] = TOKEN
        if origin and "Origin" not in headers:
            headers["Origin"] = self.origin
        return self.call("POST", path, {} if body is None else body, headers=headers, **kwargs)

    def get(self, path, token=True, **kwargs):
        headers = kwargs.pop("headers", {})
        if token:
            headers["X-Panel-Token"] = TOKEN
        return self.call("GET", path, headers=headers, **kwargs)

    # -- page and static assets -------------------------------------------
    def test_page_token_and_headers(self):
        status, headers, body = self.call("GET", "/?token=" + TOKEN)
        self.assertEqual(status, 200)
        self.assertIn("text/html", headers["content-type"])
        text = body.decode("utf-8")
        self.assertIn(TOKEN, text)
        self.assertIn("全轴停止", text)
        self.assertIn("default-src 'none'", headers["content-security-policy"])
        self.assertEqual(headers["x-content-type-options"], "nosniff")
        self.assertEqual(headers["cache-control"], "no-store")
        self.assertEqual(headers["x-frame-options"], "DENY")

    def test_page_requires_query_token(self):
        status, _, _ = self.call("GET", "/")
        self.assertEqual(status, 403)
        status, _, _ = self.call("GET", "/?token=wrong")
        self.assertEqual(status, 403)

    def test_static_assets_are_plain_files(self):
        for path, mime in (("/panel.css", "text/css"), ("/panel.js", "text/javascript"),
                           ("/index.html", "text/html")):
            status, headers, body = self.call("GET", path)
            self.assertEqual(status, 200, path)
            self.assertIn(mime, headers["content-type"])
            self.assertGreater(len(body), 100)
        status, _, _ = self.call("GET", "/../index.html")
        self.assertEqual(status, 404)
        status, _, _ = self.call("GET", "/secret.txt")
        self.assertEqual(status, 404)

    def test_page_has_no_inline_script_style_or_handlers(self):
        text = (STATIC / "index.html").read_text(encoding="utf-8")
        self.assertNotIn("<script>", text)
        self.assertNotIn("onclick=", text)
        self.assertNotIn("style=", text)
        self.assertEqual(text.count("__PANEL_TOKEN__"), 1)

    # -- authorisation ----------------------------------------------------
    def test_api_requires_token(self):
        status, _, _ = self.call("GET", "/api/status")
        self.assertEqual(status, 403)
        status, _, _ = self.get("/api/status")
        self.assertEqual(status, 200)

    def test_post_requires_origin(self):
        status, _, _ = self.post("/api/heartbeat", origin=False)
        self.assertEqual(status, 403)
        status, _, _ = self.post("/api/heartbeat", headers={"Origin": "http://evil.example"})
        self.assertEqual(status, 403)
        status, _, _ = self.post("/api/heartbeat")
        self.assertEqual(status, 200)

    def test_post_requires_token(self):
        status, _, _ = self.post("/api/stop", token=False)
        self.assertEqual(status, 403)
        self.assertFalse([call for call in self.backend.calls if call[0] == "stop"])

    def test_cross_site_fetch_metadata_rejected(self):
        status, _, _ = self.post("/api/stop", headers={"Sec-Fetch-Site": "cross-site"})
        self.assertEqual(status, 403)

    def test_bad_host_rejected(self):
        status, _, _ = self.get("/api/status", host="evil.example")
        self.assertEqual(status, 403)
        status, _, _ = self.call("GET", "/?token=" + TOKEN, host="evil.example")
        self.assertEqual(status, 403)

    # -- request bounds ---------------------------------------------------
    def test_oversized_body_rejected_without_reading(self):
        with socket.create_connection(("127.0.0.1", self.port), timeout=5) as sock:
            request = ("POST /api/preview HTTP/1.1\r\nHost: %s\r\nX-Panel-Token: %s\r\n"
                       "Origin: %s\r\nContent-Type: application/json\r\nContent-Length: 70000\r\n\r\n"
                       % (self.host, TOKEN, self.origin)).encode()
            sock.sendall(request)
            data = b""
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
        self.assertIn(b" 413 ", data.split(b"\r\n")[0])

    def test_missing_length_and_bad_json_rejected(self):
        with socket.create_connection(("127.0.0.1", self.port), timeout=5) as sock:
            sock.sendall(("POST /api/heartbeat HTTP/1.1\r\nHost: %s\r\nX-Panel-Token: %s\r\n"
                          "Origin: %s\r\n\r\n" % (self.host, TOKEN, self.origin)).encode())
            data = b""
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
        self.assertIn(b" 411 ", data.split(b"\r\n")[0])
        status, _, body = self.post("/api/preview", {})
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(body)["code"], "PLAN_INVALID")
        payload = b"{not json"
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        connection.request("POST", "/api/preview", body=payload,
                           headers={"Host": self.host, "X-Panel-Token": TOKEN, "Origin": self.origin,
                                    "Content-Type": "application/json", "Content-Length": str(len(payload))})
        response = connection.getresponse()
        self.assertEqual(response.status, 400)
        response.read()
        connection.close()

    # -- response semantics ----------------------------------------------
    def test_preview_is_completed_execute_is_only_accepted(self):
        document = {"coordinates": "absolute_deg",
                    "waypoints": [{"degrees": [0, 0, 0, 0], "move_s": 2, "dwell_s": 1}]}
        status, _, body = self.post("/api/preview", {"document": document})
        payload = json.loads(body)
        self.assertEqual(status, 200)
        self.assertTrue(payload["accepted"])
        self.assertTrue(payload["completed"])
        status, _, body = self.post("/api/execute", {"ticket": "a" * 32, "document": document,
                                                     "confirmations": {}})
        payload = json.loads(body)
        self.assertEqual(status, 202)
        self.assertTrue(payload["accepted"])
        self.assertFalse(payload["completed"])
        for path in ("/api/hold", "/api/stop", "/api/reset"):
            status, _, body = self.post(path, {})
            payload = json.loads(body)
            self.assertEqual(status, 202, path)
            self.assertTrue(payload["accepted"])
            self.assertFalse(payload["completed"], path)

    def test_execute_requires_ticket_and_document(self):
        status, _, body = self.post("/api/execute", {"document": {}})
        self.assertEqual(status, 400)
        self.assertEqual(json.loads(body)["code"], "REQUEST")
        status, _, body = self.post("/api/execute", {"ticket": "x"})
        self.assertEqual(status, 400)


class PlanModel(unittest.TestCase):
    def document(self, degrees=None, mode="relative_start_deg", move=4, dwell=1):
        return {"coordinates": mode,
                "waypoints": [{"degrees": degrees or [2, 1, -1, 2], "move_s": move, "dwell_s": dwell}]}

    def test_limits_come_from_the_installed_urdf(self):
        limits = model.load_limits(ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf")
        self.assertEqual(model.degrees_limits(limits),
                         [[-80.0, 200.0], [-10.0, 190.0], [-90.0, 90.0], [-90.0, 50.0]])

    def test_preview_uses_compile_plan_and_reports_absolute_targets(self):
        payload = model.preview_payload(self.document(), [0.0] * 4, BOUNDS, "urdf")
        self.assertEqual(payload["source_mode"], "relative_start_deg")
        self.assertEqual(payload["mode"], "absolute_deg")
        self.assertEqual(payload["waypoints"][0]["target_deg"], [2.0, 1.0, -1.0, 2.0])
        self.assertEqual(payload["total_s"], 5.0)
        frozen_points, frozen_windows = compile_plan(payload["frozen"], [0.0] * 4, BOUNDS)
        self.assertEqual(len(frozen_points), 3)
        self.assertEqual(frozen_windows[0]["target_rad"], [math.radians(v) for v in [2, 1, -1, 2]])

    def test_same_validator_rejects_real_rules(self):
        cases = [
            (self.document([201, 0, 0, 0]), "路点超出完整URDF限位；不自动裁剪"),
            (self.document([10, 0, 0, 0], move=.6), "五次插值参考峰值速度超过0.5rad/s；增加move_s"),
            (self.document([1, 2, 3]), "第1点需要J1～J4四个角度，不支持J5"),
            (self.document(dwell=0), "move_s须大于0，dwell_s至少0.5秒"),
        ]
        for document, message in cases:
            with self.subTest(message=message), self.assertRaises(ValueError) as caught:
                model.preview_payload(document, [0.0] * 4, BOUNDS)
            self.assertEqual(str(caught.exception), message)
            # The same document is refused by the production planner as well.
            with self.assertRaises(ValueError):
                compile_plan(model.normalize_document(document), [0.0] * 4, BOUNDS)

    def test_unknown_keys_are_rejected(self):
        for bad in ({"coordinates": "absolute_deg", "waypoints": [], "extra": 1},
                    {"coordinates": "absolute_deg", "waypoints": [{"degrees": [0, 0, 0, 0], "move_s": 1,
                                                                   "dwell_s": 1, "x": 1}]}):
            with self.assertRaises(ValueError):
                model.normalize_document(bad)
        with self.assertRaises(ValueError):
            model.normalize_document({"coordinates": "absolute_deg", "waypoints": []})

    def test_budget_matches_client_178_seconds(self):
        self.assertTrue(model.budget(34.1)["ok"])
        self.assertTrue(model.budget(169.0)["ok"])
        self.assertFalse(model.budget(169.1)["ok"])
        with self.assertRaises(model.PanelError) as caught:
            model.preview_payload(self.document([0, 0, 0, 0], move=169, dwell=1), [0.0] * 4, BOUNDS)
        self.assertEqual(caught.exception.code, "PLAN_BUDGET")

    def test_freeze_is_idempotent_and_returns_to_start_is_explicit(self):
        payload = model.preview_payload(self.document([0, 0, 0, 0]), [0.0] * 4, BOUNDS)
        self.assertTrue(payload["returns_to_start"])
        payload = model.preview_payload(self.document([1, 0, 0, 0]), [0.0] * 4, BOUNDS)
        self.assertFalse(payload["returns_to_start"])
        self.assertTrue(any("不会在终点持续保持" in warning for warning in payload["warnings"]))

    def test_ticket_is_one_time_and_expires(self):
        clock = [100.0]
        store = model.TicketStore(ttl=5, clock=lambda: clock[0])
        payload = model.preview_payload(self.document(), [0.0] * 4, BOUNDS)
        ticket = store.issue({"mode": "absolute_deg", "frozen": payload["frozen"],
                              "frozen_sha256": payload["frozen_sha256"],
                              "source_sha256": payload["source_sha256"], "preview": payload,
                              "start_rad": [0.0] * 4, "start_deg": payload["start_deg"]})
        self.assertTrue(store.is_live(ticket["id"]))
        store.consume(ticket["id"])
        with self.assertRaises(model.PanelError) as caught:
            store.consume(ticket["id"])
        self.assertEqual(caught.exception.code, "TICKET_USED")
        second = store.issue({"mode": "absolute_deg", "frozen": payload["frozen"],
                              "frozen_sha256": "x", "source_sha256": "y", "preview": payload,
                              "start_rad": [0.0] * 4, "start_deg": payload["start_deg"]})
        clock[0] += 6
        with self.assertRaises(model.PanelError) as caught:
            store.consume(second["id"])
        self.assertEqual(caught.exception.code, "TICKET_EXPIRED")

    def test_confirmations_must_be_explicit_true(self):
        good = {"supported": True, "clearance_and_local_power_cut": True,
                "native_output_not_hardware_clamp": True}
        self.assertEqual(model.validate_confirmations(good), good)
        for bad in ({}, dict(good, supported=False), dict(good, supported="true")):
            with self.assertRaises(model.PanelError) as caught:
                model.validate_confirmations(bad)
            self.assertEqual(caught.exception.status, 403)

    def test_result_projection_keeps_raw_errors_and_separates_accuracy(self):
        result = {"request_id": "r", "mode": "waypoints", "reason": "轨迹未通过：goal tolerance",
                  "trajectory_error_code": -5, "trajectory_error_string": "goal tolerance violated",
                  "standard_jtc_action_succeeded": False, "waypoints_observed": False,
                  "stop_observations_confirmed": True,
                  "waypoint_results": [{"arrival_s": 4.0, "end_s": 5.0,
                                        "target_rad": [0.1, 0.0, 0.0, 0.0], "samples": 3,
                                        "sampled_span_s": 0.25, "max_error_deg": [0.59, 0, 0, 0],
                                        "passed": False}]}
        projection = model.project_result(result)
        self.assertEqual(projection["trajectory_error_code"], -5)
        self.assertEqual(projection["trajectory_error_string"], "goal tolerance violated")
        self.assertFalse(projection["action_passed"])
        self.assertFalse(projection["accuracy_passed"])
        self.assertFalse(projection["overall_ok"])
        self.assertTrue(projection["stop_observations_confirmed"])
        missing = model.project_result({"reason": "x", "waypoints_observed": False,
                                        "waypoint_results": [], "stop_observations_confirmed": False})
        self.assertIsNone(missing["accuracy_passed"])
        self.assertFalse(missing["overall_ok"])

    def test_hold_result_has_no_accuracy_claim(self):
        projection = model.project_result({"reason": "finite_trial_complete", "mode": "current_pose_hold",
                                           "stop_observations_confirmed": True})
        self.assertIsNone(projection["accuracy_passed"])
        self.assertTrue(projection["overall_ok"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
