"""HTTP/static checks for the single five-axis entry. No ROS, no serial.

Run:
    python verification/hardware/five_axis_20261107/test_five_axis_panel_http.py
"""
import http.client
import json
import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
STATIC = ROOT / "src" / "slaihand_bringup" / "web" / "control_panel"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

from control_panel_http import PanelServer  # noqa: E402
import test_five_axis_plan as five  # noqa: E402

TOKEN = "<redacted-panel-token>"


class FakeBackend:
    """Transport-level fake; no ROS objects and no motion."""

    def __init__(self):
        self.calls = []
        self.preview_document = None

    def record_event(self, message, level="info"):
        self.calls.append((level, message))

    def status(self):
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        axes = []
        for index, name in enumerate(names, 1):
            axes.append({"name": name, "label": "J%d" % index,
                         "qualification": "待定：J5_ZERO_MISSING" if index == 5 else "—",
                         "actual_deg": None, "target_deg": None, "error_deg": None,
                         "feedback_age_s": None, "feedback_valid": False})
        return {"ok": True, "server": {"ros_domain_id": "176", "joint_count": 5,
                                       "joint_names": names, "plan_schema": five.model.PLAN_SCHEMA},
                "ros": {"connected": False, "stage": None}, "axes": axes,
                "unified": {"axes": 5, "joint_names": names, "schema": five.model.PLAN_SCHEMA,
                            "axis_label": "五轴",
                            "j5": {"required": True, "qualified": False,
                                   "reasons": ["J5_ZERO_MISSING: test"],
                                   "observations": {"zero_offset_rad": None}},
                            "j5_feedback": {"feedback_age_s": None},
                            "preflight": {"ok": False, "reasons": ["J5_ZERO_MISSING: test"]},
                            "gravity_identity": {"status": "PROVISIONAL",
                                                 "formal_gravity_allowed": False,
                                                 "reasons": ["J5_GEOMETRY_UNQUALIFIED: test"]},
                            "stop_axes": names},
                "operation": None, "preview": None, "stop": {"state": "not_requested"},
                "lease": {"fresh": True}, "services": {}, "events": [],
                "capabilities": {}, "gravity_shadow": {}, "gravity_mode": {},
                "teaching": {}, "supported_capture": {}}

    def preview(self, document, *args, **kwargs):
        self.preview_document = document
        return {"ok": True, "accepted": True, "completed": True,
                "ticket": {"id": "a" * 32}, "preview": {"waypoints": [], "total_s": 1.0}}

    def heartbeat(self):
        return {"ok": True}


class PanelHttp(unittest.TestCase):
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

    def call(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        supplied = {"Host": self.host}
        supplied.update(headers or {})
        payload = None
        if body is not None:
            payload = json.dumps(body).encode("utf-8")
            supplied.setdefault("Content-Type", "application/json")
            supplied["Content-Length"] = str(len(payload))
        try:
            connection.request(method, path, body=payload, headers=supplied)
            response = connection.getresponse()
            return response.status, response.read()
        finally:
            connection.close()

    def api_get(self, path):
        return self.call("GET", path, headers={"X-Panel-Token": TOKEN})

    def api_post(self, path, body=None):
        return self.call("POST", path, body or {},
                         headers={"X-Panel-Token": TOKEN, "Origin": self.origin})

    def test_same_status_entry_has_five_axis_rows(self):
        status, body = self.api_get("/api/status")
        self.assertEqual(status, 200)
        payload = json.loads(body)
        self.assertEqual(len(payload["axes"]), 5)
        self.assertEqual([row["label"] for row in payload["axes"]],
                         ["J1", "J2", "J3", "J4", "J5"])
        self.assertEqual(payload["unified"]["axes"], 5)
        self.assertEqual(len(payload["unified"]["joint_names"]), 5)
        self.assertFalse(payload["unified"]["preflight"]["ok"])
        # J5 is not painted as ready or zero.
        self.assertIsNone(payload["axes"][4]["actual_deg"])
        self.assertFalse(payload["axes"][4]["feedback_valid"])

    def test_legacy_j5_readonly_api_is_gone(self):
        for path in ("/api/j5/status", "/api/j5/reconnect",
                     "/api/j5/calibration/zero", "/api/j5/calibration/direction",
                     "/api/j5/calibration/limit", "/api/j5/calibration/save"):
            with self.subTest(path=path):
                status, _ = self.api_get(path)
                self.assertEqual(status, 404)
                status, _ = self.api_post(path)
                self.assertEqual(status, 404)

    def test_single_preview_entry_accepts_five_axis_document(self):
        document = five.five_document()
        status, body = self.api_post("/api/preview", {"document": document})
        self.assertEqual(status, 200, body)
        self.assertEqual(self.backend.preview_document, document)
        self.assertEqual(len(self.backend.preview_document["waypoints"][0]["degrees"]), 5)

    def test_page_has_one_five_axis_table_and_no_j5_card(self):
        status, body = self.api_get("/?token=" + TOKEN)
        self.assertEqual(status, 200)
        text = body.decode("utf-8")
        self.assertIn("J1–J5", text)
        self.assertNotIn("section-j5", text)
        self.assertNotIn("j5-status", text)
        self.assertNotIn('id="btn-j5', text)
        # One plan table with five axis columns; no J6 column appears anywhere.
        self.assertIn("<th>J5 °</th>", text)
        self.assertNotRegex(text, re.compile(r"<th>J6 [^<]*</th>"))

    def test_panel_js_has_no_j5_api_or_four_axis_constant(self):
        text = (STATIC / "panel.js").read_text(encoding="utf-8")
        self.assertIn("const AXIS_COUNT = 5;", text)
        self.assertIn("slaihand_joint_plan/2", text)
        self.assertNotIn("/api/j5", text)
        self.assertNotIn("renderJ5", text)
        self.assertNotIn("addRow([0, 0, 0, 0]", text)

    def test_single_tabbed_workspace_has_overview_and_latency_chip(self):
        html = (STATIC / "index.html").read_text(encoding="utf-8")
        assert html.count('class="tab') >= 5
        for tab in ("overview", "plan", "gravity", "calibration", "operation"):
            assert 'data-tab="%s"' % tab in html
        assert 'id="axis-summary"' in html
        assert 'id="chip-latency"' in html
        assert "J1–J5 实时总览" in html
        js = (STATIC / "panel.js").read_text(encoding="utf-8")
        assert "function setTab(" in js
        assert "function renderAxisSummary(" in js
        assert "S.pollMs" in js

    def test_page_script_has_no_inline_handlers_or_style(self):
        text = (STATIC / "index.html").read_text(encoding="utf-8")
        self.assertNotIn("<script>", text)
        self.assertNotIn("onclick=", text)
        self.assertNotIn("style=", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
