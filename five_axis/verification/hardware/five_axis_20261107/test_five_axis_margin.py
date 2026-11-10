"""Offline tests for the five-axis 1.5x margin evaluator and its panel API.

No ROS, no serial, no motion, no real process.  The evaluator is a pure model;
synthetic fixtures under ``panel_fixtures/`` only exercise arithmetic/loader
paths and are explicitly not measured capability or a physical qualification.
"""
import http.client
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
STATIC = ROOT / "src" / "slaihand_bringup" / "web" / "control_panel"
FIXTURES = HERE / "panel_fixtures"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

import five_axis_margin as margin  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402

TOKEN = "<redacted-panel-token>"


def capability(**overrides):
    record = {"allowed_output_nm": 16.65, "source": "verified test report",
              "source_kind": "verified_measurement", "unit": "Nm",
              "verified": True, "duration": {"kind": "continuous"},
              "condition": "pose A, 3.5kg load, 25C, 48V"}
    record.update(overrides)
    return record


def demand(**overrides):
    record = {"demand_nm": 11.1, "source": "external torque sensor",
              "measurement_type": "measured_torque",
              "condition": "pose A, 3.5kg load, 25C, 48V"}
    record.update(overrides)
    return record


class EqualityAndArithmetic(unittest.TestCase):
    def test_exact_equality_boundary_is_pass(self):
        row = margin.evaluate_axis(1, capability(), demand())
        self.assertEqual(row["status"], margin.STATUS_PASS)
        self.assertAlmostEqual(row["safety_factor"], 1.5, places=9)
        self.assertTrue(row["qualification_granted"])
        self.assertTrue(row["measured_acceptance"])
        self.assertFalse(row["pre_screen_only"])

    def test_just_below_equality_is_fail(self):
        row = margin.evaluate_axis(1, capability(allowed_output_nm=16.649),
                                   demand(demand_nm=11.1))
        self.assertEqual(row["status"], margin.STATUS_FAIL)
        self.assertLess(row["safety_factor"], 1.5)
        self.assertFalse(row["qualification_granted"])

    def test_abundant_margin_is_pass(self):
        row = margin.evaluate_axis(1, capability(allowed_output_nm=30.0), demand())
        self.assertEqual(row["status"], margin.STATUS_PASS)
        self.assertGreater(row["safety_factor"], 1.5)

    def test_insufficient_margin_is_fail_not_unknown(self):
        row = margin.evaluate_axis(1, capability(allowed_output_nm=10.0),
                                   demand(demand_nm=10.0))
        self.assertEqual(row["status"], margin.STATUS_FAIL)
        self.assertIn("INSUFFICIENT_MARGIN", row["reasons"])

    def test_rule_is_two_thirds_and_one_point_five(self):
        report = margin.evaluate_axis(1, capability(), demand())
        self.assertEqual(report["required_safety_factor"], 1.5)
        self.assertAlmostEqual(report["rule"].find("1.5") >= 0, True)
        self.assertAlmostEqual(margin.MAX_DEMAND_FRACTION, 2.0 / 3.0)


class FailClosedEvidence(unittest.TestCase):
    def test_missing_capability_is_unknown(self):
        row = margin.evaluate_axis(1, None, demand())
        self.assertEqual(row["status"], margin.STATUS_UNKNOWN)
        self.assertIn("CAPABILITY_MISSING", row["reasons"])
        self.assertFalse(row["qualification_granted"])

    def test_missing_demand_is_unknown(self):
        row = margin.evaluate_axis(1, capability(), None)
        self.assertEqual(row["status"], margin.STATUS_UNKNOWN)
        self.assertIn("DEMAND_MISSING", row["reasons"])

    def test_non_finite_values_are_unknown(self):
        for cap in (capability(allowed_output_nm=float("nan")),
                    capability(allowed_output_nm=float("inf"))):
            row = margin.evaluate_axis(1, cap, demand())
            self.assertEqual(row["status"], margin.STATUS_UNKNOWN, cap)
            self.assertTrue(any("NONFINITE" in reason for reason in row["reasons"]))
        row = margin.evaluate_axis(1, capability(), demand(demand_nm=float("nan")))
        self.assertEqual(row["status"], margin.STATUS_UNKNOWN)

    def test_missing_unit_is_unknown(self):
        cap = capability()
        del cap["unit"]
        row = margin.evaluate_axis(1, cap, demand())
        self.assertEqual(row["status"], margin.STATUS_UNKNOWN)
        self.assertIn("CAPABILITY_UNIT_MISSING", row["reasons"])

    def test_wrong_unit_is_not_qualified(self):
        row = margin.evaluate_axis(1, capability(unit="A"), demand())
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
        self.assertIn("CAPABILITY_UNIT_INVALID", row["reasons"])

    def test_missing_source_kind_or_source_is_unknown(self):
        cap = capability()
        del cap["source_kind"]
        self.assertEqual(margin.evaluate_axis(1, cap, demand())["status"],
                         margin.STATUS_UNKNOWN)
        cap = capability()
        del cap["source"]
        self.assertEqual(margin.evaluate_axis(1, cap, demand())["status"],
                         margin.STATUS_UNKNOWN)

    def test_unverified_capability_is_not_qualified(self):
        row = margin.evaluate_axis(1, capability(verified=False), demand())
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
        self.assertIn("CAPABILITY_NOT_VERIFIED", row["reasons"])

    def test_nonpositive_capability_is_not_qualified(self):
        row = margin.evaluate_axis(1, capability(allowed_output_nm=0.0), demand())
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)

    def test_missing_demand_source_or_measurement_type_is_unknown(self):
        bad = demand()
        del bad["measurement_type"]
        self.assertEqual(margin.evaluate_axis(1, capability(), bad)["status"],
                         margin.STATUS_UNKNOWN)
        bad = demand()
        del bad["source"]
        self.assertEqual(margin.evaluate_axis(1, capability(), bad)["status"],
                         margin.STATUS_UNKNOWN)

    def test_missing_condition_is_unknown(self):
        cap = capability()
        del cap["condition"]
        self.assertEqual(margin.evaluate_axis(1, cap, demand())["status"],
                         margin.STATUS_UNKNOWN)
        bad = demand()
        del bad["condition"]
        self.assertEqual(margin.evaluate_axis(1, capability(), bad)["status"],
                         margin.STATUS_UNKNOWN)


class TmaxIsNotCapability(unittest.TestCase):
    def test_protocol_tmax_200_54_28_never_pass(self):
        for value in (200.0, 54.0, 28.0):
            with self.subTest(value=value):
                row = margin.evaluate_axis(
                    1, capability(allowed_output_nm=value,
                                  source="protocol TMAX register read-back",
                                  source_kind="protocol_tmax"),
                    demand())
                self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
                self.assertIn("TMAX_NOT_CAPABILITY", row["reasons"])
                self.assertFalse(row["qualification_granted"])
                self.assertIsNone(row["safety_factor"])

    def test_tmax_word_in_source_is_rejected_even_if_labelled_verified(self):
        row = margin.evaluate_axis(
            1, capability(allowed_output_nm=200.0,
                          source="J1/J2 PMAX/VMAX/TMAX register", source_kind="verified_measurement"),
            demand())
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
        self.assertIn("TMAX_NOT_CAPABILITY", row["reasons"])

    def test_request_estimate_demand_never_gives_measured_pass(self):
        high = margin.evaluate_axis(1, capability(allowed_output_nm=200.0),
                                    demand(measurement_type="request_estimate"))
        # Even a huge claimed allowed value is still not measured acceptance.
        self.assertNotIn(high["status"], (margin.STATUS_PASS,))
        self.assertTrue(high["pre_screen_only"])
        self.assertFalse(high["qualification_granted"])
        self.assertIn(high["status"], (margin.STATUS_PRESCREEN_PASS,
                                       margin.STATUS_PRESCREEN_FAIL))
        low = margin.evaluate_axis(1, capability(allowed_output_nm=10.0),
                                   demand(measurement_type="request_estimate"))
        self.assertEqual(low["status"], margin.STATUS_PRESCREEN_FAIL)
        self.assertFalse(low["qualification_granted"])


class ConditionAndDuration(unittest.TestCase):
    def test_condition_mismatch_is_not_qualified(self):
        row = margin.evaluate_axis(1, capability(),
                                   demand(condition="dynamic swing, 60C, 48V"))
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
        self.assertIn("CONDITION_MISMATCH", row["reasons"])

    def test_condition_text_normalised_whitespace_still_matches(self):
        row = margin.evaluate_axis(
            1, capability(condition="Pose A,   3.5kg load, 25C, 48V"), demand())
        self.assertEqual(row["status"], margin.STATUS_PASS)

    def test_peak_capability_needs_duration(self):
        cap = capability(duration={"kind": "peak"})
        row = margin.evaluate_axis(1, cap, demand(duration_s=2.0, kind="peak"))
        self.assertEqual(row["status"], margin.STATUS_UNKNOWN)
        self.assertIn("CAPABILITY_DURATION_MISSING", row["reasons"])

    def test_peak_capability_shorter_than_demand_is_not_qualified(self):
        cap = capability(duration={"kind": "peak", "duration_s": 2.0})
        row = margin.evaluate_axis(1, cap, demand(duration_s=5.0, kind="peak"))
        self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
        self.assertIn("DURATION_NOT_COVERED", row["reasons"])

    def test_peak_capability_covers_demand(self):
        cap = capability(duration={"kind": "peak", "duration_s": 5.0})
        row = margin.evaluate_axis(1, cap, demand(duration_s=2.0, kind="peak"))
        self.assertEqual(row["status"], margin.STATUS_PASS)

    def test_continuous_capability_accepts_peak_demand(self):
        row = margin.evaluate_axis(1, capability(), demand(duration_s=1.0, kind="peak"))
        self.assertEqual(row["status"], margin.STATUS_PASS)


class Reports(unittest.TestCase):
    def test_default_report_stays_unknown_without_evidence(self):
        report = margin.default_unknown_report()
        self.assertEqual(len(report["axes"]), 5)
        for row in report["axes"]:
            self.assertEqual(row["status"], margin.STATUS_UNKNOWN)
        self.assertEqual(report["overall"], margin.STATUS_UNKNOWN)
        self.assertEqual(report["data_status"], margin.STATUS_UNKNOWN)
        qualification = report["qualification"]
        self.assertFalse(qualification["tuning_allowed"])
        self.assertFalse(qualification["amplitude_extension_allowed"])
        self.assertFalse(report["physical_acceptance"])
        self.assertIn("MARGIN_EVIDENCE_MISSING", report["reasons"])
        self.assertIn("不是实测验收", report["disclaimer"])

    def test_all_axes_pass_is_complete(self):
        rows = [{"axis": axis, "capability": capability(), "demand": demand()}
                for axis in range(1, 6)]
        report = margin.margin_for_axes(rows)
        self.assertEqual(report["overall"], margin.STATUS_PASS)
        self.assertEqual(report["data_status"], "COMPLETE")
        self.assertTrue(report["qualification"]["tuning_allowed"])
        self.assertFalse(report["qualification"]["physical_acceptance"])
        self.assertFalse(report["physical_acceptance"])

    def test_one_fail_blocks_overall_and_qualification(self):
        rows = [{"axis": axis, "capability": capability(), "demand": demand()}
                for axis in range(1, 6)]
        rows[2]["capability"] = capability(allowed_output_nm=4.0)
        rows[2]["demand"] = demand(demand_nm=4.0)
        report = margin.margin_for_axes(rows)
        self.assertEqual(report["overall"], margin.STATUS_FAIL)
        self.assertEqual(report["fail_axes"], 1)
        self.assertFalse(report["qualification"]["tuning_allowed"])
        self.assertFalse(report["qualification"]["amplitude_extension_allowed"])

    def test_one_not_qualified_blocks_overall(self):
        rows = [{"axis": axis, "capability": capability(), "demand": demand()}
                for axis in range(1, 6)]
        rows[2]["capability"] = capability(source_kind="protocol_tmax",
                                           source="protocol TMAX")
        report = margin.margin_for_axes(rows)
        self.assertEqual(report["overall"], margin.STATUS_NOT_QUALIFIED)
        self.assertFalse(report["qualification"]["tuning_allowed"])

    def test_missing_axis_entries_are_unknown_never_passed(self):
        rows = [{"axis": axis, "capability": capability(), "demand": demand()}
                for axis in (1, 2, 3, 4)]  # J5 evidence missing
        report = margin.margin_for_axes(rows)
        self.assertEqual(report["axes"][4]["status"], margin.STATUS_UNKNOWN)
        self.assertEqual(report["data_status"], "PARTIAL")
        self.assertNotEqual(report["overall"], margin.STATUS_PASS)
        self.assertFalse(report["qualification"]["tuning_allowed"])

    def test_duplicate_axis_evidence_is_rejected(self):
        rows = [{"axis": 1, "capability": capability(), "demand": demand()},
                {"axis": 1, "capability": capability(), "demand": demand()}]
        report = margin.margin_for_axes(rows)
        self.assertEqual(report["axes"][0]["status"], margin.STATUS_UNKNOWN)
        self.assertIn("DUPLICATE_AXIS_EVIDENCE", report["axes"][0]["reasons"])
        self.assertFalse(report["qualification"]["tuning_allowed"])

    def test_four_axis_mode(self):
        rows = [{"axis": axis, "capability": capability(), "demand": demand()}
                for axis in range(1, 5)]
        report = margin.margin_for_axes(rows, axis_count=4)
        self.assertEqual(len(report["axes"]), 4)
        self.assertEqual(report["overall"], margin.STATUS_PASS)


class EvidenceFiles(unittest.TestCase):
    def test_synthetic_fixture_loads_but_is_never_overall_pass(self):
        evidence = margin.load_evidence(FIXTURES / "margin_synthetic_evidence_fixture.json")
        self.assertTrue(evidence["loaded"])
        self.assertEqual(len(evidence["axes"]), 5)
        report = margin.build_report(evidence, axis_count=5)
        self.assertNotEqual(report["overall"], margin.STATUS_PASS)
        self.assertFalse(report["qualification"]["tuning_allowed"])
        self.assertIn("synthetic", (report["source"] or "").lower())

    def test_tmax_negative_fixture_qualified_never(self):
        evidence = margin.load_evidence(FIXTURES / "margin_tmax_negative_fixture.json")
        self.assertTrue(evidence["loaded"])
        report = margin.build_report(evidence, axis_count=5)
        for axis in (1, 2, 5):
            row = report["axes"][axis - 1]
            self.assertEqual(row["status"], margin.STATUS_NOT_QUALIFIED)
            self.assertIn("TMAX_NOT_CAPABILITY", row["reasons"])
        self.assertEqual(report["overall"], margin.STATUS_NOT_QUALIFIED)
        self.assertFalse(report["qualification"]["tuning_allowed"])

    def test_missing_file_is_unknown_not_an_exception(self):
        evidence = margin.load_evidence(FIXTURES / "does_not_exist.json")
        self.assertFalse(evidence["loaded"])
        self.assertIn("MARGIN_EVIDENCE_MISSING", evidence["reasons"])
        report = margin.build_report(evidence)
        self.assertEqual(report["overall"], margin.STATUS_UNKNOWN)

    def test_malformed_json_and_wrong_schema_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text("{not json", encoding="utf-8")
            evidence = margin.load_evidence(path)
            self.assertFalse(evidence["loaded"])
            self.assertIn("MARGIN_EVIDENCE_INVALID", evidence["reasons"])
            wrong = Path(directory) / "wrong.json"
            wrong.write_text(json.dumps({"schema": "something/else", "axes": []}),
                             encoding="utf-8")
            evidence = margin.load_evidence(wrong)
            self.assertFalse(evidence["loaded"])
            self.assertIn("MARGIN_EVIDENCE_SCHEMA", evidence["reasons"])
            report = margin.build_report(evidence)
            self.assertEqual(report["overall"], margin.STATUS_UNKNOWN)
            self.assertFalse(report["qualification"]["tuning_allowed"])

    def test_evidence_directory_path_is_not_a_file(self):
        evidence = margin.load_evidence(FIXTURES)
        self.assertFalse(evidence["loaded"])
        self.assertIn("MARGIN_EVIDENCE_MISSING", evidence["reasons"])


class FakeBackend:
    def __init__(self):
        self.calls = []

    def record_event(self, message, level="info"):
        self.calls.append((level, message))

    def status(self):
        report = margin.default_unknown_report(axis_count=5)
        two_layer = {"first_cause_available": False,
                     "first_cause_missing_reason": "FIRST_CAUSE_MISSING",
                     "thresholds": {"driver_guard_deg": 4.0, "jtc_tolerance_deg": 10.0,
                                    "matched": False},
                     "layers": [], "later_events": []}
        return {"ok": True, "margin": report,
                "diagnostics": {"schema": "slaihand_five_axis_diagnostics/1",
                                "two_layer": two_layer, "gains": {"axes": []}}}

    def margin_report(self):
        return margin.default_unknown_report(axis_count=5)

    def diagnostics_report(self):
        return {"ok": True, "schema": "slaihand_five_axis_diagnostics/1"}


class WorkspaceWithoutDiagnostics:
    def __init__(self):
        pass

    def record_event(self, message, level="info"):
        pass

    def status(self):
        return {"ok": True}


class MarginPanelHttp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.backend = FakeBackend()
        cls.server = PanelServer(cls.backend, TOKEN, STATIC, 0)
        cls.server.start()
        cls.port = cls.server.port
        cls.origin = "http://127.0.0.1:%d" % cls.port
        cls.host = "127.0.0.1:%d" % cls.port
        cls.bare = WorkspaceWithoutDiagnostics()
        cls.bare_server = PanelServer(cls.bare, TOKEN, STATIC, 0)
        cls.bare_server.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()
        cls.bare_server.stop()

    def call(self, port, method, path, body=None, headers=None, origin=None):
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        supplied = {"Host": "127.0.0.1:%d" % port}
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
        return self.call(self.port, "GET", path, headers={"X-Panel-Token": TOKEN})

    def test_status_carries_margin_and_diagnostics(self):
        status, body = self.api_get("/api/status")
        self.assertEqual(status, 200)
        payload = json.loads(body)
        self.assertIn("margin", payload)
        self.assertIn("diagnostics", payload)
        self.assertEqual(payload["margin"]["overall"], margin.STATUS_UNKNOWN)
        self.assertFalse(payload["margin"]["qualification"]["tuning_allowed"])

    def test_margin_endpoint_returns_unknown_report(self):
        status, body = self.api_get("/api/margin")
        self.assertEqual(status, 200)
        payload = json.loads(body)
        self.assertEqual(payload["schema"], margin.REPORT_SCHEMA)
        self.assertEqual(payload["overall"], margin.STATUS_UNKNOWN)
        self.assertFalse(payload["physical_acceptance"])
        self.assertEqual(len(payload["axes"]), 5)

    def test_diagnostics_endpoint_and_missing_backend_method(self):
        status, body = self.api_get("/api/diagnostics")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["ok"])
        bare_port = self.bare_server.port
        status, _ = self.call(bare_port, "GET", "/api/margin",
                              headers={"X-Panel-Token": TOKEN})
        self.assertEqual(status, 404)
        status, _ = self.call(bare_port, "GET", "/api/diagnostics",
                              headers={"X-Panel-Token": TOKEN})
        self.assertEqual(status, 404)

    def test_margin_endpoint_requires_token(self):
        status, _ = self.call(self.port, "GET", "/api/margin")
        self.assertEqual(status, 403)

    def test_page_and_script_have_margin_card_and_renderer(self):
        status, body = self.api_get("/?token=" + TOKEN)
        self.assertEqual(status, 200)
        text = body.decode("utf-8")
        self.assertIn('id="section-margin"', text)
        self.assertIn('id="margin-body"', text)
        self.assertIn("1.5", text)
        self.assertIn("TMAX", text)
        script = (STATIC / "panel.js").read_text(encoding="utf-8")
        self.assertIn("function renderMargin(", script)
        self.assertIn("renderMargin(data);", script)
        # No inline handlers/styles were added for the new cards.
        self.assertNotIn("onclick=", text)
        self.assertNotIn("style=", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
