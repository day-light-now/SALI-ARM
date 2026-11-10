"""Offline tests for per-axis Kp/Kd display and driver-first-cause vs JTC layers.

No ROS, no serial, no motion.  Two kinds of tests:

* pure parser tests against contract fixtures agreed with the C++ owner
  (``panel_fixtures/``; synthetic shape, not live hardware captures);
* panel-backend wiring tests with the usual ROS module stubs so the same
  production ``PanelBackend.status()`` path is exercised without a ROS install.

The JTC/driver threshold tests read the deployed YAML files (10 deg / 4 deg) but
only display them; nothing here changes configuration.
"""
import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
CONFIG = ROOT / "src" / "slaihand_bringup" / "config"
FIXTURES = HERE / "panel_fixtures"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE))

import five_axis_diagnostics as diag  # noqa: E402

# Installs the inert ROS stubs in sys.modules, then imports production
# ``control_panel``.  No ROS process is started.
import test_five_axis_panel_backend as panel_stubs  # noqa: E402

control_panel = panel_stubs.control_panel
SHARE = panel_stubs.SHARE


def load_fixture(name):
    return json.loads((FIXTURES / name).read_text(encoding="utf-8"))


class GainParsing(unittest.TestCase):
    def test_contract_fixture_per_axis_values_are_used_over_scalar(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        view = diag.parse_axis_gains(identity, joint_count=5)
        self.assertTrue(view["all_axes_verified"], view["reasons"])
        self.assertEqual(len(view["axes"]), 5)
        self.assertEqual(view["axes"][0]["kp_applied"], 160.0)
        self.assertEqual(view["axes"][1]["kd_applied"], 5.0)
        self.assertEqual(view["axes"][2]["kp_applied"], 120.0)
        self.assertEqual(view["axes"][4]["kp_applied"], 40.0)
        self.assertEqual(view["axes"][4]["kd_applied"], 2.0)
        # The scalar is still exposed for compatibility but never replaces a
        # present per-axis array.
        self.assertEqual(view["scalar_kp"], 160.0)
        self.assertEqual(view["scalar_kd"], 5.0)
        self.assertTrue(view["scalar_compatible"])
        self.assertTrue(all(row["display"] for row in view["axes"]))
        self.assertIn("runtime_identity.kp_axis", view["axes"][0]["kp_source"])

    def test_scalar_only_fixture_is_labelled_unproven(self):
        identity = load_fixture("runtime_identity_scalar_only_fixture.json")
        view = diag.parse_axis_gains(identity, joint_count=5)
        self.assertFalse(view["all_axes_verified"])
        for row in view["axes"]:
            self.assertEqual(row["status"], "SCALAR_ONLY")
            self.assertIsNone(row["kp_applied"])
            self.assertIsNone(row["kd_applied"])
            self.assertEqual(row["scalar_kp"], 160.0)
            self.assertEqual(row["scalar_kd"], 5.0)
            self.assertIn("SCALAR_ONLY_APPLIED_UNPROVEN",
                          "；".join(row["reasons"]))
        self.assertTrue(any("标量" in reason for reason in view["reasons"]))

    def test_encoded_values_are_shown_separately_when_present(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["encoded_kp"] = [159.9512, 159.9512, 159.9512, 159.9512, 159.9512]
        identity["encoded_kd"] = [5.0, 5.0, 5.0, 5.0, 5.0]
        view = diag.parse_axis_gains(identity, joint_count=5)
        row = view["axes"][0]
        self.assertEqual(row["kp_applied"], 160.0)
        self.assertAlmostEqual(row["kp_encoded"], 159.9512, places=6)
        self.assertEqual(row["kd_encoded"], 5.0)
        self.assertIn("runtime_identity.encoded_kp", row["kp_encoded_source"])

    def test_wrong_length_is_rejected_not_padded_or_truncated(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0, 160.0, 120.0, 80.0]
        identity["kd_axis"] = [5.0, 5.0, 4.0, 3.0]
        view = diag.parse_axis_gains(identity, joint_count=5)
        self.assertFalse(view["all_axes_verified"])
        for row in view["axes"]:
            self.assertEqual(row["status"], "REJECTED")
            self.assertIsNone(row["kp_applied"])
            joined = "；".join(row["reasons"])
            self.assertIn("KP_AXIS_LENGTH", joined)
            self.assertIn("KD_AXIS_LENGTH", joined)
        self.assertNotIn(160.0, [row["kp_applied"] for row in view["axes"]])

    def test_out_of_range_values_are_rejected_not_clamped(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0, 999.0, 120.0, 80.0, 40.0]
        identity["kd_axis"] = [5.0, 9.0, 4.0, 3.0, 2.0]
        view = diag.parse_axis_gains(identity, joint_count=5)
        row = view["axes"][1]
        self.assertEqual(row["status"], "REJECTED")
        self.assertIsNone(row["kp_applied"])
        joined = "；".join(row["reasons"])
        self.assertIn("KP_AXIS_OUT_OF_RANGE", joined)
        self.assertIn("KD_AXIS_OUT_OF_RANGE", joined)
        # Other valid axes stay verified.
        self.assertEqual(view["axes"][0]["status"], "VERIFIED")

    def test_non_finite_gain_is_rejected(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0, float("nan"), 120.0, 80.0, 40.0]
        view = diag.parse_axis_gains(identity, joint_count=5)
        self.assertEqual(view["axes"][1]["status"], "REJECTED")
        self.assertIn("KP_AXIS_NONFINITE", "；".join(view["axes"][1]["reasons"]))

    def test_dynamic_joint_interface_fallback_is_defensive_only(self):
        identity = {"kp": 160.0, "kd": 5.0}
        names = ["slaihand_joint%d" % index for index in range(1, 6)]
        axes = {name: {"applied_kp": 150.0, "applied_kd": 4.5, "position": 0.0}
                for name in names}
        view = diag.parse_axis_gains(identity, axes, joint_count=5, joint_names=names)
        self.assertTrue(view["all_axes_verified"])
        self.assertEqual(view["axes"][0]["kp_applied"], 150.0)
        self.assertIn("dynamic_joint_state", view["axes"][0]["kp_source"])

    def test_no_gain_evidence_is_unknown(self):
        view = diag.parse_axis_gains({}, joint_count=5)
        self.assertFalse(view["all_axes_verified"])
        for row in view["axes"]:
            self.assertEqual(row["status"], "UNKNOWN")
            self.assertIsNone(row["kp_applied"])
            self.assertIn("AXIS_GAIN_MISSING", "；".join(row["reasons"]))

    def test_four_axis_session(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0, 160.0, 120.0, 80.0]
        identity["kd_axis"] = [5.0, 5.0, 4.0, 3.0]
        view = diag.parse_axis_gains(identity, joint_count=4)
        self.assertEqual(len(view["axes"]), 4)
        self.assertTrue(view["all_axes_verified"])

    def test_scalar_legacy_expanded_arrays_are_labelled(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0] * 5
        identity["kd_axis"] = [5.0] * 5
        identity["kp_axis_source"] = "scalar_legacy"
        identity["kd_axis_source"] = "scalar_legacy"
        view = diag.parse_axis_gains(identity, joint_count=5)
        self.assertTrue(view["all_axes_verified"])
        self.assertEqual(view["axes"][0]["source_kind"], "scalar_expanded")
        self.assertIn("标量策略展开", view["axes"][0]["display"])

    def test_first_fault_encoded_vectors_fill_encoded_columns(self):
        identity = load_fixture("first_fault_following_error_fixture.json")
        gains = diag.parse_axis_gains(identity, joint_count=5)
        fault = diag.parse_first_fault(identity, joint_count=5)
        # Effective per-axis values still come from the identity arrays; the
        # deterministic scalar_legacy expansion is labelled as such.
        self.assertEqual(gains["axes"][0]["status"], "VERIFIED")
        self.assertEqual(gains["axes"][0]["source_kind"], "scalar_expanded")
        merged = diag.merge_encoded_gains(gains, fault)
        self.assertTrue(merged["encoded_from_first_fault"])
        self.assertAlmostEqual(merged["axes"][0]["kp_encoded"], 159.9512, places=6)
        self.assertEqual(merged["axes"][0]["kp_encoded_source"],
                         "first_fault.encoded_kp")
        self.assertAlmostEqual(merged["axes"][0]["kd_encoded"], 5.0, places=6)
        self.assertAlmostEqual(merged["axes"][0]["kp_applied"], 160.0, places=6)


class FirstFaultParsing(unittest.TestCase):
    def setUp(self):
        self.identity = load_fixture("first_fault_following_error_fixture.json")

    def test_driver_first_cause_fields(self):
        view = diag.parse_first_fault(self.identity, joint_count=5)
        self.assertTrue(view["available"])
        self.assertEqual(view["layer"], "DRIVER")
        self.assertEqual(view["code"], "FOLLOWING_ERROR")
        self.assertEqual(view["axis"], 2)
        self.assertEqual(view["axis_label"], "J2")
        self.assertAlmostEqual(view["error_deg"], 4.0047, places=6)
        self.assertAlmostEqual(view["limit_deg"], 4.0, places=6)
        self.assertIs(view["exceeded"], True)
        self.assertAlmostEqual(view["target_deg"], 40.1998, places=2)
        self.assertAlmostEqual(view["actual_deg"], 36.1951, places=2)
        self.assertAlmostEqual(view["feedback_age_s"], 0.012, places=6)
        self.assertIs(view["feedback_valid"], True)
        self.assertAlmostEqual(view["encoded_kp"], 159.9512, places=6)
        self.assertAlmostEqual(view["encoded_kd"], 5.0, places=6)
        # Per-axis tau_ff vector is indexed at J2 (second entry).
        self.assertAlmostEqual(view["encoded_tau_ff_nm"], -0.04884, places=6)
        self.assertEqual(view["command_id"], 14)
        self.assertEqual(view["cycle_id"], 253166)
        self.assertEqual(view["boot_id"], "fixture-boot-first-fault")
        self.assertAlmostEqual(view["last_complete_write_at"], 15324.0075, places=6)
        self.assertGreaterEqual(view["last_write_age_s"], 0.0)
        self.assertTrue(view["frozen"])
        # Per-axis request estimate is J2's last completed cycle value, and it
        # is always labelled last-active/stale, never current output.
        estimate = view["request_estimate"]
        self.assertAlmostEqual(estimate["value_nm"], 11.117, places=6)
        self.assertEqual(estimate["state"], "LAST_ACTIVE_STALE")
        self.assertTrue(estimate["stale"])
        self.assertFalse(estimate["is_current_output"])
        self.assertIn("不是当前输出", estimate["label"])
        self.assertIn("J2", view["display"])

    def test_jtc_layer_is_never_promoted_to_driver_first_cause(self):
        view = diag.parse_first_fault(
            {"first_fault": {"schema": "slaihand_first_fault/1", "layer": "JTC",
                             "axis": 1, "code": "FOLLOWING_ERROR", "error_deg": 10.57,
                             "limit_deg": 10.0}},
            joint_count=5)
        self.assertFalse(view["available"])
        self.assertIn("FIRST_FAULT_NOT_DRIVER", "；".join(view["reasons"]))

    def test_wrong_schema_is_rejected(self):
        view = diag.parse_first_fault(
            {"first_fault": {"schema": "something/else", "axis": 2,
                             "error_deg": 4.0}}, joint_count=5)
        self.assertFalse(view["available"])
        self.assertIn("FIRST_FAULT_SCHEMA_UNKNOWN", "；".join(view["reasons"]))

    def test_missing_snapshot_is_unknown(self):
        view = diag.parse_first_fault({"schema": "slaihand_runtime_identity/1",
                                       "first_fault": None}, joint_count=5)
        self.assertFalse(view["available"])
        self.assertIn("驱动首因", view["display"])
        self.assertIn("未提供", view["display"])

    def test_invalid_json_string_does_not_raise(self):
        view = diag.parse_first_fault("{not json", joint_count=5)
        self.assertFalse(view["available"])

    def test_missing_limit_is_not_invented(self):
        fault = {"schema": "slaihand_first_fault/1", "layer": "DRIVER", "axis": 2,
                 "code": "FOLLOWING_ERROR", "error_deg": 4.0047,
                 "target_deg": 40.0, "actual_deg": 36.0, "encoded_kp": 160.0,
                 "encoded_kd": 5.0, "command_id": 1, "cycle_id": 2, "boot_id": "b"}
        view = diag.parse_first_fault(fault, joint_count=5)
        self.assertTrue(view["available"])
        self.assertIsNone(view["limit_deg"])
        self.assertIsNone(view["exceeded"])
        self.assertIn("FIRST_FAULT_LIMIT_MISSING", "；".join(view["reasons"]))

    def test_axis_zero_is_not_painted_as_j1(self):
        fault = {"schema": "slaihand_first_fault/1", "layer": "DRIVER", "axis": 0,
                 "code": "TEMPERATURE", "error_deg": 55.0, "limit_deg": 50.0,
                 "encoded_kp": [160.0] * 5, "encoded_kd": [5.0] * 5,
                 "command_id": 1, "cycle_id": 2, "boot_id": "b"}
        view = diag.parse_first_fault(fault, joint_count=5)
        self.assertIsNone(view["axis"])
        self.assertEqual(view["axis_scope"], "not_axis_specific")
        self.assertEqual(view["axis_label"], "全轴/非单轴")
        self.assertNotIn("J1", view["display"])
        self.assertIn("FIRST_FAULT_AXIS_NOT_SPECIFIC", "；".join(view["reasons"]))
        # Axis=0 can still evidence the full encoded vector.
        self.assertEqual(view["encoded_vectors"]["kp"], [160.0] * 5)
        self.assertIsNone(view["encoded_kp"])

    def test_error_below_limit_is_kept_but_flagged(self):
        fault = {"schema": "slaihand_first_fault/1", "layer": "DRIVER", "axis": 2,
                 "code": "FOLLOWING_ERROR", "error_deg": 3.0, "limit_deg": 4.0,
                 "encoded_kp": [160.0] * 5, "encoded_kd": [5.0] * 5,
                 "command_id": 1, "cycle_id": 2, "boot_id": "b"}
        view = diag.parse_first_fault(fault, joint_count=5)
        self.assertIs(view["exceeded"], False)
        self.assertIn("FIRST_FAULT_BELOW_LIMIT", "；".join(view["reasons"]))

    def test_driver_last_write_and_estimate_sentinels_are_unavailable(self):
        # The real driver serialises -1 sentinels as finite JSON, so they must
        # not be displayed as times or used to compute an age.
        fault = {"schema": "slaihand_first_fault/1", "layer": "DRIVER", "axis": 2,
                 "joint": "slaihand_joint2", "code": "FEEDBACK_STALE_OR_CONFLICTED",
                 "detected_at": 500.0, "error_deg": None, "limit_deg": None,
                 "target_deg": None, "actual_deg": None,
                 "feedback_age_s": None, "feedback_valid": False,
                 "encoded_kp": [0.0] * 5, "encoded_kd": [0.0] * 5,
                 "encoded_tau_ff_nm": [0.0] * 5,
                 "request_estimate_nm": [0.0] * 5, "request_estimate_at": -1,
                 "request_estimate_state": "LAST_ACTIVE",
                 "command_id": 0, "cycle_id": 7, "boot_id": "b",
                 "last_complete_write_at": -1, "frozen": True}
        view = diag.parse_first_fault(fault, joint_count=5)
        self.assertTrue(view["available"])
        self.assertIsNone(view["last_complete_write_at"])
        self.assertIsNone(view["last_write_age_s"])
        self.assertIn("FIRST_FAULT_LAST_WRITE_MISSING", "；".join(view["reasons"]))
        self.assertIsNone(view["request_estimate"]["at"])
        self.assertEqual(view["request_estimate"]["state"], "LAST_ACTIVE_STALE")
        self.assertIn("FIRST_FAULT_ERROR_MISSING", "；".join(view["reasons"]))


class EstimateAndTwoLayer(unittest.TestCase):
    def setUp(self):
        self.ordering = load_fixture("first_fault_then_jtc_after_stop_fixture.json")
        self.jtc_config = diag.parse_jtc_config(
            (CONFIG / "canusb_five_axis_controllers.yaml").read_text(encoding="utf-8"))
        self.guard = diag.parse_driver_guard_deg(
            (CONFIG / "canusb_five_axis_motion_policy.yaml").read_text(encoding="utf-8"))

    def test_deployed_thresholds_are_10_jtc_and_4_driver(self):
        self.assertAlmostEqual(self.jtc_config["tolerance_deg"], 10.0, places=6)
        self.assertAlmostEqual(self.jtc_config["goal_tolerance_deg"], 1.0, places=6)
        self.assertEqual(self.jtc_config["joint_count"], 5)
        self.assertAlmostEqual(self.guard, 4.0, places=6)

    def test_four_axis_config_is_4_and_matches_driver(self):
        four = diag.parse_jtc_config(
            (CONFIG / "canusb_motion_controllers.yaml").read_text(encoding="utf-8"))
        four_guard = diag.parse_driver_guard_deg(
            (CONFIG / "canusb_motion_policy.yaml").read_text(encoding="utf-8"))
        self.assertAlmostEqual(four["tolerance_deg"], 4.0, places=6)
        self.assertAlmostEqual(four_guard, 4.0, places=6)
        matched = diag.two_layer_view(None, jtc_config=four,
                                      driver_guard_deg=four_guard)
        self.assertIs(matched["thresholds"]["matched"], True)
        recommendation = matched["recommendation"]
        self.assertEqual(recommendation["status"], "MATCHED_NO_CHANGE_NEEDED")
        self.assertIs(recommendation["apply_now"], False)
        self.assertIsNone(recommendation["recommended_jtc_tolerance_deg"])
        self.assertTrue(any("无需恢复" in change for change in recommendation["changes"]))

    def test_first_cause_preserved_through_later_jtc_and_operator_stop(self):
        identity = {"schema": "slaihand_runtime_identity/1", "boot_id": "fixture-boot-ordering",
                    "following_error_deg": 4.0,
                    "first_fault": self.ordering["driver_first_fault"],
                    "request_estimate_state": "LAST_ACTIVE_STALE"}
        first = diag.parse_first_fault(identity, joint_count=5)
        after = self.ordering["after_stop"]
        operation_view = {
            "kind": "trajectory", "phase": "stopped",
            "stop_reason": after["operator_stop_reason"],
            "result": {"trajectory_error_code": after["jtc_error_code"],
                       "trajectory_error_string": after["jtc_error_string"]},
        }
        stop_view = {"state": "requested", "reason": after["operator_stop_reason"]}
        two = diag.two_layer_view(first, jtc_config=self.jtc_config,
                                  driver_guard_deg=self.guard,
                                  operation=operation_view, stop=stop_view,
                                  current_jtc_error_deg=[10.57, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(two["first_cause_available"])
        self.assertEqual(two["first_cause"]["axis"], 2)
        self.assertTrue(two["first_cause_preserved"])
        self.assertEqual(two["layers"][0]["layer"], "DRIVER")
        self.assertEqual(two["layers"][0]["axis_label"], "J2")
        self.assertEqual(two["layers"][1]["layer"], "JTC")
        self.assertAlmostEqual(two["layers"][1]["tolerance_deg"], 10.0, places=6)
        later = two["later_events"]
        self.assertGreaterEqual(len(later), 2)
        layers = {event["layer"] for event in later}
        self.assertIn("JTC", layers)
        self.assertTrue(any("STOP" in event["layer"] for event in later))
        self.assertTrue(all(event["is_first_cause"] is False for event in later))
        # Explicit two thresholds, not one merged tolerance.
        thresholds = two["thresholds"]
        self.assertAlmostEqual(thresholds["driver_guard_deg"], 4.0, places=6)
        self.assertAlmostEqual(thresholds["jtc_tolerance_deg"], 10.0, places=6)
        self.assertIs(thresholds["matched"], False)
        self.assertAlmostEqual(thresholds["mismatch_deg"], 6.0, places=6)
        self.assertIn("J2", two["headline"])
        self.assertNotIn("首因：驱动层 J1", two["headline"])

    def test_candidate_restore_to_4_degrees_is_display_only(self):
        two = diag.two_layer_view(None, jtc_config=self.jtc_config,
                                  driver_guard_deg=self.guard)
        recommendation = two["recommendation"]
        self.assertEqual(recommendation["status"], "CANDIDATE_NOT_DEPLOYED")
        self.assertIs(recommendation["apply_now"], False)
        self.assertAlmostEqual(recommendation["recommended_jtc_tolerance_deg"], 4.0,
                               places=6)
        self.assertAlmostEqual(recommendation["current_jtc_tolerance_deg"], 10.0,
                               places=6)
        self.assertIn("不能消除负载偏差", recommendation["note"])
        self.assertIn("不部署", recommendation["note"])
        # The client overrides goal_tolerance per goal mode; a YAML goal-only
        # edit would not take effect, which the display must say.
        goals = recommendation["client_goal_tolerance"]
        self.assertAlmostEqual(goals["normal_plan_deg"], 0.5, places=6)
        self.assertAlmostEqual(goals["teaching_deg"], 4.0, places=6)
        self.assertAlmostEqual(goals["calibration_deg"], 4.0, places=6)
        self.assertIn("goal_tolerance", recommendation["note"])
        self.assertTrue(any("goal_tolerance" in change
                            for change in recommendation["changes"]))

    def test_no_driver_snapshot_never_attributes_the_jtc_error(self):
        two = diag.two_layer_view(
            {"schema": "slaihand_first_fault_view/1", "available": False},
            jtc_config=self.jtc_config, driver_guard_deg=self.guard,
            operation={"stop_reason": "OPERATOR_GLOBAL_STOP"},
            current_jtc_error_deg=[10.57, 0.0, 0.0, 0.0, 0.0])
        self.assertFalse(two["first_cause_available"])
        self.assertIsNone(two["first_cause"])
        self.assertIn("FIRST_CAUSE_MISSING", two["first_cause_missing_reason"])
        self.assertTrue(two["later_events"])
        self.assertFalse(two["first_cause_preserved"])
        self.assertIn("UNKNOWN", two["headline"])
        self.assertNotIn("J1", str(two["first_cause"]))

    def test_request_estimate_states(self):
        current = diag.estimate_view(11.117, state="CURRENT",
                                     source="first_fault.request_estimate_nm")
        self.assertEqual(current["state"], "CURRENT")
        self.assertFalse(current["stale"])
        self.assertFalse(current["is_current_output"])
        stale = diag.estimate_view(11.117, state="LAST_ACTIVE_STALE")
        self.assertTrue(stale["stale"])
        self.assertIn("不是当前输出", stale["label"])
        unavailable = diag.estimate_view(None)
        self.assertEqual(unavailable["state"], "UNAVAILABLE")
        inactive = diag.estimate_view(11.117, active=False)
        self.assertEqual(inactive["state"], "LAST_ACTIVE_STALE")
        active = diag.estimate_view(11.117, active=True)
        self.assertEqual(active["state"], "CURRENT")


class BackendWiring(unittest.TestCase):
    joint_count = 5

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="five-axis-gain-fault-")
        self.addCleanup(self.tmp.cleanup)
        self._saved_share = dict(SHARE)
        self.addCleanup(self._restore_share)
        SHARE["slaihand_bringup"] = str(ROOT / "src" / "slaihand_bringup")
        SHARE["slaihand_description"] = str(ROOT / "src" / "slaihand_description")
        self.backend = control_panel.PanelBackend(
            panel_stubs.FakeNode(), self.tmp.name, 60, 30, "token",
            gravity_enabled=False, joint_count=self.joint_count)

    def _restore_share(self):
        SHARE.clear()
        SHARE.update(self._saved_share)

    def inject(self, identity=None):
        names = ["slaihand_joint%d" % index for index in range(1, self.joint_count + 1)]
        now = time.monotonic()
        self.backend._dynamic = {"at": now, "axes": panel_stubs.axes_for(names)}
        self.backend._joints = {"at": now, "positions": [0.0] * self.joint_count}
        self.backend._controller = None
        self.backend._identity = None if identity is None else {"at": now, "payload": identity}

    def test_status_exposes_gains_and_two_thresholds(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        self.inject(identity)
        status = self.backend.status()
        diagnostics = status["diagnostics"]
        self.assertTrue(diagnostics["gains"]["all_axes_verified"])
        self.assertEqual(diagnostics["gains"]["axes"][2]["kp_applied"], 120.0)
        self.assertFalse(diagnostics["first_fault"]["available"])
        self.assertAlmostEqual(diagnostics["driver"]["following_error_deg"], 4.0, places=6)
        self.assertAlmostEqual(diagnostics["jtc"]["tolerance_deg"], 10.0, places=6)
        self.assertIs(diagnostics["two_layer"]["thresholds"]["matched"], False)
        self.assertIs(diagnostics["recommendation"]["apply_now"], False)
        self.assertTrue(any("首因" in note for note in diagnostics["notes"]))
        margin_view = status["margin"]
        self.assertEqual(margin_view["overall"], "UNKNOWN")
        self.assertFalse(margin_view["qualification"]["tuning_allowed"])
        self.assertEqual([row["label"] for row in margin_view["axes"]],
                         ["J1", "J2", "J3", "J4", "J5"])
        # The HTTP layer serialises with allow_nan=False; the new sections must
        # not leak NaN/inf from the driver payloads.
        json.dumps(status, allow_nan=False)

    def test_status_exposes_frozen_first_fault_and_stale_estimate(self):
        identity = load_fixture("first_fault_following_error_fixture.json")
        self.inject(identity)
        diagnostics = self.backend.status()["diagnostics"]
        first = diagnostics["first_fault"]
        self.assertTrue(first["available"])
        self.assertEqual(first["axis"], 2)
        self.assertAlmostEqual(first["error_deg"], 4.0047, places=6)
        self.assertAlmostEqual(first["limit_deg"], 4.0, places=6)
        estimate = diagnostics["request_estimate"]
        self.assertTrue(estimate["stale"])
        self.assertEqual(estimate["state"], "LAST_ACTIVE_STALE")
        self.assertFalse(estimate["is_current_output"])
        self.assertEqual(diagnostics["two_layer"]["first_cause"]["axis"], 2)
        # The identity has no separate encoded arrays in this fixture; the
        # first-fault snapshot's decoded MIT fields fill the encoded columns.
        gains = diagnostics["gains"]
        self.assertTrue(gains.get("encoded_from_first_fault"))
        self.assertAlmostEqual(gains["axes"][1]["kp_encoded"], 159.9512, places=6)
        self.assertEqual(gains["axes"][1]["kp_encoded_source"],
                         "first_fault.encoded_kp")

    def test_stop_does_not_replace_first_cause_and_marks_estimate_stale(self):
        identity = load_fixture("first_fault_following_error_fixture.json")
        self.inject(identity)
        response = self.backend.stop("操作员全轴停止")
        self.assertTrue(response["ok"])
        status = self.backend.status()
        diagnostics = status["diagnostics"]
        self.assertEqual(diagnostics["two_layer"]["first_cause"]["axis"], 2)
        later = diagnostics["two_layer"]["later_events"]
        self.assertTrue(any("STOP" in event["layer"] for event in later))
        self.assertTrue(diagnostics["request_estimate"]["stale"])
        self.assertEqual(diagnostics["request_estimate"]["state"], "LAST_ACTIVE_STALE")

    def test_panel_stop_relabels_current_estimate_as_stale(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["request_estimate_state"] = "CURRENT"
        self.inject(identity)
        before = self.backend.status()["diagnostics"]["request_estimate"]
        self.assertEqual(before["state"], "CURRENT")
        self.backend.stop("操作员全轴停止")
        after = self.backend.status()["diagnostics"]["request_estimate"]
        self.assertEqual(after["state"], "LAST_ACTIVE_STALE")
        self.assertTrue(after["stale"])
        self.assertFalse(after["is_current_output"])

    def test_later_jtc_error_keeps_driver_first_cause(self):
        identity = load_fixture("first_fault_following_error_fixture.json")
        self.inject(identity)
        operation = control_panel.Operation(
            "trajectory", "a" * 32, Path(self.tmp.name) / "operations" / "op")
        operation.finished = True
        operation.phase = "stopped"
        operation.stop_reason = "OPERATOR_GLOBAL_STOP"
        operation.projection = {
            "trajectory_error_code": 5,
            "trajectory_error_string":
                "slaihand_joint1: 10.57 deg exceeds trajectory tolerance 10.0 deg",
        }
        self.backend._operation = operation
        diagnostics = self.backend.status()["diagnostics"]
        self.assertEqual(diagnostics["two_layer"]["first_cause"]["axis"], 2)
        later = diagnostics["two_layer"]["later_events"]
        jtc_events = [event for event in later if event["layer"] == "JTC"]
        self.assertTrue(jtc_events)
        self.assertIn("10.57", jtc_events[0]["message"])
        self.assertTrue(all(event["is_first_cause"] is False for event in later))

    def test_cpp_null_sentinel_snapshot_serialises_cleanly(self):
        # The driver renders non-finite snapshot fields as JSON null and can
        # report a non-axis-specific fault (axis=0, joint=null).
        null = None
        identity = {
            "schema": "slaihand_runtime_identity/1", "boot_id": "fixture-boot-null",
            "active_axes": 5, "following_error_deg": 4.0,
            "kp_axis": [160.0] * 5, "kd_axis": [5.0] * 5,
            "kp_axis_source": "scalar_legacy", "kd_axis_source": "scalar_legacy",
            "request_estimate_state": "LAST_ACTIVE_STALE",
            "first_fault": {
                "schema": "slaihand_first_fault/1", "layer": "DRIVER",
                "axis": 0, "joint": null, "code": "FEEDBACK_STALE_OR_CONFLICTED",
                "detected_at": 100.0, "error_rad": null, "limit_rad": null,
                "error_deg": null, "limit_deg": null, "target_rad": null,
                "actual_rad": null, "target_deg": null, "actual_deg": null,
                "feedback_age_s": null, "feedback_valid": False,
                "encoded_kp": [0.0] * 5, "encoded_kd": [0.0] * 5,
                "encoded_tau_ff_nm": [0.0] * 5,
                "request_estimate_nm": [0.0] * 5, "request_estimate_at": null,
                "request_estimate_state": "LAST_ACTIVE",
                "command_id": 0, "cycle_id": 1, "boot_id": "fixture-boot-null",
                "last_complete_write_at": null, "frozen": True,
            },
        }
        self.inject(identity)
        status = self.backend.status()
        json.dumps(status, allow_nan=False)
        first = status["diagnostics"]["first_fault"]
        self.assertTrue(first["available"])
        self.assertIsNone(first["axis"])
        self.assertEqual(first["axis_label"], "全轴/非单轴")
        self.assertIsNone(first["last_complete_write_at"])
        self.assertIsNone(first["request_estimate"]["at"])
        self.assertTrue(first["request_estimate"]["stale"])

    def test_direct_reports_match_status_sections(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        self.inject(identity)
        status = self.backend.status()
        self.assertEqual(self.backend.margin_report()["schema"],
                         status["margin"]["schema"])
        self.assertEqual(self.backend.diagnostics_report()["schema"],
                         status["diagnostics"]["schema"])

    def test_margin_evidence_file_is_loaded_fail_closed(self):
        bringup = Path(self.tmp.name) / "bringup"
        config = bringup / "config"
        config.mkdir(parents=True)
        (config / "hardware_mapping.yaml").write_text(
            (CONFIG / "hardware_mapping.yaml").read_text(encoding="utf-8"),
            encoding="utf-8")
        (config / "five_axis_margin_evidence.json").write_text(
            (FIXTURES / "margin_synthetic_evidence_fixture.json").read_text(encoding="utf-8"),
            encoding="utf-8")
        SHARE["slaihand_bringup"] = str(bringup)
        # Keep this regression independent of other tests that also mutate the
        # shared package-share stub.  The production lookup path is unchanged;
        # only the package-share callback is pinned for this temporary tree.
        with mock.patch.object(control_panel, "get_package_share_directory",
                               lambda name: str(bringup) if name == "slaihand_bringup"
                               else SHARE.get(name, str(HERE / "_missing_share_" / name))):
            backend = control_panel.PanelBackend(
                panel_stubs.FakeNode(), (Path(self.tmp.name) / "archive2").as_posix(),
                60, 30, "token", gravity_enabled=False, joint_count=5)
            report = backend.margin_report()
        self.assertNotEqual(report["overall"], "UNKNOWN")
        self.assertFalse(report["qualification"]["tuning_allowed"])
        self.assertEqual(report["source"], str(config / "five_axis_margin_evidence.json"))


class FourAxisDiagnostics(unittest.TestCase):
    """Four-axis mode keeps the same unified view without the 10-degree layer."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="four-axis-gain-fault-")
        self.addCleanup(self.tmp.cleanup)
        self._saved_share = dict(SHARE)
        self.addCleanup(self._restore_share)
        SHARE["slaihand_bringup"] = str(ROOT / "src" / "slaihand_bringup")
        SHARE["slaihand_description"] = str(ROOT / "src" / "slaihand_description")
        self.backend = control_panel.PanelBackend(
            panel_stubs.FakeNode(), self.tmp.name, 60, 30, "token",
            gravity_enabled=False, joint_count=4)

    def _restore_share(self):
        SHARE.clear()
        SHARE.update(self._saved_share)

    def test_four_axis_mode_has_matched_thresholds(self):
        identity = load_fixture("runtime_identity_axis_gains_fixture.json")
        identity["kp_axis"] = [160.0, 160.0, 120.0, 80.0]
        identity["kd_axis"] = [5.0, 5.0, 4.0, 3.0]
        identity["active_axes"] = 4
        now = time.monotonic()
        names = ["slaihand_joint%d" % index for index in range(1, 5)]
        self.backend._dynamic = {"at": now, "axes": panel_stubs.axes_for(names)}
        self.backend._joints = {"at": now, "positions": [0.0] * 4}
        self.backend._controller = None
        self.backend._identity = {"at": now, "payload": identity}
        status = self.backend.status()
        diagnostics = status["diagnostics"]
        self.assertEqual(len(diagnostics["gains"]["axes"]), 4)
        self.assertTrue(diagnostics["gains"]["all_axes_verified"])
        self.assertAlmostEqual(diagnostics["jtc"]["tolerance_deg"], 4.0, places=6)
        self.assertIs(diagnostics["two_layer"]["thresholds"]["matched"], True)
        self.assertEqual(diagnostics["recommendation"]["status"],
                         "MATCHED_NO_CHANGE_NEEDED")
        self.assertEqual(len(status["margin"]["axes"]), 4)
        self.assertEqual(status["margin"]["overall"], "UNKNOWN")


if __name__ == "__main__":
    unittest.main(verbosity=2)
