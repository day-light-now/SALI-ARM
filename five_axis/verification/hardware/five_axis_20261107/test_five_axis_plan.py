"""Offline five-axis plan/version/qualification tests. No ROS, no serial.

Run:
    python verification/hardware/five_axis_20261107/test_five_axis_plan.py
"""
import hashlib
import json
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src" / "slaihand_bringup" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import control_panel_model as model  # noqa: E402
import joint_waypoint_plan as planner  # noqa: E402
from point_to_point import StableEndpoint  # noqa: E402

URDF = ROOT / "src" / "slaihand_description" / "urdf" / "slaihand_four_axis.urdf"
MAPPING = ROOT / "src" / "slaihand_bringup" / "config" / "hardware_mapping.yaml"

FOUR_LIMITS_DEG = [(-80, 200), (-10, 190), (-90, 90), (-50, 90)]
J5_LIMITS_DEG = (-30, 30)
FOUR_BOUNDS = [tuple(map(math.radians, pair)) for pair in FOUR_LIMITS_DEG]
FIVE_BOUNDS = FOUR_BOUNDS + [tuple(map(math.radians, J5_LIMITS_DEG))]


def five_document(degrees=(1, 2, -3, 4, 5), mode="absolute_deg", move=4, dwell=1,
                  schema=model.PLAN_SCHEMA):
    document = {"coordinates": mode,
                "waypoints": [{"degrees": list(degrees), "move_s": move, "dwell_s": dwell}]}
    if schema is not None:
        document["schema"] = schema
    return document


def qualified_mapping_text(*, timeout=2000, sign=1, zero=0.25, limits="[-0.5, 0.5]",
                            pmax=12.5, vmax=10.0, tmax=28.0, mode=1):
    return "\n".join([
        "joints:",
        "  slaihand_joint5:",
        "    hardware_sign: %s" % sign,
        "    zero_offset_rad: %s" % zero,
        "    limits_rad: %s" % limits,
        "    commissioning_control_mode: %s" % mode,
        "    commissioning_timeout_count: %s" % timeout,
        "    commissioning_mit_pmax_rad: %s" % pmax,
        "    commissioning_mit_vmax_rad_s: %s" % vmax,
        "    commissioning_mit_tmax_nm: %s" % tmax,
        "    calibration_status: calibrated_test_only",
    ])


def fresh_five_axes(position=0.1):
    names = list(planner.FIVE_AXIS_JOINT_NAMES)
    return {
        "names": names,
        "axes": {name: {"position": position, "velocity": 0.0, "feedback_age": 0.01}
                 for name in names},
        "dynamic_age": 0.02,
        "feedback_valid": [True] * 5,
        "positions": [position] * 5,
    }


class PlanVersions(unittest.TestCase):
    def test_five_axis_plan_is_accepted_with_explicit_version(self):
        points, windows = planner.compile_plan(five_document(), [0.0] * 5, FIVE_BOUNDS)
        self.assertEqual(len(points), 3)
        self.assertEqual(len(windows[0]["target_rad"]), 5)
        self.assertAlmostEqual(math.degrees(windows[0]["target_rad"][4]), 5.0)

        payload = model.preview_payload(five_document(), [0.0] * 5, FIVE_BOUNDS, "test")
        self.assertEqual(payload["schema"], planner.SCHEMA)
        self.assertEqual(payload["axes"], 5)
        self.assertEqual(payload["joint_names"], list(planner.FIVE_AXIS_JOINT_NAMES))
        self.assertEqual(payload["waypoints"][0]["target_deg"], [1.0, 2.0, -3.0, 4.0, 5.0])
        self.assertEqual(payload["frozen"]["schema"], planner.SCHEMA)
        self.assertEqual(len(payload["frozen"]["waypoints"][0]["degrees"]), 5)
        self.assertTrue(any("五轴计划" in warning for warning in payload["warnings"]))

    def test_wrong_dimension_is_never_padded_or_truncated(self):
        cases = [
            five_document(degrees=(1, 2, 3, 4)),          # five-axis header, four angles
            five_document(degrees=(1, 2, 3, 4, 5, 6)),    # six angles
        ]
        for document in cases:
            with self.subTest(document=document), self.assertRaises(ValueError):
                model.normalize_document(document)
            with self.assertRaises(ValueError):
                planner.compile_plan(document, [0.0] * 5, FIVE_BOUNDS)
        # A valid five-axis document cannot be previewed with four-axis feedback/limits.
        with self.assertRaises(ValueError) as caught:
            model.preview_payload(five_document(), [0.0] * 5, FOUR_BOUNDS)
        self.assertIn("5", str(caught.exception))
        with self.assertRaises(ValueError):
            model.preview_payload(five_document(), [0.0] * 4, FIVE_BOUNDS)

    def test_missing_or_non_finite_values_are_rejected(self):
        for bad in (None, True, float("nan"), float("inf"), "1"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                model.normalize_document(five_document(degrees=(0, 0, 0, 0, bad)))
        row = five_document()
        del row["waypoints"][0]["degrees"]
        with self.assertRaises(ValueError):
            model.normalize_document(row)

    def test_four_axis_plan_is_explicitly_legacy_and_not_five_axis(self):
        legacy = five_document(degrees=(1, 2, 3, 4), schema=None)
        normalized = model.normalize_document(legacy)
        self.assertEqual(normalized["schema"], planner.LEGACY_SCHEMA)
        self.assertEqual(len(normalized["waypoints"][0]["degrees"]), 4)
        # It still works in four-axis mode, preserving compatibility.
        payload = model.preview_payload(legacy, [0.0] * 4, FOUR_BOUNDS, "test")
        self.assertEqual(payload["axes"], 4)
        self.assertEqual(payload["schema"], planner.LEGACY_SCHEMA)
        self.assertTrue(any("旧四轴计划" in warning for warning in payload["warnings"]))
        # It is refused by every five-axis caller.
        with self.assertRaises(ValueError):
            planner.compile_plan(normalized, [0.0] * 5, FIVE_BOUNDS)
        with self.assertRaises(ValueError):
            model.preview_payload(legacy, [0.0] * 5, FIVE_BOUNDS)
        with self.assertRaises(ValueError):
            model.compile_document(normalized, [0.0] * 5, FIVE_BOUNDS)

    def test_explicit_versions_are_honoured_and_unknown_refused(self):
        four_v1 = five_document(degrees=(0, 0, 0, 0), schema=planner.LEGACY_SCHEMA)
        self.assertEqual(model.normalize_document(four_v1)["schema"], planner.LEGACY_SCHEMA)
        five_v1 = five_document(degrees=(0, 0, 0, 0, 0), schema=planner.LEGACY_SCHEMA)
        with self.assertRaises(ValueError):
            model.normalize_document(five_v1)
        for schema in ("slaihand_joint_plan/99", "four_axis", 2):
            document = five_document(degrees=(0, 0, 0, 0, 0))
            document["schema"] = schema
            with self.subTest(schema=schema), self.assertRaises(ValueError):
                model.normalize_document(document)
        # A missing schema is an explicit legacy version, never a five-axis plan.
        legacy = five_document(degrees=(0, 0, 0, 0), schema=None)
        self.assertEqual(model.normalize_document(legacy)["schema"], planner.LEGACY_SCHEMA)
        with self.assertRaises(ValueError):
            model.normalize_document(five_document(degrees=(0, 0, 0, 0, 0), schema=None))

    def test_installed_limits_stay_the_four_axis_source(self):
        limits = model.load_limits(URDF)
        self.assertEqual(model.degrees_limits(limits),
                         [[-80.0, 200.0], [-10.0, 190.0], [-90.0, 90.0], [-50.0, 90.0]])
        with self.assertRaises(ValueError):
            model.load_limits(URDF, list(planner.FIVE_AXIS_JOINT_NAMES))  # J5 absent in this URDF

    def test_point_to_point_width_follows_plan(self):
        endpoint = StableEndpoint([0.0] * 5)
        for step in range(13):
            endpoint.observe(10.0 + step * 0.05, [0.0] * 5, [0.0] * 5)
        result = endpoint.result(10.61)
        self.assertEqual(len(result["max_error_deg"]), 5)
        self.assertTrue(result["passed"])
        with self.assertRaises(RuntimeError):
            endpoint.observe(10.7, [0.0] * 4, [0.0] * 4)


class J5Qualification(unittest.TestCase):
    def mapping(self, text=None):
        text = qualified_mapping_text() if text is None else text
        return text, hashlib.sha256(text.encode("utf-8")).hexdigest()

    def identity(self, sha, **extra):
        payload = {"mapping_sha256": sha, "j5_geometry": True,
                   "j5_geometry_sha256": "d" * 64, "j5_motion_qualified": True,
                   "active_axes": 5, "load_id": "j5_3p5kg_candidate"}
        payload.update(extra)
        return payload

    def preflight(self, text, sha, identity, axes_state=None):
        state = axes_state or fresh_five_axes()
        return model.unified_motion_preflight(
            joint_count=5, names=state["names"], axes=state["axes"],
            dynamic_age=state["dynamic_age"], feedback_valid=state["feedback_valid"],
            positions=state["positions"], identity_payload=identity, identity_fresh=True,
            mapping_text=text, mapping_sha256=sha)

    def test_real_hardware_mapping_has_adopted_values_but_empty_feedback_is_rejected(self):
        text = MAPPING.read_text(encoding="utf-8")
        sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
        empty = fresh_five_axes()
        empty["axes"] = {}
        empty["dynamic_age"] = None
        empty["feedback_valid"] = [False] * 5
        empty["positions"] = None
        result = self.preflight(text, sha, self.identity(sha), empty)
        self.assertFalse(result["ok"])
        codes = " ".join(result["reasons"])
        # The historical zero/sign, supplied +/-90 deg travel and the real
        # TIMEOUT2000 readback are now adopted.  Empty feedback is still a hard
        # rejection and this is not a formal calibration.
        for code in ("J5_FEEDBACK_STALE", "J5_FEEDBACK_MISSING", "J5_FEEDBACK_INVALID"):
            self.assertIn(code, codes)
        self.assertNotIn("J5_SIGN_MISSING", codes)
        self.assertNotIn("J5_ZERO_MISSING", codes)
        self.assertNotIn("J5_LIMIT_MISSING", codes)
        self.assertNotIn("J5_TIMEOUT_INVALID", codes)
        observations = result["j5"]["observations"]
        self.assertEqual(observations["hardware_sign"], 1)
        self.assertAlmostEqual(observations["zero_offset_rad"], 2.5389105058365757)
        self.assertEqual(observations["calibration_qualified"], True)
        self.assertEqual(observations["timeout_count"], 2000)
        self.assertTrue(result["j5"]["qualified"])
        self.assertFalse(result["ok"])
        self.assertEqual(observations["limits_rad"], [-math.pi / 2, math.pi / 2])
        self.assertIn("manual_software_zero_direction_and_limits_adopted",
                      observations["calibration_status"])
        record = model.parse_j5_mapping(text)
        self.assertEqual(record["hardware_sign"], 1)
        self.assertAlmostEqual(record["zero_offset_rad"], 2.5389105058365757)
        self.assertEqual([float(value) for value in record["limits_rad"]],
                         [-math.pi / 2, math.pi / 2])
        self.assertEqual(record["software_zero_record"],
                         "verification/hardware/five_axis_20261107/j5_zero_direction_adopted_20261108.json")
        provenance = json.loads((ROOT / "verification/hardware/five_axis_20261107"
                                 / "j5_zero_direction_adopted_20261108.json").read_text(encoding="utf-8"))
        self.assertEqual(provenance["schema"], "slaihand_historical_zero_direction_adoption/1")
        self.assertEqual(provenance["hardware_sign"], 1)
        self.assertAlmostEqual(provenance["zero_offset_rad"], 2.5389105058365757)
        self.assertFalse(provenance["motion_qualified"])
        self.assertFalse(provenance["formal_full_calibration"])
        self.assertEqual(len(provenance["sources"]), 2)
        # Profile readbacks exist, so the profile itself is not missing.
        self.assertNotIn("J5_PROFILE_MISSING", codes)
        self.assertNotIn("J5_MODE_INVALID", codes)
        self.assertEqual(result["gravity"]["status"], "PROVISIONAL")

    def test_unqualified_fixture_still_reports_missing_sign_and_zero(self):
        text = ("joints:\n  slaihand_joint5:\n    hardware_sign: null\n"
                "    zero_offset_rad: null\n    limits_rad: null\n"
                "    calibration_status: unqualified_fixture\n"
                "    commissioning_control_mode: 1\n"
                "    commissioning_timeout_count: 0\n"
                "    commissioning_mit_pmax_rad: 12.5\n"
                "    commissioning_mit_vmax_rad_s: 10.0\n"
                "    commissioning_mit_tmax_nm: 28.0\n")
        sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
        result = self.preflight(text, sha, self.identity(sha))
        codes = " ".join(result["reasons"])
        for code in ("J5_SIGN_MISSING", "J5_ZERO_MISSING", "J5_LIMIT_MISSING",
                     "J5_TIMEOUT_INVALID"):
            self.assertIn(code, codes)

    def test_timeout_zero_alone_blocks_otherwise_qualified_j5(self):
        text, sha = self.mapping(qualified_mapping_text(timeout=0))
        result = self.preflight(text, sha, self.identity(sha))
        self.assertFalse(result["ok"])
        codes = [reason.split(":", 1)[0] for reason in result["reasons"]]
        self.assertEqual(codes, ["J5_TIMEOUT_INVALID"])
        self.assertIn("TIMEOUT0", " ".join(result["reasons"]))

    def test_fully_qualified_j5_is_accepted(self):
        text, sha = self.mapping()
        result = self.preflight(text, sha, self.identity(sha))
        self.assertTrue(result["ok"], result["reasons"])
        self.assertTrue(result["j5"]["qualified"])
        self.assertEqual(result["j5"]["limits_rad"], [-0.5, 0.5])
        self.assertEqual(result["gravity"]["status"], "QUALIFIED")
        self.assertTrue(result["gravity"]["formal_gravity_allowed"])

    def test_missing_identity_or_mapping_mismatch_rejects(self):
        text, sha = self.mapping()
        missing = self.preflight(text, sha, None)
        self.assertIn("J5_IDENTITY_NOT_READY", " ".join(missing["reasons"]))
        mismatch = self.preflight(text, sha, self.identity("0" * 64,
                                                          j5_geometry=True))
        self.assertIn("J5_MAPPING_MISMATCH", " ".join(mismatch["reasons"]))
        undeclared = self.preflight(text, sha, {"mapping_sha256": sha, "load_id": "x"})
        self.assertIn("J5_IDENTITY_MISSING", " ".join(undeclared["reasons"]))

    def test_missing_or_stale_j5_feedback_rejects(self):
        text, sha = self.mapping()
        empty = fresh_five_axes()
        empty["axes"].pop(planner.FIVE_AXIS_JOINT_NAMES[-1])
        empty["feedback_valid"] = [True] * 4
        empty["positions"] = [0.0] * 4
        empty["dynamic_age"] = None
        result = self.preflight(text, sha, self.identity(sha), empty)
        codes = " ".join(result["reasons"])
        self.assertIn("J5_FEEDBACK_STALE", codes)
        self.assertIn("J5_FEEDBACK_MISSING", codes)
        stale = fresh_five_axes()
        stale["dynamic_age"] = 5.0
        result = self.preflight(text, sha, self.identity(sha), stale)
        self.assertIn("J5_FEEDBACK_STALE", " ".join(result["reasons"]))

    def test_no_end_load_is_provisional_not_formal_gravity(self):
        text, sha = self.mapping()
        for load_id, code in (("no_end_load", "LOAD_ID_STALE"),
                              ("new_load_3p5kg_j5_provisional", "LOAD_ID_PROVISIONAL")):
            with self.subTest(load_id=load_id):
                identity = self.identity(sha, load_id=load_id)
                result = self.preflight(text, sha, identity)
                self.assertTrue(result["ok"])
                self.assertEqual(result["gravity"]["status"], "PROVISIONAL")
                self.assertFalse(result["gravity"]["formal_gravity_allowed"])
                self.assertIn(code, " ".join(result["gravity"]["reasons"]))

    def test_four_axis_mode_is_unchanged_and_does_not_need_j5(self):
        result = model.unified_motion_preflight(
            joint_count=4, names=list(planner.FOUR_AXIS_JOINT_NAMES), axes={},
            dynamic_age=None, feedback_valid=[False] * 4, positions=None,
            identity_payload=None, identity_fresh=False, mapping_text=None,
            mapping_sha256=None)
        self.assertTrue(result["ok"])
        self.assertEqual(result["reasons"], [])
        self.assertFalse(result["j5"]["required"])

    def test_j5_mapping_parser_does_not_invent_missing_values(self):
        record = model.parse_j5_mapping("joints:\n  slaihand_joint5:\n    hardware_sign: null\n"
                                        "    zero_offset_rad: null\n    limits_rad: null\n")
        self.assertIsNone(record["hardware_sign"])
        self.assertIsNone(record["zero_offset_rad"])
        self.assertIsNone(record["limits_rad"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
