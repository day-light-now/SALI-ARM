"""Evidence-level checks for the offline scheme matrix."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_schemes  # noqa: E402
import five_axis_control_lab as lab  # noqa: E402


class SchemeMatrix(unittest.TestCase):
    def setUp(self):
        self.baseline = run_schemes.constant()
        self.high = run_schemes.constant_high()

    def test_measured_high_file_is_actually_read(self):
        rows = self.high["rows"]["J4"]
        by_angle = {row["angle_deg"]: row for row in rows}
        # Values are from the real GAIN_0_40 file; they are not the 1/Kp prediction.
        self.assertAlmostEqual(
            by_angle[20.0]["measured_high_gain_error_magnitude_deg"], 1.575, places=2)
        self.assertAlmostEqual(
            by_angle[40.0]["measured_high_gain_error_magnitude_deg"], 2.100, places=2)
        self.assertGreater(
            by_angle[40.0]["predicted_high_gain_magnitude_deg"],
            by_angle[40.0]["measured_high_gain_error_magnitude_deg"])
        self.assertNotEqual(
            by_angle[20.0]["predicted_high_gain_magnitude_deg"],
            by_angle[20.0]["measured_high_gain_error_magnitude_deg"])

    def test_j4_improvement_range_is_not_overstated(self):
        rows = self.high["rows"]["J4"]
        improvements = {row["angle_deg"]: row["measured_improvement_pct"] for row in rows}
        self.assertAlmostEqual(improvements[20.0], 9.95, places=1)
        self.assertAlmostEqual(improvements[40.0], 26.14, places=1)
        self.assertGreaterEqual(min(improvements[20.0], improvements[30.0],
                                    improvements[35.0], improvements[40.0]), 9.0)
        self.assertLessEqual(max(improvements[20.0], improvements[30.0],
                                 improvements[35.0], improvements[40.0]), 27.0)

    def test_blocked_entries_have_null_metrics_and_no_performance_rank(self):
        for name in ("gravity_ff", "disturbance_observer", "model_gravity"):
            result = run_schemes.SCHEMES[name]()
            self.assertIn(result["category"], ("BLOCKED", "IDEALIZED"))
            self.assertIsNone(result["summary"].get("metrics"))
            self.assertFalse(result["existing_command_path"])
            self.assertFalse(result["commissioned"])
            self.assertFalse(result["acceptance_passed"])
        payload = json.loads(Path(run_schemes.HERE / "scheme_results.json").read_text(
            encoding="utf-8"))
        self.assertFalse(payload["ranking_performed"])

    def test_no_unsigned_data_is_presented_as_signed_ff(self):
        result = run_schemes.gravity_ff()
        self.assertEqual(result["status"], "BLOCKED_SIGNED_STATIC_DATA_MISSING")
        self.assertFalse(result["summary"]["deployable_feedforward"])
        self.assertFalse(result["signed_static_data"])
        self.assertIn("signed_static_time_series_missing", result["blocked_reasons"])
        report = lab.compare(lab.load_observations(lab.BASELINE))
        self.assertFalse(report["deployable_gravity_feedforward"])
        self.assertEqual(report["data_kind"], "UNSIGNED_MAGNITUDE_ONLY_PATH_LOCAL")

    def test_schedule_parameters_are_explicit_and_bounded(self):
        result = run_schemes.angle_schedule()
        schedule = result["summary"]["schedule"]
        self.assertEqual(schedule["low"], 280.0)
        self.assertEqual(schedule["high"], 500.0)
        self.assertEqual(schedule["full_angle_deg"], 30.0)
        kp = lab.schedule_kp("J1", 0.0, **schedule)
        self.assertEqual(kp, 280.0)
        self.assertEqual(lab.schedule_kp("J1", 100.0, **schedule), 500.0)

    def test_load_rejects_empty_missing_axis_and_nonfinite(self):
        document = json.loads(lab.BASELINE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for mutate in (
                lambda d: d.update({"initial_model_rad": []}),
                lambda d: d["waypoint_results"][1].update({"target_rad": [0.0] * 4}),
                lambda d: d["waypoint_results"][1].update({"max_error_deg": [float("nan")] * 5}),
                lambda d: d.update({"waypoint_results": []}),
            ):
                broken = copy.deepcopy(document)
                mutate(broken)
                path.write_text(json.dumps(broken), encoding="utf-8")
                with self.subTest(mutate=mutate):
                    with self.assertRaises(ValueError):
                        lab.load_observations(path)

    def test_no_tautological_scheme_assertion(self):
        names = [run_schemes.SCHEMES[name]()["scheme"] for name in run_schemes.SCHEMES]
        self.assertEqual(len(names), len(set(names)))
        self.assertNotIn("hybrid", names)


if __name__ == "__main__":
    unittest.main(verbosity=2)
