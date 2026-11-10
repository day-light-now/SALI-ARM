"""Pure-file checks for the signed dwell analyzer (no ROS/hardware)."""
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analyze_signed  # noqa: E402


def state(t, reference, feedback):
    return {"t": t, "kind": "controller_state", "payload": {
        "joint_names": list(analyze_signed.JOINTS),
        "reference_positions": list(reference),
        "feedback_positions": list(feedback),
        "error_positions": [f - r for f, r in zip(feedback, reference)],
        "reference_velocities": [0.0] * 5,
        "feedback_velocities": [0.0] * 5,
        "output": [0.0] * 5,
    }}


class SignedAnalysis(unittest.TestCase):
    def test_signed_mean_is_feedback_minus_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "signed.jsonl"
            lines = []
            for index in range(200):
                t = index * 0.01
                lines.append(state(t, [0.1] * 5, [0.1 - 0.01] * 5))
            path.write_text("\n".join(json.dumps(row) for row in lines) + "\n",
                            encoding="utf-8")
            summary = analyze_signed.analyze(path, window_s=1.0)
            self.assertEqual(len(summary["segments"]), 1)
            stats = summary["segments"][0]["axes"]["slaihand_joint1"]["signed_error_deg"]
            self.assertAlmostEqual(stats["mean"], math.degrees(-0.01), places=5)
            self.assertAlmostEqual(stats["max_abs"], math.degrees(0.01), places=5)

    def test_schedule_kp_is_used_for_pd_estimate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "signed.jsonl"
            # actual q about 0.19 rad = 10.9 deg; 1 deg below target.
            lines = [json.dumps(state(index * 0.01, [0.2] * 5, [0.2 - math.radians(1)] * 5))
                     for index in range(200)]
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            schedule = analyze_signed.parse_kp_schedule(
                "0:160,40:280;0:160;0:160;0:160;0:160")
            summary = analyze_signed.analyze(path, 1.0,
                analyze_signed.parse_kp_axis("160,160,160,160,160"), schedule)
            axis = summary["segments"][0]["axes"]["slaihand_joint1"]
            self.assertGreater(axis["kp_used"]["mean"], 160.0)
            self.assertLess(axis["kp_used"]["mean"], 280.0)
            self.assertGreater(axis["pd_request_estimate_nm"]["mean"], 2.0)
            self.assertEqual(summary["kp_source"], "schedule")

    def test_plan_target_windows_keep_origin_and_labels(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            path = directory / "signed.jsonl"
            rows = []
            for index in range(600):
                t = index * 0.01
                angle = 0.2 if t >= 3.0 else 0.0
                rows.append(json.dumps(state(t, [angle] * 5,
                                             [angle - math.radians(0.5)] * 5)))
            path.write_text("\n".join(rows) + "\n", encoding="utf-8")
            client = directory / "client.json"
            client.write_text(json.dumps({
                "initial_model_rad": [0.0] * 5,
                "waypoint_results": [
                    {"target_rad": [0.0] * 5},
                    {"target_rad": [0.2] * 5},
                ],
            }), encoding="utf-8")
            summary = analyze_signed.analyze_with_client(path, client, 0.3)
            self.assertEqual(len(summary["segments"]), 2)
            self.assertAlmostEqual(summary["segments"][0]["relative_deg"][0], 0.0, places=4)
            self.assertAlmostEqual(summary["segments"][1]["relative_deg"][0],
                                   math.degrees(0.2), places=3)

    def test_too_few_samples_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "short.jsonl"
            path.write_text(json.dumps(state(0.0, [0.0] * 5, [0.0] * 5)) + "\n",
                            encoding="utf-8")
            with self.assertRaises(ValueError):
                analyze_signed.analyze(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
