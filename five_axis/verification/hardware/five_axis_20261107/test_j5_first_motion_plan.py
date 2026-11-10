"""Validate the first J5-only-target candidate plan shape, limits, and speed."""
import json
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import joint_waypoint_plan as planner  # noqa: E402
import control_panel_model as model  # noqa: E402

PLAN = HERE / "j5_first_motion_10deg.json"
BOUNDS = model.load_limits(ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf") + [
    (-math.pi / 2, math.pi / 2)]


class FirstJ5Motion(unittest.TestCase):
    def test_only_j5_moves_and_target_is_ten_degrees(self):
        document = json.loads(PLAN.read_text(encoding="utf-8"))
        self.assertEqual(document["schema"], planner.SCHEMA)
        self.assertEqual(document["coordinates"], "relative_start_deg")
        points, windows = planner.compile_plan(document, [0.0] * 5, BOUNDS)
        targets = [window["target_rad"] for window in windows]
        self.assertTrue(all(values[:4] == [0.0, 0.0, 0.0, 0.0] for values in targets))
        self.assertTrue(any(abs(math.degrees(values[4]) - 10.0) < 1e-9 for values in targets))
        self.assertTrue(all(abs(math.degrees(values[4])) <= 90.0 for values in targets))
        self.assertEqual(targets[-1], [0.0] * 5)

    def test_reference_speed_does_not_exceed_half_rad_per_second(self):
        document = json.loads(PLAN.read_text(encoding="utf-8"))
        points, _ = planner.compile_plan(document, [0.0] * 5, BOUNDS)
        previous = None
        for at, q in points:
            if previous is not None:
                previous_at, previous_q = previous
                dt = at - previous_at
                self.assertGreater(dt, 0.0)
                for before, after in zip(previous_q, q):
                    self.assertLessEqual(abs(after - before) / dt, 0.5 + 1e-6)
            previous = (at, q)

    def test_finite_plan_under_session_budget(self):
        document = json.loads(PLAN.read_text(encoding="utf-8"))
        points, windows = planner.compile_plan(document, [0.0] * 5, BOUNDS)
        self.assertLessEqual(windows[-1]["end_s"] + 8.0, 178.0)
        self.assertEqual(len(points) >= 2, True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
