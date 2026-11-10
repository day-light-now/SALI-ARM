"""Validate the five-axis local +2/-2 degree diagnostic plans offline."""
import json
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import control_panel_model as model  # noqa: E402

PLANS = HERE / "five_axis_2deg_diagnostics"
BOUNDS = model.load_limits(ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf") + [
    tuple(map(math.radians, (-90, 90)))]


class FiveAxisLocalDiagnostics(unittest.TestCase):
    def test_all_ten_axis_direction_plans_are_finite_and_one_axis(self):
        files = sorted(PLANS.glob("J*_POS.json")) + sorted(PLANS.glob("J*_NEG.json"))
        self.assertEqual(len(files), 10)
        for path in files:
            axis = int(path.name[1]) - 1
            sign = 1 if path.name.endswith("_POS.json") else -1
            with self.subTest(path=path.name):
                document = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(document["schema"], model.PLAN_SCHEMA)
                self.assertEqual(document["coordinates"], "relative_start_deg")
                _, points, windows = model.compile_document(document, [0.0] * 5, BOUNDS)
                self.assertEqual(len(document["waypoints"]), 3)
                self.assertEqual(document["waypoints"][1]["degrees"][axis], sign * 2.0)
                self.assertEqual(document["waypoints"][2]["degrees"], [0.0] * 5)
                for index in range(5):
                    if index != axis:
                        self.assertEqual(document["waypoints"][1]["degrees"][index], 0.0)
                self.assertGreaterEqual(document["waypoints"][1]["move_s"], 8.0)
                self.assertLessEqual(document["waypoints"][1]["move_s"], 10.0)
                self.assertGreaterEqual(document["waypoints"][1]["dwell_s"], 3.0)
                self.assertLessEqual(document["waypoints"][1]["dwell_s"], 5.0)
                self.assertTrue(all(window["end_s"] <= 178.0 for window in windows))

    def test_plans_are_not_an_execution_queue(self):
        for path in PLANS.glob("J*.json"):
            document = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(set(document), {"schema", "coordinates", "waypoints"})
            self.assertFalse(any("auto" in key.lower() for key in document))
            self.assertNotIn("execute", path.read_text(encoding="utf-8").lower())


if __name__ == "__main__":
    unittest.main(verbosity=2)
