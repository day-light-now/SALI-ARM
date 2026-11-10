"""Static-pose gravity-identification candidates: offline shape/budget checks only."""
import json
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
TRAJECTORIES = HERE / "gravity_trajectories"
ROOT = HERE.parents[2]
SCRIPTS = ROOT / "src/slaihand_bringup/scripts"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(HERE.parent))

import control_panel_model as model  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
FIVE_BOUNDS = model.load_limits(URDF) + [tuple(map(math.radians, (-30, 30)))]
WIDE_BOUNDS = model.load_limits(URDF) + [tuple(map(math.radians, (-90, 90)))]


class GravityTrajectoryLibrary(unittest.TestCase):
    def test_all_candidate_files_are_schema2_five_axis_and_j5_static(self):
        files = sorted(TRAJECTORIES.glob("GRAV_*.json"))
        self.assertEqual(len(files), 5)
        for path in files:
            with self.subTest(path=path.name):
                document = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(document["schema"], model.PLAN_SCHEMA)
                self.assertEqual(document["coordinates"], "relative_start_deg")
                self.assertGreaterEqual(len(document["waypoints"]), 3)
                for waypoint in document["waypoints"]:
                    self.assertEqual(len(waypoint["degrees"]), 5)
                    self.assertEqual(waypoint["degrees"][4], 0.0)
                    self.assertGreaterEqual(waypoint["dwell_s"], 0.5)
                # Use the same planner as the panel; J5 is fixed at relative zero.
                payload = model.preview_payload(document, [0.0] * 5, FIVE_BOUNDS)
                self.assertLessEqual(payload["total_s"] + 8.0, model.SESSION_BUDGET_S)
                self.assertEqual(payload["axes"], 5)

    def test_j5_has_no_motion_in_legacy_j1_j4_candidates(self):
        for path in TRAJECTORIES.glob("GRAV_*.json"):
            document = json.loads(path.read_text(encoding="utf-8"))
            j5_offsets = {waypoint["degrees"][4] for waypoint in document["waypoints"]}
            self.assertEqual(j5_offsets, {0.0})

    def test_j5_inclusive_candidate_files_are_bounded_five_axis_plans(self):
        files = sorted(TRAJECTORIES.glob("J5_GRAV_*.json"))
        self.assertEqual(len(files), 4)
        for path in files:
            with self.subTest(path=path.name):
                document = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(document["schema"], model.PLAN_SCHEMA)
                self.assertEqual(document["coordinates"], "relative_start_deg")
                payload = model.preview_payload(document, [0.0] * 5, FIVE_BOUNDS)
                self.assertEqual(payload["axes"], 5)
                self.assertLessEqual(payload["total_s"] + 8.0, model.SESSION_BUDGET_S)
                self.assertTrue(any(waypoint["degrees"][4] != 0.0
                                    for waypoint in document["waypoints"]))
                self.assertTrue(all(abs(waypoint["degrees"][4]) <= 30.0
                                    for waypoint in document["waypoints"]))

    def test_panel_group_module_exposes_only_bounded_position_plans(self):
        import five_axis_trajectory_groups as panel_groups
        self.assertEqual(set(panel_groups.NAMES), {
            "WIDE_J5_60", "WIDE_J5_80", "WIDE_J1_J2_J5",
            "WIDE_J3_J4_J5", "WIDE_MIXED_GRID", "WIDE_J5_COMBINED"})
        for name, document in panel_groups.groups().items():
            with self.subTest(name=name):
                self.assertEqual(document["schema"], model.PLAN_SCHEMA)
                self.assertEqual(document["coordinates"], "relative_start_deg")
                model.preview_payload(document, [0.0] * 5, WIDE_BOUNDS)
                source = json.loads((TRAJECTORIES / (name + ".json")).read_text(encoding="utf-8"))
                self.assertEqual(document, source)

    def test_wide_gravity_candidates_stay_within_planner_limits(self):
        files = sorted(TRAJECTORIES.glob("WIDE_*.json"))
        self.assertEqual(len(files), 6)
        for path in files:
            with self.subTest(path=path.name):
                document = json.loads(path.read_text(encoding="utf-8"))
                self.assertEqual(document["schema"], model.PLAN_SCHEMA)
                self.assertEqual(document["coordinates"], "relative_start_deg")
                payload = model.preview_payload(document, [0.0] * 5, WIDE_BOUNDS)
                self.assertEqual(payload["axes"], 5)
                self.assertLessEqual(payload["total_s"] + 8.0, model.SESSION_BUDGET_S)
                self.assertTrue(any(abs(waypoint["degrees"][4]) > 30.0
                                    for waypoint in document["waypoints"]))
                self.assertTrue(all(abs(value) <= 90.0
                                    for waypoint in document["waypoints"]
                                    for value in waypoint["degrees"][:4]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
