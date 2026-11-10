"""Pure planning and fake-transport guards; no ROS or motor connection."""
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location("panel", Path(__file__).parent /
    "src/slaihand_bringup/scripts/offline_panel.py")
panel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(panel)


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.limits = [(-1.3963, 3.4907), (-0.17453, 3.3161),
                       (-1.5707963, 1.5708), (-1.5707963, 0.87266)]

    def test_moves_and_demo(self):
        for command, values in [("move", [10, 5, -5, 5]), ("zero", None), ("demo", None)]:
            points = panel.plan(command, values, [0.0] * 4, self.limits)
            previous, elapsed = [0.0] * 4, 0.0
            for q, seconds in points:
                self.assertGreater(seconds, elapsed)
                self.assertLessEqual(max(abs(a-b) for a, b in zip(q, previous)) / (seconds-elapsed), 0.200001)
                previous, elapsed = q, seconds
            if command == "demo":
                self.assertEqual(len(points), 11)
                self.assertEqual(points[-1][0], [0.0]*4)

    def test_invalid_commands(self):
        for q in [[0, 0, 0], [0, 0, 0, 60], [0, math.nan, 0, 0], [0, True, 0, 0], [0, "5", 0, 0]]:
            with self.assertRaises(ValueError):
                panel.plan("move", q, [0.0]*4, self.limits)
        with self.assertRaises(ValueError):
            panel.plan("enable_motor", None, [0.0]*4, self.limits)

    def test_description_guards(self):
        xml = (Path(__file__).parent / "verification/slaihand_expanded.urdf").read_text(encoding="utf-8")
        self.assertEqual(panel.read_limits(xml), self.limits)
        for invalid in [xml.replace(">fake<", ">real<"), xml.replace("OpenArmJointFake", "OpenArmHW"),
                        xml.replace("slaihand_joint4", "slaihand_joint5")]:
            with self.assertRaises(ValueError):
                panel.read_limits(invalid)

    def test_j2_direction_and_zero(self):
        robot = ET.parse(Path(__file__).parent / "src/slaihand_description/urdf/slaihand_four_axis.urdf").getroot()
        joint = robot.find("joint[@name='slaihand_joint2']")
        self.assertEqual(joint.find("axis").get("xyz"), "0 0 1")
        self.assertEqual(joint.find("origin").get("xyz"), "0.039 0.0068669849 0.1351999710")
        self.assertEqual(joint.find("origin").get("rpy"), "1.5707963268 0 1.5707963268")
        for name in ["slaihand_joint1", "slaihand_joint3", "slaihand_joint4"]:
            self.assertEqual(robot.find(f"joint[@name='{name}']/axis").get("xyz"), "0 0 -1")
        # About +Z, a positive angle sends local +X toward +Y; the old -Z sent it toward -Y.
        theta = math.radians(5)
        self.assertGreater(math.sin(theta), 0)
        self.assertEqual(math.cos(theta), math.cos(-theta))


if __name__ == "__main__":
    unittest.main()
