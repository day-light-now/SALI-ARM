"""Regression for the four-axis gravity feedforward (offline, provisional masses)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gravity_ff  # noqa: E402
from gravity_model import GravityModel  # noqa: E402


class GravityFeedforwardCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.feedforward = gravity_ff.FourAxisGravityFeedforward()

    def test_signs_come_from_the_hardware_mapping(self):
        self.assertEqual(self.feedforward.signs,
                         {"slaihand_joint1": 1, "slaihand_joint2": 1,
                          "slaihand_joint3": 1, "slaihand_joint4": -1})

    def test_motor_torque_is_sign_times_urdf_torque(self):
        for degrees in ([-4.109, 0.109, 1.596, 0.809], [5.891, 10.109, 11.596, 10.809],
                        [0, 0, 0, 0], [-80, -10, -30, 3.3]):
            row = self.feedforward.torques(degrees)
            for index, name in enumerate(gravity_ff.NAMES):
                expected = self.feedforward.signs[name] * row["tau_urdf_nm"][index]
                self.assertAlmostEqual(row["tau_motor_nm"][index], expected, places=12)

    def test_matches_the_existing_reference_model(self):
        rng = np.random.default_rng(20261104)
        for _ in range(25):
            q = [rng.uniform(low, high) for low, high in self.feedforward.limits]
            reference, _ = self.feedforward.model.evaluate(np.array(q))
            row = self.feedforward.torques(np.rad2deg(q))
            np.testing.assert_allclose(row["tau_urdf_nm"], reference, rtol=0, atol=1e-12)

    def test_out_of_limit_is_rejected_not_clipped(self):
        with self.assertRaises(ValueError):
            self.feedforward.torques([201, 0, 0, 0])
        with self.assertRaises(ValueError):
            self.feedforward.torques([0, 0, 0])

    def test_live_poses_stay_far_below_the_estimate_guards(self):
        for degrees, ceiling in (([-4.109, 0.109, 1.596, 0.809], 0.2),
                                 ([5.891, 10.109, 11.596, 10.809], 1.0)):
            row = self.feedforward.torques(degrees)
            self.assertLess(max(abs(v) for v in row["tau_motor_nm"]), ceiling)
            for value, guard in zip(row["tau_motor_nm"], gravity_ff.ESTIMATE_GUARDS):
                self.assertLess(abs(value), guard)

    def test_guard_headroom_is_positive_over_the_workspace(self):
        rows = self.feedforward.guard_headroom()
        self.assertEqual([row["joint"] for row in rows], gravity_ff.NAMES)
        for row in rows:
            self.assertGreater(row["headroom_nm"], 0.0)
            self.assertGreater(row["allowed_error_deg_at_kp"], 0.5)

    def test_cli_requires_provisional_acknowledgement(self):
        import subprocess
        result = subprocess.run([sys.executable, str(HERE / "gravity_ff.py"), "--degrees", "0", "0", "0", "0"],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("allow-provisional", result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
