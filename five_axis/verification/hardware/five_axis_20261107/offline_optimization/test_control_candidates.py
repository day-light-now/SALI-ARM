"""Unit checks for the offline five-axis control study (no hardware)."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import control_candidates as lab  # noqa: E402


class ControlCandidates(unittest.TestCase):
    def setUp(self):
        self.observations = lab.make_observations({
            "J1": [(5, 0.869), (10, 1.673), (20, 2.820), (30, 3.553), (35, 3.548), (40, 3.564)],
            "J2": [(5, 1.088), (10, 1.367), (20, 2.230), (30, 3.072), (35, 3.395), (40, 3.805)],
            "J3": [(5, 0.235), (10, 0.317), (20, 0.154), (30, 0.318), (35, 0.080), (40, 0.020)],
            "J4": [(5, 0.519), (10, 1.061), (20, 1.749), (30, 2.329), (35, 2.608), (40, 2.843)],
            "J5": [(5, 0.366), (10, 0.143), (20, 0.110), (30, 0.100), (35, 0.007), (40, 0.133)],
        })

    def test_constant_high_gain_still_loses_large_angle(self):
        rows = lab.constant_gain_residual(self.observations, "J1", 280.0)
        self.assertAlmostEqual(rows[-1], 2.037, places=2)
        self.assertGreater(rows[-1], 0.5)

    def test_angle_schedule_is_bounded_and_not_a_fix(self):
        for axis in ("J1", "J2", "J4"):
            for angle in (0, 5, 10, 20, 30, 40, 90):
                kp = lab.scheduled_gain(axis, angle)
                self.assertGreaterEqual(kp, lab.BASE_KP)
                self.assertLessEqual(kp, lab.PROTOCOL_KP_MAX)
            rows = lab.scheduled_gain_residual(self.observations, axis)
            self.assertGreater(rows[-1], 0.5)
        self.assertEqual(lab.scheduled_gain("J3", 40), lab.BASE_KP)
        self.assertEqual(lab.scheduled_gain("J5", 40), lab.BASE_KP)

    def test_feedforward_polynomial_fits_and_leave_one_out_stays_bounded(self):
        fit = lab.feedforward_fit(self.observations)
        loo = lab.leave_one_out_torque_error(self.observations)
        residual = lab.feedforward_residual(self.observations, fit)
        for axis in ("J1", "J2", "J4"):
            self.assertIn(axis, fit)
            self.assertLess(loo[axis], 1.0)  # Nm, only a loose sanity bound
            self.assertLess(max(abs(value) for value in residual[axis]), 0.5)
        self.assertGreater(max(lab.constant_gain_residual(self.observations, "J1", 160)), 1.0)

    def test_integral_trim_can_reduce_static_error_in_ideal_model(self):
        error, trim = lab.integral_final_error(3.564, 200.0)
        self.assertLess(abs(error), 0.5)
        self.assertGreater(trim, 0)

    def test_no_nan_or_infinite_candidate_values(self):
        fit = lab.feedforward_fit(self.observations)
        for axis in ("J1", "J2", "J4"):
            self.assertTrue(all(math.isfinite(value) for value in fit[axis]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
