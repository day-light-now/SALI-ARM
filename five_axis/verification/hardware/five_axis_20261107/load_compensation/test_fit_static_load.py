"""Pure checks for the path-local load-proxy fit."""
import math
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import fit_static_load as load  # noqa: E402


class PathLoadProxy(unittest.TestCase):
    def test_quadratic_recovery(self):
        points = [(x, 1.0 + 0.2 * x + 0.01 * x * x) for x in (0, 5, 10, 20, 30, 40)]
        result = load.fit(points)
        for got, expected in zip(result["coefficients"], [1.0, 0.2, 0.01]):
            self.assertAlmostEqual(got, expected, places=6)
        self.assertLess(result["leave_one_out_max_error_nm"], 1e-6)
        self.assertFalse(result["deployable"])

    def test_validity_domain_is_not_extrapolated(self):
        result = load.fit([(0, 0.0), (10, 1.0), (20, 2.0), (30, 3.0)])
        self.assertEqual(result["validity_domain_deg"], [0, 30])

    def test_too_few_points_are_refused(self):
        with self.assertRaises(ValueError):
            load.fit([(0, 0.0), (10, 1.0), (20, 2.0)])


if __name__ == "__main__":
    unittest.main(verbosity=2)
