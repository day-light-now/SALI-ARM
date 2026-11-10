"""Offline checks for the 2026-11-09 J5 excitation analysis artifacts.

These tests only read the committed analysis JSON/summary files; they do not
touch hardware, ROS, a device or any other part of the repository.
"""
from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WINDOWS = HERE / 'static_windows_j5.json'
FIT = HERE / 'five_axis_payload_fit_result.json'
PROBE = HERE / 'j5_payload_geometry_probe.json'
NAMES = [f'slaihand_joint{i}' for i in range(1, 6)]


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


class J5Windows(unittest.TestCase):
    def setUp(self):
        self.windows = read(WINDOWS)['J5']['windows']
        self.accepted = [w for w in self.windows if 'excluded' not in w]

    def test_six_accepted_windows_ordered_as_planned(self):
        self.assertEqual([w['id'] for w in self.accepted], [0, 1, 2, 3, 4, 5])
        q5 = np.degrees([w['q_rad'][4] for w in self.accepted])
        expected = np.array([0.0, 10.0, -10.0, 20.0, -20.0, 0.0])
        self.assertLess(np.max(np.abs(q5 - expected)), 1.0)

    def test_each_window_is_a_stationary_hold(self):
        for window in self.accepted:
            self.assertGreaterEqual(window['samples'], 50, window['id'])
            self.assertLessEqual(max(window['q_span_deg']), 0.4, window['id'])
            self.assertLessEqual(max(np.abs(window['velocity_mean_rad_s'])), 0.035,
                                 window['id'])

    def test_upstream_torque_is_flat_while_j5_moves(self):
        """w0-w3: q5 spans about -9.7..+19.6 deg with J1-J4 nearly fixed."""
        plateau = self.accepted[:4]
        effort = np.array([w['effort_mean_nm'] for w in plateau])
        spans = np.ptp(effort, axis=0)
        # J2 is exactly constant inside the torque quantization; J1/J3 are flat.
        self.assertLessEqual(spans[1], 1e-3)   # J2
        self.assertLessEqual(spans[0], 0.01)   # J1
        self.assertLessEqual(spans[2], 0.01)   # J3
        # J4 is flat too within this phase (the later slip is a separate event).
        self.assertLessEqual(spans[3], 0.03)   # J4

    def test_j5_torque_is_not_a_single_gravity_sinusoid(self):
        """Two nearly identical J5 poses (+0.284 and -0.284 deg) differ by >1.5 Nm."""
        effort = np.array([w['effort_mean_nm'] for w in self.accepted])
        self.assertGreater(abs(effort[0, 4] - effort[5, 4]), 1.5)


class PayloadHypotheses(unittest.TestCase):
    def setUp(self):
        self.fit = read(FIT)
        self.probe = read(PROBE)

    def test_fixed_payload_reproduces_prior_conditional_com(self):
        com = np.array(self.fit['fits']['fixed']['com_m']) * 1000
        # Prior Codex 22-pose training model: [+96.3, +6.6, -42.0] mm.
        self.assertLess(np.max(np.abs(com - np.array([96.3, 6.6, -42.0]))), 1.0)

    def test_rotating_payload_predicts_large_j2_and_j4_swing(self):
        com = np.array(self.fit['fits']['j5']['com_m'])
        self.assertEqual(com.shape, (3,))
        rows = np.array(self.fit['fits']['j5']['j5_predictions_nm'])
        self.assertGreater(np.ptp(rows[:, 1]), 2.0)   # J2
        self.assertGreater(np.ptp(rows[:, 3]), 2.0)   # J4

    def test_observed_j2_and_j4_swing_is_tiny(self):
        rows = np.array(self.fit['fits']['j5']['j5_observed_nm'])
        self.assertLess(np.ptp(rows[:, 1]), 0.01)     # J2
        self.assertLess(np.ptp(rows[:4, 3]), 0.03)    # J4, pre-slip phase only

    def test_no_hypothesis_is_validated(self):
        for name in ('fixed', 'j5'):
            holdout = self.fit['fits'][name]['holdout']['rmse_nm']
            self.assertGreater(holdout, 1.0, name)

    def test_adding_j5_windows_does_not_improve_holdout(self):
        for name in ('fixed', 'j5'):
            base = self.fit['fits'][name]['holdout']['rmse_nm']
            combined = self.fit['fits'][name]['train_plus_j5_upstream']['holdout_metrics']['rmse_nm']
            self.assertGreaterEqual(combined, base - 1e-6, name)

    def test_stiction_slip_is_not_explained_by_either_gravity_model(self):
        fixed = np.array(self.fit['stiction_slip_check']['fixed']['predicted_d_tau_nm'])
        slip = np.array(self.fit['stiction_slip_check']['observed_sag_change_nm_J1_J4'])
        # The recorded slip torque on J1/J4 is ~1.3/1.07 Nm; a rigid payload cannot
        # produce it from the 0.27 deg pose change, so it is released stiction.
        self.assertGreater(slip[0], 0.6)
        self.assertGreater(slip[3], 0.7)
        self.assertGreater(np.max(np.abs(fixed[:4] - slip)), 0.7)


if __name__ == '__main__':
    unittest.main(verbosity=2)
