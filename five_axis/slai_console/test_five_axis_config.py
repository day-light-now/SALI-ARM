"""Offline tests for the operator-confirmed five-axis config gate."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from .five_axis_config import (AxisSetting, FiveAxisSettings, build_envelopes,
                               ensure_template, load_settings, template_settings)
from .j5_calibration import build_j5_calibration


def settings(confirmed=True, j5_enabled=True, overrides=None):
    overrides = overrides or {}
    axes = {i: AxisSetting(-78. if i == 1 else (-8. if i == 2 else (-88. if i == 3 else -88.)),
                           198. if i == 1 else (188. if i == 2 else (88. if i == 3 else 48.)),
                           2., 32., 2.) for i in (1, 2, 3, 4)}
    axes[5] = AxisSetting(None, None, 1., 32., 2.)
    axes.update(overrides)
    return FiveAxisSettings(confirmed, axes, 'test', j5_enabled=j5_enabled)


def calibration():
    return build_j5_calibration(zero_position_rad=0., positive_position_rad=.2,
                                travel_min_position_rad=-.5,
                                travel_max_position_rad=.8,
                                user_zero_confirmed=True,
                                user_direction_confirmed=True,
                                user_limits_confirmed=True)


class Tests(unittest.TestCase):
    def test_template_is_not_authorized(self):
        settings0 = template_settings()
        self.assertFalse(settings0.user_confirmed)
        with self.assertRaises(ValueError):
            build_envelopes(settings0)
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as folder:
            path = Path(folder)/'config.json'
            ensure_template(path)
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertFalse(data['user_confirmed'])
            with self.assertRaises(ValueError):
                load_settings(path)

    def test_without_j5_calibration_only_four_axes_are_built(self):
        envelopes, active = build_envelopes(settings())
        self.assertEqual(active, (1, 2, 3, 4))
        self.assertNotIn(5, envelopes)
        self.assertAlmostEqual(math.degrees(envelopes[1].lower), -78.)
        self.assertAlmostEqual(math.degrees(envelopes[1].upper), 198.)

    def test_j5_calibration_enables_fifth_but_only_within_calibrated_travel(self):
        envelopes, active = build_envelopes(settings(), calibration())
        self.assertEqual(active, (1, 2, 3, 4, 5))
        self.assertAlmostEqual(math.degrees(envelopes[5].lower), -.5*180/math.pi, places=6)
        self.assertAlmostEqual(math.degrees(envelopes[5].upper), .8*180/math.pi, places=6)
        narrower = settings(overrides={5: AxisSetting(.2, .4, 1., 32., 2.)})
        envelopes, active = build_envelopes(narrower, calibration())
        self.assertAlmostEqual(math.degrees(envelopes[5].lower), .2, places=6)
        self.assertAlmostEqual(math.degrees(envelopes[5].upper), .4, places=6)
        wider = settings(overrides={5: AxisSetting(-40., 60., 1., 32., 2.)})
        envelopes, active = build_envelopes(wider, calibration())
        self.assertAlmostEqual(math.degrees(envelopes[5].lower), -.5*180/math.pi, places=6)
        self.assertAlmostEqual(math.degrees(envelopes[5].upper), .8*180/math.pi, places=6)

    def test_j5_can_be_explicitly_disabled(self):
        envelopes, active = build_envelopes(settings(j5_enabled=False), calibration())
        self.assertEqual(active, (1, 2, 3, 4))

    def test_j1_j4_cannot_widen_model_limits(self):
        with self.assertRaises(ValueError):
            build_envelopes(settings(overrides={1: AxisSetting(200., 210., 2., 32., 2.)}))
        with self.assertRaises(ValueError):
            build_envelopes(settings(overrides={4: AxisSetting(-90., 60., 2., 32., 2.)}))
        with self.assertRaises(ValueError):
            build_envelopes(settings(overrides={2: AxisSetting(10., 5., 2., 32., 2.)}))

    def test_speed_error_temperature_and_gain_cannot_widen(self):
        for override in (
                AxisSetting(-1., 1., 2., 32., 2., speed_rad_s=.03),
                AxisSetting(-1., 1., 2., 32., 2., following_error_deg=4.5),
                AxisSetting(-1., 1., 2., 32., 2., feedback_speed_rad_s=1.1),
                AxisSetting(-1., 1., 2., 32., 2., temperature_c=55.),
                AxisSetting(-1., 1., 2., 501., 2.),
                AxisSetting(-1., 1., 2., 32., 6.),
                AxisSetting(-1., 1., 2., 32., 2., arrival_error_deg=1.5),
                AxisSetting(-1., 1., 0., 32., 2.)):
            with self.assertRaises(ValueError):
                build_envelopes(settings(overrides={3: override}))

    def test_load_requires_confirmation_and_exact_axes(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as folder:
            path = Path(folder)/'config.json'
            base = dict(user_confirmed=True, axes={str(i): dict(
                travel_min_deg=None, travel_max_deg=None,
                estimate_guard_nm=2., kp=32., kd=2.) for i in (1, 2, 3, 4, 5)})
            path.write_text(json.dumps(base), encoding='utf-8')
            loaded = load_settings(path)
            self.assertTrue(loaded.user_confirmed)
            envelopes, active = build_envelopes(loaded)
            self.assertEqual(active, (1, 2, 3, 4))
            del base['axes']['5']
            path.write_text(json.dumps(base), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_settings(path)


if __name__ == '__main__':
    unittest.main()
