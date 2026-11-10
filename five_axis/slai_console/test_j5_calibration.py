"""Offline tests for the manual J5 calibration record. No motor command."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from .j5_calibration import (J5Calibration, build_j5_calibration,
                             load_j5_calibration, save_j5_calibration)


class Tests(unittest.TestCase):
    def build(self, **over):
        fields = dict(zero_position_rad=0., positive_position_rad=.2,
                      travel_min_position_rad=-.5, travel_max_position_rad=.8,
                      user_zero_confirmed=True, user_direction_confirmed=True,
                      user_limits_confirmed=True, now='2026-11-02T10:00:00+08:00')
        fields.update(over)
        return build_j5_calibration(**fields)

    def test_positive_direction(self):
        record = self.build()
        self.assertEqual(record.sign, 1)
        self.assertAlmostEqual(record.zero_offset_rad, 0.)
        self.assertAlmostEqual(record.travel_min_rad, -.5)
        self.assertAlmostEqual(record.travel_max_rad, .8)
        self.assertTrue(record.is_usable())

    def test_negative_direction(self):
        record = self.build(zero_position_rad=1., positive_position_rad=.8,
                            travel_min_position_rad=.2, travel_max_position_rad=1.8)
        self.assertEqual(record.sign, -1)
        self.assertAlmostEqual(record.zero_offset_rad, 1.)
        self.assertAlmostEqual(record.travel_min_rad, -.8)
        self.assertAlmostEqual(record.travel_max_rad, .8)
        self.assertTrue(record.is_usable())

    def test_ambiguous_or_incomplete_is_rejected(self):
        with self.assertRaises(ValueError):
            self.build(positive_position_rad=.01)  # too small to infer a sign
        with self.assertRaises(ValueError):
            self.build(user_zero_confirmed=False)
        with self.assertRaises(ValueError):
            self.build(user_direction_confirmed=False)
        with self.assertRaises(ValueError):
            self.build(user_limits_confirmed=False)
        with self.assertRaises(ValueError):
            self.build(travel_min_position_rad=.3, travel_max_position_rad=.5)  # zero outside
        with self.assertRaises(ValueError):
            self.build(travel_min_position_rad=-.5, travel_max_position_rad=-.45)  # span too small
        with self.assertRaises(ValueError):
            self.build(travel_min_position_rad=float('nan'))

    def test_save_load_round_trip_and_invalid_record_refused(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as folder:
            path = Path(folder)/'j5.json'
            record = self.build()
            save_j5_calibration(record, path)
            loaded = load_j5_calibration(path, required=True)
            self.assertEqual(loaded, record)
            data = json.loads(path.read_text(encoding='utf-8'))
            self.assertTrue(data['validated'])
            data['hardware_sign'] = 1
            data['sign'] = 0
            path.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_j5_calibration(path, required=True)

    def test_missing_returns_none_unless_required(self):
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as folder:
            path = Path(folder)/'missing.json'
            self.assertIsNone(load_j5_calibration(path))
            with self.assertRaises(ValueError):
                load_j5_calibration(path, required=True)

    def test_unusable_record_cannot_be_saved(self):
        record = self.build()
        broken = J5Calibration(**{**record.__dict__, 'sign': 0})
        self.assertFalse(broken.is_usable())
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as folder:
            with self.assertRaises(ValueError):
                save_j5_calibration(broken, Path(folder)/'bad.json')


if __name__ == '__main__':
    unittest.main()
