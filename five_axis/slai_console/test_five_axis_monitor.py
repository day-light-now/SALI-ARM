"""Offline five-axis monitor/session integration with a synthetic 5-axis bus."""
import math
import struct
import tempfile
import unittest
from pathlib import Path

from .core import LIMITS
from .five_axis_config import AxisSetting, FiveAxisSettings
from .five_axis_mit import (AXES5, AxisProfile, FiveAxisAcceptance,
                            decode_mit_control_data)
from .five_axis_monitor import IDS, load_mapping5, request, usb_packet
from .five_axis_session import build_five_session, fresh_active_feedback
from .j5_calibration import build_j5_calibration, save_j5_calibration
from .test_reader_fixture import make_reader5
from .test_watchdog_check import Clock


class Bus5:
    def __init__(self, mapping, clock):
        self.mapping = mapping; self.clock = clock
        self.buf = bytearray(); self.frames = []; self.history = []
        self.states = {i: 0 for i in IDS}
        self.raw = {i: self.default_code(i) for i in IDS}

    def default_code(self, i):
        m = self.mapping[i]
        return round((m['zero_offset_rad']+12.5)*65535/25)

    def feedback(self, i, code=None):
        code = self.raw[i] if code is None else code
        return bytes([self.states[i]*16+i, code >> 8, code & 255,
                      0x7f, 0xf7, 0xff, 25, 25])

    def _append(self, payload):
        self.buf.extend(bytes([0xAA, 0x11, 8, 0, 0, 0, 0])+payload+b'\x55')

    def write(self, frame):
        self.frames.append(bytes(frame))
        cid = int.from_bytes(frame[13:17], 'little'); p = frame[21:29]
        self.history.append((self.clock.now(), cid, p))
        if cid == 0x7FF:
            if p[2] == 0x33:
                i, rid = p[0], p[3]
                value = {7: 0, 8: i, 9: 2000, 10: 1, 21: 12.5, 22: 45., 23: 54.}[rid]
                self._append(bytes([i, 0, 0x33, rid]) +
                             struct.pack('<I' if rid < 21 else '<f', value))
            elif p[2] == 0xCC:
                self._append(self.feedback(p[0]))
            return len(frame)
        if p[:7] == b'\xff'*7:
            self.states[cid] = 1 if p[-1] == 0xFC else 0
            self._append(self.feedback(cid))
        else:
            self.raw[cid] = (p[0] << 8) | p[1]
            self._append(self.feedback(cid, self.raw[cid]))
        return len(frame)

    def read(self, n):
        result = bytes(self.buf[:n]); del self.buf[:n]; return result

    @property
    def in_waiting(self):
        return len(self.buf)

    def reset_input_buffer(self):
        self.buf.clear()

    def close(self):
        pass


def settings(j5_enabled=True):
    axes = {
        1: AxisSetting(-78., 198., 2., 32., 2.),
        2: AxisSetting(-8., 188., 2., 32., 2.),
        3: AxisSetting(-88., 88., 2., 32., 2.),
        4: AxisSetting(-88., 48., 2., 32., 2.),
        5: AxisSetting(None, None, 1., 32., 2.),
    }
    return FiveAxisSettings(True, axes, 'test', j5_enabled=j5_enabled)


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.temp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.cal_path = Path(self.temp.name)/'j5.json'
        self.cal = build_j5_calibration(
            zero_position_rad=.2, positive_position_rad=.4,
            travel_min_position_rad=-.5, travel_max_position_rad=.8,
            user_zero_confirmed=True, user_direction_confirmed=True,
            user_limits_confirmed=True)
        save_j5_calibration(self.cal, self.cal_path)
        self.mapping = load_mapping5(self.cal_path)
        self.bus = Bus5(self.mapping, self.clock)
        self.reader = make_reader5('FAKE_ONLY', self.mapping,
                                   serial_factory=lambda *a, **k: self.bus,
                                   cal_path=self.cal_path)
        self.feed_all(self.reader, self.bus)

    def tearDown(self):
        try:
            self.reader.close()
        except Exception:
            pass
        self.temp.cleanup()

    def feed_all(self, reader, bus):
        for i in IDS:
            for rid, value in {7: 0, 8: i, 9: 2000, 10: 1,
                               21: 12.5, 22: 45., 23: 54.}.items():
                payload = bytes([i, 0, 0x33, rid]) + \
                    struct.pack('<I' if rid < 21 else '<f', value)
                reader.state.feed(0, payload, self.clock.now())
            reader.state.feed(0, bus.feedback(i), self.clock.now())

    def test_mapping_without_calibration_marks_j5_uncalibrated(self):
        missing = Path(self.temp.name)/'missing.json'
        mapping = load_mapping5(missing)
        self.assertFalse(mapping[5]['calibrated'])
        self.assertEqual(mapping[5]['hardware_sign'], 1)
        self.assertEqual(mapping[5]['zero_offset_rad'], 0.)
        self.assertIsNone(mapping[5]['calibration_error'])

    def test_mapping_with_calibration_uses_record(self):
        mapping = load_mapping5(self.cal_path)
        self.assertTrue(mapping[5]['calibrated'])
        self.assertEqual(mapping[5]['hardware_sign'], self.cal.sign)
        self.assertAlmostEqual(mapping[5]['zero_offset_rad'], self.cal.zero_offset_rad)
        self.assertAlmostEqual(mapping[5]['calibration'].travel_min_rad, self.cal.travel_min_rad)

    def test_only_j1_to_j5_read_requests_are_supported(self):
        for i in IDS:
            self.assertEqual(usb_packet(i)[21], i)
            self.assertEqual(usb_packet(i)[23], 0xCC)
        with self.assertRaises(ValueError):
            request(6)
        with self.assertRaises(ValueError):
            usb_packet(5, 33)

    def test_reader_sees_all_five_axes(self):
        self.assertEqual(IDS, (1, 2, 3, 4, 5))
        for i in IDS:
            self.assertTrue(self.reader.state.ready(i), i)
            self.assertTrue(self.reader.state.fresh(i, self.clock.now()), i)
        self.assertAlmostEqual(self.reader.state.feedback[5]['model_deg'], 0., delta=1e-2)
        self.assertTrue(self.reader.state.mapping[5]['calibrated'])

    def test_build_rejects_j5_without_reader_calibration(self):
        missing = Path(self.temp.name)/'missing.json'
        mapping = load_mapping5(missing)
        bus = Bus5(mapping, self.clock)
        reader = make_reader5('FAKE_ONLY2', mapping,
                              serial_factory=lambda *a, **k: bus,
                              cal_path=missing)
        try:
            self.feed_all(reader, bus)
            with self.assertRaises(ValueError):
                build_five_session(reader, settings(), self.ticket(), lambda *a, **k: None,
                                   clock=self.clock.now, calibration=self.cal)
        finally:
            reader.close()

    def ticket(self):
        return FiveAxisAcceptance(True, True, True, True, 'test', 'POLICY', 'CONFIG')

    def test_fresh_preflight_bursts_only_active_axes(self):
        before = len(self.bus.history)
        self.assertTrue(fresh_active_feedback(self.reader, (1, 2, 3, 4),
                                              clock=self.clock.now,
                                              sleep=self.clock.sleep))
        now = self.clock.now()
        for i in (1, 2, 3, 4):
            self.assertLessEqual(now-self.reader.state.at[i], .05)
        queried = [p[0] for _, cid, p in self.bus.history[before:] if cid == 0x7FF]
        self.assertEqual(queried, [1, 2, 3, 4])
        self.assertNotIn(5, queried)

    def test_build_allows_out_of_range_recovery_axis(self):
        m = self.mapping[3]
        code = round((m['zero_offset_rad'] + m['hardware_sign']*math.radians(93.) + 12.5)
                     * 65535/25)
        self.bus.raw[3] = code
        self.feed_all(self.reader, self.bus)
        session = build_five_session(self.reader, settings(), self.ticket(),
                                     lambda *a, **k: None, clock=self.clock.now,
                                     calibration=self.cal, recovery_axes=(3,))
        self.assertEqual(session.controller.recovery_axes, frozenset({3}))
        session.close()

    def test_session_runs_five_axes_and_stops_all(self):
        session = build_five_session(self.reader, settings(), self.ticket(),
                                     lambda *a, **k: None, clock=self.clock.now,
                                     calibration=self.cal)
        self.assertEqual(session.controller.active_axes, AXES5)
        session.start()
        state = None
        for _ in range(320):
            self.clock.sleep(.010001)
            state = session.cycle()
            if state == 'HOLD':
                break
        self.assertEqual(state, 'HOLD', session.controller.reason)
        j5_mit = [(cid, p) for _, cid, p in self.bus.history
                  if cid == 5 and p[:7] != b'\xff'*7]
        self.assertTrue(j5_mit)
        profile = AxisProfile(5, 1, 0, 2000, 12.5, 45., 54.,
                              self.cal.sign, self.cal.zero_offset_rad)
        decoded = [decode_mit_control_data(profile, p) for _, p in j5_mit]
        self.assertTrue(any(d.kp > 31. for d in decoded))
        session.stop()
        self.clock.sleep(.010001)
        self.assertEqual(session.cycle(), 'DISABLED')
        fd = [cid for _, cid, p in self.bus.history if p == b'\xff'*7+b'\xfd']
        self.assertEqual(fd, [1, 2, 3, 4, 5])
        session.close()


if __name__ == '__main__':
    unittest.main()
