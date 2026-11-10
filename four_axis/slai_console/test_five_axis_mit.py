"""Offline five-axis MIT supervisor tests. No serial, no hardware, no writes."""
import math
import unittest

from .five_axis_mit import (AXES5, AxisEnvelope, AxisObservation, AxisProfile,
                            FiveAxisAcceptance, FiveAxisController,
                            FiveAxisEndpoint, FiveSnapshot)
from .openarm_mit import decode_mit_control_data


class Wire:
    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame)


class Harness:
    def __init__(self, active=AXES5, velocity_ack=False, follow=True):
        self.now = 10.
        self.wire = Wire()
        self.events = []
        self.audit = lambda event, **fields: self.events.append((event, fields))
        self.endpoint = FiveAxisEndpoint(self.wire, self.audit, lambda: self.now)
        self.profiles = {i: AxisProfile(i, 1, 0, 2000, 12.5, 45., 54.,
                                        -1 if i == 4 else 1, 0.) for i in active}
        self.envelopes = {i: AxisEnvelope(-math.radians(10), math.radians(10), .02,
                                          2., math.radians(4), 32., 2., 50., .2,
                                          math.radians(.5)) for i in active}
        self.ticket = FiveAxisAcceptance(True, True, True, True, 'test',
                                         'POLICY', 'CONFIG', velocity_ack)
        self.controller = FiveAxisController(self.endpoint, self.profiles,
                                             self.envelopes, self.ticket,
                                             self.audit, active_axes=active)
        self.states = {i: 0 for i in active}
        self.q = {i: 0. for i in active}
        self.dq = {i: 0. for i in active}
        self.torque = {i: 0. for i in active}
        self.follow = follow
        self.controller.start(self.snap(), self.now)

    def snap(self, states=None, q=None, dq=None, at=None):
        states = self.states if states is None else states
        q = self.q if q is None else q
        dq = self.dq if dq is None else dq
        at = self.now if at is None else at
        rows = tuple(AxisObservation(p, p.raw_q(q[i]), p.raw_dq(dq[i]),
                                     self.torque[i], states[i], at, 25., 25., '')
                     for i, p in self.profiles.items())
        return FiveSnapshot('test', at, rows, 'sha', 'log')

    def tick(self, states=None, q=None, dq=None):
        self.now += .010001
        before = len(self.wire.frames)
        self.controller.tick(self.snap(states, q, dq), self.now)
        for frame in self.wire.frames[before:]:
            cid, payload = frame[13], frame[21:29]
            if payload[:7] == b'\xff'*7:
                if payload[-1] == 0xFC:
                    self.states[cid] = 1
                elif payload[-1] == 0xFD:
                    self.states[cid] = 0
            elif cid in self.q and self.follow:
                decoded = decode_mit_control_data(self.profiles[cid], payload)
                self.q[cid] = self.profiles[cid].model_q(decoded.q)
                self.dq[cid] = self.profiles[cid].model_dq(decoded.dq)
        return self.controller.state

    def hold(self):
        for _ in range(300):
            if self.tick() == 'HOLD':
                return
        raise AssertionError(self.controller.reason)

    def mit_frames(self, cid=None):
        result = []
        for frame in self.wire.frames:
            cid0, payload = frame[13], frame[21:29]
            if cid0 in self.profiles and payload[:7] != b'\xff'*7:
                if cid is None or cid0 == cid:
                    result.append((cid0, decode_mit_control_data(self.profiles[cid0], payload)))
        return result

    def fd_frames(self):
        return [f for f in self.wire.frames
                if f[21:29] == b'\xff'*7+b'\xfd']


class ProfileTests(unittest.TestCase):
    def test_no_default_sign_zero_or_wrong_timeout(self):
        with self.assertRaises(ValueError):
            AxisProfile(5, 1, 0, 2000, 12.5, 45., 54., 0, 0.)
        with self.assertRaises(ValueError):
            AxisProfile(5, 1, 0, 2000, 12.5, 45., 54., 1, float('nan'))
        with self.assertRaises(ValueError):
            AxisProfile(5, 1, 0, 1000, 12.5, 45., 54., 1, 0.)
        with self.assertRaises(ValueError):
            AxisProfile(6, 1, 0, 2000, 12.5, 45., 54., 1, 0.)

    def test_envelope_cannot_widen_beyond_bench_boundary(self):
        with self.assertRaises(ValueError):
            AxisEnvelope(-1, 1, .03, 2., math.radians(4), 32., 2., 50., .2, .01)
        with self.assertRaises(ValueError):
            AxisEnvelope(-1, 1, .02, 2., math.radians(4.1), 32., 2., 50., .2, .01)
        with self.assertRaises(ValueError):
            AxisEnvelope(-1, 1, .02, 2., math.radians(4), 32., 2., 51., .2, .01)

    def test_acceptance_requires_five_axis_and_velocity_ticket(self):
        with self.assertRaises(ValueError):
            FiveAxisAcceptance(True, True, True, False, 'id', 'p', 'c').validate()
        with self.assertRaises(ValueError):
            FiveAxisAcceptance(True, True, True, True, '', 'p', 'c').validate()
        FiveAxisAcceptance(True, True, True, True, 'id', 'p', 'c').validate()


class ControllerTests(unittest.TestCase):
    def test_start_emits_nothing(self):
        h = Harness()
        self.assertEqual(h.wire.frames, [])
        self.assertEqual(h.controller.state, 'TAKEOVER')
        self.assertEqual(set(h.controller.anchor), set(AXES5))

    def test_no_mit_to_unenabled_axis_and_sequential_enable(self):
        h = Harness()
        h.tick()
        self.assertEqual([f[13] for f in h.wire.frames], [1])
        preload = decode_mit_control_data(h.profiles[1], h.wire.frames[0][21:29])
        self.assertEqual((preload.kp, preload.kd), (0., 0.))
        h.hold()
        self.assertEqual(h.controller.state, 'HOLD')
        enabled = [fields['joint'] for event, fields in h.events
                   if event == 'FIVE_AXIS_ENABLED']
        self.assertEqual(enabled, [1, 2, 3, 4, 5])
        self.assertEqual(sum(event == 'FIVE_AXIS_ENABLE_SENT' for event, _ in h.events), 5)
        self.assertEqual(h.fd_frames(), [])
        last = h.mit_frames()[-1][1]
        self.assertGreater(last.kp, 31.8)
        self.assertGreater(last.kd, 1.99)

    def test_all_five_axes_get_mit_after_hold(self):
        h = Harness()
        h.hold()
        cids = {cid for cid, _ in h.mit_frames()}
        self.assertEqual(cids, set(AXES5))
        for i in AXES5:
            self.assertTrue(any(cid == i for cid, _ in h.mit_frames(i)))

    def test_single_waypoint_trajectory_reaches_hold(self):
        h = Harness()
        h.hold()
        target = math.radians(3.)
        h.controller.trajectory(((target,)*5,), 5., h.now)
        self.assertEqual(h.controller.state, 'TRACK')
        for _ in range(520):
            if h.tick() == 'HOLD':
                break
        self.assertEqual(h.controller.state, 'HOLD')
        for i in AXES5:
            self.assertAlmostEqual(h.controller.targets[i], target, places=9)
            self.assertAlmostEqual(h.q[i], target, delta=1e-3)
        self.assertTrue(any(event == 'FIVE_AXIS_TRAJECTORY_COMPLETE'
                            for event, _ in h.events))

    def test_multi_waypoint_trajectory_and_speed_rejection(self):
        h = Harness()
        h.hold()
        points = ((math.radians(1.5),)*5, (math.radians(3.),)*5)
        h.controller.trajectory(points, 5., h.now)
        for _ in range(520):
            if h.tick() == 'HOLD':
                break
        self.assertEqual(h.controller.state, 'HOLD')
        with self.assertRaises(ValueError):
            h.controller.trajectory(((math.radians(8.),)*5,), 5., h.now)

    def test_velocity_mode_has_kp_zero_and_full_kd(self):
        h = Harness(velocity_ack=True)
        h.hold()
        h.controller.velocity((.01,)*5, 1., h.now)
        start = len(h.wire.frames)
        for _ in range(110):
            if h.tick() == 'HOLD':
                break
        frames = [decode_mit_control_data(h.profiles[f[13]], f[21:29])
                  for f in h.wire.frames[start:] if f[13] in h.profiles
                  and f[21:28] != b'\xff'*7]
        self.assertTrue(frames)
        velocity_frames = frames[:5]
        self.assertTrue(all(abs(p.kp) < 1e-9 for p in velocity_frames))
        self.assertTrue(all(p.kd > 1.99 for p in velocity_frames))
        self.assertTrue(all(abs(abs(p.dq)-.01) < .001 for p in velocity_frames))
        self.assertEqual(h.controller.state, 'HOLD')
        for i in AXES5:
            self.assertAlmostEqual(h.controller.targets[i], h.q[i], places=6)

    def test_velocity_without_ticket_is_rejected(self):
        h = Harness(velocity_ack=False)
        h.hold()
        with self.assertRaises(ValueError):
            h.controller.velocity((.01,)*5, 1., h.now)

    def test_feedback_speed_fault_stops_all_five(self):
        h = Harness()
        h.hold()
        before = len(h.wire.frames)
        h.dq[3] = .201
        state = h.tick()
        self.assertEqual(state, 'STOPPING')
        new = h.wire.frames[before:]
        self.assertEqual([f[13] for f in new], [1, 2, 3, 4, 5])
        self.assertTrue(all(f[21:29] == b'\xff'*7+b'\xfd' for f in new))
        self.assertIn('feedback speed', h.controller.reason)
        h.tick()
        self.assertEqual(h.controller.state, 'DISABLED')

    def test_following_error_fault(self):
        h = Harness()
        h.hold()
        q = dict(h.q)
        q[2] = math.radians(4.1)
        before = len(h.wire.frames)
        h.tick(q=q)
        self.assertEqual(h.controller.state, 'STOPPING')
        self.assertTrue(all(f[21:29] == b'\xff'*7+b'\xfd'
                            for f in h.wire.frames[before:]))
        self.assertIn('following error', h.controller.reason)

    def test_estimate_guard_fault(self):
        h = Harness()
        h.hold()
        q = dict(h.q)
        q[4] = math.radians(3.9)
        before = len(h.wire.frames)
        h.tick(q=q)
        self.assertEqual(h.controller.state, 'STOPPING')
        new = h.wire.frames[before:]
        fd = [f for f in new if f[21:29] == b'\xff'*7+b'\xfd']
        self.assertEqual([f[13] for f in fd], [1, 2, 3, 4, 5])
        self.assertIn('estimate', h.controller.reason)

    def test_recovery_axis_may_start_outside_envelope(self):
        h = Harness()
        h.envelopes[3] = AxisEnvelope(-math.radians(90), math.radians(90), .02,
                                      2., math.radians(4), 32., 2., 50., .2,
                                      math.radians(.5))
        h.q[3] = math.radians(93.)
        h.controller = FiveAxisController(h.endpoint, h.profiles, h.envelopes,
                                          h.ticket, h.audit, active_axes=AXES5,
                                          recovery_axes=(3,), max_runtime=180.)
        h.controller.start(h.snap(), h.now)
        h.hold()
        self.assertEqual(h.controller.state, 'HOLD')
        self.assertIn(3, h.controller.recovery_axes)

    def test_stop_unknown_without_new_state0(self):
        h = Harness()
        h.hold()
        h.controller.stop('operator', h.now)
        states = {i: 1 for i in AXES5}
        for _ in range(60):
            h.now += .010001
            h.controller.tick(h.snap(states=states, at=h.now), h.now)
        self.assertEqual(h.controller.state, 'UNKNOWN')

    def test_no_parameter_or_register_writes(self):
        h = Harness()
        h.hold()
        for frame in h.wire.frames:
            cid, payload = frame[13], frame[21:29]
            self.assertIn(cid, AXES5)
            if payload[:7] == b'\xff'*7:
                self.assertIn(payload[-1], (0xFC, 0xFD))
            else:
                self.assertEqual(len(payload), 8)


if __name__ == '__main__':
    unittest.main()
