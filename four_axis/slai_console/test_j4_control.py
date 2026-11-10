"""Offline evidence for bounded start/brake shaping, motion classes and replay.

No port, no controller, no motor. The synthetic stick-slip plant is algorithm
evidence, not a hardware guarantee. The real-log replay reads recorded files
only and never writes to them.
"""
from collections import deque
import json
import math
from pathlib import Path
import random
import unittest

from .actuation import PD_TIERS, motion_summary
from .j4_control import (CandidateControl, classify_motion, native_mit_estimate,
                         shape_torque, symmetric_slew)

ROOT = Path(__file__).resolve().parents[1]
TRIAL_2NM = ROOT / 'tmp/live_trial/20261101_155927_e9ad73'
RAW_2NM = ROOT / 'tmp/four_axis_readonly/20261101_155917_340800.jsonl'
RESULT_1NM_SLOW = ROOT / 'tmp/live_trial/20261101_155439_df9c6a.json'
PMAX, VMAX, TMAX = 12.5, 45., 54.
J4_SIGN, J4_ZERO = -1, -0.40951400015259054
RES_DEG = math.degrees(2 * PMAX / 65535)


def minjerk(u):
    u = min(1., max(0., u))
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


class ShaperTests(unittest.TestCase):
    def control(self):
        return PD_TIERS['kp32_2nm_brake_candidate'].control

    def test_release_is_much_faster_than_legacy_symmetric_slew(self):
        control = self.control()
        previous = 1.2
        cycles_old = 0
        value = previous
        while value > 1e-9 and cycles_old < 1000:
            value = symmetric_slew(value, 0., .01, 1.)
            cycles_old += 1
        shaped = shape_torque(requested=0., previous=previous, dt=.01, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        cycles_new = 0
        value = previous
        while value > 1e-9 and cycles_new < 1000:
            shaped = shape_torque(requested=0., previous=value, dt=.01, cap=2.,
                                  control=control, tracking=True, motion_confirmed=True,
                                  model_velocity=0., reference_velocity=0.)
            value = shaped['torque']
            cycles_new += 1
        self.assertEqual(cycles_old, 120)
        self.assertEqual(cycles_new, 12)
        self.assertLessEqual(cycles_new * 10, cycles_old)
        self.assertGreaterEqual(previous - symmetric_slew(previous, 0., .02, 10.), .2 - 1e-12)
        self.assertLessEqual(previous - symmetric_slew(previous, 0., .02, 1.), .02 + 1e-12)
        # A reduction is allowed to move by the fast rate, an increase is not.
        up = shape_torque(requested=2., previous=0., dt=.01, cap=2., control=control,
                          tracking=True, motion_confirmed=True,
                          model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(up['torque'], .01, places=12)

    def test_brake_only_removes_force_and_stays_bounded(self):
        control = self.control()
        shaped = shape_torque(requested=1.21, previous=1.2, dt=.01, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=.2088, reference_velocity=.018)
        self.assertGreater(shaped['brake_nm'], .3)
        self.assertLess(shaped['pre_slew'], 1.21 - .3)
        self.assertLessEqual(abs(shaped['torque']), 2.)
        # Opposite measured motion gets an opposing (positive) correction.
        back = shape_torque(requested=0.5, previous=.5, dt=.01, cap=2.,
                            control=control, tracking=True, motion_confirmed=True,
                            model_velocity=-.2088, reference_velocity=-.018)
        self.assertGreater(back['pre_slew'], .5)
        # Below the deadband the brake does nothing.
        quiet = shape_torque(requested=.4, previous=.4, dt=.01, cap=2.,
                             control=control, tracking=True, motion_confirmed=True,
                             model_velocity=.033, reference_velocity=0.)
        self.assertEqual(quiet['brake_nm'], 0.)

    def test_startup_cap_lifts_only_after_confirmed_motion(self):
        control = PD_TIERS['kp32_1nm_brake_candidate'].control
        capped = shape_torque(requested=1., previous=.4, dt=.01, cap=1.,
                              control=control, tracking=True, motion_confirmed=False,
                              model_velocity=0., reference_velocity=0.)
        self.assertTrue(capped['startup_capped'])
        self.assertLessEqual(capped['torque'], .5 + 1e-12)
        lifted = shape_torque(requested=1., previous=.5, dt=.01, cap=1.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        self.assertFalse(lifted['startup_capped'])
        self.assertLessEqual(lifted['torque'], 1.)

    def test_zero_crossing_does_not_use_fast_release_for_reverse_torque(self):
        control = PD_TIERS['kp32_2nm_brake_candidate'].control  # up=1, down=10
        # Codex counterexample: +0.01 -> -2 in one 10ms cycle.
        shaped = shape_torque(requested=-2., previous=.01, dt=.01, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        # 0.01 takes 1ms to reach zero; the remaining 9ms builds at 1Nm/s.
        self.assertAlmostEqual(shaped['torque'], -.009, places=12)
        self.assertAlmostEqual(shaped['rate_nm_s'], 1., places=12)
        self.assertTrue(shaped['zero_crossed'])
        self.assertTrue(shaped['reverse'])
        self.assertFalse(shaped['release'])
        self.assertGreater(shaped['torque'], -.01)
        self.assertLess(shaped['torque'], 0.)
        # Mirror direction.
        mirror = shape_torque(requested=2., previous=-.01, dt=.01, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(mirror['torque'], .009, places=12)
        self.assertAlmostEqual(mirror['rate_nm_s'], 1., places=12)
        # Exactly zero: the fast release stops at zero, no reverse.
        exact = shape_torque(requested=-2., previous=.1, dt=.01, cap=2.,
                             control=control, tracking=True, motion_confirmed=True,
                             model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(exact['torque'], 0., places=12)
        self.assertEqual(exact['rate_nm_s'], 10.)
        self.assertTrue(exact['zero_crossed'])
        self.assertFalse(exact['reverse'])
        # Shorter than the release time: only the release rate, no reverse.
        short = shape_torque(requested=-2., previous=1., dt=.05, cap=2.,
                             control=control, tracking=True, motion_confirmed=True,
                             model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(short['torque'], .5, places=12)
        self.assertEqual(short['rate_nm_s'], 10.)
        self.assertFalse(short['reverse'])
        # Longer than the release time: reverse magnitude is up-limited.
        long = shape_torque(requested=-2., previous=1., dt=.2, cap=2.,
                            control=control, tracking=True, motion_confirmed=True,
                            model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(long['torque'], -.1, places=12)
        self.assertEqual(long['rate_nm_s'], 1.)
        self.assertTrue(long['reverse'])
        # Same-side growth still uses the slow build-up rate; same-side shrink
        # uses the fast release rate.
        grow = shape_torque(requested=2., previous=.1, dt=.05, cap=2.,
                            control=control, tracking=True, motion_confirmed=True,
                            model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(grow['torque'], .15, places=12)
        shrink = shape_torque(requested=0., previous=1., dt=.05, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        self.assertAlmostEqual(shrink['torque'], .5, places=12)

    def test_startup_cap_bounds_brake_composite_in_both_directions(self):
        control = PD_TIERS['kp32_2nm_brake_candidate'].control  # startup cap 1.0
        # Codex counterexample: -1 demand, +0.19 measured velocity, brake would
        # push the composite to -1.28 during the startup phase.
        shaped = shape_torque(requested=-1., previous=-1., dt=.04, cap=2.,
                              control=control, tracking=True, motion_confirmed=False,
                              model_velocity=.19, reference_velocity=0.)
        self.assertTrue(shaped['startup_capped'])
        self.assertAlmostEqual(shaped['limit_nm'], 1., places=12)
        self.assertLessEqual(abs(shaped['pre_slew']), 1. + 1e-12)
        self.assertLessEqual(abs(shaped['torque']), 1. + 1e-12)
        self.assertGreater(shaped['brake_nm'], .2)
        # Opposite direction with opposite measured velocity.
        mirror = shape_torque(requested=1., previous=1., dt=.04, cap=2.,
                              control=control, tracking=True, motion_confirmed=False,
                              model_velocity=-.19, reference_velocity=0.)
        self.assertTrue(mirror['startup_capped'])
        self.assertLessEqual(abs(mirror['pre_slew']), 1. + 1e-12)
        self.assertLessEqual(abs(mirror['torque']), 1. + 1e-12)
        # Position error and measured velocity opposite: still bounded.
        opposite = shape_torque(requested=-1., previous=-1., dt=.04, cap=2.,
                                control=control, tracking=True, motion_confirmed=False,
                                model_velocity=-.19, reference_velocity=0.)
        self.assertLessEqual(abs(opposite['pre_slew']), 1. + 1e-12)
        self.assertLessEqual(abs(opposite['torque']), 1. + 1e-12)
        # After confirmation the tier cap is available again, not more.
        lifted = shape_torque(requested=-1.5, previous=-1., dt=.04, cap=2.,
                              control=control, tracking=True, motion_confirmed=True,
                              model_velocity=0., reference_velocity=0.)
        self.assertFalse(lifted['startup_capped'])
        self.assertAlmostEqual(lifted['limit_nm'], 2., places=12)
        self.assertLessEqual(abs(lifted['torque']), 2.)

    def test_motion_confirmation_and_deadband_are_bounded(self):
        control = self.control()
        self.assertAlmostEqual(control.startup_confirmation_rad(PMAX),
                               3 * 2 * PMAX / 65535, places=12)
        with self.assertRaises(ValueError):
            CandidateControl(1., .5, 1., 1., 3, 2., .4, .05)  # up > down
        with self.assertRaises(ValueError):
            CandidateControl(1., 10., 1., 0., 3, 2., .4, .05)  # zero window
        with self.assertRaises(ValueError):
            CandidateControl(1., 10., 1., 1., 0, 2., .4, .05)  # zero confirm
        with self.assertRaises(ValueError):
            CandidateControl(1., 10., 1., 1., 3, 2., .4, .2)  # deadband too high


class MotionClassTests(unittest.TestCase):
    def test_jitter_start_and_tracking_are_distinct(self):
        self.assertEqual(classify_motion([], RES_DEG), 'NO_CODE_DATA')
        self.assertEqual(classify_motion([31694], RES_DEG), 'NO_CODE_CHANGE')
        self.assertEqual(classify_motion([31694, 31695, 31694], RES_DEG), 'CODE_JITTER')
        self.assertEqual(classify_motion([31694, 31695, 31696], RES_DEG), 'CODE_JITTER')
        self.assertEqual(classify_motion([31705, 31706, 31707, 31708], RES_DEG),
                         'OBSERVABLE_START')
        self.assertEqual(classify_motion([31691, 31689, 31683], RES_DEG),
                         'OBSERVABLE_START')
        self.assertEqual(classify_motion([31700, 31650], RES_DEG), 'PARTIAL_TRACKING')
        self.assertEqual(classify_motion([31900, 31600], RES_DEG), 'EFFECTIVE_TRACKING')

    def test_small_code_change_is_not_effective_motion(self):
        samples = [
            dict(at=0., state=1, angle_deg=0., error_deg=0., position_code=31693,
                 output=dict(requested_nm=1., encoded_nm=.99, at_encoded_cap=True,
                             request_clipped=False)),
            dict(at=.01, state=1, angle_deg=.0219, error_deg=.5, position_code=31694,
                 output=dict(requested_nm=1., encoded_nm=.99, at_encoded_cap=True,
                             request_clipped=False)),
            dict(at=.02, state=1, angle_deg=.0437, error_deg=2., position_code=31695,
                 output=dict(requested_nm=1., encoded_nm=.99, at_encoded_cap=True,
                             request_clipped=False)),
        ]
        metrics = motion_summary(samples, cap_nm=1., encoded_cap_nm=.989,
                                 encoder_resolution_deg=RES_DEG, motion_required=True,
                                 following_error_deg=2.)
        self.assertEqual(metrics['motion_class'], 'CODE_JITTER')
        self.assertFalse(metrics['moved'])
        self.assertEqual(metrics['assessment'], 'CODE_JITTER_NO_EFFECTIVE_MOTION')


class NativeMitTests(unittest.TestCase):
    def test_native_total_can_exceed_a_two_nm_host_bound(self):
        estimate = native_mit_estimate(kp=32., kd=.5, q_des=math.radians(4.),
                                       dq_des=0., q=0., dq=-.2, tau_ff=0.,
                                       tmax=54., command_cap=2.)
        self.assertGreater(estimate['total_nm'], 2.)
        self.assertTrue(estimate['ff_alone_within_command_cap'])
        self.assertFalse(estimate['total_within_command_cap'])
        self.assertFalse(estimate['host_can_bound_total_from_ff_field'])
        self.assertIn('does not bound', estimate['note'])
        # A small error stays inside the 2Nm command bound; the bound is state
        # dependent, which is exactly why the host cannot claim it from FF alone.
        small = native_mit_estimate(kp=32., kd=.5, q_des=math.radians(1.),
                                    dq_des=0., q=0., dq=0., tau_ff=0.,
                                    tmax=54., command_cap=2.)
        self.assertTrue(small['total_within_command_cap'])


class RealLogReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not TRIAL_2NM.with_suffix('.json').exists() or not RAW_2NM.exists():
            raise unittest.SkipTest('recorded 2Nm run is not present in this workspace')
        cls.rows = [json.loads(line) for line in
                    TRIAL_2NM.with_suffix('.jsonl').read_text(encoding='utf-8').splitlines()]
        cls.result = json.loads(TRIAL_2NM.with_suffix('.json').read_text(encoding='utf-8'))
        cls.stop = next(r['unix_s'] for r in cls.rows
                        if r['kind'] == 'GLOBAL_STOP_REQUESTED')
        cls.commands = [r['payload'] for r in cls.rows if r['kind'] == 'SERIAL_WRITTEN'
                        and r.get('frame_kind') == 'mit' and r.get('joint') == 4]

    def decode_command(self, payload):
        data = bytes.fromhex(payload)
        raw = ((data[6] & 15) << 8) | data[7]
        return raw * 2 * TMAX / 4095 - TMAX

    def raw_frames(self, lo, hi):
        frames = []
        for line in RAW_2NM.read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            if row['direction'] != 'rx' or not (lo <= row['time'] <= hi):
                continue
            data = bytes.fromhex(row['payload'])
            if len(data) != 8 or (data[1] == 0 and data[2] == 0x33):
                continue
            if (data[0] & 15) != 4:
                continue
            frames.append(dict(time=row['time'], state=data[0] >> 4,
                               code=(data[1] << 8) | data[2],
                               vel=((data[3] << 4) | (data[4] >> 4)) * 90 / 4095 - 45.,
                               tau=(((data[4] & 15) << 8) | data[5]) * 108 / 4095 - 54.))
        return frames

    def test_recorded_final_state_and_post_stop_frames(self):
        last = self.decode_command(self.commands[-1])
        self.assertAlmostEqual(abs(last), 1.2, places=6)
        frames = self.raw_frames(self.stop - 1.5, self.stop + .3)
        pre = [f for f in frames if f['time'] <= self.stop]
        post = [f for f in frames if f['time'] > self.stop and f['state'] == 0]
        self.assertTrue(pre, 'no pre-stop J4 feedback in the recorded window')
        self.assertTrue(post, 'no post-stop state-0 feedback in the recorded window')
        self.assertLessEqual(abs(pre[-1]['vel']), .21)
        self.assertGreaterEqual(abs(pre[-1]['vel']), .15)
        self.assertGreaterEqual(abs(post[0]['vel']), .4)
        self.assertLessEqual(abs(post[0]['code'] - pre[-1]['code']), 20)
        self.assertGreater(abs(post[0]['code'] - pre[-1]['code']), 6)
        self.assertTrue(self.result['reason'].startswith('J4: feedback speed -0.208791'))

    def test_old_symmetric_release_delay_replayed_from_recorded_state(self):
        control = PD_TIERS['kp32_2nm_brake_candidate'].control
        previous = 1.2
        old = 0
        value = previous
        while value > 1e-9 and old < 1000:
            value = symmetric_slew(value, 0., .01, 1.)
            old += 1
        newcycles = 0
        value = previous
        while value > 1e-9 and newcycles < 1000:
            value = shape_torque(requested=0., previous=value, dt=.01, cap=2.,
                                 control=control, tracking=True, motion_confirmed=True,
                                 model_velocity=.2088, reference_velocity=0.)['torque']
            newcycles += 1
        self.assertEqual(old, 120)
        self.assertEqual(newcycles, 12)
        self.assertGreater(old * .01, 1.)
        self.assertLess(newcycles * .01, .2)

    def test_recorded_0044deg_run_is_jitter_not_effective_motion(self):
        result = json.loads(RESULT_1NM_SLOW.read_text(encoding='utf-8'))
        metrics = result['motion_metrics']
        self.assertLess(metrics['total_span_deg'], .05)
        self.assertEqual(metrics['position_code_changes'], 2)
        self.assertEqual(classify_motion(metrics['position_codes'], RES_DEG),
                         'CODE_JITTER')


class StickSlipPlant:
    """Quantized, delayed, noisy stick-slip joint; algorithm evidence only."""

    def __init__(self, *, fs, fk, J=.05, b=.05, delay=2, seed=7):
        self.q = 0.
        self.v = 0.
        self.fs, self.fk, self.J, self.b = fs, fk, J, b
        self.delay = delay
        self.buf = deque([(0., 0.)] * max(1, delay), maxlen=max(1, delay))
        self.rng = random.Random(seed)

    def step(self, torque, dt):
        if abs(self.v) < 1e-3 and abs(torque) <= self.fs:
            self.v = 0.
            return
        if abs(self.v) < 1e-9:
            self.v = math.copysign(1e-6, torque)
        accel = (torque - self.b * self.v - math.copysign(self.fk, self.v)) / self.J
        v_new = self.v + accel * dt
        if self.v * v_new < 0 and abs(torque) <= self.fs:
            v_new = 0.  # friction stops the joint; it does not reverse it
        self.v = v_new
        self.q += self.v * dt

    def feedback(self):
        enc = J4_ZERO + J4_SIGN * self.q
        code = round((enc + PMAX) * 65535 / (2 * PMAX))
        q_fb = J4_SIGN * (code * 2 * PMAX / 65535 - PMAX - J4_ZERO)
        vcode = round((J4_SIGN * self.v + VMAX) * 4095 / (2 * VMAX))
        if self.rng.random() < .05:
            vcode += self.rng.choice((-1, 1))
        vcode = max(0, min(4095, vcode))
        v_model = J4_SIGN * (vcode * 2 * VMAX / 4095 - VMAX)
        self.buf.append((q_fb, v_model))
        return self.buf[0]


def simulate(control, *, fs, fk, legacy_slew=None, duration=18., cap=2., kp=32., kd=.5,
             abort=.2, following_deg=4., dt=.01, J=.05, b=.05):
    plant = StickSlipPlant(fs=fs, fk=fk, J=J, b=b)
    target = 0.
    previous = 0.
    motion_confirmed = False
    peak = 0.
    stopped = None
    q_used = v_used = 0.
    window = (control.startup_window_s if control else None)
    confirm = (control.startup_confirmation_rad(PMAX) if control else None)
    history = []
    for k in range(int(duration / dt) + 1):
        t = k * dt
        u = min(1., t / duration)
        target = math.radians(10) * minjerk(u)
        v_ref = math.radians(10) * 30 * u * u * (1 - u) ** 2 / duration
        error = target - q_used
        requested = max(-cap, min(cap, kp * error + kd * (v_ref - v_used)))
        if control is not None:
            if not motion_confirmed and q_used >= confirm - 1e-12:
                motion_confirmed = True
            if not motion_confirmed and t >= window:
                stopped = 'startup'
                break
            shaped = shape_torque(requested=requested, previous=previous, dt=dt, cap=cap,
                                  control=control, tracking=True,
                                  motion_confirmed=motion_confirmed,
                                  model_velocity=v_used,
                                  reference_velocity=v_ref)
            torque = shaped['torque']
        else:
            shaped = None
            torque = symmetric_slew(previous, requested, dt, legacy_slew)
        previous = torque
        plant.step(torque, dt)
        q_used, v_used = plant.feedback()
        history.append(dict(t=t, torque=torque, q_used=q_used, v_used=v_used,
                            requested=requested, error=math.degrees(error),
                            confirmed=motion_confirmed,
                            brake=(shaped['brake_nm'] if shaped else 0.),
                            startup=(shaped['startup_capped'] if shaped else False)))
        peak = max(peak, abs(v_used))
        if abs(v_used) > abort:
            stopped = 'speed'
            break
        if abs(math.degrees(target - q_used)) > following_deg:
            stopped = 'error'
            break
    travel = math.degrees(plant.q)
    # FD: zero torque, friction/inertia only.
    for _ in range(100):
        plant.step(0., dt)
        if abs(plant.v) < 1e-4:
            break
    return dict(stop=stopped, travel_deg=travel, peak_speed=peak,
                coast_deg=math.degrees(plant.q) - travel,
                peak_after=peak, history=history)


class PlantTests(unittest.TestCase):
    def test_high_stiction_new_profile_stops_before_a_high_torque_release(self):
        old = simulate(None, fs=1.2, fk=.8, legacy_slew=1., J=.15)
        new = simulate(PD_TIERS['kp32_2nm_brake_candidate'].control, fs=1.2, fk=.8, J=.15)
        self.assertEqual(old['stop'], 'speed')
        self.assertGreater(old['peak_speed'], .2)
        self.assertGreaterEqual(old['travel_deg'], .05)
        self.assertEqual(new['stop'], 'startup')
        self.assertLess(new['peak_speed'], .05)
        self.assertLess(new['travel_deg'], .05)
        self.assertLess(abs(new['coast_deg']), .05)

    def test_medium_stiction_new_brake_tracks_where_old_aborts(self):
        old = simulate(None, fs=.9, fk=.5, legacy_slew=1., J=.15)
        new = simulate(PD_TIERS['kp32_2nm_brake_candidate'].control, fs=.9, fk=.5, J=.15)
        self.assertEqual(old['stop'], 'speed')
        self.assertLess(old['travel_deg'], 1.)
        self.assertNotEqual(new['stop'], 'speed')
        self.assertGreater(new['travel_deg'], 5.)
        self.assertLess(new['peak_speed'], .2)


if __name__ == '__main__':
    unittest.main()
