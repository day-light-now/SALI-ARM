"""Five-axis drive-side MIT supervisor (OpenArm-style, no host PD).

This is a new, explicitly scoped layer. It does not change the accepted
four-axis host-PD path or Codex's J4-only native bench. It supports J1-J5, but
J5 is only usable after a valid manual calibration record exists; a missing
sign/zero/travel never defaults to a guess.

Safety is deliberately conservative: 0.02 rad/s target speed, 0.20 rad/s
feedback abort, 4 deg following error, 50 C, 60 ms feedback age, 40 ms host
gap, per-axis observed-state request estimate guard (NOT a continuous drive
torque clamp), and FD on every active axis for a fault.
"""
from collections import deque
from dataclasses import dataclass
import math

from .core import minjerk
from .openarm_mit import (MITParam, model_command, decode_mit_control_data,
                         estimate_drive_request)

AXES5 = (1, 2, 3, 4, 5)


def _finite(value, name):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


@dataclass(frozen=True)
class AxisProfile:
    """One physical axis, values read from the device/config, never guessed."""
    joint: int
    mode: int
    master_id: int
    timeout: int
    pmax: float
    vmax: float
    tmax: float
    sign: int
    zero: float

    def __post_init__(self):
        if type(self.joint) is not int or self.joint not in AXES5:
            raise ValueError('axis must be 1..5')
        if self.mode != 1:
            raise ValueError('native MIT requires CTRL_MODE=1')
        if type(self.master_id) is not int or not 0 <= self.master_id <= 0x7FF:
            raise ValueError('master_id must be a CAN id')
        if self.timeout != 2000:
            raise ValueError('only the verified TIMEOUT2000 setup is accepted')
        for value, name in ((self.pmax, 'pmax'), (self.vmax, 'vmax'), (self.tmax, 'tmax')):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must come from a device reply')
        if self.sign not in (-1, 1):
            raise ValueError('hardware sign must be +1/-1; no default')
        if not math.isfinite(self.zero):
            raise ValueError('software zero must be finite; no default')

    def model_q(self, raw):
        return self.sign*(raw-self.zero)

    def model_dq(self, raw):
        return self.sign*raw

    def raw_q(self, model):
        return self.zero+self.sign*model

    def raw_dq(self, model):
        return self.sign*model


@dataclass(frozen=True)
class AxisObservation:
    profile: AxisProfile
    position: float
    velocity: float
    torque: float
    state: int
    received_at: float
    driver_temp: float
    motor_temp: float
    payload: str

    @property
    def q(self):
        return self.profile.model_q(self.position)

    @property
    def dq(self):
        return self.profile.model_dq(self.velocity)

    def __post_init__(self):
        for value, name in ((self.position, 'position'), (self.velocity, 'velocity'),
                            (self.torque, 'torque'), (self.received_at, 'received_at'),
                            (self.driver_temp, 'driver_temp'),
                            (self.motor_temp, 'motor_temp')):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')


@dataclass(frozen=True)
class FiveSnapshot:
    session_id: str
    observed_at: float
    rows: tuple
    mapping_sha: str
    readonly_log: str
    clock_source: str = 'perf_counter'


@dataclass(frozen=True)
class AxisEnvelope:
    lower: float
    upper: float
    speed: float
    estimate_guard_nm: float
    following_error: float
    kp: float
    kd: float
    temperature: float
    feedback_speed: float
    arrival_error: float
    velocity_error: float = .05

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (
                self.lower, self.upper, self.speed, self.estimate_guard_nm,
                self.following_error, self.kp, self.kd, self.temperature,
                self.feedback_speed, self.arrival_error, self.velocity_error)):
            raise ValueError('envelope values must be finite')
        if self.lower >= self.upper:
            raise ValueError('lower must be below upper')
        if not 0 < self.speed <= .02 + 1e-12:
            raise ValueError('target speed must be in (0,0.02]rad/s')
        if not 0 < self.estimate_guard_nm:
            raise ValueError('estimate guard must be positive')
        if not 0 < self.feedback_speed <= 1.0 + 1e-12:
            raise ValueError('feedback speed abort must be <=1.0rad/s')
        if not 0 < self.following_error <= math.radians(4)+1e-12:
            raise ValueError('following error must be <=4deg')
        if not 0 <= self.kp <= 500 or not 0 <= self.kd <= 5:
            raise ValueError('drive gains outside MIT protocol')
        if not 0 < self.temperature <= 50:
            raise ValueError('temperature limit must be <=50C')
        if not 0 < self.arrival_error <= math.radians(1):
            raise ValueError('arrival error must be in (0,1deg]')
        if not 0 < self.velocity_error <= .2:
            raise ValueError('velocity error guard must be in (0,0.2]rad/s')

    def check(self, q, where):
        if not self.lower-1e-12 <= q <= self.upper+1e-12:
            raise ValueError(f'J{where}: {math.degrees(q):.3f}deg outside local travel '
                             f'[{math.degrees(self.lower):.3f},{math.degrees(self.upper):.3f}]')


@dataclass(frozen=True)
class FiveAxisAcceptance:
    support_confirmed: bool
    clearance_confirmed: bool
    native_output_acknowledged: bool
    five_axis_acknowledged: bool
    request_id: str
    output_policy_reference: str
    config_reference: str
    velocity_acknowledged: bool = False
    execution_backend: str = 'drive_mit'

    def validate(self):
        if not all(v is True for v in (self.support_confirmed, self.clearance_confirmed,
                                       self.native_output_acknowledged,
                                       self.five_axis_acknowledged)):
            raise ValueError('Five-axis native MIT needs support/clearance/output/five-axis acknowledgement')
        if not self.request_id or not self.output_policy_reference or not self.config_reference:
            raise ValueError('request id, output policy and config reference are required')
        if self.execution_backend != 'drive_mit':
            raise ValueError('wrong execution backend')

    def validate_scope(self, active_axes, ramp):
        if tuple(active_axes) not in ((1, 2, 3, 4), AXES5):
            raise ValueError('active axes must be J1-J4 or J1-J5')
        if ramp != 1.:
            raise ValueError('first five-axis build requires a 1s gain ramp')


class FiveAxisEndpoint:
    """Exclusive injected serial. Basic IDs 1-5, MIT and FC/FD only."""
    def __init__(self, serial, audit, clock):
        self.serial, self.audit, self.clock = serial, audit, clock
        self.closed = False
        self.receipts = deque(maxlen=6000)

    def send(self, joint, payload, kind, command_id):
        if self.closed or type(joint) is not int or joint not in AXES5:
            raise ValueError('Closed connection or forbidden axis')
        if not isinstance(payload, bytes) or len(payload) != 8:
            raise ValueError('Eight-byte payload required')
        special = payload[:7] == b'\xff'*7
        if kind not in ('mit', 'enable', 'disable'):
            raise ValueError('Forbidden frame kind')
        if kind == 'mit' and special:
            raise ValueError('Reserved MIT special command rejected')
        if kind != 'mit' and payload != b'\xff'*7+bytes([0xFC if kind == 'enable' else 0xFD]):
            raise ValueError('Only explicit enable/disable special commands allowed')
        frame = bytearray([0x55,0xAA,0x1E,3,1,0,0,0,0x0A,0,0,0,0,
                           joint,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0])
        frame[21:29] = payload
        receipt = dict(command_id=command_id, joint=joint, kind=kind,
                       can_id=joint, payload=payload.hex(' '),
                       accepted_at=self.clock(), status='accepted')
        try:
            self.audit('CONTROL_ACCEPTED', **receipt)
        except Exception:
            if kind != 'disable':
                raise
        self.receipts.append(receipt)
        try:
            receipt['write_started_at'] = self.clock()
            count = self.serial.write(bytes(frame))
            if count != len(frame):
                raise IOError('Partial USB write; delivery unknown')
        except Exception:
            receipt.update(status='delivery_unknown', finished_at=self.clock())
            raise
        receipt.update(status='serial_written_not_drive_ack', finished_at=self.clock())
        try:
            self.audit('SERIAL_WRITTEN', **receipt)
        except Exception:
            if kind != 'disable':
                raise
        return receipt


def _minjerk_velocity(t):
    """Derivative of core.minjerk with respect to normalized t."""
    return 30.*t*t*(1.-t)*(1.-t)


class FiveAxisController:
    """Manual 100Hz supervisor for five native-MIT axes."""
    execution_backend = 'drive_mit'

    def __init__(self, endpoint, profiles, envelopes, acceptance, audit, *,
                 active_axes=AXES5, period=.01, max_age=.06, max_gap=.04,
                 ramp=1., ack_timeout=.5, max_runtime=60., hold_duration=2.,
                 velocity_duration_max=5., recovery_axes=()):
        if not isinstance(acceptance, FiveAxisAcceptance):
            raise ValueError('Host-PD tickets cannot authorize five-axis native MIT')
        acceptance.validate()
        self.endpoint, self.audit = endpoint, audit
        self.profiles = {i: profiles[i] for i in profiles}
        self.envelopes = {i: envelopes[i] for i in envelopes}
        self.acceptance = acceptance
        self.active_axes = tuple(active_axes)
        if self.active_axes not in ((1, 2, 3, 4), AXES5):
            raise ValueError('active axes must be J1-J4 or J1-J5')
        self.recovery_axes = frozenset(recovery_axes)
        if not self.recovery_axes <= set(self.active_axes):
            raise ValueError('recovery axis must be an active axis')
        if set(self.active_axes) - set(self.profiles) or set(self.active_axes) - set(self.envelopes):
            raise ValueError('profile/envelope missing for an active axis')
        if set(self.profiles) != set(self.envelopes):
            raise ValueError('profiles and envelopes must cover the same axes')
        if len(self.profiles) != len(self.active_axes):
            raise ValueError('no unused axis profiles in a five-axis session')
        for i in self.active_axes:
            if self.profiles[i].joint != i:
                raise ValueError('profile joint mismatch')
            if self.profiles[i].sign not in (-1, 1):
                raise ValueError('uncalibrated axis cannot be active')
        acceptance.validate_scope(self.active_axes, ramp)
        self.period, self.max_age, self.max_gap = period, max_age, max_gap
        self.ramp, self.ack_timeout = ramp, ack_timeout
        self.max_runtime = max_runtime
        self.hold_duration = hold_duration
        self.velocity_duration_max = velocity_duration_max
        self.state = 'IDLE'
        self.reason = ''
        self.anchor, self.targets, self.output = {}, {}, {}
        self.enabled, self.preloaded, self.fc_sent = set(), set(), set()
        self.ramp_at, self.fc_at = {}, {}
        self.plan = None
        self.plan_at = None
        self.velocity_at = None
        self.velocity_targets = {}
        self.stop_time = None
        self.started_at = None
        self.last_tick = None
        self.samples = 0

    # ---- snapshot validation -------------------------------------------------
    def _rows(self, snapshot):
        if not isinstance(snapshot, FiveSnapshot):
            raise ValueError('FiveSnapshot required')
        rows = tuple(snapshot.rows)
        if len(rows) != len(self.active_axes):
            raise ValueError('snapshot must contain exactly the active axes')
        if tuple(r.profile.joint for r in rows) != self.active_axes:
            raise ValueError('snapshot axis order/identity mismatch')
        for row in rows:
            if not isinstance(row, AxisObservation):
                raise ValueError('AxisObservation required')
            i = row.profile.joint
            if row.profile != self.profiles[i]:
                raise ValueError(f'J{i}: device parameters changed; reconnect')
        return rows

    def _validate(self, snapshot, now, initial=False):
        if not math.isfinite(now):
            raise ValueError('non-finite clock')
        rows = self._rows(snapshot)
        if self.last_tick is not None:
            gap = now-self.last_tick
            if gap < 0:
                raise ValueError('clock went backwards')
            if not initial and gap > self.max_gap+1e-9:
                raise ValueError(f'host gap {gap:.3f}s exceeds {self.max_gap:.3f}s')
        for row in rows:
            i = row.profile.joint
            env = self.envelopes[i]
            age = now-row.received_at
            if age < -1e-9 or age > self.max_age+1e-9:
                raise ValueError(f'J{i}: feedback age {age:.4f}s outside 0..{self.max_age:.3f}s')
            if not -1e-9 <= row.driver_temp <= env.temperature+1e-9:
                raise ValueError(f'J{i}: driver temperature {row.driver_temp:.1f}C')
            if not -1e-9 <= row.motor_temp <= env.temperature+1e-9:
                raise ValueError(f'J{i}: motor temperature {row.motor_temp:.1f}C')
            if abs(row.dq) > env.feedback_speed+1e-9:
                raise ValueError(f'J{i}: feedback speed {row.dq:+.4f}rad/s exceeds '
                                 f'{env.feedback_speed:.3f}')
            if abs(row.torque) > 2*env.estimate_guard_nm+1e-9:
                raise ValueError(f'J{i}: feedback torque {row.torque:+.3f}Nm exceeds '
                                 f'2x estimate guard')
            if initial:
                if row.state != 0:
                    raise ValueError(f'J{i}: state {row.state} before takeover; expected disabled')
            elif i in self.enabled:
                if row.state != 1:
                    raise ValueError(f'J{i}: enabled axis reports state {row.state}')
            elif i in self.fc_sent:
                if row.state not in (0, 1):
                    raise ValueError(f'J{i}: enable-pending state {row.state}')
            elif row.state != 0:
                raise ValueError(f'J{i}: unexpected enabled state {row.state}')
            if i not in self.recovery_axes:
                env.check(row.q, i)
            if self.state in ('HOLD', 'TRACK') and i in self.enabled:
                target = self._target_for_check(i, now)
                if target is not None and abs(target-row.q) > env.following_error+1e-9:
                    raise ValueError(f'J{i}: following error {math.degrees(target-row.q):+.3f}deg '
                                     f'exceeds {math.degrees(env.following_error):.3f}deg')
            if self.state == 'VELOCITY' and i in self.enabled:
                target = self.velocity_targets.get(i)
                if target is not None and abs(target-row.dq) > env.velocity_error+1e-9:
                    raise ValueError(f'J{i}: velocity error {target-row.dq:+.4f}rad/s exceeds '
                                     f'{env.velocity_error:.3f}')

    def _target_for_check(self, i, now):
        if self.state == 'TRACK' and self.plan is not None:
            q, _ = self._plan_target(now)
            return q[i]
        return self.targets.get(i)

    # ---- start / tick --------------------------------------------------------
    def start(self, snapshot, now):
        if self.state != 'IDLE':
            raise ValueError('controller already started')
        self._validate(snapshot, now, initial=True)
        self._rows_now = {r.profile.joint: r for r in snapshot.rows}
        for row in snapshot.rows:
            i = row.profile.joint
            self.anchor[i] = row.q
            self.targets[i] = row.q
        self.state = 'TAKEOVER'
        self.started_at = now
        self.last_tick = now
        self.audit('FIVE_AXIS_TAKEOVER', request_id=self.acceptance.request_id,
                   axes=list(self.active_axes), backend=self.execution_backend,
                   drive_kp={i: self.envelopes[i].kp for i in self.active_axes},
                   drive_kd={i: self.envelopes[i].kd for i in self.active_axes},
                   estimate_guard_nm={i: self.envelopes[i].estimate_guard_nm
                                      for i in self.active_axes},
                   host_total_torque_clamp=False,
                   output_policy_reference=self.acceptance.output_policy_reference,
                   config_reference=self.acceptance.config_reference)
        return self.state

    def tick(self, snapshot, now):
        if self.state in ('IDLE', 'DISABLED', 'UNKNOWN'):
            return self.state
        if self.state == 'STOPPING':
            self._observe_stop(snapshot, now)
            return self.state
        if self.last_tick is not None and 0 <= now-self.last_tick < self.period:
            return self.state
        self.samples += 1
        try:
            self._validate(snapshot, now)
            self._rows_now = {r.profile.joint: r for r in snapshot.rows}
            self.last_tick = now
            if self.state == 'TAKEOVER':
                self._takeover_step(snapshot, now)
            elif self.state == 'HOLD':
                self._hold_step(snapshot, now)
            elif self.state == 'TRACK':
                self._track_step(snapshot, now)
            elif self.state == 'VELOCITY':
                self._velocity_step(snapshot, now)
            else:
                raise ValueError(f'unknown state {self.state}')
        except Exception as exc:
            self._fail(str(exc), now)
        return self.state

    def _takeover_step(self, snapshot, now):
        rows = {r.profile.joint: r for r in snapshot.rows}
        if len(self.enabled) == len(self.active_axes):
            for i in self.active_axes:
                self._command(i, self.anchor[i], 0., 1., now, 'hold')
            if all(now-self.ramp_at[i] >= self.ramp-1e-12 for i in self.active_axes):
                self.targets = dict(self.anchor)
                self.state = 'HOLD'
                self.audit('FIVE_AXIS_HOLD_OBSERVED', axes=list(self.active_axes),
                           targets={i: math.degrees(self.anchor[i]) for i in self.active_axes})
            return
        # Keep every already-enabled axis fed while the next one is enabled.
        # Otherwise a four-axis sequential takeover can leave J1 without an MIT
        # frame (and therefore without new feedback) for >60ms.
        for i in self.enabled:
            scale = min(1., max(0., (now-self.ramp_at[i])/self.ramp))
            self._command(i, self.anchor[i], 0., scale, now, 'ramp')
        for i in self.active_axes:
            if i not in self.enabled:
                if i not in self.preloaded:
                    payload = model_command(self.profiles[i], q=self.anchor[i],
                                            dq=0., kp=0., kd=0.)
                    self.endpoint.send(i, payload, 'mit', 'preload')
                    self.preloaded.add(i)
                    self.audit('FIVE_AXIS_PRELOAD', joint=i,
                               q_deg=math.degrees(self.anchor[i]))
                    return
                if i not in self.fc_sent:
                    self.endpoint.send(i, b'\xff'*7+b'\xfc', 'enable', 'enable')
                    self.fc_sent.add(i)
                    self.fc_at[i] = now
                    self.audit('FIVE_AXIS_ENABLE_SENT', joint=i)
                    return
                if rows[i].state == 1:
                    self.enabled.add(i)
                    self.ramp_at[i] = now
                    self.audit('FIVE_AXIS_ENABLED', joint=i)
                    self._command(i, self.anchor[i], 0., 0., now, 'ramp')
                    return
                if now-self.fc_at[i] > self.ack_timeout+1e-9:
                    raise ValueError(f'J{i}: enable acknowledgement timeout')
                return

    def _hold_step(self, snapshot, now):
        for i in self.active_axes:
            self._command(i, self.targets[i], 0., 1., now, 'hold')

    def _track_step(self, snapshot, now):
        if self.plan is None:
            raise ValueError('track state without plan')
        elapsed = now-self.plan_at
        if elapsed >= self.plan['duration']-1e-12:
            rows = {r.profile.joint: r for r in snapshot.rows}
            for i in self.active_axes:
                final = self.plan['points'][-1][i]
                if abs(final-rows[i].q) > self.envelopes[i].arrival_error+1e-9:
                    raise ValueError(f'J{i}: arrival error {math.degrees(final-rows[i].q):+.3f}deg')
            self.targets = dict(self.plan['points'][-1])
            self.state = 'HOLD'
            self.plan = None
            self.audit('FIVE_AXIS_TRAJECTORY_COMPLETE',
                       targets={i: math.degrees(self.targets[i]) for i in self.active_axes})
            for i in self.active_axes:
                self._command(i, self.targets[i], 0., 1., now, 'hold')
            return
        q, dq = self._plan_target(now)
        for i in self.active_axes:
            self._command(i, q[i], dq[i], 1., now, 'track')

    def _velocity_step(self, snapshot, now):
        rows = {r.profile.joint: r for r in snapshot.rows}
        if now-self.velocity_at >= self.plan['duration']-1e-12:
            self.targets = {i: rows[i].q for i in self.active_axes}
            self.state = 'HOLD'
            self.plan = None
            self.audit('FIVE_AXIS_VELOCITY_COMPLETE',
                       targets={i: math.degrees(self.targets[i]) for i in self.active_axes})
            for i in self.active_axes:
                self._command(i, self.targets[i], 0., 1., now, 'hold')
            return
        for i in self.active_axes:
            target_dq = self.velocity_targets[i]
            self._command(i, rows[i].q, target_dq, 1., now, 'velocity',
                          kp_override=0., kd_scale=1.)

    def _plan_target(self, now):
        plan = self.plan
        elapsed = max(0., min(now-plan['plan_at'], plan['duration']))
        seg = min(int(elapsed/plan['seg_duration']), plan['segments']-1)
        local = min(1., max(0., (elapsed-seg*plan['seg_duration'])/plan['seg_duration']))
        s = minjerk(local)
        ds = _minjerk_velocity(local)
        q, dq = {}, {}
        for i in self.active_axes:
            start = plan['start'][i] if seg == 0 else plan['points'][seg-1][i]
            end = plan['points'][seg][i]
            delta = end-start
            q[i] = start+delta*s
            dq[i] = delta*ds/plan['seg_duration']
        return q, dq

    def _command(self, i, q, dq, kp_scale, now, mode, kp_override=None, kd_scale=None):
        profile = self.profiles[i]
        env = self.envelopes[i]
        kp = env.kp if kp_override is None else kp_override
        kd_scale = kp_scale if kd_scale is None else kd_scale
        payload = model_command(profile, q=q, dq=dq, kp=kp_scale*kp,
                                kd=kd_scale*env.kd, tau_ff=0.)
        estimated = estimate_drive_request(profile, payload,
                                           position=self._last_position(i),
                                           velocity=self._last_velocity(i))
        if abs(estimated) > env.estimate_guard_nm+1e-9:
            raise ValueError(f'J{i}: observed-state request estimate {estimated:+.3f}Nm '
                             f'exceeds {env.estimate_guard_nm:.3f}Nm')
        decoded = decode_mit_control_data(profile, payload)
        self.endpoint.send(i, payload, 'mit', mode)
        self.output[i] = dict(at=now, mode=mode, target=q, position=self._last_q(i),
                              error=q-self._last_q(i), drive_kp=decoded.kp,
                              drive_kd=decoded.kd, drive_q=decoded.q, drive_dq=decoded.dq,
                              encoded_tau_ff_nm=decoded.tau, estimate_nm=estimated,
                              ramp_scale=kp_scale, host_total_torque_clamp=False)

    def _last_q(self, i):
        return self._last_row(i).q

    def _last_position(self, i):
        return self._last_row(i).position

    def _last_velocity(self, i):
        return self._last_row(i).velocity

    def _last_row(self, i):
        if not hasattr(self, '_rows_now'):
            raise ValueError('no current observation')
        return self._rows_now[i]

    # ---- plans ---------------------------------------------------------------
    def trajectory(self, points, duration, now):
        if self.state != 'HOLD':
            raise ValueError('trajectory requires a stable hold')
        if not 1. <= duration <= self.max_runtime:
            raise ValueError(f'duration must be 1..{self.max_runtime:.0f}s')
        if not points or len(points) > 32:
            raise ValueError('1..32 waypoints required')
        rows = []
        for point in points:
            if len(point) != len(self.active_axes):
                raise ValueError('each waypoint must have one value per active axis')
            row = {}
            for i, value in zip(self.active_axes, point):
                value = _finite(value, f'J{i} waypoint')
                self.envelopes[i].check(value, i)
                row[i] = value
            rows.append(row)
        seg_count = len(rows)
        seg_duration = duration/seg_count
        previous = self.targets
        for row in rows:
            for i in self.active_axes:
                delta = abs(row[i]-previous[i])
                peak = 1.875*delta/seg_duration
                if peak > self.envelopes[i].speed+1e-9:
                    minimum = 1.875*delta*seg_count/self.envelopes[i].speed
                    raise ValueError(
                        f'J{i}: segment peak {peak:.4f}rad/s exceeds '
                        f'{self.envelopes[i].speed:.3f}; delta '
                        f'{math.degrees(delta):.2f}deg, minimum total duration '
                        f'{minimum:.2f}s')
            previous = row
        self.plan = dict(points=rows, start=dict(self.targets), duration=duration,
                         segments=seg_count, seg_duration=seg_duration, plan_at=now)
        self.plan_at = now
        self.state = 'TRACK'
        self.audit('FIVE_AXIS_PLAN_ACCEPTED_NOT_SENT', duration_s=duration,
                   waypoints=[[math.degrees(row[i]) for i in self.active_axes] for row in rows])

    def velocity(self, targets, duration, now):
        if self.state != 'HOLD':
            raise ValueError('velocity requires a stable hold')
        if not self.acceptance.velocity_acknowledged:
            raise ValueError('velocity mode needs a separate explicit acknowledgement')
        if not 0 < duration <= self.velocity_duration_max:
            raise ValueError(f'velocity duration must be in (0,{self.velocity_duration_max}]s')
        values = {}
        for i, value in zip(self.active_axes, targets):
            value = _finite(value, f'J{i} velocity')
            if abs(value) > self.envelopes[i].speed+1e-9:
                raise ValueError(f'J{i}: velocity target exceeds speed envelope')
            values[i] = value
        self.velocity_targets = values
        self.plan = dict(duration=duration, mode='velocity', plan_at=now)
        self.velocity_at = now
        self.state = 'VELOCITY'
        self.audit('FIVE_AXIS_VELOCITY_ACCEPTED_NOT_SENT', duration_s=duration,
                   targets={i: values[i] for i in self.active_axes})

    def hold(self, now):
        if self.state not in ('HOLD', 'TRACK', 'VELOCITY'):
            raise ValueError('hold requires an active controller')
        rows = getattr(self, '_rows_now', None)
        if rows:
            self.targets = {i: rows[i].q for i in self.active_axes}
        self.plan = None
        self.state = 'HOLD'
        self.audit('FIVE_AXIS_HOLD_AT_CURRENT')

    # ---- stop / fault --------------------------------------------------------
    def _fail(self, reason, now):
        self.reason = reason
        self.state = 'STOPPING'
        self.plan = None
        self.stop_time = now
        self.audit('FIVE_AXIS_FAULT', reason=reason, axes=list(self.active_axes))
        self._send_fd_all()

    def stop(self, reason, now):
        if self.state in ('IDLE', 'DISABLED', 'UNKNOWN'):
            return
        if self.state == 'STOPPING':
            return
        self.reason = reason
        self.state = 'STOPPING'
        self.plan = None
        self.stop_time = now
        self.audit('FIVE_AXIS_STOP', reason=reason, axes=list(self.active_axes))
        self._send_fd_all()

    def _send_fd_all(self):
        for i in self.active_axes:
            try:
                self.endpoint.send(i, b'\xff'*7+b'\xfd', 'disable', 'stop')
            except Exception as exc:
                self.audit('FIVE_AXIS_FD_FAILED', joint=i, error=str(exc))

    def _observe_stop(self, snapshot, now):
        if self.stop_time is None:
            return
        try:
            rows = self._rows(snapshot)
        except Exception:
            rows = ()
        if rows and all(r.state == 0 and r.received_at > self.stop_time for r in rows):
            self.state = 'DISABLED'
            self.audit('FIVE_AXIS_DISABLED_CONFIRMED',
                       axes=list(self.active_axes), elapsed_s=now-self.stop_time)
            return
        if now-self.stop_time > self.ack_timeout+1e-9:
            self.state = 'UNKNOWN'
            self.audit('FIVE_AXIS_STOP_UNKNOWN', axes=list(self.active_axes),
                       elapsed_s=now-self.stop_time)
