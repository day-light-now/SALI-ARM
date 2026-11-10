"""Four-axis MIT control backend; no port discovery, auto-connect or UI enable.

Transport is injected explicitly. Physical acceptance evidence and numerical
per-axis envelopes are required; no gains/torque limits are taken from simulation.
Feedback confirms observed drive state, never execution of an individual packet.

Host PD candidates are a closed, explicitly selected set. The verified Kp=4 tier
stays the default; a stiffer tier only applies when the operator ticket names it
and the envelope gains match that tier exactly. Nothing here tunes or escalates
gains, raises the encoded torque bound or retries on its own.
"""
from dataclasses import dataclass
from collections import deque
import math
import uuid

from .commissioning import JointCommand, Snapshot, encode_shadow
from .core import ACTIVE, minjerk

# Hard ceilings for any commissioning host PD candidate, independent of the tier
# table: a future tier cannot quietly exceed these without an explicit review.
MAX_CANDIDATE_KP = 32.
MAX_CANDIDATE_KD = .5
TORQUE_RAW_STEPS = 4095


@dataclass(frozen=True)
class HostPdTier:
    """A named host PD candidate that an operator must select explicitly.

    Merely existing in this table grants nothing: the J4 acceptance ticket names
    one tier, and the controller rejects any envelope whose gains differ.
    """
    name: str
    kp: float
    kd: float
    label: str
    hardware_validated: bool
    torque_cap: float = .3
    slew_nm_s: float | None = None
    feedback_speed_abort: float = .1
    following_error_deg: float = 2.

    def __post_init__(self):
        if not math.isfinite(self.following_error_deg) or not 0 < self.following_error_deg <= 4.:
            raise ValueError('Candidate following error exceeds 4deg')
        if not math.isfinite(self.feedback_speed_abort) or not 0 < self.feedback_speed_abort <= .2:
            raise ValueError('Candidate feedback speed threshold exceeds 0.2rad/s')
        if not math.isfinite(self.torque_cap) or not 0 < self.torque_cap <= 2.:
            raise ValueError('Candidate torque cap exceeds 2Nm')
        if self.slew_nm_s is not None and (not math.isfinite(self.slew_nm_s) or not 0 < self.slew_nm_s <= 1.):
            raise ValueError('Invalid candidate slew')
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('Host PD tier needs a stable name')
        if not isinstance(self.label, str) or not self.label.strip():
            raise ValueError('Host PD tier needs an operator-visible label')
        if not (math.isfinite(self.kp) and math.isfinite(self.kd)
                and 0 < self.kp <= MAX_CANDIDATE_KP and 0 <= self.kd <= MAX_CANDIDATE_KD):
            raise ValueError('Host PD tier gains exceed the accepted candidate ceiling')


VERIFIED_PD_TIER = HostPdTier('kp4_verified', 4., .5,
                              '已验证档 Kp=4（默认，此前实机试验所用）', True)
STIFFER_PD_TIER = HostPdTier('kp12_candidate', 12., .5,
                             '较硬候选档 Kp=12（未实机验证，仍受0.30Nm编码上限）', False)
ONE_NM_PD_TIER = HostPdTier('kp32_1nm_candidate', 32., .5,
    '1Nm候选档 Kp=32 / Kd=0.5（未实机验证；力矩斜坡0.5Nm/s；速度中止0.20rad/s）', False, 1., .5, .2)
TWO_NM_PD_TIER = HostPdTier('kp32_2nm_candidate', 32., .5,
    '2Nm候选档 Kp=32 / Kd=0.5（未实机验证；误差4°；斜坡1Nm/s；速度中止0.20rad/s）',
    False, torque_cap=2., slew_nm_s=1., feedback_speed_abort=.2, following_error_deg=4.)
PD_TIERS = {tier.name: tier for tier in (VERIFIED_PD_TIER, STIFFER_PD_TIER, ONE_NM_PD_TIER, TWO_NM_PD_TIER)}
DEFAULT_PD_TIER = VERIFIED_PD_TIER.name


def pd_tier(name):
    """Resolve an explicitly named tier; an absent name is never defaulted here."""
    if not isinstance(name, str) or name not in PD_TIERS:
        raise ValueError('Host PD tier must be named explicitly and be an accepted candidate')
    return PD_TIERS[name]


def torque_raw_bounds(cap, tmax):
    """Largest raw torque indices whose decoded value stays inside +-cap."""
    if not (math.isfinite(cap) and math.isfinite(tmax) and 0 < tmax and 0 <= cap < tmax):
        raise ValueError('Invalid torque cap/protocol range')
    low = math.ceil((-cap + tmax) * TORQUE_RAW_STEPS / (2 * tmax))
    high = math.floor((cap + tmax) * TORQUE_RAW_STEPS / (2 * tmax))
    if low > high:
        raise ValueError('No representable torque inside the accepted bound')
    return low, high


def encoded_cap_torque_nm(cap, tmax):
    """Largest |torque| the 12-bit field can actually carry inside cap.

    For a 0.30Nm bound with TMAX=54Nm this is 0.276923Nm, not 0.30Nm: reporting
    the encoded value avoids pretending the quantization gap is usable output.
    """
    low, high = torque_raw_bounds(cap, tmax)
    return max(abs(low * 2 * tmax / TORQUE_RAW_STEPS - tmax),
               abs(high * 2 * tmax / TORQUE_RAW_STEPS - tmax))


@dataclass(frozen=True)
class Envelope:
    lower: float
    upper: float
    speed: float
    torque: float
    following_error: float
    kp: float
    kd: float
    temperature: float
    feedback_speed: float | None = None
    arrival_error: float | None = None

    def __post_init__(self):
        values = (self.lower, self.upper, self.speed, self.torque,
                  self.following_error, self.kp, self.kd, self.temperature)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Envelope must be finite')
        if not self.lower < self.upper or min(values[2:5]) <= 0:
            raise ValueError('Invalid travel/speed/torque/error bounds')
        if self.kp <= 0 or self.kd < 0 or not 0 < self.temperature <= 60:
            raise ValueError('Invalid accepted gains/temperature')
        if self.feedback_speed is not None and (not math.isfinite(self.feedback_speed)
                                                or self.feedback_speed<self.speed):
            raise ValueError('Invalid independent measured-speed threshold')
        if self.arrival_error is not None and (not math.isfinite(self.arrival_error)
                                               or not 0<self.arrival_error<=self.following_error):
            raise ValueError('Invalid arrival tolerance')


@dataclass(frozen=True)
class Acceptance:
    """Evidence references, not defaults or a UI checkbox granting acceptance."""
    passive_catch: str
    watchdog_action: str
    local_envelopes: str
    preload_behavior: str
    identity: str
    bus_timing: str

    def validate(self):
        if not all(isinstance(v, str) and v.strip() for v in vars(self).values()):
            raise ValueError('Physical acceptance evidence incomplete')


@dataclass(frozen=True)
class LimitedJ4Trial:
    """Operator-authorized commissioning experiment, NOT full acceptance.

    Explicitly permits only a supported two-second current-pose J4 hold to
    investigate the proposed envelope. It grants no trajectory or other-axis
    enable authority, and does not fabricate physical watchdog/torque-off proof.
    The named host PD tier is part of the frozen ticket, so a gain change needs a
    new operator request rather than an in-place edit.
    """
    support_confirmed: bool
    local_clearance_confirmed: bool
    low_output_and_stop_risk_acknowledged: bool
    watchdog_report: str
    pd_tier: str = DEFAULT_PD_TIER

    def validate(self):
        if not all(v is True for v in (self.support_confirmed,self.local_clearance_confirmed,
                                      self.low_output_and_stop_risk_acknowledged)):
            raise ValueError('Supported commissioning conditions not confirmed')
        pd_tier(self.pd_tier)
        import json
        from pathlib import Path
        from .core import ROOT
        path=Path(self.watchdog_report).resolve()
        if not path.is_relative_to((ROOT/'tmp/watchdog_check').resolve()):
            raise ValueError('Expected a local J4 watchdog observation record')
        report=json.loads(path.read_text(encoding='utf-8'))
        if report.get('joint')!=4 or report.get('watchdog_d_observed') is not True:
            raise ValueError('J4 D-state observation missing; not physical torque-off proof')


@dataclass(frozen=True)
class LimitedJ4RoundTrip(LimitedJ4Trial):
    hold_report: str = ''

    def validate(self):
        super().validate()
        import json
        from pathlib import Path
        from .core import ROOT
        path=Path(self.hold_report).resolve()
        if not path.is_relative_to((ROOT/'tmp/live_trial').resolve()):
            raise ValueError('Expected a local completed supported J4 hold record')
        report=json.loads(path.read_text(encoding='utf-8'))
        if (report.get('trial')!='supported_J4_current_pose_hold_2s'
                or report.get('position_hold_criterion_met') is not True
                or report.get('fresh_stop_confirmation') is not True
                or report.get('final_controller_state')!='DISABLED'
                or report.get('enabled_observed_axes')!=[4]):
            raise ValueError('Completed J4 supported hold/stop record required')


class USB2CANEndpoint:
    """Exclusive injected serial connection. Never opens a port itself.

    Only basic IDs 1–4, MIT commands and FC/FD. No parameter/Flash/zero writes.
    Closing this connection is NOT a confirmed stop; controller.stop comes first.
    """
    def __init__(self, serial, audit, clock):
        self.serial, self.audit, self.clock = serial, audit, clock
        self.closed = False
        self.receipts = deque(maxlen=4000)

    def send(self, joint, payload, command_id, kind):
        if self.closed or type(joint) is not int or joint not in ACTIVE:
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
                       can_id=joint,payload=payload.hex(' '),
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

    def close(self):
        self.closed = True
        self.serial.close()


class FourAxisController:
    """Single-owner, manually scheduled 100Hz candidate backend.

    Start freezes the measured current pose. Host PD supplies bounded torque;
    drive kp/kd remain zero. Ramp affects total requested torque. A fault cancels
    the plan for all axes and attempts FD on every axis independently. A broken
    bus leaves state UNKNOWN, never falsely DISABLED; watchdog/support are vital.
    """
    def __init__(self, endpoint, envelopes, acceptance, audit, *, period=.01,
                 max_age=.06, max_gap=.04, ramp=1., ack_timeout=.3,
                 active_axes=ACTIVE, max_runtime=None):
        self.endpoint, self.audit = endpoint, audit
        self.envelopes = tuple(envelopes)
        self.acceptance = acceptance
        if len(self.envelopes) != 4:
            raise ValueError('Exactly four accepted axis envelopes required')
        self.active_axes=tuple(active_axes)
        self.max_runtime=max_runtime
        self.trial_leg=0
        self.tier=None
        if isinstance(acceptance,LimitedJ4Trial):
            e=self.envelopes[3]
            round_trip=isinstance(acceptance,LimitedJ4RoundTrip)
            tier=pd_tier(acceptance.pd_tier)
            self.tier=tier
            if (self.active_axes!=(4,) or max_runtime!=(45. if round_trip else 2.) or ramp!=1.
                    or e.torque>tier.torque_cap or e.kp>MAX_CANDIDATE_KP or e.kd>MAX_CANDIDATE_KD
                    or e.speed>.02
                    or e.following_error>math.radians(tier.following_error_deg)+1e-12
                    or e.upper-e.lower>math.radians((10 if round_trip else 0)+2*tier.following_error_deg)+1e-12):
                raise ValueError('Limited J4 trial cannot expand scope/output/runtime')
            # The ticket names one tier; the live envelope must match it exactly.
            # This is what stops an envelope edit from silently raising the gain.
            if abs(e.kp-tier.kp)>1e-12 or abs(e.kd-tier.kd)>1e-12:
                raise ValueError('J4 envelope gains must match the explicitly selected tier')
            if round_trip and (e.arrival_error is None or e.arrival_error>math.radians(.5)+1e-12):
                raise ValueError('Round trip requires arrival tolerance <=0.5deg')
            for index,envelope in enumerate(self.envelopes):
                ceiling=tier.feedback_speed_abort if index==3 else .1
                if envelope.feedback_speed is not None and envelope.feedback_speed>ceiling:
                    raise ValueError('Limited trial feedback-speed threshold exceeds selected tier')
        elif self.active_axes!=ACTIVE or max_runtime is not None:
            raise ValueError('Subset/runtime only implemented for explicit J4 trial')
        if not all(math.isfinite(v) for v in (period,max_age,max_gap,ramp,ack_timeout)):
            raise ValueError('Nonfinite timing')
        if not .005 <= period <= .02 or not period < max_gap < .1:
            raise ValueError('Loop must fit nominal 100ms watchdog')
        if not period <= max_age < .1 or not .5 <= ramp <= 2 or not 0 < ack_timeout <= 1:
            raise ValueError('Invalid feedback/ramp/ack timing')
        self.period,self.max_age,self.max_gap = period,max_age,max_gap
        self.ramp,self.ack_timeout = ramp,ack_timeout
        self.state = 'IDLE'
        self.reason = None
        self.plan = None
        self.last_time = None
        self.last_received = {}
        self.enabled = set()
        self.pending = None
        self.enable_floor = None
        self.pending_at = None
        self.ramp_at = {}
        self.stop_floor = {}
        self.stop_time = None
        self.output = {}  # joint -> last encoded/requested torque actually built

    def _check(self, snapshot, now, initial=False):
        if not isinstance(snapshot, Snapshot) or not math.isfinite(now):
            raise ValueError('Invalid snapshot/time')
        if tuple(o.profile.joint for o in snapshot.rows) != ACTIVE:
            raise ValueError('Need four ordered axes; J5 excluded')
        if not snapshot.session_id or snapshot.clock != 'perf_counter':
            raise ValueError('Missing session or wrong clock')
        if not initial:
            if (snapshot.session_id, snapshot.clock, snapshot.mapping_sha256) != self.identity:
                raise ValueError('Session/clock/mapping changed')
            if tuple(o.profile for o in snapshot.rows) != self.profiles:
                raise ValueError('Live configuration changed')
        for o,e in zip(snapshot.rows,self.envelopes):
            p = o.profile
            vals = (o.position,o.velocity,o.received_at,o.driver_temp,o.motor_temp)
            if not all(math.isfinite(v) for v in vals):
                raise ValueError(f'J{p.joint}: nonfinite feedback')
            if not 0 <= now-o.received_at <= self.max_age:
                raise ValueError(f'J{p.joint}: stale feedback')
            if o.received_at < self.last_received.get(p.joint, -math.inf):
                raise ValueError(f'J{p.joint}: feedback time regressed')
            if p.mode != 1 or p.timeout != 2000:
                raise ValueError(f'J{p.joint}: require fresh MIT/2000 configuration')
            if e.torque >= p.tmax or e.speed > p.vmax:
                raise ValueError('Accepted bounds outside live protocol')
            # Quantized zero must itself fit the accepted output bound.
            if e.torque < p.tmax/4095:
                raise ValueError('Torque limit below nearest representable zero')
            if not e.lower <= o.q <= e.upper:
                raise ValueError(f'J{p.joint}: position {o.q:.6f}rad outside [{e.lower:.6f},{e.upper:.6f}]')
            measured_limit=e.speed if e.feedback_speed is None else e.feedback_speed
            if abs(o.velocity)>measured_limit:
                raise ValueError(f'J{p.joint}: feedback speed {o.velocity:.6f}rad/s exceeds '
                                 f'{measured_limit:.6f}; command limit remains {e.speed:.6f}')
            if max(o.driver_temp,o.motor_temp) >= e.temperature:
                raise ValueError(f'J{p.joint}: temperature')
            allowed = {0} if initial or p.joint not in self.enabled else {1}
            if not initial and p.joint == self.pending:
                allowed = {0,1}
            if o.state not in allowed:
                raise ValueError(f'J{p.joint}: unexpected drive state {o.state}')

    def start(self, snapshot, now):
        if self.state != 'IDLE':
            raise ValueError('New controller/session required after stop or fault')
        self.acceptance.validate()
        self._check(snapshot, now, initial=True)
        self.profiles = tuple(o.profile for o in snapshot.rows)
        self.identity = (snapshot.session_id,snapshot.clock,snapshot.mapping_sha256)
        self.targets = tuple(o.q for o in snapshot.rows)
        self.anchor = self.targets
        self.last_snapshot = snapshot
        self.last_time = now
        self.started_at = now
        self.state = 'TAKEOVER'
        self.audit('TAKEOVER_PREPARED', command_id=uuid.uuid4().hex,
                   targets=self.targets, evidence=vars(self.acceptance),
                   pd_tier=self.tier.name if self.tier else None,
                   host_kp=self.envelopes[3].kp if self.tier else None,
                   host_kd=self.envelopes[3].kd if self.tier else None,
                   torque_cap_nm=self.envelopes[3].torque if self.tier else None)

    def _send(self, i, payload, kind, command_id=None):
        return self.endpoint.send(i,payload,command_id or uuid.uuid4().hex,kind)

    def _torque_frame(self, o, e, target, velocity, now):
        scale = min(1., max(0.,(now-self.ramp_at[o.profile.joint])/self.ramp))
        requested = scale*(e.kp*(target-o.q)+e.kd*(velocity-o.profile.sign*o.velocity))
        torque = max(-e.torque,min(e.torque,requested))
        if self.tier is not None and self.tier.slew_nm_s is not None:
            previous=self.output.get(o.profile.joint)
            before=previous['limited_nm'] if previous else 0.
            elapsed=max(0.,now-previous['at']) if previous else 0.
            delta=self.tier.slew_nm_s*elapsed
            torque=max(before-delta,min(before+delta,torque))
        p = o.profile
        # Select a representable output within bounds, not clamp only float FF.
        lo,hi = torque_raw_bounds(e.torque,p.tmax)
        raw = max(lo,min(hi,round((p.sign*torque+p.tmax)*4095/(2*p.tmax))))
        actual = raw*2*p.tmax/4095-p.tmax
        encoded = encode_shadow(p,JointCommand(target,speed_limit=e.speed))
        payload = bytearray.fromhex(encoded['candidate_payload'])
        # Driver gains and target velocity are zero; all PD is on the host.
        payload[6] = (payload[6]&0xF0)|(raw>>8)
        payload[7] = raw&255
        if abs(actual) > e.torque+1e-12:
            raise ValueError('Encoded total torque outside accepted bounds')
        # Record what was built, not what the drive later reported: the drive
        # torque estimate is not a measurement of external load.
        self.output[o.profile.joint]=dict(
            at=now,target=target,position=o.q,error=target-o.q,
            requested_nm=requested,limited_nm=torque,encoded_nm=actual,
            cap_nm=e.torque,encoded_cap_nm=encoded_cap_torque_nm(e.torque,p.tmax),
            ramp_scale=scale,raw=raw,raw_low=lo,raw_high=hi,
            request_clipped=abs(requested)>e.torque+1e-12,
            at_encoded_cap=raw in (lo,hi))
        return bytes(payload)

    def trajectory(self, targets, duration, now):
        targets=tuple(targets)
        if isinstance(self.acceptance,LimitedJ4RoundTrip):
            if not hasattr(self,'anchor'):raise ValueError('Takeover has not started')
            expected=list(self.anchor)
            if self.trial_leg==0:expected[3]+=math.radians(10)
            if (self.trial_leg>=2 or duration!=18. or len(targets)!=4
                    or any(not math.isfinite(v) or abs(v-x)>1e-10 for v,x in zip(targets,expected))):
                raise ValueError('Only frozen J4 +10deg then original anchor, 18s per leg allowed')
        elif isinstance(self.acceptance,LimitedJ4Trial):
            raise ValueError('Limited J4 current-pose trial does not authorize trajectories')
        if self.state != 'HOLD' or self.plan is not None:
            raise ValueError('Trajectory only from confirmed HOLD')
        targets = tuple(targets)
        if len(targets) != 4 or not math.isfinite(duration) or not 1 <= duration <= 60:
            raise ValueError('Four targets and bounded duration required')
        if self.last_time is None or not math.isfinite(now) or not 0 <= now-self.last_time <= self.max_gap:
            raise ValueError('Trajectory requires current controller cycle')
        for target,start,anchor,e in zip(targets,self.targets,self.anchor,self.envelopes):
            if (not math.isfinite(target) or not e.lower<=target<=e.upper
                    or abs(target-anchor)>math.radians(20)
                    or 1.875*abs(target-start)/duration>e.speed):
                raise ValueError('Trajectory outside accepted travel/speed')
        command_id = uuid.uuid4().hex
        self.plan = (command_id,self.targets,targets,float(duration),now)
        if isinstance(self.acceptance,LimitedJ4RoundTrip):self.trial_leg+=1
        self.state = 'TRACK'
        self.audit('PLAN_ACCEPTED_NOT_SENT', command_id=command_id, targets=targets,duration=duration)
        return command_id

    def tick(self, snapshot, now):
        if self.state in ('IDLE','DISABLED','UNKNOWN'):
            return
        if self.state == 'STOPPING':
            self.observe_stop(snapshot,now)
            return
        try:
            self._check(snapshot,now)
            if self.max_runtime is not None and now-self.started_at>=self.max_runtime:
                self.stop('trial_duration_complete',now)
                return
            if not 0 < now-self.last_time <= self.max_gap:
                raise ValueError('Host loop time regressed or missed deadline')
            if now-self.last_time < self.period:
                return  # Never catch up with a burst of control frames.
            self.last_time = now
            self.last_snapshot = snapshot
            self.last_received = {o.profile.joint:o.received_at for o in snapshot.rows}
            if self.pending is not None:
                o = snapshot.rows[self.pending-1]
                if o.state == 1 and o.received_at > max(self.enable_floor,self.pending_at):
                    self.enabled.add(self.pending)
                    self.ramp_at[self.pending] = now
                    self.audit('DRIVE_ENABLED_OBSERVED', joint=self.pending,
                               command_id=self.pending_command_id,
                               note='state observation, not per-command acknowledgement')
                    self.pending = None
                elif now-self.pending_at > self.ack_timeout:
                    raise ValueError('Enable confirmation timeout')
            velocities = (0.,)*4
            if self.plan:
                cid,starts,ends,duration,at = self.plan
                u = min(1.,max(0.,(now-at)/duration))
                self.targets = tuple(a+(b-a)*minjerk(u) for a,b in zip(starts,ends))
                velocities = tuple((b-a)*30*u*u*(1-u)**2/duration for a,b in zip(starts,ends))
                if u == 1. and all(abs(t-o.q)<=(e.following_error if e.arrival_error is None else e.arrival_error)
                                  for t,o,e in zip(self.targets,snapshot.rows,self.envelopes)):
                    self.plan = None
                    self.state = 'HOLD'
                    self.audit('PLAN_REACHED_OBSERVED',command_id=cid)
                elif now-at > duration+(1. if isinstance(self.acceptance,LimitedJ4RoundTrip) else self.ack_timeout):
                    raise ValueError('Trajectory arrival timeout')
            # Build all frames before any write: invalid axis prevents whole cycle.
            frames = []
            cycle_id = uuid.uuid4().hex
            for o,e,target,velocity in zip(snapshot.rows,self.envelopes,self.targets,velocities):
                if o.profile.joint in self.enabled:
                    if abs(target-o.q)>e.following_error:
                        raise ValueError('Following error')
                    frames.append((o.profile.joint,self._torque_frame(o,e,target,velocity,now)))
            for i,payload in frames:
                self._send(i,payload,'mit',self.plan[0] if self.plan else cycle_id)
                if self.endpoint.clock()-now > self.max_gap:
                    raise ValueError('USB cycle write deadline exceeded')
            self.audit('CONTROL_CYCLE',cycle_id=cycle_id,at=now,finished_at=self.endpoint.clock(),
                       axes=[i for i,_ in frames],plan_id=self.plan[0] if self.plan else None)
            if self.state == 'TAKEOVER' and self.pending is None:
                candidates = [i for i in self.active_axes if i not in self.enabled]
                # Finish current ramp before enabling the next axis.
                if candidates and all(now-self.ramp_at[i]>=self.ramp for i in self.enabled):
                    i = candidates[0]
                    o = snapshot.rows[i-1]
                    self.ramp_at[i] = now
                    self._send(i,self._torque_frame(o,self.envelopes[i-1],self.targets[i-1],0.,now),'mit')
                    self.pending,self.pending_at,self.enable_floor = i,now,o.received_at
                    self.pending_command_id = uuid.uuid4().hex
                    self._send(i,b'\xff'*7+b'\xfc','enable',self.pending_command_id)
                elif not candidates and all(now-self.ramp_at[i]>=self.ramp for i in self.enabled):
                    self.state = 'HOLD'
                    self.audit('FOUR_AXIS_HOLD_OBSERVED' if self.active_axes==ACTIVE else 'SCOPED_HOLD_OBSERVED',
                               axes=self.active_axes,targets=self.targets)
        except Exception as exc:
            self.stop(str(exc),now)

    def stop(self, reason, now):
        """Supported-arm FD policy only; no automatic resume or reset."""
        if self.state in ('IDLE','STOPPING','DISABLED','UNKNOWN'):
            return
        self.reason = reason
        self.plan = None
        self.state = 'STOPPING'
        self.stop_time = now
        self.stop_floor = {o.profile.joint:o.received_at for o in self.last_snapshot.rows}
        failures = []
        for i in ACTIVE:
            try:
                self._send(i,b'\xff'*7+b'\xfd','disable')
            except Exception:
                failures.append(i)  # Keep attempting other axes, even if audit fails.
        if failures:
            self.state = 'UNKNOWN'
        try:
            self.audit('GLOBAL_STOP_REQUESTED', reason=reason,delivery_unknown_axes=failures,
                       physical_state='unknown_until_new_feedback')
        except Exception:
            pass

    def observe_stop(self,snapshot,now):
        if not math.isfinite(now) or now < self.stop_time:
            self.state = 'UNKNOWN'
            return
        same = ((snapshot.session_id,snapshot.clock,snapshot.mapping_sha256)==self.identity
                and tuple(o.profile for o in snapshot.rows)==self.profiles)
        if same and all(o.state==0 and self.stop_floor[o.profile.joint]<o.received_at<=now
                        and o.received_at>self.stop_time
                        and now-o.received_at<=self.max_age for o in snapshot.rows):
            self.state = 'DISABLED'
        elif now-self.stop_time > self.ack_timeout:
            self.state = 'UNKNOWN'


def _dwell_s(items, flag):
    """Time covered by consecutive samples whose current sample satisfies flag."""
    total = 0.
    for previous, current in zip(items, items[1:]):
        if flag(current):
            total += max(0., current['at'] - previous['at'])
    return total


def motion_summary(samples, *, cap_nm, encoded_cap_nm, encoder_resolution_deg,
                   motion_required, following_error_deg):
    """Facts about whether output was asked for and whether the axis moved.

    Distinguishes 'the command never requested the budget' from 'the encoded
    limit was reached and the axis still did not move'. It never proposes or
    performs an output increase: the caller only reports these facts.
    """
    for name,value in (('cap_nm',cap_nm),('encoded_cap_nm',encoded_cap_nm),
                       ('encoder_resolution_deg',encoder_resolution_deg),
                       ('following_error_deg',following_error_deg)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {name}')
    samples = list(samples)
    enabled = [s for s in samples if s.get('state') == 1]
    result = dict(sample_count=len(samples), enabled_samples=len(enabled),
                  output_sample_count=0, sampled_span_s=0., cap_nm=cap_nm,
                  encoded_cap_nm=encoded_cap_nm,
                  encoder_resolution_deg=encoder_resolution_deg,
                  motion_required=bool(motion_required), moved=False, stalled=False,
                  position_codes=[], position_code_changes=0,
                  first_angle_deg=None, last_angle_deg=None,
                  max_positive_travel_deg=0., total_span_deg=0.,
                  net_travel_deg=0., peak_requested_nm=None, peak_encoded_nm=None,
                  peak_fraction_of_cap=None, peak_fraction_of_encoded_cap=None,
                  cycles_at_encoded_cap=0, share_at_encoded_cap=0.,
                  time_at_encoded_cap_s=0., time_share_at_encoded_cap=0.,
                  cycles_near_encoded_cap=0, share_near_encoded_cap=0.,
                  time_near_encoded_cap_s=0., cycles_request_clipped=0,
                  max_error_deg=0., final_error_deg=None,
                  assessment='NO_ENABLED_SAMPLES')
    if not enabled:
        return result
    angles = [s['angle_deg'] for s in enabled]
    first, last = angles[0], angles[-1]
    span = max(angles) - min(angles)
    codes = sorted({s['position_code'] for s in enabled
                    if s.get('position_code') is not None})
    outputs = [s['output'] for s in enabled if s.get('output')]
    peak_requested = max((abs(o['requested_nm']) for o in outputs), default=None)
    peak_encoded = max((abs(o['encoded_nm']) for o in outputs), default=None)
    at_cap = [s for s in enabled if s.get('output') and s['output']['at_encoded_cap']]
    near = [s for s in enabled if s.get('output')
            and abs(s['output']['encoded_nm']) >= .9 * encoded_cap_nm]
    clipped = [s for s in enabled if s.get('output') and s['output']['request_clipped']]
    sampled_span = enabled[-1]['at'] - enabled[0]['at']
    time_at_cap = _dwell_s(enabled, lambda s: bool(s.get('output') and s['output']['at_encoded_cap']))
    time_near = _dwell_s(enabled, lambda s: bool(s.get('output')
                                                and abs(s['output']['encoded_nm']) >= .9 * encoded_cap_nm))
    errors = [abs(s['error_deg']) for s in enabled]
    moved = (len(codes) >= 2) or (span > encoder_resolution_deg / 2)
    result.update(
        sampled_span_s=sampled_span, first_angle_deg=first, last_angle_deg=last,
        max_positive_travel_deg=max(angles) - first, total_span_deg=span,
        net_travel_deg=last - first, position_codes=codes,
        output_sample_count=len(outputs),
        position_code_changes=max(0, len(codes) - 1),
        peak_requested_nm=peak_requested, peak_encoded_nm=peak_encoded,
        peak_fraction_of_cap=(None if peak_encoded is None else peak_encoded / cap_nm),
        peak_fraction_of_encoded_cap=(None if peak_encoded is None
                                      else peak_encoded / encoded_cap_nm),
        cycles_at_encoded_cap=len(at_cap),
        share_at_encoded_cap=(len(at_cap) / len(outputs)) if outputs else 0.,
        time_at_encoded_cap_s=time_at_cap,
        time_share_at_encoded_cap=(time_at_cap / sampled_span) if sampled_span > 0 else 0.,
        cycles_near_encoded_cap=len(near),
        share_near_encoded_cap=(len(near) / len(outputs)) if outputs else 0.,
        time_near_encoded_cap_s=time_near,
        cycles_request_clipped=len(clipped),
        max_error_deg=max(errors), final_error_deg=enabled[-1]['error_deg'],
        moved=moved)
    result['stalled'] = bool(motion_required and not moved
                             and result['max_error_deg'] >= following_error_deg - 1e-9)
    if moved:
        result['assessment'] = 'MOVED'
    elif not motion_required:
        result['assessment'] = 'NO_MOTION_REQUIRED'
    elif at_cap:
        result['assessment'] = 'OUTPUT_AT_ENCODED_CAP_NO_MOTION'
    elif outputs:
        result['assessment'] = 'COMMAND_BELOW_CAP_NO_MOTION'
    else:
        result['assessment'] = 'NO_OUTPUT_RECORDED'
    return result
