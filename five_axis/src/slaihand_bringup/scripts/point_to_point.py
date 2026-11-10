"""Endpoint verification and bounded retiming of the official ROS JTC.

No motor encoding, PD controller, gravity estimate or serial access.
"""
from collections import deque
import copy
import math
import time

EXTRA_SECONDS = 6.0
ENDPOINT_TIMEOUT_S = 3.0
STABLE_SECONDS = .5
ERROR_RAD = math.radians(.5)
VELOCITY_RAD_S = .05
MAX_SAMPLE_GAP_S = .06


def effective_document(document):
    result = copy.deepcopy(document)
    for row in result['waypoints']:
        row['dwell_s'] = max(1., row['dwell_s'])
    return result


def axis_names(target):
    """Canonical names for the planned width; never assume a fixed four axes."""
    width = len(target)
    return ['slaihand_joint%d' % index for index in range(1, width + 1)]


class StableEndpoint:
    def __init__(self, target):
        self.target = list(target)
        self.width = len(self.target)
        self.samples = deque()
        self.last_at = None
        self.total = 0
        self.worst = [0.] * self.width

    def observe(self, at, q, velocity):
        if len(q) != self.width or len(velocity) != self.width:
            raise RuntimeError('PTP_DIMENSION_MISMATCH')
        if self.last_at is not None and at <= self.last_at:
            return  # repeated cached message is not another observation
        errors = [abs(a-b) for a, b in zip(q, self.target)]
        self.total += 1
        self.worst = [max(a,b) for a,b in zip(self.worst, errors)]
        gap = self.last_at is not None and at-self.last_at > MAX_SAMPLE_GAP_S
        self.last_at = at
        if gap or max(errors) > ERROR_RAD or max(map(abs, velocity)) > VELOCITY_RAD_S:
            self.samples.clear()
        if max(errors) <= ERROR_RAD and max(map(abs, velocity)) <= VELOCITY_RAD_S:
            self.samples.append((at, errors, [abs(v) for v in velocity]))
            while len(self.samples)>1 and at-self.samples[1][0] >= STABLE_SECONDS:
                self.samples.popleft()

    def passed(self, now):
        return bool(self.samples and now-self.samples[-1][0] <= MAX_SAMPLE_GAP_S
                    and self.samples[-1][0]-self.samples[0][0] >= STABLE_SECONDS)

    def result(self, now):
        good = self.passed(now)
        values = ([max(row[1][i] for row in self.samples) for i in range(self.width)]
                  if good else self.worst if self.total else None)
        return dict(samples=len(self.samples) if good else self.total,
                    sampled_span_s=(self.samples[-1][0]-self.samples[0][0]) if len(self.samples)>1 else 0.,
                    max_error_deg=None if values is None else [math.degrees(v) for v in values],
                    passed=good, stable_seconds_required=STABLE_SECONDS,
                    max_velocity_rad_s=([max(row[2][i] for row in self.samples)
                                         for i in range(self.width)] if good else None),
                    velocity_limit_rad_s=VELOCITY_RAD_S)


class PointToPointSession:
    def __init__(self, windows, duration):
        self.windows = windows
        self.duration = duration
        self.endpoints = [StableEndpoint(w['target_rad']) for w in windows]
        self.entered = [None]*len(windows)
        self.completed = [None]*len(windows)
        self.index = 0
        self.last_phase = -1.
        self.last_wall = None
        self.factor = 1.
        self.minimum_factor = 1.
        self.scaled_samples = 0

    def observe(self, phase, now, q, velocity, reference, sample_at):
        if not all(math.isfinite(v) for v in [phase,now,sample_at]+list(q)+list(velocity)+list(reference)):
            raise RuntimeError('PTP_NONFINITE_FEEDBACK')
        if now-sample_at > MAX_SAMPLE_GAP_S:
            raise RuntimeError('PTP_STALE_FEEDBACK')
        if phase < self.last_phase-1e-6:
            raise RuntimeError('PTP_PHASE_REGRESSED')
        self.last_phase = phase
        dt = 0. if self.last_wall is None else max(0.,min(.1,now-self.last_wall))
        self.last_wall = now
        while self.index < len(self.windows) and phase > self.windows[self.index]['end_s']+.001:
            verifier = self.endpoints[self.index]
            if not verifier.passed(now):
                raise RuntimeError('ENDPOINT_NOT_SETTLED:'+str(self.index+1))
            self.completed[self.index] = verifier.result(now)
            self.index += 1
        wait_endpoint = False
        if self.index < len(self.windows):
            window = self.windows[self.index]
            verifier = self.endpoints[self.index]
            if phase >= window['arrival_s']:
                if self.entered[self.index] is None:self.entered[self.index]=now
                verifier.observe(sample_at,q,velocity)
                if now-self.entered[self.index] > ENDPOINT_TIMEOUT_S and not verifier.passed(now):
                    raise RuntimeError('ENDPOINT_NOT_SETTLED:'+str(self.index+1))
                wait_endpoint = phase >= window['end_s']-.08 and not verifier.passed(now)
        error = max(abs(a-b) for a,b in zip(q,reference))
        # Stop advancing the reference well before the unchanged 4-degree guard.
        # This pauses trajectory time only; the drive continues its native hold.
        if wait_endpoint or error >= math.radians(2.):
            self.factor=0.
        else:
            desired=max(.1,min(1.,(math.radians(2.)-error)/math.radians(1.25)))
            rate=4. if desired<self.factor else .5
            self.factor+=max(-rate*dt,min(rate*dt,desired-self.factor))
        self.factor=max(0.,min(1.,self.factor))
        self.minimum_factor=min(self.minimum_factor,self.factor)
        self.scaled_samples+=int(self.factor<.999)
        return self.factor

    def results(self, now):
        return [dict(w, **(done if done is not None else verifier.result(now)))
                for w,verifier,done in zip(self.windows,self.endpoints,self.completed)]


def scaling_publisher(node):
    from control_msgs.msg import SpeedScalingFactor
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    return node.create_publisher(SpeedScalingFactor,
        '/slaihand/slaihand_joint_trajectory_controller/speed_scaling_input',
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))


def publish_factor(publisher, factor):
    from control_msgs.msg import SpeedScalingFactor
    message=SpeedScalingFactor();message.factor=float(factor);publisher.publish(message)


def run_retimed(node, seen, status, future, publisher, windows, duration, movement_at, result,
                joint_names=None):
    import rclpy
    if joint_names is None:
        joint_names = axis_names(windows[0]['target_rad']) if windows else []
    names = list(joint_names)
    if windows and len(windows[0]['target_rad']) != len(names):
        raise RuntimeError('PTP_DIMENSION_MISMATCH')
    session=PointToPointSession(windows,duration)
    synced=False;last_publish=0.;last_factor=None
    try:
        while not future.done():
            rclpy.spin_once(node,timeout_sec=.01)
            now=time.monotonic()
            if now-movement_at > duration+EXTRA_SECONDS:
                raise RuntimeError('PTP_RETIMING_BUDGET_EXHAUSTED')
            axes=status()
            if any(axes.get(n,{}).get('control_state') in (7,8,9) for n in names):
                raise RuntimeError('PTP_BACKEND_STOPPED')
            if now<movement_at:continue
            message=seen.get('controller')
            if message is None or now-seen.get('controller_at',0)>.12:
                raise RuntimeError('PTP_CONTROLLER_STATE_STALE')
            phase=message.reference.time_from_start.sec+message.reference.time_from_start.nanosec/1e9
            if not synced:
                if 0<=phase<=.2:synced=True
                elif now-movement_at>1:raise RuntimeError('PTP_PHASE_NOT_SYNCHRONISED')
                else:continue
            if any(axes.get(n,{}).get('feedback_valid')!=1 for n in names):
                raise RuntimeError('PTP_INVALID_FEEDBACK')
            # JTC's success-hold resets the published phase before the action
            # result callback necessarily arrives. Require that actual result,
            # with a bounded grace window; never accept an in-flight rewind.
            if phase<session.last_phase-1e-6 and phase<=.02 and session.last_phase>=duration-.04:
                grace=now+.25
                while not future.done() and time.monotonic()<grace:
                    rclpy.spin_once(node,timeout_sec=.01)
                    fresh=status();stamp=seen.get('at',0);at=time.monotonic()
                    if any(fresh.get(n,{}).get('feedback_valid')!=1 for n in names):
                        raise RuntimeError('PTP_INVALID_FEEDBACK')
                    session.endpoints[-1].observe(stamp,[fresh[n]['position'] for n in names],
                                                  [fresh[n]['velocity'] for n in names])
                if future.done():
                    result['terminal_phase_reset_observed']=True
                    break
                raise RuntimeError('PTP_GOAL_RESULT_MISSING')
            factor=session.observe(phase,now,[axes[n]['position'] for n in names],
                [axes[n]['velocity'] for n in names],list(message.reference.positions),seen.get('at',0))
            # Quantisation limits logging/chatter; the hard pause is immediate.
            factor=round(factor*20)/20
            if last_factor!=factor or now-last_publish>.1:
                publish_factor(publisher,factor);last_factor=factor;last_publish=now
        if not synced:raise RuntimeError('PTP_NO_PHASE_OBSERVATION')
    finally:
        rows=session.results(time.monotonic())
        result['waypoint_results']=rows
        result['waypoints_observed']=all(row['passed'] for row in rows)
        result['point_to_point']=dict(minimum_scaling=session.minimum_factor,
            scaled_samples=session.scaled_samples,extra_time_limit_s=EXTRA_SECONDS,
            endpoint_timeout_s=ENDPOINT_TIMEOUT_S,terminal_tolerance_deg=.5,
            stable_seconds=STABLE_SECONDS,official_jtc_scaling=True)
