"""Bounded host-side start/brake shaping and OpenArm comparison helpers.

Pure functions only: no serial, no controller, no file writes. The production
controller imports this module; the offline tests replay recorded states and a
synthetic stick-slip plant through the same code.

The native-MIT estimator at the bottom is an evaluation prototype. It is NOT
used by the production path: the host cannot bound a drive-side MIT total torque
by limiting only the feedforward field.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CandidateControl:
    """Explicit, frozen control profile attached to one candidate tier.

    Every value is a bound, not an automatic gain: the build-up is rate limited,
    the release is faster but still bounded, the startup phase has its own lower
    ceiling and a finite window, and the damping term only removes force.
    """
    slew_up_nm_s: float
    slew_down_nm_s: float
    startup_cap_nm: float
    startup_window_s: float
    motion_confirm_codes: int
    brake_gain_nm_per_rad_s: float
    brake_cap_nm: float
    brake_deadband_rad_s: float

    def __post_init__(self):
        finite = (self.slew_up_nm_s, self.slew_down_nm_s, self.startup_cap_nm,
                  self.startup_window_s, self.brake_gain_nm_per_rad_s,
                  self.brake_cap_nm, self.brake_deadband_rad_s)
        if not all(math.isfinite(v) and v >= 0 for v in finite):
            raise ValueError('Invalid candidate control parameter')
        if not 0 < self.slew_up_nm_s <= self.slew_down_nm_s:
            raise ValueError('Build-up must be slower than or equal to release')
        if not 0 < self.startup_cap_nm:
            raise ValueError('Startup cap must be positive')
        if not 0 < self.startup_window_s <= 10:
            raise ValueError('Startup window must be bounded')
        if not isinstance(self.motion_confirm_codes, int) or not 1 <= self.motion_confirm_codes <= 100:
            raise ValueError('Motion confirmation must be a bounded code count')
        if not 0 < self.brake_cap_nm:
            raise ValueError('Brake cap must be positive')
        if not 0 < self.brake_deadband_rad_s <= .1:
            raise ValueError('Brake deadband must stay below the speed abort')

    def startup_confirmation_rad(self, pmax):
        if not math.isfinite(pmax) or pmax <= 0:
            raise ValueError('Invalid PMAX for motion confirmation')
        return self.motion_confirm_codes * 2 * pmax / 65535


def position_code(raw_payload):
    """Exact 16-bit encoder code behind a feedback payload, or None."""
    try:
        data = bytes.fromhex(raw_payload)
    except (TypeError, ValueError):
        return None
    return ((data[1] << 8) | data[2]) if len(data) == 8 else None


def symmetric_slew(previous, demand, dt, rate):
    """Legacy law: same rate for build-up and release (kept for comparison)."""
    if not all(math.isfinite(v) for v in (previous, demand, dt, rate)) or dt < 0 or rate <= 0:
        raise ValueError('Invalid symmetric slew input')
    delta = demand - previous
    return previous + max(-rate * dt, min(rate * dt, delta))


def asymmetric_slew(previous, target, dt, up, down):
    """Bounded slew through zero: the reduction to zero is fast, the reverse
    build-up is limited by the build-up rate.

    A reverse torque is force in the opposite direction, not a release. It must
    not inherit the fast release rate. This function returns the piecewise
    result plus the rate/time accounting, so the caller cannot accidentally
    claim a fast rate for a reverse output.
    """
    if not all(math.isfinite(v) for v in (previous, target, dt, up, down)):
        raise ValueError('Invalid asymmetric slew input')
    if dt < 0 or up <= 0 or down <= 0:
        raise ValueError('Invalid slew rate/time')
    if dt == 0:
        return dict(torque=previous, rate_nm_s=0., zero_crossed=False,
                    release_time_s=0., build_time_s=0., reverse=False)
    if previous == 0.:
        shaped = max(-up * dt, min(up * dt, target))
        return dict(torque=shaped, rate_nm_s=up, zero_crossed=False,
                    release_time_s=0., build_time_s=dt, reverse=False)
    if target == 0. or target * previous > 0:
        # Same side (or target zero): growing magnitude uses the build-up rate,
        # shrinking magnitude uses the release rate.
        rate = up if abs(target) > abs(previous) else down
        delta = max(-rate * dt, min(rate * dt, target - previous))
        return dict(torque=previous + delta, rate_nm_s=rate, zero_crossed=False,
                    release_time_s=(dt if rate == down else 0.),
                    build_time_s=(dt if rate == up else 0.), reverse=False)
    # Opposite signs: spend release time to reach zero, then spend the rest of
    # the cycle building in the new direction at the build-up rate.
    release_time = abs(previous) / down
    if dt <= release_time:
        step = down * dt
        shaped = previous - math.copysign(step, previous)
        if shaped * previous < 0:
            shaped = 0.
        return dict(torque=shaped, rate_nm_s=down, zero_crossed=shaped == 0.,
                    release_time_s=dt, build_time_s=0., reverse=False)
    build_time = dt - release_time
    reverse = min(abs(target), up * build_time)
    return dict(torque=math.copysign(reverse, target), rate_nm_s=up,
                zero_crossed=True, release_time_s=release_time,
                build_time_s=build_time, reverse=True)


def shape_torque(*, requested, previous, dt, cap, control, tracking,
                 motion_confirmed, model_velocity, reference_velocity):
    """Shape one host-PD demand into the bounded command for this cycle.

    Order: phase ceiling -> bounded velocity-error brake -> phase ceiling ->
    piecewise asymmetric slew -> final phase ceiling. During the startup phase
    the phase ceiling is `startup_cap_nm`, so every intermediate and the encoded
    output stay inside the stated startup boundary. A reverse output is built at
    the build-up rate even though the reduction to zero is fast.
    """
    values = (requested, previous, dt, cap, model_velocity, reference_velocity)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite shaping input')
    if cap <= 0 or dt < 0:
        raise ValueError('Invalid cap/time')
    if not isinstance(control, CandidateControl):
        raise ValueError('A candidate control profile is required')
    if tracking and not motion_confirmed and control.startup_cap_nm > cap:
        raise ValueError('Startup cap exceeds the tier cap')

    startup_capped = bool(tracking and not motion_confirmed)
    limit = min(cap, control.startup_cap_nm) if startup_capped else cap
    torque = max(-limit, min(limit, requested))

    brake_nm = 0.
    if tracking:
        excess = abs(model_velocity) - max(abs(reference_velocity),
                                           control.brake_deadband_rad_s)
        if excess > 0:
            brake_nm = min(control.brake_cap_nm, control.brake_gain_nm_per_rad_s * excess)
            torque -= math.copysign(brake_nm, model_velocity)
            # The brake must not push the startup-phase composite past its cap.
            torque = max(-limit, min(limit, torque))

    slew = asymmetric_slew(previous, torque, dt,
                           control.slew_up_nm_s, control.slew_down_nm_s)
    shaped = max(-limit, min(limit, slew['torque']))
    return dict(torque=shaped, requested=requested, pre_slew=torque,
                startup_capped=startup_capped, brake_nm=brake_nm,
                rate_nm_s=slew['rate_nm_s'], release=slew['rate_nm_s'] == control.slew_down_nm_s,
                zero_crossed=slew['zero_crossed'], reverse=slew['reverse'],
                release_time_s=slew['release_time_s'],
                build_time_s=slew['build_time_s'], limit_nm=limit)


def classify_motion(codes, resolution_deg, *, observable_deg=0.0,
                    partial_deg=1.0, effective_deg=5.0):
    """Quantization-aware motion class; a few codes are not effective motion."""
    if not math.isfinite(resolution_deg) or resolution_deg <= 0:
        raise ValueError('Invalid encoder resolution')
    codes = [c for c in codes if isinstance(c, int)]
    if not codes:
        return 'NO_CODE_DATA'
    span = (max(codes) - min(codes)) * resolution_deg
    if len(set(codes)) <= 1 or span <= resolution_deg / 2:
        return 'NO_CODE_CHANGE'
    if span < max(observable_deg, 3 * resolution_deg) - 1e-12:
        return 'CODE_JITTER'
    if span < partial_deg:
        return 'OBSERVABLE_START'
    if span < effective_deg:
        return 'PARTIAL_TRACKING'
    return 'EFFECTIVE_TRACKING'


def native_mit_estimate(*, kp, kd, q_des, dq_des, q, dq, tau_ff=0., tmax=54.,
                        command_cap=None):
    """Offline estimate of the total torque a drive-side MIT loop would ask for.

    The host can clamp `tau_ff`, but the drive computes kp*(q_des-q)+kd*(dq_des-dq)
    itself and applies its own internal limit. This estimator makes that explicit;
    it is not a measured torque and not used in production.
    """
    values = (kp, kd, q_des, dq_des, q, dq, tau_ff, tmax)
    if not all(math.isfinite(v) for v in values) or tmax <= 0:
        raise ValueError('Invalid native MIT estimate input')
    total = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
    result = dict(total_nm=total, ff_nm=tau_ff, within_tmax=abs(total) <= tmax,
                  command_cap=command_cap)
    if command_cap is not None:
        if not math.isfinite(command_cap) or command_cap <= 0:
            raise ValueError('Invalid command cap')
        result['ff_alone_within_command_cap'] = abs(tau_ff) <= command_cap
        result['total_within_command_cap'] = abs(total) <= command_cap
        result['host_can_bound_total_from_ff_field'] = False
    result['note'] = ('drive-side MIT total is computed by the drive; limiting the '
                      'FF field does not bound it without changing drive limits')
    return result
