# Copyright 2025 Enactic, Inc.
# Licensed under the Apache License, Version 2.0.
# https://www.apache.org/licenses/LICENSE-2.0
# Adapted 2026-11-02: Python, runtime motor ranges, strict validation and SLAI mapping.
"""OpenArm's drive-side MIT packet encoder, with no transport or host PD.

Source: enactic/openarm_can f340d4b808fb177e1f297af54eb55fd51c6c7c10,
src/openarm/damiao_motor/dm_motor_control.cpp::pack_mit_control_data,
double_to_uint. The field quantizer truncates exactly as the official C++ code.
We reject out-of-range inputs before encoding instead of silently clipping them.
"""
from dataclasses import dataclass, asdict
import math

OFFICIAL_REPOSITORY = 'https://github.com/enactic/openarm_can'
OFFICIAL_COMMIT = 'f340d4b808fb177e1f297af54eb55fd51c6c7c10'
# SLAI bench tuning, not gains copied from a different OpenArm mechanism.
NATIVE_KP = 160.
NATIVE_KD = 5.
# User explicitly selected 1rad/s on 2026-11-02. This is measured-speed
# abort only; reference velocity and legacy controllers are unchanged.
NATIVE_SPEED_ABORT = 1.


@dataclass(frozen=True)
class MITParam:
    kp: float
    kd: float
    q: float
    dq: float
    tau: float


def _uint(value, lo, hi, bits):
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError('MIT field outside protocol range; no silent clipping')
    return int((value-lo)/(hi-lo)*((1 << bits)-1))


def pack_mit_control_data(profile, param):
    """Pack native motor coordinates using the official OpenArm layout."""
    if profile.mode != 1:
        raise ValueError('Native MIT requires CTRL_MODE=1')
    if not isinstance(param, MITParam):
        raise ValueError('Explicit MITParam required')
    kp = _uint(param.kp, 0., 500., 12)
    kd = _uint(param.kd, 0., 5., 12)
    q = _uint(param.q, -profile.pmax, profile.pmax, 16)
    dq = _uint(param.dq, -profile.vmax, profile.vmax, 12)
    tau = _uint(param.tau, -profile.tmax, profile.tmax, 12)
    return bytes([q >> 8, q & 255, dq >> 4, ((dq & 15) << 4) | (kp >> 8),
                  kp & 255, kd >> 4, ((kd & 15) << 4) | (tau >> 8), tau & 255])


def decode_mit_control_data(profile, payload):
    if not isinstance(payload, bytes) or len(payload) != 8:
        raise ValueError('Eight-byte MIT payload required')
    b = payload
    raw = ((b[0] << 8) | b[1], (b[2] << 4) | (b[3] >> 4),
           ((b[3] & 15) << 8) | b[4], (b[5] << 4) | (b[6] >> 4),
           ((b[6] & 15) << 8) | b[7])
    q = raw[0]*2*profile.pmax/65535-profile.pmax
    dq = raw[1]*2*profile.vmax/4095-profile.vmax
    kp = raw[2]*500/4095
    kd = raw[3]*5/4095
    tau = raw[4]*2*profile.tmax/4095-profile.tmax
    return MITParam(kp, kd, q, dq, tau)


def model_command(profile, *, q, dq, kp, kd, tau_ff=0.):
    """Map SLAI joint coordinates into the five native drive-side fields."""
    if abs(dq) > .02:
        raise ValueError('Reference velocity exceeds 0.02rad/s')
    param = MITParam(kp, kd, profile.zero+profile.sign*q,
                     profile.sign*dq, profile.sign*tau_ff)
    packet = pack_mit_control_data(profile, param)
    # A 12-bit velocity field with VMAX=45 has ~0.02198rad/s steps.
    # Official floor quantization can turn a 0.018 reference into 0.03297.
    # Keep the official packer unchanged, but constrain the encoded reference
    # to a representable value inside this bench's +/-0.02rad/s envelope.
    if abs(decode_mit_control_data(profile, packet).dq) > .02:
        low = math.ceil((profile.vmax-.02)*4095/(2*profile.vmax))
        high = math.floor((profile.vmax+.02)*4095/(2*profile.vmax))
        if low > high:
            raise ValueError('No velocity code fits the reference envelope')
        raw = max(low, min(high, _uint(param.dq,-profile.vmax,profile.vmax,12)))
        bounded = (raw+.25)*2*profile.vmax/4095-profile.vmax
        packet = pack_mit_control_data(profile, MITParam(kp,kd,param.q,bounded,param.tau))
    if packet[:7] == b'\xff'*7:
        raise ValueError('Reserved special-command pattern')
    return packet


def estimate_drive_request(profile, payload, *, position, velocity):
    """Estimate at one observed state, NOT a continuous/hardware torque bound."""
    if not all(math.isfinite(v) for v in (position, velocity)):
        raise ValueError('Finite motor feedback required')
    d = decode_mit_control_data(profile, payload)
    return d.kp*(d.q-position)+d.kd*(d.dq-velocity)+d.tau


@dataclass(frozen=True)
class NativeMITAcceptance:
    """Separate explicit authority; never inferred from a host-PD trial ticket.

    The acknowledgement changes the meaning of the old host 2Nm limit: it is
    now a feedback-state estimate guard, NOT a continuous driver torque clamp.
    A UI must explain this before constructing the acknowledged ticket.
    """
    support_confirmed: bool
    clearance_confirmed: bool
    native_output_acknowledged: bool
    request_id: str
    output_policy_reference: str
    execution_backend: str = 'drive_mit'

    def validate(self):
        if not all(v is True for v in (self.support_confirmed,
                                      self.clearance_confirmed,
                                      self.native_output_acknowledged)):
            raise ValueError('Native MIT needs explicit support/clearance/output acknowledgement')
        if not self.request_id or not self.output_policy_reference:
            raise ValueError('Native MIT request and output-policy reference required')
        if self.execution_backend != 'drive_mit':
            raise ValueError('Wrong execution backend')

    def validate_scope(self, envelopes, axes, runtime, ramp):
        if tuple(axes) != (4,) or runtime != 45. or ramp != 1.:
            raise ValueError('First native MIT trial is J4-only, 45s, 1s gain ramp')
        e = envelopes[3]
        if (e.kp != NATIVE_KP or e.kd != NATIVE_KD or e.torque != 2.
                or e.speed > .02 or e.feedback_speed is None or e.feedback_speed > NATIVE_SPEED_ABORT
                or e.following_error > math.radians(4)+1e-12
                or e.upper-e.lower > math.radians(18)+1e-12
                or e.temperature > 50. or e.arrival_error is None
                or e.arrival_error > math.radians(.5)+1e-12):
            raise ValueError('Native MIT scope/gain/estimate guard mismatch')
        if any(e.feedback_speed is not None and e.feedback_speed > .1
               for e in envelopes[:3]):
            raise ValueError('Other-axis observation thresholds cannot expand')


def command_description(profile, payload):
    return dict(backend='drive_mit', source=OFFICIAL_REPOSITORY,
                commit=OFFICIAL_COMMIT,
                motor_fields=asdict(decode_mit_control_data(profile, payload)),
                host_total_torque_clamp=False,
                note='tau is feedforward only; PD runs inside the motor drive')
