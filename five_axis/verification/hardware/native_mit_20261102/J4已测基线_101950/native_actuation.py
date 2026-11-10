"""Native MIT adapter over the existing takeover/trajectory/stop supervisor."""
import math

from .actuation import FourAxisController
from .openarm_mit import (NativeMITAcceptance, NATIVE_KP, NATIVE_KD, model_command,
                         decode_mit_control_data, estimate_drive_request)


class NativeMITController(FourAxisController):
    execution_backend = 'drive_mit'

    def __init__(self, endpoint, envelopes, acceptance, audit, **options):
        if not isinstance(acceptance, NativeMITAcceptance):
            raise ValueError('Host-PD tickets cannot authorize native MIT')
        super().__init__(endpoint, envelopes, acceptance, audit, **options)

    def start(self, snapshot, now):
        # Base start freezes fresh current pose; does NOT command zero.
        super().start(snapshot, now)
        self.audit('NATIVE_MIT_TAKEOVER', request_id=self.acceptance.request_id,
                   backend=self.execution_backend, drive_kp=NATIVE_KP, drive_kd=NATIVE_KD,
                   tau_ff_nm=0., host_total_torque_clamp=False,
                   observed_state_estimate_guard_nm=2.,
                   output_policy_reference=self.acceptance.output_policy_reference)

    def _torque_frame(self, observation, envelope, target, velocity, now):
        # The inherited supervisor calls this for preloading, gain ramp, hold,
        # and trajectory. There is no host PD torque or host startup shaper.
        p = observation.profile
        scale = min(1., max(0., (now-self.ramp_at[p.joint])/self.ramp))
        payload = model_command(p, q=target, dq=velocity,
                                kp=scale*envelope.kp, kd=scale*envelope.kd)
        estimated = estimate_drive_request(p, payload,
            position=observation.position, velocity=observation.velocity)
        if abs(estimated) > envelope.torque:
            raise ValueError('Native MIT observed-state request estimate exceeds 2Nm; not a hard clamp')
        d = decode_mit_control_data(p, payload)
        self.output[p.joint] = dict(at=now, backend='drive_mit',
            target=target, position=observation.q, error=target-observation.q,
            drive_kp=d.kp, drive_kd=d.kd, drive_q=d.q, drive_dq=d.dq,
            encoded_tau_ff_nm=d.tau, observed_state_estimate_nm=estimated,
            ramp_scale=scale, host_total_torque_clamp=False)
        return payload

    def trajectory(self, targets, duration, now):
        if not hasattr(self, 'anchor'):
            raise ValueError('Native takeover has not started')
        expected = list(self.anchor)
        if self.trial_leg == 0:
            expected[3] += math.radians(10)
        targets = tuple(targets)
        if (self.trial_leg >= 2 or duration != 18. or len(targets) != 4
                or any(not math.isfinite(q) or abs(q-e)>1e-10
                       for q,e in zip(targets, expected))):
            raise ValueError('Native first trial: frozen J4 +10deg then anchor, 18s/leg only')
        result = super().trajectory(targets, duration, now)
        self.trial_leg += 1
        return result
