"""Native MIT adapter over the existing takeover/trajectory/stop supervisor."""
import math

from .actuation import FourAxisController
from .openarm_mit import (NativeMITAcceptance, NATIVE_KP, NATIVE_KD, model_command,
                         decode_mit_control_data, estimate_drive_request)
from .holding_bias import HoldingBias, bias_packet


class NativeMITController(FourAxisController):
    execution_backend = 'drive_mit'

    def __init__(self, endpoint, envelopes, acceptance, audit, **options):
        if not isinstance(acceptance, NativeMITAcceptance):
            raise ValueError('Host-PD tickets cannot authorize native MIT')
        if (type(acceptance.holding_bias_acknowledged) is not bool or
            (acceptance.holding_bias_acknowledged and acceptance.joint!=2 and not acceptance.calibration_only)):
            raise ValueError('Explicit holding-bias candidate is J2 only')
        self.holding_bias=HoldingBias() if acceptance.holding_bias_acknowledged else None
        super().__init__(endpoint, envelopes, acceptance, audit, **options)

    def start(self, snapshot, now):
        if self.holding_bias is not None:self.holding_bias.reset()
        # Base start freezes fresh current pose; does NOT command zero.
        super().start(snapshot, now)
        self.audit('NATIVE_MIT_TAKEOVER', request_id=self.acceptance.request_id,
                   joint=self.acceptance.joint,
                   backend=self.execution_backend, drive_kp=NATIVE_KP, drive_kd=NATIVE_KD,
                   tau_ff_nm=0. if self.holding_bias is None else None,
                   holding_bias_enabled=self.holding_bias is not None,
                   holding_bias_settings=vars(self.holding_bias.settings) if self.holding_bias else None,
                   host_total_torque_clamp=False,
                   observed_state_estimate_guard_nm=self.envelopes[self.acceptance.joint-1].torque,
                   output_policy_reference=self.acceptance.output_policy_reference)

    def stop(self,reason,now):
        if self.holding_bias is not None:self.holding_bias.reset()
        return super().stop(reason,now)

    def _torque_frame(self, observation, envelope, target, velocity, now):
        # The inherited supervisor calls this for preloading, gain ramp, hold,
        # and trajectory. There is no host PD torque or host startup shaper.
        p = observation.profile
        scale = min(1., max(0., (now-self.ramp_at[p.joint])/self.ramp))
        payload = model_command(p, q=target, dq=velocity,
                                kp=scale*envelope.kp, kd=scale*envelope.kd)
        bias=None
        if self.holding_bias is not None:
            bias=self.holding_bias.step(target-observation.q,p.sign*observation.velocity,now,
                allow=bool(p.joint in self.enabled and scale>=1.))
            previous=bias['previous_model_bias_nm']
            proposed=bias_packet(p,payload,bias['model_bias_nm'],self.holding_bias.settings.cap_nm)
            before=bias_packet(p,payload,previous,self.holding_bias.settings.cap_nm)
            def estimate(packet):
                return estimate_drive_request(p,packet,position=observation.position,velocity=observation.velocity)
            if abs(estimate(proposed))>envelope.torque and abs(estimate(proposed))>abs(estimate(before)):
                self.holding_bias.value=previous
                bias.update(model_bias_nm=previous,reason='total_estimate_guard')
                payload=before
            else:payload=proposed
        estimated = estimate_drive_request(p, payload,
            position=observation.position, velocity=observation.velocity)
        if abs(estimated) > envelope.torque:
            raise ValueError(f'Native MIT observed-state request estimate exceeds {envelope.torque:g}Nm; not a hard clamp')
        d = decode_mit_control_data(p, payload)
        self.output[p.joint] = dict(at=now, backend='drive_mit',
            target=target, position=observation.q, error=target-observation.q,
            drive_kp=d.kp, drive_kd=d.kd, drive_q=d.q, drive_dq=d.dq,
            encoded_tau_ff_nm=d.tau, observed_state_estimate_nm=estimated,
            holding_bias=bias,
            ramp_scale=scale, host_total_torque_clamp=False)
        return payload

    def trajectory(self, targets, duration, now):
        if self.acceptance.calibration_only:
            raise ValueError('Calibration hold cannot authorize any trajectory')
        if not hasattr(self, 'anchor'):
            raise ValueError('Native takeover has not started')
        expected = list(self.anchor)
        if self.trial_leg == 0:
            expected[self.acceptance.joint-1] += math.radians(10)
        targets = tuple(targets)
        if (self.trial_leg >= 2 or duration != 18. or len(targets) != 4
                or any(not math.isfinite(q) or abs(q-e)>1e-10
                       for q,e in zip(targets, expected))):
            raise ValueError(f'Native first trial: frozen J{self.acceptance.joint} +10deg then anchor, 18s/leg only')
        result = super().trajectory(targets, duration, now)
        self.trial_leg += 1
        return result
