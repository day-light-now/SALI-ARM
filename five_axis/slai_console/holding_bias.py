"""Measured-error integral bias; no robot masses, simulation or gravity model.

This estimates a local holding bias, not gravity. Caps and gains are bench
candidate settings, not motor ratings or a continuous total-torque clamp.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BiasSettings:
    ki_nm_per_rad_s: float = 12.
    cap_nm: float = 2.
    slew_nm_s: float = .5
    speed_freeze_rad_s: float = .05
    error_window_rad: float = math.radians(1.5)
    deadband_rad: float = math.radians(.05)
    max_gap_s: float = .04

    def __post_init__(self):
        values=vars(self).values()
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('Finite positive bias settings required')
        if self.deadband_rad>=self.error_window_rad:
            raise ValueError('Bias deadband must be smaller than error window')


class HoldingBias:
    def __init__(self,settings=BiasSettings()):
        self.settings=settings;self.reset()

    def reset(self):
        self.value=0.;self.last=None;self.faulted=False

    def step(self,error,model_velocity,now,*,allow):
        try:
            if self.faulted:raise ValueError('Bias fault is latched')
            if not all(math.isfinite(v) for v in (error,model_velocity,now)):
                raise ValueError('Nonfinite holding bias feedback')
            if type(allow) is not bool:raise ValueError('Explicit bias update state required')
            s=self.settings
            dt=0. if self.last is None else now-self.last
            if self.last is not None and not 0<dt<=s.max_gap_s:
                raise ValueError('Bias clock regressed or missed deadline')
            self.last=now
            if not allow:reason='preload_or_gain_ramp'
            elif abs(model_velocity)>s.speed_freeze_rad_s:reason='feedback_speed'
            elif abs(error)>s.error_window_rad:reason='error_window'
            elif abs(error)<=s.deadband_rad:reason='deadband'
            else:reason='integrating'
            previous=self.value
            if reason=='integrating':
                delta=s.ki_nm_per_rad_s*error*dt
                delta=max(-s.slew_nm_s*dt,min(s.slew_nm_s*dt,delta))
                # Store the capped output itself: no hidden integral windup.
                self.value=max(-s.cap_nm,min(s.cap_nm,self.value+delta))
                if self.value==previous and abs(self.value)==s.cap_nm:reason='bias_cap'
            return dict(model_bias_nm=self.value,previous_model_bias_nm=previous,
                        error_rad=error,reason=reason,update_dt_s=dt,
                        not_a_gravity_estimate=True)
        except Exception:
            self.value=0.;self.faulted=True
            raise


def bias_packet(profile,original,model_bias_nm,cap_nm):
    """Replace only the native tau_ff bits, using existing measured profile.

    Quantization is included in the FF bound. Zero request preserves the
    ordinary native zero-FF payload (which has a small quantization residual).
    """
    if (profile.joint not in (1,2,3,4) or type(original) is not bytes or len(original)!=8 or
        original[:7]==b'\xff'*7):
        raise ValueError('Holding bias requires J1-J4 ordinary MIT')
    if (not all(math.isfinite(v) for v in (model_bias_nm,cap_nm)) or
        not 0<cap_nm<=profile.tmax or abs(model_bias_nm)>cap_nm):
        raise ValueError('Bias request outside bound')
    quantum=2*profile.tmax/4095
    low=math.ceil((profile.tmax-cap_nm)/quantum)
    high=math.floor((profile.tmax+cap_nm)/quantum)
    if low>high:raise ValueError('No representable bias inside bound')
    raw=math.floor((profile.sign*model_bias_nm+profile.tmax)/quantum)
    raw=max(low,min(high,raw))
    packet=bytearray(original)
    packet[6]=(packet[6]&0xf0)|(raw>>8);packet[7]=raw&255
    return bytes(packet)
