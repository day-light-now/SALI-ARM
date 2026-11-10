"""Bounded gravity candidate and MIT field builder, OFFLINE ONLY.

No serial imports, transport, live entry or permission to enable motors.
The current mass/COM/base orientation are provisional. A candidate packet is
data for replay, never a hardware command authorization.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from verification.gravity_model import GravityModel
from .core import ROOT
from .openarm_mit import decode_mit_control_data


@dataclass(frozen=True)
class GravityBounds:
    # Offline starting hypotheses, NOT identified hardware limits or gains.
    fraction: float = .5
    cap_nm: float = 2.
    slew_nm_s: float = 1.
    ramp_s: float = 1.
    max_gap_s: float = .1

    def __post_init__(self):
        if not all(math.isfinite(x) for x in
                   (self.fraction,self.cap_nm,self.slew_nm_s,self.ramp_s,self.max_gap_s)):
            raise ValueError('Finite gravity bounds required')
        if not (0 <= self.fraction <= 1 and self.cap_nm > 0 and
                self.slew_nm_s > 0 and self.ramp_s > 0 and self.max_gap_s > 0):
            raise ValueError('Invalid gravity bounds')


class GravityCandidate:
    """One selected J2, one replay session; compute from all four measured q.

    Failed/stale steps latch failure, clear output and require a new instance.
    Hardware fault handling must use FD, never this candidate's slew limiter.
    """
    def __init__(self, model, bounds=GravityBounds(), *, joint=2):
        if type(joint) is not int or joint != 2:
            raise ValueError('Gravity candidate currently scoped to J2 only')
        self.model,self.bounds,self.joint=model,bounds,joint
        self.started=None;self.last=None;self.previous=0.;self.failed=False

    def step(self, q, now, received_at):
        try:
            if self.failed:
                raise ValueError('Gravity candidate fault is latched')
            if (len(q)!=4 or len(received_at)!=4 or
                not all(math.isfinite(x) for x in (*q,now,*received_at))):
                raise ValueError('Four finite measured positions/timestamps required')
            b=self.bounds
            if any(not 0 <= now-t <= b.max_gap_s for t in received_at):
                raise ValueError('Stale or future gravity feedback')
            if self.last is not None and not 0 < now-self.last <= b.max_gap_s:
                raise ValueError('Gravity clock regressed or missed deadline')
            raw=float(self.model.evaluate(q)[0][self.joint-1])
            if not math.isfinite(raw):
                raise ValueError('Nonfinite gravity model output')
            if self.started is None:
                self.started=now;self.last=now
                return dict(raw_gravity_nm=raw,model_ff_nm=0.,ramp_scale=0.,
                            clipped=False,hardware_authorized=False)
            dt=now-self.last
            ramp=min(1.,(now-self.started)/b.ramp_s)
            requested=b.fraction*ramp*raw
            bounded=max(-b.cap_nm,min(b.cap_nm,requested))
            ff=max(self.previous-b.slew_nm_s*dt,
                   min(self.previous+b.slew_nm_s*dt,bounded))
            ff=max(-b.cap_nm,min(b.cap_nm,ff))
            self.previous=ff;self.last=now
            return dict(raw_gravity_nm=raw,model_ff_nm=ff,ramp_scale=ramp,
                        clipped=abs(requested)>b.cap_nm,hardware_authorized=False)
        except Exception:
            self.failed=True;self.previous=0.
            raise


def provisional_model():
    """Use canonical URDF + existing MuJoCo J3 fixed +pi placement.

    +Y-down is documented from CAD, not independently measured installation.
    No URDF, mass data, mapping or simulation artifact is mutated here.
    """
    urdf=ROOT/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
    mass=ROOT/'verification/gravity_preparation/provisional_gravity.json'
    placement=ROOT/'verification/impedance_demo/model.py'
    report=json.loads(mass.read_text(encoding='utf-8'))
    if report['status']!='ESTIMATE_ONLY_NOT_FOR_HARDWARE':
        raise ValueError('Re-review changed mass provenance before using this loader')
    model=GravityModel(urdf,report['links'],report['gravity_base_m_s2'],
                       fixed_joint_offsets={'slaihand_joint3':math.pi})
    if model.names!=[f'slaihand_joint{i}' for i in range(1,5)]:
        raise ValueError('Unexpected gravity joint ordering')
    evidence=dict(status=report['status'],gravity_base_m_s2=report['gravity_base_m_s2'],
        fixed_joint_offsets_rad={'slaihand_joint3':math.pi},
        hardware_authorized=False,assumptions=report['assumptions'],
        sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (urdf,mass,placement)})
    return model,evidence


def candidate_mit_packet(profile, original, model_ff_nm, cap_nm):
    """Offline: replace only tau_ff bits, sign-map and enforce encoded FF cap.

    PD fields are unchanged. A bounded FF field is not a bounded total torque.
    This API rejects J1/J3/J4/J5 and never opens any hardware interface.
    """
    if profile.joint!=2 or len(original)!=8 or original[:7]==b'\xff'*7:
        raise ValueError('Expected a J2 ordinary MIT candidate packet')
    if not all(math.isfinite(v) for v in (model_ff_nm,cap_nm)) or not 0<cap_nm<=profile.tmax:
        raise ValueError('Invalid feedforward bound')
    if abs(model_ff_nm)>cap_nm:
        raise ValueError('Requested FF outside candidate bound')
    quantum=2*profile.tmax/4095
    low=math.ceil((profile.tmax-cap_nm)/quantum)
    high=math.floor((profile.tmax+cap_nm)/quantum)
    if low>high:
        raise ValueError('No representable FF value inside bound')
    # Match official truncation; zero request retains its existing quantization
    # residual (there is no exact zero in this signed 12-bit range).
    raw=max(low,min(high,math.floor((profile.sign*model_ff_nm+profile.tmax)/quantum)))
    packet=bytearray(original)
    packet[6]=(packet[6]&0xf0)|(raw>>8);packet[7]=raw&255
    d=decode_mit_control_data(profile,bytes(packet))
    if abs(d.tau)>cap_nm+1e-12:
        raise ValueError('Encoded FF exceeded bound')
    return bytes(packet)
