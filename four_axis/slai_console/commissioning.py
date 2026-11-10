"""Pure commissioning policy and shadow encoders. No serial/transport imports.

Candidate CAN bytes in this module are data for inspection, never a transmit
capability. Real output remains blocked by core.CommandGateway's R2 rule.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import struct
from .core import ACTIVE, LIMITS, ROOT


@dataclass(frozen=True)
class JointCommand:
    pos: float
    vel: float = 0.
    kp: float = 0.
    kd: float = 0.
    tau_ff: float = 0.
    speed_limit: float = .02


@dataclass(frozen=True)
class Profile:
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
        if self.joint not in ACTIVE or self.mode not in (1,2):
            raise ValueError('Only J1–J4 MIT/PV shadow profiles supported')
        if self.sign not in (-1,1) or not math.isfinite(self.zero):
            raise ValueError('Invalid software mapping')
        if not 0<=self.master_id<=0x7FF or not 0<=self.timeout<=0xFFFFFFFF:
            raise ValueError('Invalid ID/timeout')
        if not all(math.isfinite(v) and v>0 for v in (self.pmax,self.vmax,self.tmax)):
            raise ValueError('Invalid live protocol ranges')


@dataclass(frozen=True)
class Observation:
    profile: Profile
    position: float
    velocity: float
    state: int
    received_at: float
    driver_temp: float
    motor_temp: float
    raw_payload: str

    @property
    def q(self):
        return self.profile.sign*(self.position-self.profile.zero)


@dataclass(frozen=True)
class Snapshot:
    session_id: str
    observed_at: float
    rows: tuple
    mapping_sha256: str
    readonly_log: str
    clock: str='legacy_or_fixture'

    def serializable(self):
        result=asdict(self)
        result.update(schema=1,source='existing_readonly_worker',actuation_tx=0,
            model_limits_only_deg=LIMITS,
            physical_envelopes=dict(J1='historical mode4 only: 2deg/.02rad_s/1.4961A; limit issue unresolved',
                J2='historical mode4 only: 2deg/.02rad_s/1.4961A; ID persistence pending',
                J3='not accepted',J4='not accepted'),
            limitations='No unique hardware serial identity read; no mechanical safe-range measurement')
        return result


def capture(sample,mapping,now):
    if not sample.get('session_id') or set(sample.get('rows',{}))!=set(ACTIVE):
        raise ValueError('Need one identified read-only session with four fresh axes')
    rows=[]
    for i in ACTIVE:
        f=sample['rows'][i]
        r=f['regs']
        if r[8]!=i:
            raise ValueError('CAN identity mismatch')
        p=Profile(i,r[10],r[7],r[9],r[21],r[22],r[23],int(mapping[i]['hardware_sign']),mapping[i]['zero_offset_rad'])
        at=float(f.get('received_at',float(sample['at'])-float(f['age'])))
        values=(f['position_rad'],f['velocity_rad_s'],at,f['driver_temp_c'],f['motor_temp_c'])
        if not all(math.isfinite(v) for v in values) or not 0<=now-at<=.35:
            raise ValueError(f'J{i}: missing/nonfinite/older than 350ms')
        payload=bytes.fromhex(f['payload'])
        if len(payload)!=8 or payload[0]&15!=i or payload[0]>>4!=f['state_code']:
            raise ValueError('Raw payload disagrees with parsed state')
        if f['state_code']!=0:
            raise ValueError(f'J{i}: shadow preparation requires disabled state')
        rows.append(Observation(p,f['position_rad'],f['velocity_rad_s'],f['state_code'],at,
                                f['driver_temp_c'],f['motor_temp_c'],f['payload']))
    path=ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml'
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    return Snapshot(sample['session_id'],now,tuple(rows),digest,sample.get('readonly_log','unknown'),sample.get('clock','legacy_or_fixture'))


def quantize(value,lo,hi,bits):
    if not math.isfinite(value) or not lo<=value<=hi:
        raise ValueError('Command outside protocol range; no silent clipping')
    raw=round((value-lo)*((1<<bits)-1)/(hi-lo))
    decoded=raw*(hi-lo)/((1<<bits)-1)+lo
    return raw,decoded


def encode_shadow(p,c):
    if not all(math.isfinite(v) for v in asdict(c).values()):
        raise ValueError('Nonfinite command')
    if not 0<c.speed_limit<=p.vmax or abs(c.vel)>c.speed_limit:
        raise ValueError('Velocity exceeds command limit')
    position=p.zero+p.sign*c.pos
    if not -p.pmax<=position<=p.pmax:
        raise ValueError('Position outside live PMAX')
    if p.mode==2:
        # PV's second float is a positive speed bound, not signed target velocity.
        if c.kp!=0 or c.kd!=0 or c.tau_ff!=0 or c.vel!=0:
            raise ValueError('PV does not support MIT gains/torque or signed velocity feedforward')
        payload=struct.pack('<ff',position,c.speed_limit)
        decoded_position,decoded_speed=struct.unpack('<ff',payload)
        return dict(candidate_can_id=p.joint+0x100,candidate_payload=payload.hex(' '),
            decoded_position=decoded_position,decoded_speed_limit=decoded_speed,
            effort_limit_supported=False,source='shadow_not_transmitted')
    pos,_=quantize(position,-p.pmax,p.pmax,16)
    vel,_=quantize(p.sign*c.vel,-p.vmax,p.vmax,12)
    kp,_=quantize(c.kp,0,500,12)
    kd,_=quantize(c.kd,0,5,12)
    tau,decoded_tau=quantize(p.sign*c.tau_ff,-p.tmax,p.tmax,12)
    payload=bytes([pos>>8,pos&255,vel>>4,((vel&15)<<4)|(kp>>8),kp&255,
                   kd>>4,((kd&15)<<4)|(tau>>8),tau&255])
    return dict(candidate_can_id=p.joint,candidate_payload=payload.hex(' '),
        decoded_tau_ff_nm=decoded_tau,zero_tau_is_exact=decoded_tau==0,
        torque_quantum_nm=2*p.tmax/4095,source='shadow_not_transmitted')


class GlobalSupervisor:
    """Sticky diagnostic latch. It grants no motion or torque authority."""
    def __init__(self):
        self.reason=None

    def check(self,snapshot,now,session_id,expected_profiles):
        if self.reason:
            return self.reason
        if snapshot.session_id!=session_id:
            self.reason='SESSION_CHANGED'
        elif len(snapshot.rows)!=4 or tuple(o.profile for o in snapshot.rows)!=expected_profiles:
            self.reason='PROFILE_CHANGED'
        else:
            for o in snapshot.rows:
                age=now-o.received_at
                if not 0<=age<=.35:
                    self.reason=f'STALE_J{o.profile.joint}'
                elif o.state!=0:
                    self.reason=f'STATE_J{o.profile.joint}_{o.state}'
                elif max(o.driver_temp,o.motor_temp)>=60:
                    self.reason=f'TEMPERATURE_J{o.profile.joint}'
                if self.reason:
                    break
        return self.reason


def readiness(snapshot):
    blocks=['ACTUATION_ADAPTER_NOT_IMPLEMENTED','PASSIVE_CATCH_NOT_ACCEPTED',
            'FAULT_STOP_AND_WATCHDOG_ACTION_NOT_ACCEPTED','CONTROL_BUS_CYCLE_NOT_MEASURED',
            'PHYSICAL_ENVELOPES_NOT_ACCEPTED','J1_MECHANICAL_LIMIT_UNRESOLVED',
            'J2_ID_POWER_CYCLE_RECORD_PENDING','STAGGERED_TAKEOVER_NOT_ACCEPTED']
    for row in snapshot.rows:
        if row.profile.timeout==0:
            blocks.append(f'J{row.profile.joint}_WATCHDOG_DISABLED')
        if row.profile.mode==2:
            blocks.append(f'J{row.profile.joint}_PV_EFFORT_LIMIT_NOT_ACCEPTED')
    return dict(motion_permitted=False,blocks=blocks)


class ShadowEngine:
    """Capture current positions, compute at host clock, record only.

No inverse dynamics prediction is claimed; targets are a frozen current-pose
candidate, with zero gains/feedforward. Encoded zero has quantization residue.
"""
    def __init__(self,snapshot):
        if tuple(o.profile.joint for o in snapshot.rows)!=ACTIVE:
            raise ValueError('Exactly four ordered axes required; J5 excluded')
        self.initial=snapshot
        self.profiles=tuple(o.profile for o in snapshot.rows)
        self.targets=tuple(o.q for o in snapshot.rows)
        self.supervisor=GlobalSupervisor()
        self.last_time=None
        self.last_source={}
        self.observed_updates={i:[] for i in ACTIVE}
        self.cycle_ms=[]
        self.ages_80ms_exceeded=0
        self.cycles=0

    def step(self,snapshot,now):
        if snapshot.clock!=self.initial.clock:
            self.supervisor.reason='CLOCK_DOMAIN_CHANGED'
        if snapshot.mapping_sha256!=self.initial.mapping_sha256:
            self.supervisor.reason='MAPPING_CHANGED'
        if self.last_time is not None:
            elapsed=now-self.last_time
            if elapsed<0 or elapsed>.050:
                self.supervisor.reason='HOST_CLOCK_OR_LOOP_DELAY'
            self.cycle_ms.append(elapsed*1000)
        self.last_time=now
        fault=self.supervisor.check(snapshot,now,self.initial.session_id,self.profiles)
        self.cycles+=1
        ages=[now-o.received_at for o in snapshot.rows]
        if any(a>.08 for a in ages):
            self.ages_80ms_exceeded+=1
        for o in snapshot.rows:
            i=o.profile.joint
            previous=self.last_source.get(i)
            if previous is not None and o.received_at<previous:
                self.supervisor.reason=f'OUT_OF_ORDER_J{i}'
            if previous is not None and o.received_at>previous:
                self.observed_updates[i].append((o.received_at-previous)*1000)
            self.last_source[i]=o.received_at
        fault=self.supervisor.reason or fault
        errors=[target-o.q for target,o in zip(self.targets,snapshot.rows)]
        if not fault:
            for i,error in zip(ACTIVE,errors):
                if abs(error)>math.radians(2):
                    self.supervisor.reason=f'SHADOW_REFERENCE_DEPARTURE_J{i}'
                    fault=self.supervisor.reason
                    break
        frames=[] if fault else [dict(joint=p.joint,command=asdict(JointCommand(q)),
            **encode_shadow(p,JointCommand(q))) for p,q in zip(self.profiles,self.targets)]
        return dict(kind='shadow_cycle',at_monotonic=now,cycle=self.cycles,global_latch=fault,
            actuation_tx=0,hardware_writes=0,feedback_age_s=ages,
            candidate_reference_error_rad=errors,reference_error_limit_is_shadow_only=True,
            hypothetical_motion_fresh_80ms=bool(ages) and all(0<=a<=.08 for a in ages),
            frames=frames,readiness=readiness(snapshot))

    def summary(self):
        def distribution(values):
            values=sorted(values)
            return dict(count=len(values),minimum=min(values),maximum=max(values),
                p95=values[min(len(values)-1,math.ceil(len(values)*.95)-1)]) if values else dict(count=0)
        return dict(cycles=self.cycles,global_latch=self.supervisor.reason,actuation_tx=0,
            host_cycle_ms=distribution(self.cycle_ms),feedback_update_ms={i:distribution(v) for i,v in self.observed_updates.items()},
            cycles_exceeding_80ms_feedback_age=self.ages_80ms_exceeded,
            bus_four_axis_control_cycle='NOT_MEASURED',motion_permitted=False)
