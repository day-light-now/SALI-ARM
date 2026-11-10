"""Explicit finite single-axis automatic acquisition over native MIT.

No executable main or automatic connection. A UI must supply a reviewed plan
and separate whole-route acknowledgement. No simulation weights are used.
"""
from dataclasses import dataclass,replace
import math
import statistics
import uuid

from .actuation import Envelope
from .actuation_io import ControlSession
from .native_actuation import NativeMITController
from .openarm_mit import NativeMITAcceptance,NATIVE_KP,NATIVE_KD,NATIVE_SPEED_ABORT,native_estimate_guard_nm
from .auto_load_acquisition import AutoLoadPlan,AcquisitionLedger
from .clock import now as clock_now


@dataclass(frozen=True)
class AutoLoadTicket(NativeMITAcceptance):
    plan: object = None
    route_acknowledged: bool = False

    def validate(self):
        super().validate()
        if (not isinstance(self.plan,AutoLoadPlan) or self.plan.joint!=self.joint or
            self.route_acknowledged is not True or not self.calibration_only or not self.holding_bias_acknowledged):
            raise ValueError('Automatic acquisition needs its explicit finite route ticket')

    def validate_scope(self,envelopes,axes,runtime,ramp):
        self.validate();p=self.plan
        if p.bias_cap_nm!=(4. if self.joint==1 else 2.) or p.arrival_settle_s!=8. or p.takeover_settle_s!=30.:
            raise ValueError('Automatic bias/settling policy mismatch')
        if tuple(axes)!=(self.joint,) or runtime!=p.total_budget_s or ramp!=1.:
            raise ValueError('Automatic acquisition scope/budget mismatch')
        e=envelopes[self.joint-1]
        if (abs(e.lower-math.radians(p.scan_limits_deg[0]))>1e-12 or
            abs(e.upper-math.radians(p.scan_limits_deg[1]))>1e-12 or
            e.kp!=NATIVE_KP or e.kd!=NATIVE_KD or e.torque!=native_estimate_guard_nm(self.joint) or
            e.speed!=p.reference_speed_rad_s or e.feedback_speed!=NATIVE_SPEED_ABORT or
            e.following_error!=math.radians(4) or e.arrival_error!=math.radians(.5) or e.temperature!=50.):
            raise ValueError('Automatic acquisition cannot alter accepted output/protection')
        for i,e in enumerate(envelopes):
            if i==self.joint-1:continue
            if (e.feedback_speed!=.1 or e.lower!=p.initial_pose_rad[i]-math.radians(2) or
                e.upper!=p.initial_pose_rad[i]+math.radians(2)):
                raise ValueError('Other-axis monitoring scope mismatch')


class AutoLoadController(NativeMITController):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        from .holding_bias import HoldingBias
        self.holding_bias=HoldingBias(replace(self.holding_bias.settings,
            cap_nm=self.acceptance.plan.bias_cap_nm))

    def start(self,snapshot,now):
        p=self.acceptance.plan
        if snapshot.mapping_sha256!=p.mapping_sha256:
            raise ValueError('Plan zero/sign differs from fresh hardware mapping')
        if any(abs(o.q-q)>math.radians(.5) for o,q in zip(snapshot.rows,p.initial_pose_rad)):
            raise ValueError('Initial pose moved after preview; generate a new preview')
        self.next_point=0
        self.arrival_deadline=None
        super().start(snapshot,now)

    def tick(self,snapshot,now):
        if self.state=='TRACK' and self.plan is not None:
            cid,starts,ends,duration,at=self.plan
            if now>=at+duration:
                # End the reference trajectory, hold its final target, and let
                # the bounded bias converge. Actual arrival remains <=0.5deg.
                self.targets=ends;self.plan=None;self.state='SETTLING'
                self.arrival_deadline=at+duration+self.acceptance.plan.arrival_settle_s
                self.audit('AUTO_LOAD_REFERENCE_COMPLETE_SETTLING',command_id=cid,
                    deadline=self.arrival_deadline,arrival_error_deg=.5,
                    bias_cap_nm=self.acceptance.plan.bias_cap_nm)
        super().tick(snapshot,now)
        if self.state=='SETTLING':
            if all(abs(t-o.q)<=e.arrival_error for t,o,e in zip(self.targets,snapshot.rows,self.envelopes)):
                self.state='HOLD';self.arrival_deadline=None
                self.audit('AUTO_LOAD_ACTUAL_ARRIVAL_OBSERVED',point=self.next_point-1)
            elif now>=self.arrival_deadline:
                self.stop('arrival_settle_timeout',now)

    def trajectory(self,targets,duration,now):
        p=self.acceptance.plan
        if self.next_point>=len(p.points) or self.state!='HOLD' or self.plan is not None:
            raise ValueError('Automatic move not due')
        point=p.points[self.next_point];targets=tuple(targets)
        if (targets!=point.target_pose_rad or duration!=point.duration_s or
            self.last_time is None or not 0<=now-self.last_time<=self.max_gap):
            raise ValueError('Move must match the next frozen acquisition point')
        for target,start,e in zip(targets,self.targets,self.envelopes):
            if not e.lower<=target<=e.upper or 1.875*abs(target-start)/duration>e.speed+1e-12:
                raise ValueError('Automatic point outside fixed global bounds or reference speed')
        cid=uuid.uuid4().hex
        self.plan=(cid,self.targets,targets,duration,now)
        self.arrival_deadline=None
        self.state='TRACK';self.next_point+=1
        self.audit('AUTO_LOAD_POINT_ACCEPTED_NOT_SENT',command_id=cid,
                   point=point.index,targets=targets,duration=duration,plan_sha256=p.fingerprint)
        return cid


def session_for_plan(reader,plan,audit,*,acknowledged,clock=clock_now):
    def serializable_audit(event_type,**data):
        if 'evidence' in data and isinstance(data['evidence'].get('plan'),AutoLoadPlan):
            data['evidence']=dict(data['evidence'],plan=data['evidence']['plan'].serializable())
        audit(event_type,**data)
    ticket=AutoLoadTicket(True,True,True,uuid.uuid4().hex,'measured physical scan envelope',
        joint=plan.joint,holding_bias_acknowledged=True,calibration_only=True,
        plan=plan,route_acknowledged=acknowledged)
    envelopes=[]
    for i,q in enumerate(plan.initial_pose_rad,1):
        lo,hi=(map(math.radians,plan.scan_limits_deg) if i==plan.joint else
               (q-math.radians(2),q+math.radians(2)))
        envelopes.append(Envelope(lo,hi,plan.reference_speed_rad_s,
            native_estimate_guard_nm(i) if i==plan.joint else .5,math.radians(4),
            NATIVE_KP if i==plan.joint else 4.,NATIVE_KD if i==plan.joint else .5,
            50.,feedback_speed=NATIVE_SPEED_ABORT if i==plan.joint else .1,arrival_error=math.radians(.5)))
    return ControlSession(reader,envelopes,ticket,serializable_audit,clock,
        controller_factory=AutoLoadController,active_axes=(plan.joint,),max_runtime=plan.total_budget_s,ramp=1.)


def stable_point(samples,joint,mapping_sha256):
    if len(samples)<2 or samples[-1]['at']-samples[0]['at']<1.5:return None
    if any(s['state']!=1 or abs(s['velocity'])>.05 for s in samples):return None
    poses=[[s['pose_rad'][i] for s in samples] for i in range(4)]
    torque=[s['request_model_nm'] for s in samples];bias=[s['bias_model_nm'] for s in samples]
    if (any(max(q)-min(q)>math.radians(.15) for q in poses) or
        max(torque)-min(torque)>.5 or max(bias)-min(bias)>.15):return None
    return dict(accepted=True,joint=joint,mapping_sha256=mapping_sha256,
        pose_rad=[statistics.mean(q) for q in poses],
        holding_request_estimate_model_nm=statistics.mean(torque),
        request_estimate_range_nm=[min(torque),max(torque)],
        bias_model_nm=statistics.mean(bias),sample_count=len(samples),
        window_span_s=samples[-1]['at']-samples[0]['at'],simulated_mass_used=False,
        quantity='drive_request_estimate_not_measured_gravity',not_a_gravity_model=True)


def axis_observations(controller,now):
    """Keep all four axes, including explicit reasons inactive torques aren't labels."""
    axes=[]
    for o in controller.last_snapshot.rows:
        p=o.profile;command=controller.output.get(p.joint,{})
        b=bytes.fromhex(o.raw_payload) if isinstance(o.raw_payload,str) else o.raw_payload
        feedback_tau=(((b[4]&15)<<8)|b[5])*2*p.tmax/4095-p.tmax if len(b)==8 else None
        fresh=0<=now-o.received_at<=controller.max_age
        enabled=o.state==1 and p.joint in controller.enabled
        command_fresh=command.get('at')==controller.last_time
        axes.append(dict(joint=p.joint,angle_rad=o.q,model_velocity_rad_s=p.sign*o.velocity,
            state=o.state,feedback_age_s=now-o.received_at,
            feedback_torque_estimate_model_nm=None if feedback_tau is None else p.sign*feedback_tau,
            request_estimate_model_nm=(p.sign*command['observed_state_estimate_nm']
                if enabled and command_fresh else None),
            target_angle_rad=command.get('target') if enabled and command_fresh else None,
            encoded_ff_model_nm=p.sign*command['encoded_tau_ff_nm'] if enabled and command_fresh else None,
            integral_model_bias_nm=(command.get('holding_bias') or {}).get('model_bias_nm') if enabled and command_fresh else None,
            bias_update_reason=(command.get('holding_bias') or {}).get('reason') if enabled and command_fresh else None,
            stationary_candidate=bool(enabled and fresh and command_fresh and abs(o.velocity)<=.05),
            static_fit_qualified=False,
            exclusion=None if enabled and fresh and command_fresh else 'not_fresh_enabled_controlled_axis',
            driver_temp_c=o.driver_temp,motor_temp_c=o.motor_temp))
    return axes


def run_acquisition(reader,plan,audit,cancel,*,acknowledged,clock=clock_now,sleep=None,progress=None,
                    checkpoint_callback=None):
    import time
    sleep=sleep or time.sleep;progress=progress or (lambda text:None)
    checkpoint_callback=checkpoint_callback or (lambda data:None)
    ledger=AcquisitionLedger(plan);session=None;last=None;stage='takeover';window=[]
    entered=None;deadline=None;settle_notice=None;takeover_notice=False
    result=dict(mode='auto_load_acquisition',joint=plan.joint,plan_sha256=plan.fingerprint,
        source_request_id=uuid.uuid4().hex,readonly_log=str(reader.path),
        bias_cap_nm=plan.bias_cap_nm,arrival_settle_max_s=plan.arrival_settle_s,
        simulated_mass_used=False,final_controller_state='IDLE',fresh_stop_confirmation=False,
        automatic_resume=False,full_range_gravity_model_validated=False)
    try:
        if cancel.is_set():raise ValueError('Cancelled before enable')
        session=session_for_plan(reader,plan,audit,acknowledged=acknowledged,clock=clock)
        session.start();c=session.controller;began=clock()
        audit('AUTO_LOAD_BATCH_STARTED',plan=plan.serializable(),plan_sha256=plan.fingerprint)
        while c.state not in ('DISABLED','UNKNOWN'):
            if cancel.is_set():ledger.halt('operator_stop');session.stop()
            if clock()-began>plan.total_budget_s+.7:
                ledger.halt('campaign_deadline');c.stop('campaign_deadline',clock())
            f=reader.state.feedback.get(plan.joint)
            if f and abs(f['torque_feedback_nm'])>max(4.,native_estimate_guard_nm(plan.joint)):
                ledger.halt('feedback_torque_abort');c.stop('feedback_torque_abort',clock())
            session.cycle();now=clock()
            if c.state=='SETTLING' and settle_notice!=ledger.index:
                settle_notice=ledger.index
                progress(f'点{ledger.index+1}参考轨迹结束，等待实际到位（最多{plan.arrival_settle_s:g}秒，仍要求±0.5°）')
            if c.last_time!=last and c.enabled and c.last_snapshot is not None:
                last=c.last_time;o=c.last_snapshot.rows[plan.joint-1]
                command=c.output.get(plan.joint,{})
                window.append(dict(at=now,state=o.state,velocity=o.velocity,
                    pose_rad=[r.q for r in c.last_snapshot.rows],
                    error=command.get('error',math.inf),
                    request_model_nm=o.profile.sign*command.get('observed_state_estimate_nm',0.),
                    bias_model_nm=(command.get('holding_bias') or {}).get('model_bias_nm',0.)))
                window=[s for s in window if now-s['at']<=2.]
                audit('AUTO_LOAD_OBSERVATION',at=now,stage=stage,axes=axis_observations(c,now),
                      note='Motion/disabled samples are not gravity labels; stable window qualifies selected axis only')
            if c.state=='HOLD':
                if stage=='takeover' and now-began>=2.:
                    recent=[s for s in window if now-s['at']<=.5]
                    if (len(recent)<2 or recent[-1]['at']-recent[0]['at']<.4 or
                        any(s['state']!=1 or abs(s['error'])>math.radians(.5) for s in recent)):
                        if now-began>=plan.takeover_settle_s:
                            ledger.halt('takeover_unstable');c.stop('takeover_unstable',now)
                        elif not takeover_notice:
                            takeover_notice=True
                            progress(f'当前位置接管收敛中（最多{plan.takeover_settle_s:g}秒）；实际稳定后才开始路线')
                    else:stage='next'
                if stage=='moving':
                    ledger.arrived(fresh=True,within_tolerance=True)
                    stage='sampling' if ledger.phase=='SAMPLING' else 'next'
                    entered=now;deadline=now+plan.settle_budget_s;window=[]
                if stage=='sampling':
                    point=stable_point(window,plan.joint,plan.mapping_sha256)
                    if now-entered>=3. and point is not None:
                        ledger.record(point);audit('AUTO_LOAD_STABLE_POINT',**ledger.samples[-1])
                        checkpoint_callback(dict(ledger.checkpoint(),stop_confirmed=False))
                        progress(f'已记录 {len(ledger.samples)} 个姿态点；当前J{plan.joint}={math.degrees(point["pose_rad"][plan.joint-1]):.2f}°')
                        stage='next'
                    elif now>deadline:
                        ledger.halt('point_stability_timeout');c.stop('point_stability_timeout',now)
                if stage=='next':
                    point=ledger.next_move(plan.mapping_sha256)
                    if point is None:c.stop('auto_load_route_complete',now);stage='stopping'
                    else:
                        c.trajectory(point.target_pose_rad,point.duration_s,now);stage='moving';window=[]
                        progress(f'点{point.index+1}/{len(plan.points)} → J{plan.joint} {math.degrees(point.target_pose_rad[plan.joint-1]):.1f}°')
            sleep(.001)
        if ledger.phase=='AWAITING_STOP':ledger.confirm_stop(c.state=='DISABLED')
        elif ledger.phase!='HALTED':ledger.halt(c.reason or 'controller_stopped')
    except Exception as exc:
        result['error']=str(exc);ledger.halt(str(exc))
    finally:
        if session and session.started:
            c=session.controller
            if c.state not in ('DISABLED','UNKNOWN'):
                c.stop('auto_acquisition_cleanup',clock());end=clock()+.7
                while c.state not in ('DISABLED','UNKNOWN') and clock()<end:
                    session.cycle();sleep(.001)
                if c.state not in ('DISABLED','UNKNOWN'):c.state='UNKNOWN'
            result.update(final_controller_state=c.state,fresh_stop_confirmation=c.state=='DISABLED',
                          reason=c.reason,protocol_profiles=[vars(p) for p in c.profiles],
                          last_reference_target_deg=math.degrees(c.targets[plan.joint-1]),
                          last_pre_stop_observed_deg=math.degrees(c.last_snapshot.rows[plan.joint-1].q),
                          last_built_bias_model_nm=(c.output.get(plan.joint,{}).get('holding_bias') or {}).get('model_bias_nm'))
            session.endpoint.closed=True
        elif session:session.endpoint.closed=True
        result['checkpoint']=ledger.checkpoint()
        result['collected_points']=len(ledger.samples)
        result['campaign_complete']=ledger.phase=='COMPLETE' and result['fresh_stop_confirmation']
        result['qualified_points']=ledger.samples if result['fresh_stop_confirmation'] else []
        try:checkpoint_callback(dict(ledger.checkpoint(),stop_confirmed=result['fresh_stop_confirmation']))
        except Exception as exc:result['checkpoint_write_error']=str(exc)
        audit('AUTO_LOAD_BATCH_RESULT',**result)
    return result
