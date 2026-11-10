"""Operator-controlled supported J4 current-pose commissioning window.

Opening/importing never connects or enables. Read-only connection is explicit;
trial start is a separate explicit action. Full-arm and trajectory authority
remain blocked; J5 never enters the transport. No parameter writes/auto-clear.

The operator names a host PD tier in the immutable request: Kp=4 (default),
Kp=12 (0.3Nm candidate), Kp=32 (explicit 1Nm/2Nm candidates), or the bounded
start/brake candidates (1Nm/2Nm with a startup ceiling+window, a dissipative
velocity-error brake and a faster bounded release). Nondefault candidates require
extra acknowledgement. Nothing here tunes gains, escalates a tier, changes the
selected bound or retries after a stop. The 1Nm/2Nm tiers use Kp=32/Kd=0.5 with
0.5/1Nm/s host torque slew; the brake candidates use the frozen CandidateControl.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

from .core import ROOT, LIMITS
from .actuation import (Envelope, LimitedJ4Trial, LimitedJ4RoundTrip,
                        DEFAULT_PD_TIER, VERIFIED_PD_TIER, STIFFER_PD_TIER,
                        ONE_NM_PD_TIER, TWO_NM_PD_TIER,
                        ONE_NM_BRAKE_TIER, TWO_NM_BRAKE_TIER,
                        pd_tier, encoded_cap_torque_nm, motion_summary)
from .j4_control import position_code
from .actuation_io import ControlSession
from .clock import now as clock_now
from .watchdog_check import make_event_writer
from four_axis_monitor import Reader, State, REGS, load_mapping, usb_packet


WATCHDOG_REPORT=ROOT/'tmp/watchdog_check/20261101_131809_aee5b0.json'
HOLD_REPORT=ROOT/'tmp/live_trial/20261101_134732_a325bf.json'
# Post-stop observation: after the four-axis DISABLED confirmation only, a
# bounded read-only CC poll of J4 is allowed. In UNKNOWN no query is sent.
POST_STOP_WINDOW_S=.30
POST_STOP_MAX_POLLS=32
POST_STOP_MIN_COVERAGE_S=.01


@dataclass(frozen=True)
class TrialRequest:
    support: bool
    clearance: bool
    risk_ack: bool
    request_id: str
    mode: str = 'hold'
    tier: str = DEFAULT_PD_TIER
    tier_ack: bool = False


def stop_reason_code(reason,metrics):
    """Stable code that separates a stall from a generic following error."""
    if reason=='round_trip_complete':return 'ROUND_TRIP_COMPLETE'
    if reason=='trial_duration_complete':return 'HOLD_COMPLETE'
    if reason=='operator_stop':return 'OPERATOR_STOP'
    if isinstance(reason,str) and 'Startup no breakaway' in reason:
        return 'STARTUP_NO_BREAKAWAY'
    if isinstance(reason,str) and 'feedback speed' in reason:
        return 'FEEDBACK_SPEED_ABORT'
    if isinstance(reason,str) and 'feedback torque' in reason:
        return 'FEEDBACK_TORQUE_ABORT'
    if isinstance(reason,str) and 'Trajectory arrival timeout' in reason:
        return 'TRAJECTORY_ARRIVAL_TIMEOUT'
    if isinstance(reason,str) and 'USB cycle write deadline' in reason:
        return 'USB_WRITE_DEADLINE'
    if reason=='Following error':
        motion=(metrics or {}).get('motion_class')
        if motion=='CODE_JITTER':return 'FOLLOWING_ERROR_CODE_JITTER'
        if motion=='OBSERVABLE_START':return 'FOLLOWING_ERROR_OBSERVABLE_START'
        return {'OUTPUT_AT_ENCODED_CAP_NO_MOTION':'FOLLOWING_ERROR_AT_ENCODED_CAP_NO_MOTION',
                'COMMAND_BELOW_CAP_NO_MOTION':'FOLLOWING_ERROR_BELOW_CAP_NO_MOTION',
                'NO_OUTPUT_RECORDED':'FOLLOWING_ERROR_NO_OUTPUT_RECORDED'
                }.get((metrics or {}).get('assessment'),'FOLLOWING_ERROR')
    return 'STOPPED:'+str(reason)


def next_step_hint(metrics, stop_code=None, post_stop=None):
    """Mechanical/load checks for the next operator decision. Never an auto action."""
    if not metrics or not metrics.get('enabled_samples'):
        return '未进入使能采样，无法评价输出与位移；先检查预检、使能确认和日志。'
    if stop_code=='STARTUP_NO_BREAKAWAY':
        return ('起动窗口内未确认位移，已在编码峰值{:.3f}Nm以内受控停止：先检查支撑/夹紧、静摩擦和负载，'
                '再由操作员决定是否换档；不会自动延长窗口或升力。'
                .format(metrics['peak_encoded_nm'] or 0.))
    if metrics['assessment']=='CODE_JITTER_NO_EFFECTIVE_MOTION':
        return ('只有{}个码值（{:.4f}°）变化，属于量化抖动，不是有效运动：不要据此判断已起动；'
                '先检查支撑/静摩擦/负载和反馈时序。'.format(
                    len(metrics['position_codes']), metrics['total_span_deg']))
    if metrics['assessment']=='OBSERVABLE_START_NO_TRACKING':
        return ('可观察起动（{:.4f}°）但未形成有效跟随：检查静摩擦释放后的阻尼/负载，'
                '不要自动升档或加力。'.format(metrics['total_span_deg']))
    if metrics['assessment']=='OUTPUT_AT_ENCODED_CAP_NO_MOTION':
        return ('已在编码力矩上限持续{:.2f}s仍无实际位移：优先检查机械支撑是否夹死、'
                '静摩擦或负载是否超过{:.3f}Nm可达上限；不要自动提高所选档位上限。'
                .format(metrics['time_at_encoded_cap_s'],metrics['encoded_cap_nm']))
    if metrics['assessment']=='COMMAND_BELOW_CAP_NO_MOTION':
        return ('本次指令峰值{:.3f}Nm仅占编码上限{:.3f}Nm的{:.0f}%，没有用满输出：'
                '先检查所选跟随误差门槛、试验时长和支撑/夹紧，再由操作员决定是否单独选择较硬候选档；'
                '不要自动升档或加力。'
                .format(metrics['peak_encoded_nm'] or 0.,metrics['encoded_cap_nm'],
                        100*(metrics['peak_fraction_of_encoded_cap'] or 0.)))
    if metrics['assessment'] in ('PARTIAL_TRACKING','EFFECTIVE_TRACKING'):
        return ('已有实际位移{:.3f}°（编码器{}个码值），按停止原因和跟随误差判断是否继续；'
                '本结果不构成主动运动验收。'.format(metrics['total_span_deg'],
                                                   len(metrics['position_codes'])))
    if metrics['assessment']=='NO_MOTION_REQUIRED':
        return '保持试验未要求位移；本结果只说明短时保持判据，不是主动运动证明。'
    return '没有可分类的输出/位移记录；先检查日志和预检。'


def trigger_sentence(trigger):
    if not trigger:
        return '保护触发帧：无（正常完成、操作员停止或未捕获）。'
    return ('保护触发帧：t={:.3f}s 状态{} 码{} 角{:+.4f}° 原始速度{:+.4f} 模型速度{:+.4f}rad/s '
            '误差{:+.3f}° 在用编码命令{} 原因={}。'.format(
                trigger['at_s'], trigger['state'], trigger['position_code'],
                trigger['angle_deg'], trigger['velocity_rad_s'],
                trigger['model_velocity_rad_s'], trigger['error_deg'],
                ('—' if trigger['command_encoded_nm'] is None
                 else '{:+.4f}Nm'.format(trigger['command_encoded_nm'])),
                trigger['reason']))


def post_stop_sentence(post_stop):
    if not post_stop:
        return '停止后新反馈：未采集。'
    observation=post_stop.get('observation') or {}
    if not post_stop.get('frames'):
        return ('停止后新反馈：无可见新帧（{}，主动查询={}，UNKNOWN锁定={}）；'
                '不保证首帧、可能丢帧；不能判断滑行，不能声称0滑行。').format(
            post_stop.get('insufficient_reason') or 'no_new_feedback',
            bool(observation.get('active_query')),bool(observation.get('unknown_lock')))
    first=post_stop.get('first_state0') or post_stop['frames'][0]
    last=post_stop.get('last_state0') or post_stop['frames'][-1]
    text=('停止后新反馈{}帧（可见，state0={}，不保证首帧；可能丢帧；来源{}）：'
          '首可见t=+{:.3f}s 状态{} 码{} 角{:+.4f}° 原始速度{:+.4f}rad/s；'
          '末可见码{} ').format(
        post_stop['count'],post_stop.get('state0_count',0),
        ','.join(post_stop.get('sources') or ['—']),
        first['at_s'],first['state'],first['position_code'],first['angle_deg'],
        first['velocity_rad_s'],last['position_code'])
    travel=post_stop.get('travel_deg')
    if travel is None:
        text+='位置再变化不可判定（单帧/覆盖不足）；'
    else:
        text+='位置再变化{:+.4f}°（覆盖{:.3f}s）；'.format(travel,post_stop.get('coverage_s',0.))
    peak=post_stop.get('peak_speed_rad_s')
    text+=('峰值速度{}；').format('不可判定' if peak is None else '{:.4f}rad/s'.format(peak))
    if post_stop.get('insufficient'):
        text+='证据不足（{}），不能判断滑行，不能据此声称0滑行。'.format(
            post_stop.get('insufficient_reason') or 'insufficient')
    return text


def metrics_sentence(metrics,reason,trigger=None,post_stop=None):
    if not metrics or not metrics.get('enabled_samples'):
        return '未进入使能采样；停止原因：{}。'.format(reason)
    return ('使能采样{:.2f}s；实际行程{:.4f}°（{}个编码器码值，分辨率{:.4f}°）；'
            '跟随误差峰值{:.3f}°；指令峰值{:.3f}Nm/请求峰值{}；'
            '到达编码上限{}/{}周期（{:.2f}s，{:.1f}%）；起动相限幅{}/{}周期；运动分类{}；判定{}；停止原因：{}。'
            .format(metrics['sampled_span_s'],metrics['total_span_deg'],
                    len(metrics['position_codes']),metrics['encoder_resolution_deg'],
                    metrics['max_error_deg'],metrics['peak_encoded_nm'] or 0.,
                    ('—' if metrics['peak_requested_nm'] is None
                     else '{:.3f}Nm'.format(metrics['peak_requested_nm'])),
                    metrics['cycles_at_encoded_cap'],metrics['output_sample_count'],
                    metrics['time_at_encoded_cap_s'],100*metrics['share_at_encoded_cap'],
                    metrics.get('cycles_startup_capped',0),metrics['output_sample_count'],
                    metrics['motion_class'],metrics['assessment'],reason)) \
        + ' ' + trigger_sentence(trigger) + ' ' + post_stop_sentence(post_stop)


def metrics_fields(c,measurements,round_trip,trigger=None,post_stop=None):
    """Facts for one report, computed from the recorded production samples."""
    metrics=motion_summary(measurements,cap_nm=c.envelopes[3].torque,
        encoded_cap_nm=encoded_cap_torque_nm(c.envelopes[3].torque,c.profiles[3].tmax),
        encoder_resolution_deg=math.degrees(2*c.profiles[3].pmax/65535),
        motion_required=round_trip,
        following_error_deg=math.degrees(c.envelopes[3].following_error))
    code=stop_reason_code(c.reason,metrics)
    return (metrics,code,metrics_sentence(metrics,c.reason,trigger,post_stop),
            next_step_hint(metrics,code,post_stop))


def attach_metrics(result,c,measurements,round_trip,audit,trigger=None,post_stop=None):
    """Add output/motion facts and the stop classification exactly once."""
    metrics,code,summary,hint=metrics_fields(c,measurements,round_trip,trigger,post_stop)
    tier=c.tier
    result.update(max_observed_error_deg=max((abs(v['error_deg']) for v in measurements),default=None),
        sample_count=len(measurements),torque_cap_nm=c.envelopes[3].torque,
        encoded_torque_cap_nm=encoded_cap_torque_nm(c.envelopes[3].torque,c.profiles[3].tmax),
        stop_reason_code=code,stop_summary=summary,
        next_step_hint=hint,motion_metrics=metrics,
        motion_class=metrics['motion_class'],output_class=metrics['output_class'],
        trigger_frame=trigger,post_stop=post_stop,
        pd_tier=tier.name if tier else None,host_kp=tier.kp if tier else None,
        host_kd=tier.kd if tier else None,
        pd_tier_hardware_validated=tier.hardware_validated if tier else None,
        candidate_control=(None if not tier or not tier.control else
            dict(slew_up_nm_s=tier.control.slew_up_nm_s,
                 slew_down_nm_s=tier.control.slew_down_nm_s,
                 startup_cap_nm=tier.control.startup_cap_nm,
                 startup_window_s=tier.control.startup_window_s,
                 motion_confirm_codes=tier.control.motion_confirm_codes,
                 brake_gain_nm_per_rad_s=tier.control.brake_gain_nm_per_rad_s,
                 brake_cap_nm=tier.control.brake_cap_nm,
                 brake_deadband_rad_s=tier.control.brake_deadband_rad_s)))
    audit('TRIAL_METRICS',stop_reason_code=code,stop_summary=summary,
          next_step_hint=hint,pd_tier=result.get('pd_tier'),
          host_kp=result.get('host_kp'),host_kd=result.get('host_kd'),
          trigger_frame=trigger,post_stop=post_stop,
          **{k:v for k,v in metrics.items() if k!='position_codes'},
          position_codes=metrics['position_codes'])
    audit('PHASE',message=summary)
    return metrics


def build_session(reader,request,audit,clock=clock_now):
    if not isinstance(request,TrialRequest) or not request.request_id:
        raise ValueError('Immutable operator trial request required')
    if request.mode not in ('hold','roundtrip10'):
        raise ValueError('Only fixed hold or J4 +10deg round trip supported')
    round_trip=request.mode=='roundtrip10'
    tier=pd_tier(request.tier)
    if tier.name!=DEFAULT_PD_TIER and request.tier_ack is not True:
        raise ValueError('较硬候选档必须由操作员单独确认；不会自动选择或升档')
    args=(request.support,request.clearance,request.risk_ack,str(WATCHDOG_REPORT))
    approval=(LimitedJ4RoundTrip(*args,hold_report=str(HOLD_REPORT),pd_tier=tier.name)
              if round_trip else LimitedJ4Trial(*args,pd_tier=tier.name))
    approval.validate()
    audit('PD_TIER_SELECTED',pd_tier=tier.name,host_kp=tier.kp,host_kd=tier.kd,
          hardware_validated=tier.hardware_validated,explicit_tier_ack=bool(request.tier_ack),
          is_default=tier.name==DEFAULT_PD_TIER,output_cap_nm=tier.torque_cap,torque_slew_nm_s=tier.slew_nm_s,
          note='operator-selected host PD candidate; no automatic tuning or escalation')
    if reader.state.mapping!=load_mapping():
        raise ValueError('软件零偏已变化，请重新连接')
    envelopes=[]
    for i in range(1,5):
        if i not in reader.state.feedback or not reader.state.ready(i):
            raise ValueError(f'J{i}参数/反馈尚未齐全')
        f=reader.state.feedback[i];m=reader.state.mapping[i]
        q=m['hardware_sign']*(f['position_rad']-m['zero_offset_rad'])
        deg=math.degrees(q)
        margin=tier.following_error_deg if i==4 else 2
        upper_margin=10+margin if round_trip and i==4 else margin
        if not LIMITS[i-1][0]+margin<=deg<=LIMITS[i-1][1]-upper_margin:
            raise ValueError(f'J{i}当前角度{deg:.2f}°靠近模型边界或角度分支异常；不会自动回零/折算360°')
        envelopes.append(Envelope(q-math.radians(margin),q+math.radians(upper_margin),.02,
                                  (tier.torque_cap if i==4 else (.5 if i in (1,2) else .3)),math.radians(margin),
                                  tier.kp if i==4 else 4.,tier.kd if i==4 else .5,50.,
                                  feedback_speed=tier.feedback_speed_abort if i==4 else .1,
                                  arrival_error=math.radians(.5) if round_trip and i==4 else None))
    return ControlSession(reader,envelopes,approval,audit,clock,
                          active_axes=(4,),max_runtime=45. if round_trip else 2.,ramp=1.)


def capture_post_frame(state, reader, c, source, joint=4):
    """Keep one actually visible post-stop feedback frame (deduplicated by time).

    This is deliberately 'visible' rather than 'first': a drain can overwrite an
    earlier frame before it can be copied, and a passive bus may not send
    anything at all. The caller reports that limitation instead of calling the
    cached frame a guaranteed first sample.
    """
    if type(joint) is not int or joint not in (3,4):
        raise ValueError('Post-stop observer supports J3/J4 only')
    feedback = reader.state.feedback.get(joint)
    at = reader.state.at.get(joint)
    if (feedback is None or at is None or not getattr(c, 'stop_time', None)
            or at <= c.stop_time or at in state['seen']
            or not hasattr(c, 'profiles')):
        return False
    state['seen'].add(at)
    state['frames'].append(dict(
        at_s=at - c.stop_time, state=feedback['state_code'],
        position_code=position_code(feedback['payload']),
        angle_deg=math.degrees(c.profiles[joint-1].sign*(feedback['position_rad']-c.profiles[joint-1].zero)),
        velocity_rad_s=feedback['velocity_rad_s'],
        model_velocity_rad_s=c.profiles[joint-1].sign*feedback['velocity_rad_s'],
        torque_feedback_nm=feedback['torque_feedback_nm'], source=source))
    if source == 'cc_poll':
        state['meta']['cc_new_frames'] += 1
    elif source == 'stop_cycle':
        state['meta']['cycle_frames'] += 1
    else:
        state['meta']['passive_frames'] += 1
    return True


def run_trial(reader,request,audit,cancel,publish=lambda row:None,clock=clock_now,sleep=time.sleep):
    """Reader already has a NEW preflight; single owner throughout the trial."""
    session=None;measurements=[];hold_seen=False
    round_trip=request.mode=='roundtrip10'
    runtime=45. if round_trip else 2.
    result=dict(request_id=request.request_id,trial='supported_J4_plus10deg_round_trip' if round_trip else 'supported_J4_current_pose_hold_2s',
        enabled_axes_permitted=[4],trajectory_permitted=round_trip,j5_queried=False,
        parameter_writes=0,full_arm_accepted=False,physical_torque_off_verified=False,
        watchdog_latency_measured=False,actual_motion_envelope_accepted=False,
        readonly_log=str(reader.path))
    # Post-stop observation state exists even if build/start fails early.
    post_state=dict(frames=[],seen=set(),tx=[],meta=dict(
        window_s=POST_STOP_WINDOW_S,active_query=False,unknown_lock=False,
        tx=0,polls=0,cc_new_frames=0,cycle_frames=0,passive_frames=0,
        may_miss_frames=True,first_guaranteed=False))
    try:
        if cancel.is_set():raise RuntimeError('开始前已取消；未使能')
        session=build_session(reader,request,audit,clock)
        session.start();c=session.controller
        tier=c.tier
        if tier is None:
            raise RuntimeError('Host PD tier not bound to this session')
        audit('TRIAL_STARTED',request_id=request.request_id,axes=[4],max_runtime_s=runtime,
              targets_deg=[math.degrees(q) for q in c.targets],candidate_cap_nm=tier.torque_cap,torque_slew_nm_s=tier.slew_nm_s,
              torque_slew_up_nm_s=(tier.control.slew_up_nm_s if tier.control else tier.slew_nm_s),
              torque_slew_down_nm_s=(tier.control.slew_down_nm_s if tier.control else tier.slew_nm_s),
              startup_cap_nm=(tier.control.startup_cap_nm if tier.control else None),
              startup_window_s=(tier.control.startup_window_s if tier.control else None),
              motion_confirm_codes=(tier.control.motion_confirm_codes if tier.control else None),
              brake_gain_nm_per_rad_s=(tier.control.brake_gain_nm_per_rad_s if tier.control else None),
              brake_cap_nm=(tier.control.brake_cap_nm if tier.control else None),
              brake_deadband_rad_s=(tier.control.brake_deadband_rad_s if tier.control else None),
              candidate_host_kp=tier.kp,candidate_host_kd=tier.kd,
              pd_tier=tier.name,pd_tier_hardware_validated=tier.hardware_validated,
              pd_tier_explicitly_selected=True,encoded_cap_nm=encoded_cap_torque_nm(
                  c.envelopes[3].torque,c.profiles[3].tmax),
              gravity_ff_nm=0.,drive_kp=0.,drive_kd=0.,
              feedback_torque_abort_nm=2*tier.torque_cap,following_error_abort_deg=tier.following_error_deg,
              command_speed_limit_rad_s=.02,feedback_speed_abort_rad_s=tier.feedback_speed_abort)
        began=clock();last_sample=None;stage='takeover';dwell_until=None
        outward_reached=False;return_reached=False
        trigger=None;post_stop=None
        post_state=dict(frames=[],seen=set(),tx=[],meta=dict(
            window_s=POST_STOP_WINDOW_S,active_query=False,unknown_lock=False,
            tx=0,polls=0,cc_new_frames=0,cycle_frames=0,passive_frames=0,
            may_miss_frames=True,first_guaranteed=False))
        while c.state not in ('DISABLED','UNKNOWN'):
            if cancel.is_set():session.stop()
            if clock()-began>runtime+.7:
                session.stop();c.state='UNKNOWN';break
            f=reader.state.feedback.get(4)
            if f and abs(f['torque_feedback_nm'])>2*tier.torque_cap:
                c.stop('J4_feedback_torque_abort_threshold',clock())
            session.cycle()
            now=clock()
            if (trigger is None and c.state=='STOPPING'
                    and c.reason not in ('round_trip_complete','trial_duration_complete','operator_stop')):
                o=c.last_snapshot.rows[3]
                out=c.output.get(4)
                fb=reader.state.feedback.get(4) or {}
                # The faulting feedback is the reader state used to build the
                # snapshot; c.last_snapshot may be one cycle older for _check
                # faults (speed/state/staleness), so prefer the reader frame.
                if fb:
                    q_model=c.profiles[3].sign*(fb['position_rad']-c.profiles[3].zero)
                    code=position_code(fb['payload'])
                    angle=math.degrees(q_model)
                    velocity=fb['velocity_rad_s']
                    state=fb['state_code']
                else:
                    q_model=o.q;code=position_code(o.raw_payload)
                    angle=math.degrees(o.q);velocity=o.velocity;state=o.state
                receipt=reader.state.at.get(4)
                trigger=dict(
                    at_s=((receipt if fb and receipt is not None else None)
                          or c.last_time or now)-began,
                    state=state,position_code=code,
                    angle_deg=angle,velocity_rad_s=velocity,
                    model_velocity_rad_s=c.profiles[3].sign*velocity,
                    torque_feedback_nm=fb.get('torque_feedback_nm'),
                    error_deg=math.degrees(c.targets[3]-q_model),
                    command_encoded_nm=(out['encoded_nm'] if out else None),
                    command_limited_nm=(out['limited_nm'] if out else None),
                    brake_nm=(out.get('brake_nm') if out else None),
                    startup_capped=(out.get('startup_capped') if out else None),
                    motion_confirmed=(out.get('motion_confirmed') if out else None),
                    reason=c.reason)
            if c.stop_time and c.state in ('STOPPING','DISABLED'):
                capture_post_frame(post_state,reader,c,'stop_cycle')
            if c.state=='HOLD':hold_seen=True
            if round_trip and not cancel.is_set() and c.state=='HOLD':
                if stage=='takeover' and now-began>=2.:
                    targets=list(c.anchor);targets[3]+=math.radians(10)
                    c.trajectory(targets,18.,now);stage='outward'
                    audit('PHASE',message=f'J4正向10°，18秒；跟随误差超过{tier.following_error_deg:g}°即停止，不自动加力')
                elif stage=='outward':
                    outward_reached=True;stage='outward_dwell';dwell_until=now+.5
                    audit('PHASE',message='正向目标已在0.5°误差内到达，随后返回本次起点')
                elif stage=='outward_dwell' and now>=dwell_until:
                    c.trajectory(c.anchor,18.,now);stage='return'
                    audit('PHASE',message='J4返回本次接管起点，18秒；不是回软件零位')
                elif stage=='return':
                    return_reached=True;stage='return_dwell';dwell_until=now+.5
                elif stage=='return_dwell' and now>=dwell_until:
                    stage='complete';c.stop('round_trip_complete',now)
            if c.last_time!=last_sample and c.enabled:
                last_sample=c.last_time
                snapshot=c.last_snapshot
                o=snapshot.rows[3]
                error=math.degrees(c.targets[3]-o.q)
                out=c.output.get(4)
                # A faulted cycle stops before building frames; do not attribute
                # the previous cycle's torque to the newer error.
                if out is not None and out['at']!=c.last_time:
                    out=None
                measurements.append(dict(at=now-began,error_deg=error,state=o.state,
                    angle_deg=math.degrees(o.q),reference_deg=math.degrees(c.targets[3]),stage=stage,
                    position_code=position_code(o.raw_payload),output=out))
                publish(dict(state=c.state,at_s=now-began,angle_deg=math.degrees(o.q),error_deg=error))
            sleep(.001)
        tail=[v for v in measurements if 1.5<=v['at']<=2.05 and v['state']==1]
        stable=(len(tail)>=2 and tail[-1]['at']-tail[0]['at']>=.4
                and all(abs(v['error_deg'])<=.5 for v in tail))
        metrics=attach_metrics(result,c,measurements,round_trip,audit,trigger=trigger)
        result.update(final_controller_state=c.state,reason=c.reason,hold_state_observed=hold_seen,
            position_hold_criterion_met=bool(not round_trip and hold_seen and stable and c.state=='DISABLED'
                                             and c.reason=='trial_duration_complete'),
            hold_criterion='last 0.5s error<=0.5deg, >=0.4s sampled span; not a full acceptance',
            last_half_second_samples=tail,
            serial_receipts=len(session.endpoint.receipts),
            enabled_observed_axes=sorted(c.enabled),
            fresh_stop_confirmation=c.state=='DISABLED')
        if round_trip:
            anchor_deg=math.degrees(c.anchor[3])
            peak=max((v['angle_deg']-anchor_deg for v in measurements),default=0.)
            f=reader.state.feedback.get(4)
            final_error=(math.degrees(c.profiles[3].sign*(f['position_rad']-c.profiles[3].zero))-anchor_deg
                         if f is not None else None)
            result.update(outward_reached=outward_reached,return_reached=return_reached,
                actual_positive_excursion_deg=peak,final_error_from_start_deg=final_error,
                final_error_source='pre_stop_cached',
                target_outward_delta_deg=10.,leg_duration_s=18.,stage=stage,
                round_trip_criterion_met=bool(outward_reached and return_reached and peak>=9.5
                    and final_error is not None and abs(final_error)<=.5 and c.state=='DISABLED'
                    and c.reason=='round_trip_complete'),
                no_auto_output_increase=True)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if session:
            c=session.controller
            if c.state not in ('IDLE','DISABLED','UNKNOWN'):
                session.stop()
                end=clock()+.4
                while clock()<end and c.state not in ('DISABLED','UNKNOWN'):
                    session.cycle();capture_post_frame(post_state,reader,c,'stop_cycle');sleep(.001)
            capture_post_frame(post_state,reader,c,'stop_cycle_final')
            if c.state not in ('IDLE','DISABLED'):
                c.state='UNKNOWN'
            result['final_controller_state']=c.state
            result['fresh_stop_confirmation']=c.state=='DISABLED'
            # Only after four-axis new feedback confirms DISABLED may we use a
            # bounded read-only J4 CC poll. In UNKNOWN no query is sent: an
            # enabled drive could refresh its watchdog from an active request.
            if c.state=='DISABLED' and c.stop_time:
                post_state['meta']['active_query']=True
                deadline=clock()+POST_STOP_WINDOW_S
                polls=0
                while clock()<deadline and polls<POST_STOP_MAX_POLLS:
                    packet=usb_packet(4)
                    at_tx=clock()
                    try:
                        count=reader.serial.write(packet)
                        if count!=len(packet):
                            raise IOError('partial J4 CC poll write')
                        reader.tx+=1
                        try:
                            reader.event(direction='tx',can_id=0x7FF,
                                         payload=packet[21:29].hex(' '))
                        except Exception:
                            pass
                        post_state['tx'].append(dict(at=at_tx,kind='cc_j4',
                                                     payload=packet[21:29].hex(' ')))
                        post_state['meta']['tx']+=1
                    except Exception as exc:
                        post_state['meta']['tx_error']=str(exc);break
                    polls+=1
                    try:
                        session._drain()
                    except Exception as exc:
                        post_state['meta']['drain_error']=str(exc);break
                    captured=capture_post_frame(post_state,reader,c,'cc_poll')
                    if captured and reader.state.feedback[4]['state_code']!=0:
                        # A newer observation overrides the earlier disabled
                        # confirmation. Do not keep querying a possibly enabled
                        # drive and refreshing its watchdog.
                        c.state='UNKNOWN'
                        post_state['meta']['unknown_lock']=True
                        post_state['meta']['disabled_state_contradicted']=True
                        post_state['meta']['unexpected_state']=reader.state.feedback[4]['state_code']
                        break
                    sleep(.01)
                post_state['meta']['polls']=polls
            elif c.state=='UNKNOWN':
                post_state['meta']['unknown_lock']=True
                capture_post_frame(post_state,reader,c,'passive_unknown')
            else:
                capture_post_frame(post_state,reader,c,'passive')
            result['final_controller_state']=c.state
            result['fresh_stop_confirmation']=c.state=='DISABLED'
            if post_state['meta'].get('disabled_state_contradicted'):
                result['position_hold_criterion_met']=False
                if round_trip:result['round_trip_criterion_met']=False
                result['post_stop_state_conflict']=True
            visible=list(post_state['frames'])
            state0=[f for f in visible if f['state']==0]
            coverage=(state0[-1]['at_s']-state0[0]['at_s']) if len(state0)>=2 else 0.
            if post_state['meta'].get('disabled_state_contradicted'):
                reason='disabled_state_contradicted'
            elif not visible:
                reason='no_new_feedback'
            elif not state0:
                reason='no_new_state0_feedback'
            elif len(state0)<2:
                reason='too_few_state0_frames'
            elif coverage<POST_STOP_MIN_COVERAGE_S:
                reason='short_coverage'
            else:
                reason=None
            post_stop=dict(
                frames=visible[:64],count=len(visible),state0_count=len(state0),
                first=(visible[0] if visible else None),
                last=(visible[-1] if visible else None),
                first_state0=(state0[0] if state0 else None),
                last_state0=(state0[-1] if state0 else None),
                coverage_s=coverage,
                travel_deg=((state0[-1]['angle_deg']-state0[0]['angle_deg'])
                            if len(state0)>=2 else None),
                peak_speed_rad_s=(max(abs(f['velocity_rad_s']) for f in state0)
                                  if len(state0)>=2 else None),
                insufficient=reason is not None,insufficient_reason=reason,
                first_state0_guaranteed=False,visibility='visible_only',
                may_miss_frames=True,observation=post_state['meta'],
                queries=list(post_state['tx']),
                sources=sorted({f['source'] for f in visible}),
                note=('visible-only post-stop frames; a drain can overwrite earlier '
                      'frames and a passive bus may send none; CC polls only after '
                      'four-axis DISABLED, only J4 0x7FF 04 00 CC'))
            if round_trip and hasattr(c,'anchor'):
                anchor_deg=math.degrees(c.anchor[3])
                latest=state0[-1] if state0 else None
                if latest is not None:
                    post_err=latest['angle_deg']-anchor_deg
                    result['post_stop_final_error_deg']=post_err
                    result['final_error_from_start_deg']=post_err
                    result['final_error_source']='post_stop_last_visible'
                    if abs(post_err)>.5+1e-12:
                        result['post_stop_invalidates_return']=True
                        result['round_trip_criterion_met']=False
                        result['post_stop_criterion_note']=(
                            'post-stop visible position {:+.4f}deg from anchor exceeds '
                            '0.5deg; round-trip criterion revoked'.format(post_err))
                else:
                    result['post_stop_final_error_deg']=None
                    result['post_stop_criterion_note']=(
                        'no new state0 feedback after stop; final position not re-confirmed')
            session.endpoint.closed=True  # Keep the Reader for explicit read-only observation.
            if 'motion_metrics' in result:
                result['post_stop']=post_stop
                result['post_stop_observation']=post_state['meta']
                extra=post_stop_sentence(post_stop)
                result['stop_summary']=result.get('stop_summary','')+' '+extra
                if result.get('post_stop_criterion_note'):
                    result['stop_summary']+=' '+result['post_stop_criterion_note']
                audit('POST_STOP_FEEDBACK',trigger_frame=result.get('trigger_frame'),
                      post_stop=post_stop,post_stop_observation=post_state['meta'],
                      round_trip_criterion_met=result.get('round_trip_criterion_met'),
                      post_stop_final_error_deg=result.get('post_stop_final_error_deg'),
                      post_stop_invalidates_return=result.get('post_stop_invalidates_return',False),
                      stop_summary=result['stop_summary'])
                audit('PHASE',message=extra)
            elif (measurements and hasattr(c,'profiles') and hasattr(c,'envelopes')):
                # An early exception skipped the normal metrics block; still
                # report the recorded output/motion facts rather than nothing.
                try:
                    attach_metrics(result,c,measurements,round_trip,audit,trigger,post_stop)
                except Exception:
                    pass
        audit('TRIAL_RESULT',**result)
    return result


def fill_preflight(reader,cancel,clock=clock_now,sleep=time.sleep):
    """Invalidate cached registers before EVERY enabling trial, not on import."""
    reader.state=State(load_mapping())
    reader.setup=[(i,rid) for _ in range(3) for i in (1,2,3,4) for rid in REGS]
    reader.poll=0;reader.next_send=0
    end=clock()+5
    while clock()<end:
        if cancel.is_set():raise RuntimeError('预检取消；未使能')
        now=clock();previous=reader.tx
        reader.tick(now)
        if reader.tx!=previous:reader.next_send=now+.01
        if all(reader.state.ready(i) and i in reader.state.at and
               0<=now-reader.state.at[i]<=.06 for i in (1,2,3,4)):
            return
        sleep(.001)
    raise RuntimeError('预检超时，四轴新参数/反馈不齐；未使能')


def main():
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI · 受限实机接管 · J4保持 / 10°往返');root.geometry('1100x900')
    ui=ttk.Frame(root,padding=18);ui.pack(fill='both',expand=True)
    ttk.Label(ui,text='实机受限接管 · 首轮 J4',font=('Microsoft YaHei',19,'bold')).pack(anchor='w')
    ttk.Label(ui,text='J4当前位置保持2秒，或正向10°再返回起点（每程18秒，整次最多45秒）。不会回软件零位。\n'
        '总编码力矩按所选档位限幅：默认0.30Nm；1Nm档误差2°；2Nm档误差4°；速度中止最高0.20rad/s，目标限速0.02rad/s。\n'
        '主机PD档位必须明确选择：默认已验证档Kp=4；Kp=12、1Nm/2Nm及“有界起动/阻尼”候选档均未实机验证，需单独勾选确认。\n'
        '有界起动/阻尼档：起动窗口内≤0.5/1.0Nm、6s未确认位移即受控停止；减小力矩快于增加力矩，另有有界速度误差阻尼；不自动升档。\n'
        'J1–J3不使能；停止时可统一尝试失能四轴。J5不查询、不使能、不控制。四轴联动尚未开放。\n'
        '低输出可能不足以保持自重；2Nm档±4°、旧档±2°是中止边界，必须有支撑。按钮不是硬件急停。',wraplength=1010).pack(anchor='w',pady=10)
    bar=ttk.Frame(ui);bar.pack(fill='x')
    port=tk.StringVar(value='COM9');port_entry=ttk.Entry(bar,textvariable=port,width=14);port_entry.pack(side='left')
    status=tk.StringVar(value='未连接。先释放达妙工具和其他上位机的串口；打开此窗口不发指令。')
    table=ttk.Treeview(ui,columns=('joint','angle','mode','timeout','state','age'),show='headings',height=5)
    for key,name in zip(table['columns'],('关节','软件角度°','模式','TIMEOUT','状态','反馈年龄ms')):
        table.heading(key,text=name);table.column(key,width=150,anchor='center')
    for i in range(1,6):table.insert('','end',iid=str(i),values=(f'J{i}','—','—','—','未连接' if i<5 else '禁发送','—'))
    table.pack(fill='x',pady=12)
    ttk.Label(ui,textvariable=status,wraplength=1010).pack(anchor='w',pady=5)
    support=tk.BooleanVar(value=False);clearance=tk.BooleanVar(value=False);risk=tk.BooleanVar(value=False)
    for var,label in ((support,'全臂和末端有可靠机械支撑，人员离开转动范围，可立即现场切断电机电源'),
                      (clearance,'已确认所选路径无干涉：2Nm档保持需±4°、往返需起点−4°至起点+14°余量（旧档±2°/−2°至+12°），支撑不夹死转轴'),
                      (risk,'理解本次会短时使能J4，低输出不保证托住自重；D状态不等于物理力矩退出已测量')):
        ttk.Checkbutton(ui,text=label,variable=var).pack(anchor='w',pady=3)
    tier_frame=ttk.LabelFrame(ui,text='主机PD档位（每次试验必须明确选择；默认已验证档）')
    tier_frame.pack(fill='x',pady=6)
    tier=tk.StringVar(value=DEFAULT_PD_TIER);tier_buttons=[]
    for value,label in ((VERIFIED_PD_TIER.name,'已验证档 Kp=4 / Kd=0.5（默认；与既有实机记录一致）'),
                        (STIFFER_PD_TIER.name,'较硬候选档 Kp=12 / Kd=0.5（未实机验证；0.30Nm上限）'),
                        (ONE_NM_PD_TIER.name,ONE_NM_PD_TIER.label),
                        (TWO_NM_PD_TIER.name,TWO_NM_PD_TIER.label),
                        (ONE_NM_BRAKE_TIER.name,ONE_NM_BRAKE_TIER.label),
                        (TWO_NM_BRAKE_TIER.name,TWO_NM_BRAKE_TIER.label)):
        tier_buttons.append(ttk.Radiobutton(tier_frame,text=label,variable=tier,value=value))
        tier_buttons[-1].pack(anchor='w',pady=2)
    tier_ack=tk.BooleanVar(value=False)
    ack_button=ttk.Checkbutton(tier_frame,
        text='仅在选择较硬候选档时必填：我理解该档未实机验证，按明确选择的档位出力（最高2Nm），停机失能不会保持自重；不自动升档、不自动加力',
        variable=tier_ack)
    ack_button.pack(anchor='w',pady=3)
    buttons=ttk.Frame(ui);buttons.pack(fill='x',pady=10)
    log=tk.Text(ui,height=15,wrap='word');log.pack(fill='both',expand=True)
    events=queue.Queue();commands=queue.Queue();cancel=threading.Event();disconnect=threading.Event()
    worker=[None];closing=[False];busy=[False];live_ok=[False];latched=[False]
    def emit(message):events.put(('log',message))
    def sample(reader):
        now=clock_now();rows=[]
        for i in (1,2,3,4):
            s=reader.state;r=s.regs[i];f=s.feedback.get(i,{})
            age=now-s.at[i] if i in s.at else math.inf
            ready=s.ready(i) and age<=.06
            rows.append(dict(joint=i,angle=f.get('model_deg'),mode=r.get(10),timeout=r.get(9),
                state=f.get('state_code'),age=age,ready=ready))
        return rows
    def run(port_name):
        reader=None;handle=None
        try:
            reader=Reader(port_name,load_mapping());reader.serial.write_timeout=.025
            emit('只读连接建立；正在读四轴当前参数。')
            next_publish=0.
            while not disconnect.is_set():
                now=clock_now();before=reader.tx;reader.tick(now)
                if reader.tx!=before:reader.next_send=now+.01
                try:request=commands.get_nowait()
                except queue.Empty:request=None
                if request:
                    events.put(('busy',True))
                    folder=ROOT/'tmp/live_trial';folder.mkdir(exist_ok=True,parents=True)
                    path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+request.request_id[:6]+'.jsonl')
                    try:
                        with path.open('x',encoding='utf-8') as handle:
                            audit=make_event_writer(handle,emit)
                            audit('PHASE',message=('重新读取四轴寄存器，随后J4 +10°往返；不写参数' if request.mode=='roundtrip10'
                                                 else '重新读取四轴寄存器，随后仅J4当前位置接管；不写参数'))
                            fill_preflight(reader,cancel)
                            last_live_publish=[0.]
                            def publish_trial(row):
                                at=clock_now()
                                if at-last_live_publish[0]>=.05:
                                    events.put(('trial',row));events.put(('sample',sample(reader)))
                                    last_live_publish[0]=at
                            result=run_trial(reader,request,audit,cancel,publish=publish_trial)
                            path.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
                            emit(json.dumps({k:v for k,v in result.items() if k!='last_half_second_samples'},ensure_ascii=False))
                            emit('结果文件：'+str(path.with_suffix('.json')))
                            events.put(('latched',result.get('final_controller_state')=='UNKNOWN'))
                    except Exception as exc:
                        emit('预检/试验中止：'+str(exc))
                    finally:
                        handle=None;events.put(('busy',False))
                if now>=next_publish:
                    events.put(('sample',sample(reader)));next_publish=now+.1
                time.sleep(.001)
        except Exception as exc:emit('连接结束：'+str(exc))
        finally:
            try:
                if reader:reader.close()
            finally:events.put(('disconnected',None))
    def connect():
        if worker[0] and worker[0].is_alive():return
        cancel.clear();disconnect.clear();latched[0]=False
        port_entry.config(state='disabled');connect_button.config(state='disabled')
        worker[0]=threading.Thread(target=run,args=(port.get().strip(),),daemon=False);worker[0].start()
    def request_trial(mode='hold'):
        if busy[0] or not live_ok[0] or latched[0]:return
        if not all(v.get() for v in (support,clearance,risk)):
            status.set('请逐条落实并确认支撑、局部空间和低输出试验条件。');return
        try:
            tier_choice=pd_tier(tier.get())
        except ValueError:
            status.set('主机PD档位无效：请明确选择已验证档或较硬候选档。');return
        if tier_choice.name!=DEFAULT_PD_TIER and not tier_ack.get():
            status.set('较硬候选档未实机验证：请勾选对应确认，或改回默认Kp=4已验证档。');return
        busy[0]=True;trial_button.config(state='disabled');trip_button.config(state='disabled');cancel.clear()
        commands.put(TrialRequest(support.get(),clearance.get(),risk.get(),uuid.uuid4().hex,mode,
                                  tier_choice.name,tier_ack.get()))
    def stop():
        cancel.set()
        status.set('已请求停止试验；程序尝试失能并等反馈，异常时现场切电。' if busy[0]
                   else '当前只读，未发失能命令。此按钮不是硬件急停。')
    def detach():cancel.set();disconnect.set()
    connect_button=ttk.Button(bar,text='连接 · 只读预检',command=connect);connect_button.pack(side='left',padx=10)
    ttk.Button(bar,text='停止并断开',command=detach).pack(side='left')
    trial_button=ttk.Button(buttons,text='J4当前位置保持2秒',command=request_trial,state='disabled');trial_button.pack(side='left')
    trip_button=ttk.Button(buttons,text='J4 正向10°→返回起点（会使能）',command=lambda:request_trial('roundtrip10'),state='disabled');trip_button.pack(side='left',padx=10)
    ttk.Button(buttons,text='停止试验 · 尝试全臂失能',command=stop).pack(side='left',padx=12)
    def poll():
        try:
            while True:
                kind,data=events.get_nowait()
                if kind=='sample':
                    live_ok[0]=all(r['ready'] and r['state']==0 and r['mode']==1 and r['timeout']==2000 for r in data)
                    for r in data:
                        angle='—' if r['angle'] is None or not r['ready'] else f'{r["angle"]:+.2f}'
                        table.item(str(r['joint']),values=(f'J{r["joint"]}',angle,r['mode'],r['timeout'],
                            {0:'正常失能',1:'已使能',13:'D通信保护'}.get(r['state'],str(r['state'])),
                            '—' if not math.isfinite(r['age']) else f'{r["age"]*1000:.1f}'))
                    if not busy[0]:
                        status.set('上次输出状态未知；保持支撑并检查，必要时现场切电。需明确断开并检查后建立新会话。' if latched[0]
                            else ('四轴只读预检通过；可审阅条件后启动J4限定试验。' if live_ok[0]
                                  else '尚不满足：四轴MIT、TIMEOUT2000、正常失能、新鲜反馈≤60ms。不会自动改参数/清故障。'))
                elif kind=='busy':
                    busy[0]=data
                    if not data:
                        # Every run starts again from the verified default: no
                        # sticky stiffer tier and no remembered acknowledgement.
                        support.set(False);clearance.set(False);risk.set(False)
                        tier.set(DEFAULT_PD_TIER);tier_ack.set(False)
                elif kind=='latched':latched[0]=data
                elif kind=='trial':status.set(f'J4 {data["state"]} · {data["at_s"]:.2f}s · 角度{data["angle_deg"]:+.2f}° · 误差{data["error_deg"]:+.3f}°')
                elif kind=='log':log.insert('end',data+'\n');log.see('end')
                elif kind=='disconnected':
                    busy[0]=False;live_ok[0]=False;connect_button.config(state='normal');port_entry.config(state='normal')
                    status.set('已断开。若试验输出状态未知，保持支撑并现场切电。')
                    if closing[0]:root.destroy();return
        except queue.Empty:pass
        enabled='normal' if live_ok[0] and not busy[0] and not latched[0] and not disconnect.is_set() else 'disabled'
        trial_button.config(state=enabled);trip_button.config(state=enabled)
        ack_button.config(state=enabled)
        for button in tier_buttons:button.config(state=enabled)
        root.after(50,poll)
    def close():
        if worker[0] and worker[0].is_alive():closing[0]=True;detach()
        else:root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);root.after(50,poll);root.mainloop()


if __name__=='__main__':main()
