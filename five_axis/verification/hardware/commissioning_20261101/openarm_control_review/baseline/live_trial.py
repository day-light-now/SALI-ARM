"""Operator-controlled supported J4 current-pose commissioning window.

Opening/importing never connects or enables. Read-only connection is explicit;
trial start is a separate explicit action. Full-arm and trajectory authority
remain blocked; J5 never enters the transport. No parameter writes/auto-clear.

The operator names a host PD tier in the immutable request: Kp=4 (default),
Kp=12 (0.3Nm candidate), or Kp=32 (explicit 1Nm/2Nm candidates).
Nondefault candidates require extra acknowledgement. Nothing here tunes gains, escalates a tier,
changes the selected bound or retries after a stop. The explicitly authorized
The 1Nm/2Nm tiers use Kp=32/Kd=0.5 with 0.5/1Nm/s host torque slew.
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
                        DEFAULT_PD_TIER, VERIFIED_PD_TIER, STIFFER_PD_TIER, ONE_NM_PD_TIER, TWO_NM_PD_TIER,
                        pd_tier, encoded_cap_torque_nm, motion_summary)
from .actuation_io import ControlSession
from .clock import now as clock_now
from .watchdog_check import make_event_writer
from four_axis_monitor import Reader, State, REGS, load_mapping


WATCHDOG_REPORT=ROOT/'tmp/watchdog_check/20261101_131809_aee5b0.json'
HOLD_REPORT=ROOT/'tmp/live_trial/20261101_134732_a325bf.json'


@dataclass(frozen=True)
class TrialRequest:
    support: bool
    clearance: bool
    risk_ack: bool
    request_id: str
    mode: str = 'hold'
    tier: str = DEFAULT_PD_TIER
    tier_ack: bool = False


def position_code(raw_payload):
    """Exact 16-bit encoder code behind a feedback payload, or None."""
    try:
        data=bytes.fromhex(raw_payload)
    except (TypeError,ValueError):
        return None
    return ((data[1]<<8)|data[2]) if len(data)==8 else None


def stop_reason_code(reason,metrics):
    """Stable code that separates a stall from a generic following error."""
    if reason=='round_trip_complete':return 'ROUND_TRIP_COMPLETE'
    if reason=='trial_duration_complete':return 'HOLD_COMPLETE'
    if reason=='operator_stop':return 'OPERATOR_STOP'
    if reason=='Following error':
        return {'OUTPUT_AT_ENCODED_CAP_NO_MOTION':'FOLLOWING_ERROR_AT_ENCODED_CAP_NO_MOTION',
                'COMMAND_BELOW_CAP_NO_MOTION':'FOLLOWING_ERROR_BELOW_CAP_NO_MOTION',
                'NO_OUTPUT_RECORDED':'FOLLOWING_ERROR_NO_OUTPUT_RECORDED'
                }.get((metrics or {}).get('assessment'),'FOLLOWING_ERROR')
    return 'STOPPED:'+str(reason)


def next_step_hint(metrics):
    """Mechanical/load checks for the next operator decision. Never an auto action."""
    if not metrics or not metrics.get('enabled_samples'):
        return '未进入使能采样，无法评价输出与位移；先检查预检、使能确认和日志。'
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
    if metrics['assessment']=='MOVED':
        return ('已有实际位移{:.3f}°（编码器{}个码值），按停止原因和跟随误差判断是否继续；'
                '本结果不构成主动运动验收。'.format(metrics['total_span_deg'],
                                                   len(metrics['position_codes'])))
    if metrics['assessment']=='NO_MOTION_REQUIRED':
        return '保持试验未要求位移；本结果只说明短时保持判据，不是主动运动证明。'
    return '没有可分类的输出/位移记录；先检查日志和预检。'


def metrics_sentence(metrics,reason):
    if not metrics or not metrics.get('enabled_samples'):
        return '未进入使能采样；停止原因：{}。'.format(reason)
    return ('使能采样{:.2f}s；实际行程{:.4f}°（{}个编码器码值，分辨率{:.4f}°）；'
            '跟随误差峰值{:.3f}°；指令峰值{:.3f}Nm/请求峰值{}；'
            '到达编码上限{}/{}周期（{:.2f}s，{:.1f}%）；判定{}；停止原因：{}。'
            .format(metrics['sampled_span_s'],metrics['total_span_deg'],
                    len(metrics['position_codes']),metrics['encoder_resolution_deg'],
                    metrics['max_error_deg'],metrics['peak_encoded_nm'] or 0.,
                    ('—' if metrics['peak_requested_nm'] is None
                     else '{:.3f}Nm'.format(metrics['peak_requested_nm'])),
                    metrics['cycles_at_encoded_cap'],metrics['output_sample_count'],
                    metrics['time_at_encoded_cap_s'],100*metrics['share_at_encoded_cap'],
                    metrics['assessment'],reason))


def metrics_fields(c,measurements,round_trip):
    """Facts for one report, computed from the recorded production samples."""
    metrics=motion_summary(measurements,cap_nm=c.envelopes[3].torque,
        encoded_cap_nm=encoded_cap_torque_nm(c.envelopes[3].torque,c.profiles[3].tmax),
        encoder_resolution_deg=math.degrees(2*c.profiles[3].pmax/65535),
        motion_required=round_trip,
        following_error_deg=math.degrees(c.envelopes[3].following_error))
    return (metrics,stop_reason_code(c.reason,metrics),metrics_sentence(metrics,c.reason),
            next_step_hint(metrics))


def attach_metrics(result,c,measurements,round_trip,audit):
    """Add output/motion facts and the stop classification exactly once."""
    metrics,code,summary,hint=metrics_fields(c,measurements,round_trip)
    tier=c.tier
    result.update(max_observed_error_deg=max((abs(v['error_deg']) for v in measurements),default=None),
        sample_count=len(measurements),torque_cap_nm=c.envelopes[3].torque,
        encoded_torque_cap_nm=encoded_cap_torque_nm(c.envelopes[3].torque,c.profiles[3].tmax),
        stop_reason_code=code,stop_summary=summary,
        next_step_hint=hint,motion_metrics=metrics,
        pd_tier=tier.name if tier else None,host_kp=tier.kp if tier else None,
        host_kd=tier.kd if tier else None,
        pd_tier_hardware_validated=tier.hardware_validated if tier else None)
    audit('TRIAL_METRICS',stop_reason_code=code,stop_summary=summary,
          next_step_hint=hint,pd_tier=result.get('pd_tier'),
          host_kp=result.get('host_kp'),host_kd=result.get('host_kd'),
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
    try:
        if cancel.is_set():raise RuntimeError('开始前已取消；未使能')
        session=build_session(reader,request,audit,clock)
        session.start();c=session.controller
        tier=c.tier
        if tier is None:
            raise RuntimeError('Host PD tier not bound to this session')
        audit('TRIAL_STARTED',request_id=request.request_id,axes=[4],max_runtime_s=runtime,
              targets_deg=[math.degrees(q) for q in c.targets],candidate_cap_nm=tier.torque_cap,torque_slew_nm_s=tier.slew_nm_s,
              candidate_host_kp=tier.kp,candidate_host_kd=tier.kd,
              pd_tier=tier.name,pd_tier_hardware_validated=tier.hardware_validated,
              pd_tier_explicitly_selected=True,encoded_cap_nm=encoded_cap_torque_nm(
                  c.envelopes[3].torque,c.profiles[3].tmax),
              gravity_ff_nm=0.,drive_kp=0.,drive_kd=0.,
              feedback_torque_abort_nm=2*tier.torque_cap,following_error_abort_deg=tier.following_error_deg,
              command_speed_limit_rad_s=.02,feedback_speed_abort_rad_s=tier.feedback_speed_abort)
        began=clock();last_sample=None;stage='takeover';dwell_until=None
        outward_reached=False;return_reached=False
        while c.state not in ('DISABLED','UNKNOWN'):
            if cancel.is_set():session.stop()
            if clock()-began>runtime+.7:
                session.stop();c.state='UNKNOWN';break
            f=reader.state.feedback.get(4)
            if f and abs(f['torque_feedback_nm'])>2*tier.torque_cap:
                c.stop('J4_feedback_torque_abort_threshold',clock())
            session.cycle()
            now=clock()
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
        metrics=attach_metrics(result,c,measurements,round_trip,audit)
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
                    session.cycle();sleep(.001)
            if c.state not in ('IDLE','DISABLED'):
                c.state='UNKNOWN'
            result['final_controller_state']=c.state
            result['fresh_stop_confirmation']=c.state=='DISABLED'
            session.endpoint.closed=True  # Keep the Reader for explicit read-only observation.
            if ('motion_metrics' not in result and measurements
                    and hasattr(c,'profiles') and hasattr(c,'envelopes')):
                # An early exception skipped the normal metrics block; still
                # report the recorded output/motion facts rather than nothing.
                try:
                    attach_metrics(result,c,measurements,round_trip,audit)
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
        '总编码力矩按所选档位限幅：默认0.30Nm；1Nm档误差2°；2Nm档误差4°、斜坡1Nm/s。反馈速度中止0.20rad/s，目标限速0.02rad/s。\n'
        '主机PD档位必须明确选择：默认已验证档Kp=4；Kp=12、1Nm及2Nm候选档未实机验证，需单独勾选确认，不会自动升档。\n'
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
                        (TWO_NM_PD_TIER.name,TWO_NM_PD_TIER.label)):
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
