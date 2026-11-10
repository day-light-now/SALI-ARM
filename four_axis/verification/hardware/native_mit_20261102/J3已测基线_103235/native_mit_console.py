"""Explicit DMMC02/USB-CAN bench entry for OpenArm-style drive-side MIT.

No automatic connection, enable, parameter write, home command or J5 access.
The host's 2Nm observed-state estimate guard is NOT a drive torque clamp.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import math
import queue
import threading
import time
import uuid

from .core import ROOT, LIMITS
from .clock import now as clock_now
from .actuation import Envelope
from .actuation_io import ControlSession
from .native_actuation import NativeMITController
from .openarm_mit import NativeMITAcceptance, OFFICIAL_COMMIT, NATIVE_KP, NATIVE_KD, NATIVE_SPEED_ABORT
from .live_trial import fill_preflight, capture_post_frame
from .watchdog_check import make_event_writer
from four_axis_monitor import Reader, load_mapping, usb_packet

POLICY = ROOT/'verification/hardware/native_mit_20261102/输出策略说明.md'


@dataclass(frozen=True)
class NativeTrialRequest:
    support: bool
    clearance: bool
    native_output_ack: bool
    request_id: str
    mode: str = 'hold'
    joint: int = 4


def build_native_session(reader, request, audit, clock=clock_now):
    if not isinstance(request, NativeTrialRequest) or request.mode not in ('hold', 'roundtrip10'):
        raise ValueError('Explicit native hold/roundtrip request required')
    if not POLICY.is_file():
        raise ValueError('Native output policy document missing')
    authority = NativeMITAcceptance(request.support, request.clearance,
        request.native_output_ack, request.request_id, str(POLICY), joint=request.joint)
    authority.validate()
    mapping = load_mapping()
    if reader.state.mapping != mapping:
        raise ValueError('Mapping changed; reconnect')
    envelopes=[]
    for i in range(1,5):
        if not reader.state.ready(i) or i not in reader.state.feedback:
            raise ValueError(f'J{i}: fresh configuration incomplete')
        q=mapping[i]['hardware_sign']*(reader.state.feedback[i]['position_rad']-
                                      mapping[i]['zero_offset_rad'])
        low,high=(4.,14.) if i==request.joint else (2.,2.)
        if not LIMITS[i-1][0]+low <= math.degrees(q) <= LIMITS[i-1][1]-high:
            raise ValueError(f'J{i}: local path outside model bounds; no automatic wrap/home')
        envelopes.append(Envelope(q-math.radians(low), q+math.radians(high),.02,
            2. if i==request.joint else .5,math.radians(4 if i==request.joint else 2),
            NATIVE_KP if i==request.joint else 4.,NATIVE_KD if i==request.joint else .5,50.,
            feedback_speed=NATIVE_SPEED_ABORT if i==request.joint else .1,arrival_error=math.radians(.5)))
    return ControlSession(reader,envelopes,authority,audit,clock,
        controller_factory=NativeMITController,active_axes=(request.joint,),max_runtime=45.,ramp=1.)


def run_native_trial(reader, request, audit, cancel, *, connection_id,
                     prior_hold=None, clock=clock_now, sleep=time.sleep):
    """One owner of the Reader. Never called merely by opening the window."""
    joint=request.joint;index=joint-1
    session=None;samples=[];stage='takeover';outward=False;returned=False
    hold_seen=False;post=None
    result=dict(backend='drive_mit',joint=joint,enabled_axes_permitted=[joint],request_id=request.request_id,mode=request.mode,
        connection_id=connection_id,official_commit=OFFICIAL_COMMIT,
        readonly_log=str(reader.path),
        drive_kp_requested=NATIVE_KP,drive_kd_requested=NATIVE_KD,tau_ff_requested_nm=0.,
        feedback_speed_abort_rad_s=NATIVE_SPEED_ABORT,reference_speed_limit_rad_s=.02,
        host_total_torque_clamp=False,observed_state_estimate_guard_nm=2.,
        driver_current_clamp_verified=False,j5_queried=False,parameter_writes=0,
        takeover_stability_verified=False,
        hardware_motion_accepted=False,native_hold_criterion_met=False,
        round_trip_criterion_met=False,fresh_stop_confirmation=False)
    try:
        # A roundtrip includes its own fresh takeover/stability check. A saved
        # hold result is not required and never substitutes for live feedback.
        fill_preflight(reader,cancel,clock,sleep)
        if cancel.is_set():raise ValueError('Cancelled before takeover')
        session=build_native_session(reader,request,audit,clock)
        session.start();c=session.controller;began=clock();last_sample=None;dwell=None
        audit('NATIVE_TRIAL_STARTED',**result)
        while c.state not in ('DISABLED','UNKNOWN'):
            now=clock()
            if cancel.is_set():session.stop()
            if now-began>45.7:
                c.stop('native_runner_deadline',now)
                if c.state not in ('DISABLED','UNKNOWN'):c.state='UNKNOWN'
                break
            feedback=reader.state.feedback.get(joint)
            if feedback and abs(feedback['torque_feedback_nm'])>4.:
                c.stop(f'J{joint}_native_feedback_torque_abort',now)
            session.cycle();now=clock()
            if c.state=='HOLD':
                hold_seen=True
                if request.mode=='hold' and now-began>=2.:
                    c.stop('native_hold_complete',now)
                elif request.mode=='roundtrip10':
                    if stage=='takeover' and now-began>=2.:
                        recent=[s for s in samples if now-began-.5<=s['at_s']<=now-began]
                        stable=bool(len(recent)>=2 and recent[-1]['at_s']-recent[0]['at_s']>=.4
                            and all(s['state']==1 and abs(s['error_deg'])<=.5 for s in recent))
                        if not stable:
                            c.stop('native_takeover_not_stable',now)
                        else:
                            result['takeover_stability_verified']=True
                            audit('NATIVE_TAKEOVER_STABLE',sample_count=len(recent),
                                  span_s=recent[-1]['at_s']-recent[0]['at_s'])
                            target=list(c.anchor);target[index]+=math.radians(10)
                            c.trajectory(target,18.,now);stage='outward'
                    elif stage=='outward':
                        outward=True;stage='outward_dwell';dwell=now+.5
                    elif stage=='outward_dwell' and now>=dwell:
                        c.trajectory(c.anchor,18.,now);stage='return'
                    elif stage=='return':
                        returned=True;stage='return_dwell';dwell=now+.5
                    elif stage=='return_dwell' and now>=dwell:
                        c.stop('native_round_trip_complete',now);stage='complete'
            if c.last_time!=last_sample and c.enabled:
                last_sample=c.last_time;o=c.last_snapshot.rows[index]
                samples.append(dict(at_s=now-began,angle_rad=o.q,state=o.state,
                    error_deg=math.degrees(c.targets[index]-o.q),
                    motor_velocity_rad_s=o.velocity,command=dict(c.output.get(joint,{}))))
            sleep(.001)
        result.update(reason=c.reason,hold_state_observed=hold_seen,
                      outward_reached=outward,return_reached=returned,stage=stage)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if session:
            c=session.controller
            state=dict(frames=[],seen=set(),meta=dict(cc_new_frames=0,cycle_frames=0,passive_frames=0))
            if c.state not in ('IDLE','DISABLED','UNKNOWN'):
                session.stop();end=clock()+.4
                while clock()<end and c.state not in ('DISABLED','UNKNOWN'):
                    session.cycle();capture_post_frame(state,reader,c,'stop_cycle',joint=joint);sleep(.001)
            if c.state not in ('IDLE','DISABLED'):c.state='UNKNOWN'
            capture_post_frame(state,reader,c,'passive',joint=joint)
            if c.state=='DISABLED':
                end=clock()+.3
                while clock()<end:
                    packet=usb_packet(joint)
                    try:
                        if reader.serial.write(packet)!=len(packet):raise IOError('partial CC read')
                        reader.tx+=1
                        reader.event(direction='tx',can_id=0x7FF,payload=packet[21:29].hex(' '))
                        session._drain()
                        captured=capture_post_frame(state,reader,c,'cc_poll',joint=joint)
                        if captured and reader.state.feedback[joint]['state_code']!=0:
                            c.state='UNKNOWN';break
                    except Exception as exc:
                        c.state='UNKNOWN';result['post_stop_error']=str(exc);break
                    sleep(.01)
            post=state['frames'];last=post[-1] if post else None
            final_error=(last['angle_deg']-math.degrees(c.anchor[index])
                         if last and hasattr(c,'anchor') else None)
            result.update(final_controller_state=c.state,fresh_stop_confirmation=c.state=='DISABLED',
                final_error_from_start_deg=final_error,post_stop_visible_frames=post,
                post_stop_first_frame_guaranteed=False,
                post_stop_observation_sufficient=len(post)>=2)
            tail=[s for s in samples if 1.5<=s['at_s']<=2.05 and s['state']==1]
            stable=bool(len(tail)>=2 and tail[-1]['at_s']-tail[0]['at_s']>=.4
                        and all(abs(s['error_deg'])<=.5 for s in tail))
            peak=max((math.degrees(s['angle_rad']-c.anchor[index]) for s in samples),default=0.)
            result.update(actual_positive_excursion_deg=peak,
                native_hold_criterion_met=bool(request.mode=='hold' and stable and hold_seen
                    and c.reason=='native_hold_complete' and c.state=='DISABLED'
                    and final_error is not None and abs(final_error)<=.5),
                round_trip_criterion_met=bool(request.mode=='roundtrip10' and outward and returned
                    and peak>=9.5 and c.reason=='native_round_trip_complete'
                    and c.state=='DISABLED' and final_error is not None and abs(final_error)<=.5))
            session.endpoint.closed=True
        else:result['final_controller_state']='IDLE'
        result['sample_count']=len(samples)
        result['max_observed_error_deg']=max((abs(s['error_deg']) for s in samples),default=None)
        result['peak_observed_speed_rad_s']=max((abs(s['motor_velocity_rad_s']) for s in samples),default=None)
        result['peak_observed_estimate_nm']=max((abs(s['command'].get('observed_state_estimate_nm',0.))
                                                for s in samples),default=None)
        audit('NATIVE_TRIAL_RESULT',**result)
    return result


def main():
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI · OpenArm原生MIT · J3/J4 · DMMC02');root.geometry('1120x790')
    ui=ttk.Frame(root,padding=16);ui.pack(fill='both',expand=True)
    ttk.Label(ui,text='OpenArm原生MIT：位置/速度/增益交给电机驱动器',
              font=('Microsoft YaHei',16,'bold')).pack(anchor='w')
    ttk.Label(ui,text='DMMC02 / USB-CAN；一次只使能所选J3或J4，其他轴只监视，J5不查询。不会自动回零。\n'
        f'驱动Kp={NATIVE_KP:g}、Kd={NATIVE_KD:g}，1秒增益斜坡；目标速度≤0.02rad/s、反馈速度中止{NATIVE_SPEED_ABORT:g}rad/s、误差中止4°。\n'
        '重要：2Nm仅是当前反馈状态的请求估算护栏，不是电机内部连续总力矩硬限幅；驱动电流限制尚未核实。',
        wraplength=1070).pack(anchor='w',pady=10)
    port=tk.StringVar(value='COM9');bar=ttk.Frame(ui);bar.pack(fill='x')
    ttk.Entry(bar,textvariable=port,width=15).pack(side='left')
    selected_axis=tk.IntVar(value=3);axis_buttons=[]
    axis_bar=ttk.Frame(ui);axis_bar.pack(fill='x',pady=6)
    ttk.Label(axis_bar,text='本轮控制轴：').pack(side='left')
    for number,label in ((3,'J3（本轮待验证）'),(4,'J4（已完成一次往返）')):
        button=ttk.Radiobutton(axis_bar,text=label,variable=selected_axis,value=number)
        button.pack(side='left',padx=10);axis_buttons.append(button)
    support=tk.BooleanVar();clearance=tk.BooleanVar();ack=tk.BooleanVar()
    def axis_changed(*args):
        clearance.set(False);ack.set(False)
    selected_axis.trace_add('write',axis_changed)
    for var,label in [(support,'全臂与末端有支撑，可立即现场断电，人员离开转动范围'),
                      (clearance,'所选轴起点−4°至+14°没有干涉，支撑不会夹住转轴'),
                      (ack,'明确同意原生MIT台架试验：理解主机无法保证连续2Nm总输出，过流保护不等于连续限流')]:
        ttk.Checkbutton(ui,text=label,variable=var).pack(anchor='w',pady=4)
    # Pack the growing log after the controls, so it cannot hide Stop on a
    # smaller desktop. The log gets the remaining space.
    log=tk.Text(ui,height=18,wrap='word')
    events=queue.Queue();requests=queue.Queue();cancel=threading.Event();disconnect=threading.Event()
    worker=[None];connected=[False];busy=[False];latched=[False];closing=[False]
    def emit(text):events.put(('log',text))
    def work(port_name):
        reader=None;prior_hold=None;connection_id=uuid.uuid4().hex
        try:
            reader=Reader(port_name,load_mapping());reader.serial.write_timeout=.025
            fill_preflight(reader,cancel);events.put(('connected',True))
            emit('只读连接；请核对四轴状态。电机使能需单独点击，参数不会自动修改。')
            last=0.
            while not disconnect.is_set():
                now=clock_now();old=reader.tx;reader.tick(now)
                if reader.tx!=old:reader.next_send=now+.01
                try:req=requests.get_nowait()
                except queue.Empty:req=None
                if req:
                    folder=ROOT/'tmp/native_mit';folder.mkdir(parents=True,exist_ok=True)
                    path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+req.request_id[:8]+'.jsonl')
                    with path.open('x',encoding='utf-8') as handle:
                        audit=make_event_writer(handle,lambda row:None)
                        emit(f'本轮仅使能J{req.joint}：当前位置接管并检查稳定后执行所选动作。')
                        result=run_native_trial(reader,req,audit,cancel,
                            connection_id=connection_id,prior_hold=prior_hold)
                    path.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
                    if req.mode=='hold':prior_hold=result
                    events.put(('latched',result['final_controller_state']=='UNKNOWN'))
                    emit(json.dumps(result,ensure_ascii=False));emit('结果：'+str(path.with_suffix('.json')))
                    events.put(('busy',False))
                    if result['final_controller_state']=='UNKNOWN':
                        emit('输出状态未知：停止轮询并关闭串口；保持支撑，现场检查并必要时切电。')
                        break
                if now-last>.5:
                    lines=[]
                    for i in range(1,5):
                        f=reader.state.feedback.get(i,{});r=reader.state.regs[i]
                        lines.append(f'J{i} 模式{r.get(10)} / TIMEOUT {r.get(9)} / 状态{f.get("state_code")}')
                    events.put(('status','；'.join(lines)+'；J5未查询'));last=now
                time.sleep(.001)
        except Exception as exc:emit('连接/试验结束：'+str(exc))
        finally:
            if reader:reader.close()
            events.put(('connected',False));events.put(('busy',False))
    def connect():
        if worker[0] and worker[0].is_alive():return
        # Never carry an old connection's queued enable ticket into a new one.
        try:
            while True:requests.get_nowait()
        except queue.Empty:pass
        cancel.clear();disconnect.clear();latched[0]=False
        worker[0]=threading.Thread(target=work,args=(port.get().strip(),),daemon=True);worker[0].start()
    ttk.Button(bar,text='连接·只读',command=connect).pack(side='left',padx=8)
    def stop():cancel.set()
    def detach():cancel.set();disconnect.set()
    ttk.Button(bar,text='停止并断开',command=detach).pack(side='left')
    status=tk.StringVar(value='未连接；先释放达妙工具的串口。')
    ttk.Label(ui,textvariable=status,wraplength=1070).pack(anchor='w')
    buttons=ttk.Frame(ui);buttons.pack(fill='x',pady=10)
    def request(mode):
        if not connected[0] or busy[0] or latched[0]:return
        if not (support.get() and clearance.get() and ack.get()):
            status.set('请落实并逐条确认支撑、净空和原生MIT输出区别。');return
        busy[0]=True;cancel.clear()
        requests.put(NativeTrialRequest(True,True,True,uuid.uuid4().hex,mode,joint=selected_axis.get()))
        for button in axis_buttons:button.config(state='disabled')
    hold=ttk.Button(buttons,text='原生MIT当前位置保持2秒',command=lambda:request('hold'),state='disabled')
    trip=ttk.Button(buttons,text='原生MIT 接管稳定→+10°→起点',command=lambda:request('roundtrip10'),state='disabled')
    hold.pack(side='left');trip.pack(side='left',padx=8)
    ttk.Button(buttons,text='停止试验·尝试全停',command=stop).pack(side='left')
    log.pack(fill='both',expand=True,pady=8)
    def poll():
        try:
            while True:
                kind,value=events.get_nowait()
                if kind=='log':log.insert('end',value+'\n');log.see('end')
                elif kind=='status':status.set(value)
                elif kind=='connected':connected[0]=value
                elif kind=='latched':latched[0]=value
                elif kind=='busy':
                    busy[0]=value
                    if not value:ack.set(False)
        except queue.Empty:pass
        enabled='normal' if connected[0] and not busy[0] and not latched[0] else 'disabled'
        hold.config(state=enabled);trip.config(state=enabled)
        for button in axis_buttons:button.config(state='disabled' if busy[0] else 'normal')
        if closing[0] and (worker[0] is None or not worker[0].is_alive()):root.destroy();return
        root.after(60,poll)
    def close():closing[0]=True;detach()
    root.protocol('WM_DELETE_WINDOW',close);root.after(60,poll);root.mainloop()


if __name__=='__main__':main()
