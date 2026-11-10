"""Operator-started J4 near-zero-output watchdog experiment, not motion control.

No port is opened on import/window launch. No parameter writes or clear-fault.
All enabling is restricted to J4. Passive support is required: this cannot hold
the arm against gravity. D-state observation alone does not prove torque-off.
"""
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
import uuid

from .core import ROOT
from .clock import now as clock_now
from .actuation import USB2CANEndpoint
from .commissioning import Profile, JointCommand, encode_shadow

sys.path.insert(0,str(ROOT/'verification/hardware'))
from four_axis_monitor import Reader, load_mapping, usb_packet


def make_event_writer(handle, publish):
    """The same audit callback is used by the window and offline regression."""
    def event(event_type,**fields):
        row=dict(fields)
        if 'kind' in row:
            row['frame_kind']=row.pop('kind')
        row.update(kind=event_type,unix_s=time.time())
        handle.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');handle.flush()
        if event_type in ('PHASE','RESULT'):publish(json.dumps(row,ensure_ascii=False))
    return event


class J4WatchdogCheck:
    def __init__(self,reader,event,stop,clock=clock_now,sleep=time.sleep):
        self.reader,self.event,self.stop = reader,event,stop
        self.clock,self.sleep = clock,sleep
        self.endpoint=USB2CANEndpoint(reader.serial,event,clock)
        self.enable_attempted=False
        self.last_control=None
        self.anchor=None
        self.zero_frame=None
        self.last_seen=None
        self.trace=[]
        self.preflight={}

    def _cancel(self):
        if self.stop.is_set():raise RuntimeError('操作员取消')

    def _drain(self):
        r=self.reader
        for cid,data in r.stream.feed(r.serial.read(r.serial.in_waiting)):
            at=self.clock();r.rx+=1
            self.event('RX',at=at,can_id=cid,payload=data.hex(' '))
            r.state.feed(cid,data,at)
            if data[0]&15==4 and r.state.at.get(4)==at:
                self.trace.append(dict(at=at,state=r.state.feedback[4]['state_code']))

    def _query(self,i):
        packet=usb_packet(i)
        if self.reader.serial.write(packet)!=30:raise IOError('只读查询写入不完整')
        self.event('QUERY',at=self.clock(),joint=i,can_id=0x7FF,payload=packet[21:29].hex(' '))

    def _write(self,payload,kind):
        # This experiment cannot expand to another axis through its public API.
        receipt=self.endpoint.send(4,payload,uuid.uuid4().hex,kind)
        if kind=='mit':self.last_control=receipt['finished_at']

    def prepare(self):
        self.event('PHASE',message='只读预检四轴；不改参数')
        deadline=self.clock()+5
        while self.clock()<deadline:
            self._cancel();at=self.clock()
            previous=self.reader.tx
            self.reader.tick(at)
            if self.reader.tx!=previous:self.reader.next_send=at+.01
            if all(self.reader.state.ready(i) and i in self.reader.state.at
                   and 0<=at-self.reader.state.at[i]<=.06 for i in (1,2,3,4)):
                break
            self.sleep(.001)
        else:raise RuntimeError('四轴参数或新鲜反馈不齐；未使能')
        s=self.reader.state
        self.preflight={str(i):dict(mode=s.regs[i][10],timeout=s.regs[i][9],
            state=s.feedback[i]['state_code'],age_s=self.clock()-s.at[i]) for i in (1,2,3,4)}
        self.event('PREFLIGHT_AXES',axes=self.preflight)
        rejected=[]
        for i in (1,2,3,4):
            reg=s.regs[i];f=s.feedback[i]
            if reg[10]!=1 or reg[9]!=2000 or reg[8]!=i or f['state_code']!=0:
                rejected.append(f'J{i}:模式={reg[10]},TIMEOUT={reg[9]},状态={f["state_code"]}')
        if rejected:
            raise RuntimeError('预检不符（要求模式1/TIMEOUT2000/状态0）：'
                               +'；'.join(rejected)+'；未使能，未自动改参数')
        r=s.regs[4]
        if (r[21],r[22],r[23])!=(12.5,45.,54.):
            raise RuntimeError('J4协议范围与本次限定试验不符')
        self.anchor={i:s.feedback[i]['position_rad'] for i in (1,2,3,4)}
        m=s.mapping[4]
        p=Profile(4,1,r[7],2000,r[21],r[22],r[23],int(m['hardware_sign']),m['zero_offset_rad'])
        q=p.sign*(self.anchor[4]-p.zero)
        out=encode_shadow(p,JointCommand(q,speed_limit=.02))
        self.zero_frame=bytes.fromhex(out['candidate_payload'])
        residual=out['decoded_tau_ff_nm']
        if abs(residual)>.014:raise RuntimeError('近零力矩量化残余超限')
        self.event('PREPARED',anchor_encoder_rad=self.anchor,residual_request_nm=residual,
                   note='kp=kd=0; nonzero quantization residual; no holding ability')

    def _guard(self,allow_j4,require_others=True):
        s=self.reader.state;now=self.clock()
        for i in (1,2,3,4):
            if i not in s.feedback or not s.ready(i):raise RuntimeError('反馈/配置缺失')
            f=s.feedback[i];age=now-s.at[i]
            if (i==4 or require_others) and not 0<=age<=.08:
                raise RuntimeError(f'J{i}反馈过期')
            if f['state_code'] not in (allow_j4 if i==4 else (0,)):
                raise RuntimeError(f'J{i}异常状态{f["state_code"]}')
            if abs(f['position_rad']-self.anchor[i])>math.radians(2):
                raise RuntimeError(f'J{i}相对起点移动超过2度，停止试验')
            if abs(f['velocity_rad_s'])>.1 or max(f['driver_temp_c'],f['motor_temp_c'])>=50:
                raise RuntimeError(f'J{i}速度/温度超限')
            if i==4 and abs(f['torque_feedback_nm'])>.3:
                raise RuntimeError('J4反馈力矩超过本次诊断阈值')

    def run(self):
        result=dict(watchdog_d_observed=False,physical_torque_off_verified=False,
                    measured_watchdog_latency_ms=None,joint=4,other_axes_enabled=False,
                    hardware_parameter_writes=0)
        try:
            self.prepare()
            # Prime zero gains before FC. Whether this firmware caches it is
            # itself unaccepted; support and immediate post-FC zero stream apply.
            self.event('PHASE',message='J4预装零增益指令；其他轴不使能')
            self._write(self.zero_frame,'mit')
            self._drain();self._guard((0,))
            self._cancel()
            self.enable_attempted=True
            enabled_at=self.clock()
            self._write(b'\xff'*7+b'\xfc','enable')
            self._write(self.zero_frame,'mit')
            until=enabled_at+.2;next_send=self.clock()+.01;next_query=self.clock()
            query_index=0;enabled_seen=False
            self.event('PHASE',message='J4短时使能、零增益近零力矩；不会保持自重')
            while self.clock()<until:
                self._cancel();self._drain();now=self.clock()
                self._guard((0,1))
                if self.reader.state.feedback[4]['state_code']==1 and self.reader.state.at[4]>enabled_at:
                    enabled_seen=True
                if now-self.last_control>.04:raise RuntimeError('主机/USB发送间隔超期')
                if now>=next_send:
                    self._write(self.zero_frame,'mit');next_send=self.clock()+.01
                if now>=next_query:
                    self._query((1,2,3)[query_index%3]);query_index+=1;next_query=self.clock()+.01
                self.sleep(.001)
            if not enabled_seen:raise RuntimeError('没有新的J4已使能反馈，不能评价看门狗')
            last=self.last_control
            self.event('PHASE',message='停止所有发送250ms，仅被动接收，观察失联保护')
            # No CC or any other TX in the quiet interval: queries may affect
            # firmware watchdog behavior. No automatic re-enable or fault clear.
            while self.clock()<last+.25:
                self._cancel();self._drain()
                # Do not require fresh packets while intentionally silent.
                f=self.reader.state.feedback[4]
                if f['state_code'] not in (0,1,13):raise RuntimeError('J4出现其他故障')
                if abs(f['position_rad']-self.anchor[4])>math.radians(2):
                    raise RuntimeError('静默期J4位移超限')
                self.sleep(.001)
            self._query(4)
            deadline=self.clock()+.12
            while self.clock()<deadline:
                self._cancel();self._drain()
                if self.reader.state.at.get(4,0)>last+.2:break
                self.sleep(.001)
            post_at=self.reader.state.at.get(4,0)
            code=self.reader.state.feedback[4]['state_code']
            spontaneous=[e for e in self.trace if last<e['at']<last+.25 and e['state']==13]
            result.update(watchdog_d_observed=post_at>last+.2 and code==13,
                final_observed_state=code,query_observation_delay_ms=(post_at-last)*1000,
                unsolicited_d_delay_ms=(spontaneous[0]['at']-last)*1000 if spontaneous else None,
                note='Host-observed D only; query delay is not watchdog latency or torque-off proof')
            if not result['watchdog_d_observed']:
                result['error']='未观察到通信保护D状态，不能通过；不自动改参数或重试'
        except Exception as exc:
            result['error']=str(exc)
        finally:
            if self.enable_attempted:
                # Best effort FD even when cancellation or logging caused failure.
                try:self._write(b'\xff'*7+b'\xfd','disable')
                except Exception as exc:result['disable_delivery_unknown']=str(exc)
                try:
                    self._query(4);deadline=self.clock()+.1
                    while self.clock()<deadline:self._drain();self.sleep(.002)
                    result['cleanup_state']=self.reader.state.feedback.get(4,{}).get('state_code')
                except Exception:result['cleanup_state']='unknown'
            result['preflight_axes']=self.preflight
            result['enable_attempted']=self.enable_attempted
            result['readonly_log']=str(self.reader.path)
            self.event('RESULT',**result)
        return result


def main():
    # UI only: importing/testing the module never creates a window or port.
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI · J4看门狗保护试验（非运动控制）');root.geometry('860x600')
    ui=ttk.Frame(root,padding=20);ui.pack(fill='both',expand=True)
    ttk.Label(ui,text='J4 看门狗保护试验',font=('Microsoft YaHei',18,'bold')).pack(anchor='w')
    ttk.Label(ui,text='先断开我们的只读连接和达妙工具。仅J4短时使能，零增益、近零力矩；不会托住自重。\n'
        'J1–J3不使能，J5不查询。不设零、不回零、不写参数、不清故障、不保存Flash。\n'
        '有意停止控制发送250ms，观察D状态；这不能单独证明实际力矩已消失。',wraplength=800).pack(anchor='w',pady=12)
    supported=tk.BooleanVar(value=False)
    ttk.Checkbutton(ui,text='我确认所有连杆和末端已被机械支撑接住，人员离开转动范围，并可立即切断电机电源',variable=supported).pack(anchor='w')
    port=tk.StringVar(value='COM9');ttk.Entry(ui,textvariable=port,width=16).pack(anchor='w',pady=8)
    status=tk.StringVar(value='未连接；打开本窗口不会发送任何指令。')
    ttk.Label(ui,textvariable=status,wraplength=800).pack(anchor='w',pady=8)
    log=tk.Text(ui,height=15,wrap='word');log.pack(fill='both',expand=True)
    events=queue.Queue();cancel=threading.Event();worker=[None];closing=[False]
    def run(port_name):
        reader=None;handle=None
        folder=ROOT/'tmp/watchdog_check';folder.mkdir(parents=True,exist_ok=True)
        path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jsonl')
        try:
            handle=path.open('x',encoding='utf-8')
            event=make_event_writer(handle,events.put)
            reader=Reader(port_name,load_mapping())
            reader.serial.write_timeout=.025
            result=J4WatchdogCheck(reader,event,cancel).run()
            path.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            events.put('结果文件：'+str(path.with_suffix('.json')))
            events.put('试验结束。请检查机械状态；发生异常或状态未知时，保持支撑并现场切断电机电源。')
        except Exception as exc:events.put('试验中止：'+str(exc))
        finally:
            try:
                if reader:reader.close()
            finally:
                try:
                    if handle:handle.close()
                finally:events.put(None)
    def start():
        if worker[0] and worker[0].is_alive():return
        if not supported.get():status.set('请先落实机械支撑与现场切电条件，再勾选确认。');return
        cancel.clear();start_button.config(state='disabled')
        status.set('试验进行中；异常时立即现场切电。')
        worker[0]=threading.Thread(target=run,args=(port.get().strip(),),daemon=False);worker[0].start()
    start_button=ttk.Button(ui,text='开始J4保护试验（会短时使能J4）',command=start);start_button.pack(side='left',pady=10)
    ttk.Button(ui,text='取消并尝试失能J4',command=cancel.set).pack(side='left',padx=10)
    def poll():
        try:
            while True:
                message=events.get_nowait()
                if message is None:
                    status.set('已结束并释放串口；查看结果文件。');start_button.config(state='normal')
                    if closing[0]:root.destroy();return
                else:log.insert('end',message+'\n');log.see('end')
        except queue.Empty:pass
        root.after(100,poll)
    def close():
        if worker[0] and worker[0].is_alive():
            closing[0]=True;cancel.set();status.set('正在取消、尝试失能并释放串口，请勿把关闭窗口当作硬件急停。')
        else:root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);root.after(100,poll);root.mainloop()


if __name__=='__main__':main()
