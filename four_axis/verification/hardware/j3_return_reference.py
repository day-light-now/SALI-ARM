"""J3-only bounded reference return. Opens disconnected; never writes motor parameters.

Transport: locally reviewed dmBots/motor-sdk u2can framing. Requires matching
live registers, fresh disabled feedback and an explicit supported-arm checkbox.
Torque limits below constrain encoded requests, not independently measured torque.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import struct
import sys
import threading
import time

from feedback_decode import decode_feedback

WORKSPACE = Path(__file__).resolve().parents[2]
REFERENCE = -0.4289692530708784
EXPECTED = {7: 0, 8: 3, 9: 2000, 10: 1, 21: 12.5, 22: 45., 23: 54.}
READS = frozenset(EXPECTED)
DT = .02
CAP = .30
KP, KD = 4., .5
DURATION, SETTLE = 5., 1.
STALE = .080
MAX_SPEED = .15
ENABLE = bytes.fromhex('ff ff ff ff ff ff ff fc')
DISABLE = bytes.fromhex('ff ff ff ff ff ff ff fd')


def reference_matches():
    """Refuse if the authoritative mapping no longer matches this one-shot tool."""
    text = (WORKSPACE/'src/slaihand_bringup/config/hardware_mapping.yaml').read_text(encoding='utf-8')
    section = text.split('  slaihand_joint3:', 1)[1].split('  slaihand_joint4:', 1)[0]
    values = {}
    for line in section.splitlines():
        if ':' in line:
            key, value = line.strip().split(':', 1)
            values[key] = value.strip()
    return (values.get('can_id') == '3' and values.get('hardware_sign') == '1'
            and abs(float(values['zero_offset_rad'])-REFERENCE) < 1e-12)


def request(rid=None):
    if rid is not None and rid not in READS:
        raise ValueError('Register not in read-only allowlist')
    return bytes([3, 0, 0xCC if rid is None else 0x33,
                  0 if rid is None else rid, 0, 0, 0, 0])


def torque_packet(torque):
    if not math.isfinite(torque) or abs(torque) > CAP+1e-12:
        raise ValueError('Requested torque outside fixed cap')
    raw = round((torque+54.)*4095/108.)
    decoded = lambda r: r*108./4095-54.
    while abs(decoded(raw)) > CAP:
        raw += -1 if decoded(raw) > 0 else 1
    # p approximately 0, v approximately 0, kp=kd=0: pure torque only.
    return bytes([0x7F, 0xFF, 0x7F, 0xF0, 0, 0, raw >> 8, raw & 255])


def validate_send(can_id, data):
    if len(data) != 8:
        raise ValueError('Only 8-byte frames allowed')
    if can_id == 0x7FF:
        if data == request() or any(data == request(r) for r in READS):
            return
    elif can_id == 3:
        if data in (ENABLE, DISABLE):
            return
        if data[:6] == bytes.fromhex('7f ff 7f f0 00 00') and data[6] <= 15:
            torque = ((data[6] << 8) | data[7])*108./4095-54.
            if abs(torque) <= CAP+1e-12:
                return
    raise ValueError('Frame rejected by J3-only transmit allowlist')


def usb_packet(can_id, data):
    validate_send(can_id, data)
    packet = bytearray([0x55,0xAA,0x1E,3,1,0,0,0,0x0A,0,0,0,0,
                        0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0])
    packet[13:15] = struct.pack('<H',can_id)
    packet[21:29] = data
    return bytes(packet)


class Stream:
    def __init__(self): self.buffer = bytearray()
    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] != 0xAA:
                del self.buffer[0]; continue
            if len(self.buffer) < 16: break
            if self.buffer[1] != 0x11 or self.buffer[2] != 8 or self.buffer[15] != 0x55:
                del self.buffer[0]; continue
            frame = bytes(self.buffer[:16]); del self.buffer[:16]
            frames.append((int.from_bytes(frame[3:7], 'little'),frame[7:15]))
        return frames


class Guard:
    def __init__(self, start):
        if not math.isfinite(start) or abs(start-REFERENCE) > math.radians(8):
            raise ValueError('Start is over 8 degrees from recorded reference')
        self.start = start
        self.low = min(start,REFERENCE)-math.radians(1)
        self.high = max(start,REFERENCE)+math.radians(1)

    def check(self, state, age, elapsed, allowed=(1,)):
        if age < 0 or age > STALE: raise RuntimeError('Feedback stale; stopped')
        if elapsed > DURATION+SETTLE+.15: raise RuntimeError('Runtime limit; stopped')
        if state['state_code'] not in allowed: raise RuntimeError('Unexpected motor state; stopped')
        q, v = state['position_rad'],state['velocity_rad_s']
        if not math.isfinite(q) or not math.isfinite(v): raise RuntimeError('Nonfinite feedback')
        if not self.low <= q <= self.high: raise RuntimeError('Position envelope exceeded')
        if abs(v) > MAX_SPEED: raise RuntimeError('Speed limit exceeded')
        if 'torque_feedback_nm' in state and (not math.isfinite(state['torque_feedback_nm']) or abs(state['torque_feedback_nm']) > 1.):
            raise RuntimeError('Unexpected torque feedback; stopped')
        if max(state['driver_temp_c'],state['motor_temp_c']) >= 60: raise RuntimeError('Temperature limit')

    def command(self, elapsed, state):
        u = min(1.,max(0.,elapsed/DURATION))
        s = 10*u**3-15*u**4+6*u**5
        ds = (30*u**2-60*u**3+30*u**4)/DURATION if u < 1 else 0.
        qd = self.start + (REFERENCE-self.start)*s
        vd = (REFERENCE-self.start)*ds
        tau = KP*(qd-state['position_rad'])+KD*(vd-state['velocity_rad_s'])
        tau = min(CAP,max(-CAP,tau))
        return qd,vd,tau


class Link:
    def __init__(self, port):
        deps = WORKSPACE.parent/'tmp/slai_serial_deps'
        sys.path.insert(0,str(deps))
        import serial
        self.serial = serial.Serial(port,921600,timeout=0,write_timeout=.025)
        self.serial.reset_input_buffer()
        self.stream = Stream()
        self.regs = {}
        self.latest = None
        self.at = None
        self.enable_attempted = False
        self.closed = False
        logdir = WORKSPACE/'tmp/j3_return_reference'
        logdir.mkdir(parents=True,exist_ok=True)
        self.path = logdir/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000}.jsonl')
        self.log = self.path.open('x',encoding='utf-8')

    def event(self, **data):
        self.log.write(json.dumps({'time':time.time(),**data},ensure_ascii=False)+'\n')
        self.log.flush()

    def send(self, can_id, data):
        packet = usb_packet(can_id,data)
        if data == ENABLE: self.enable_attempted = True
        if self.serial.write(packet) != len(packet): raise RuntimeError('Incomplete USB write')
        self.event(direction='tx',can_id=can_id,payload=data.hex(' '))

    def receive(self):
        for can_id,data in self.stream.feed(self.serial.read(self.serial.in_waiting)):
            self.event(direction='rx',can_id=can_id,payload=data.hex(' '))
            if can_id != 0: raise RuntimeError('Unexpected feedback CAN ID')
            if data[:3] == bytes([3,0,0x33]) and data[3] in READS:
                rid = data[3]
                self.regs[rid] = struct.unpack('<I' if rid in (7,8,9,10) else '<f',data[4:])[0]
            elif data[0]&15 == 3:
                self.latest = decode_feedback(data,12.5,45.,54.)
                self.at = time.monotonic()
            else:
                raise RuntimeError('Unexpected device/data; verify only J3 is connected')

    def prepare(self, stop):
        for rid in EXPECTED:
            if stop.is_set(): raise RuntimeError('Cancelled')
            self.send(0x7FF,request(rid))
            deadline = time.monotonic()+.6
            while rid not in self.regs:
                if stop.is_set(): raise RuntimeError('Cancelled')
                if time.monotonic()>deadline: raise RuntimeError(f'No reply for register {rid}')
                self.receive(); time.sleep(.003)
            if not math.isclose(self.regs[rid],EXPECTED[rid],rel_tol=0,abs_tol=1e-6):
                raise RuntimeError(f'Register {rid} mismatch: {self.regs[rid]}')
        self.latest = None; self.at = None
        self.send(0x7FF,request())
        deadline = time.monotonic()+.6
        while self.latest is None:
            if stop.is_set(): raise RuntimeError('Cancelled')
            if time.monotonic()>deadline: raise RuntimeError('No fresh status reply')
            self.receive(); time.sleep(.003)
        guard = Guard(self.latest['position_rad'])
        guard.check(self.latest,time.monotonic()-self.at,0,allowed=(0,))
        return guard

    def finish(self):
        """Always disable J3 after any enable attempt, even on a partial write/error."""
        errors = []
        if self.enable_attempted:
            for _ in range(3):
                try: self.send(3,DISABLE)
                except Exception as e: errors.append(str(e))
                time.sleep(.01)
            try:
                self.latest = None; self.at = None
                self.send(0x7FF,request())
                end=time.monotonic()+.15
                while time.monotonic()<end:
                    self.receive();time.sleep(.003)
                if self.latest is None or self.latest['state_code'] not in (0,13):
                    errors.append('Final disabled/timeout state NOT confirmed; use power cutoff')
            except Exception as e: errors.append('Final status: '+str(e))
        self.event(event='finish',errors=errors,final_state=self.latest)
        self.serial.close(); self.log.close(); self.closed=True
        return errors


def run_return(link, stop, publish):
    guard = link.prepare(stop)
    if stop.is_set(): raise RuntimeError('Cancelled before enable')
    if abs(guard.start-REFERENCE) <= math.radians(.3):
        return 'Already within 0.3 degrees; no enable sent'
    link.send(3,torque_packet(0.))
    link.send(3,ENABLE)
    handshake=time.monotonic()
    while True:
        if stop.is_set(): raise RuntimeError('Cancelled')
        link.receive()
        now=time.monotonic()
        guard.check(link.latest,now-link.at,0,allowed=(0,1))
        if link.latest['state_code']==1: break
        if now-handshake>.075: raise RuntimeError('Enable not confirmed')
        time.sleep(.003)
    start=time.monotonic(); previous=start
    while True:
        if stop.is_set(): raise RuntimeError('Cancelled')
        now=time.monotonic();elapsed=now-start
        if now-previous>.060: raise RuntimeError('Control loop delayed; stopped')
        previous=now
        link.receive()
        now=time.monotonic();elapsed=now-start
        guard.check(link.latest,now-link.at,elapsed)
        if elapsed>=DURATION+SETTLE: break
        if elapsed>3 and abs(guard.start-REFERENCE)>math.radians(.5) and abs(link.latest['position_rad']-guard.start)<math.radians(.15):
            raise RuntimeError('No appreciable motion; stopped without increasing torque')
        qd,vd,tau=guard.command(elapsed,link.latest)
        link.send(3,torque_packet(tau))
        link.event(event='control',elapsed=elapsed,target_rad=qd,
                   requested_torque_nm=tau,feedback=link.latest)
        publish(link.latest,tau,elapsed)
        if stop.wait(max(0.,DT-(time.monotonic()-now))): raise RuntimeError('Cancelled')
    err=math.degrees(link.latest['position_rad']-REFERENCE)
    return f'Trial finished; before disable, error {err:+.2f} deg. No continuous hold.'


def main():
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI J3 受限回软件零位 · 仅编号3');root.geometry('740x570')
    ui=ttk.Frame(root,padding=18);ui.pack(fill='both',expand=True)
    ttk.Label(ui,text='J3 回人工软件零位',font=('Microsoft YaHei',19,'bold')).pack(anchor='w')
    ttk.Label(ui,text='仅J3；不改电机参数、不设零、不自动清故障。',wraplength=680).pack(anchor='w',pady=8)
    ttk.Label(ui,text='参考 −0.428969 rad；约5秒轨迹＋1秒收敛；编码力矩上限0.30 N·m。\n输出可能不足，未到位会停止，不会自动加力。结束即失能，必须机械支撑。',wraplength=680).pack(anchor='w',pady=8)
    port=tk.StringVar(value='COM9');line=ttk.Frame(ui);line.pack(fill='x',pady=8)
    ttk.Label(line,text='USB2CAN 串口：').pack(side='left');ttk.Entry(line,textvariable=port,width=14).pack(side='left')
    checked=tk.BooleanVar(value=False)
    ttk.Checkbutton(ui,text='确认只有J3接入；支撑可靠；手已离开；可立即断电',variable=checked).pack(anchor='w',pady=10)
    status=tk.StringVar(value='未连接。先在 DMTool 点击“关闭设备”释放串口。打开本窗口不会使能。')
    feedback=tk.StringVar(value='无实时反馈')
    ttk.Label(ui,textvariable=status,wraplength=680).pack(anchor='w',pady=12)
    ttk.Label(ui,textvariable=feedback,wraplength=680,font=('Microsoft YaHei',12)).pack(anchor='w',pady=12)
    events=queue.Queue();stop=threading.Event();worker=[None];closing=[False]
    def post(state,tau,elapsed):
        events.put(('feedback',f"相对零位 {math.degrees(state['position_rad']-REFERENCE):+.2f}°；速度 {state['velocity_rad_s']:+.3f} rad/s；指令 {tau:+.3f} N·m；{elapsed:.1f}s"))
    def job(selected):
        link=None
        try:
            if not reference_matches(): raise RuntimeError('Reference/configuration changed; refusing')
            link=Link(selected)
            events.put(('status','连接后核对参数和失能状态，通过后执行一次限时回位。'))
            result=run_return(link,stop,post)
            events.put(('status',result))
        except Exception as e: events.put(('status','停止：'+str(e)))
        finally:
            if link:
                try:
                    errors=link.finish()
                    events.put(('final','结束：'+('失能确认失败，请切断电源；' if errors else '未使能或已结束并检查失能；')+'日志 '+str(link.path)+('；'+'; '.join(errors) if errors else '')))
                except Exception as e: events.put(('final','退出检查失败，请切断电源：'+str(e)))
            events.put(('done',''))
    def begin():
        if worker[0] and worker[0].is_alive(): return
        if not checked.get():status.set('先确认支撑、隔离和断电措施。');return
        stop.clear();startbutton.config(state='disabled')
        worker[0]=threading.Thread(target=job,args=(port.get().strip(),),daemon=False);worker[0].start()
    def halt():stop.set();status.set('请求停止，后台将失能J3。软件停止不是硬件急停。')
    startbutton=ttk.Button(ui,text='连接核对并执行一次 J3 受限回位',command=begin);startbutton.pack(fill='x',pady=10)
    ttk.Button(ui,text='停止并失能 J3',command=halt).pack(fill='x',pady=5)
    def close():
        closing[0]=True;stop.set()
        if not worker[0] or not worker[0].is_alive():root.destroy()
        else:status.set('等待后台失能退出；若异常，使用电源切断措施。')
    def tick():
        while not events.empty():
            kind,msg=events.get()
            if kind=='feedback':feedback.set(msg)
            elif kind=='final':feedback.set(msg)
            elif kind=='status':status.set(msg)
            elif kind=='done':startbutton.config(state='normal');checked.set(False)
        if closing[0] and (not worker[0] or not worker[0].is_alive()):root.destroy();return
        root.after(50,tick)
    root.protocol('WM_DELETE_WINDOW',close);root.after(50,tick);root.mainloop()


if __name__=='__main__':
    main()
