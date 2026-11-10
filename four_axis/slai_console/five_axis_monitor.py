"""J1-J5 read-only monitor. No enable, motion, parameter write or zero write.

J5 is shown raw until a valid manual calibration record exists. The monitor
never activates J5 by itself; the five-axis controller has a separate gate.
"""
import json
import math
from pathlib import Path
import struct
import sys
import time

from .core import ROOT
from .robot_contract import load_mapping
from .j5_calibration import DEFAULT_PATH as J5_PATH, load_j5_calibration

sys.path.insert(0, str(ROOT/'verification/hardware'))
from feedback_decode import decode_feedback

WORKSPACE = ROOT
IDS = (1, 2, 3, 4, 5)
REGS = (7, 8, 9, 10, 21, 22, 23)
MAX_AGE = 1.0
STATES = {0:'正常失能',1:'已使能',3:'输出轴校准异常',4:'传感器异常',
          5:'编码器校准异常',8:'过压',9:'欠压',10:'过流',11:'驱动过温',
          12:'电机过温',13:'通信丢失保护',14:'过载'}


def load_mapping5(cal_path=J5_PATH):
    path = WORKSPACE/'src/slaihand_bringup/config/hardware_mapping.yaml'
    result = load_mapping(); active = None
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('  slaihand_joint') and line.endswith(':'):
            active = int(line.split('slaihand_joint', 1)[1][:-1])
            if active == 5:
                result[active] = {}
        elif active == 5 and line.startswith('    ') and ':' in line and not line.lstrip().startswith('#'):
            key, value = line.strip().split(':', 1)
            if key in ('can_id', 'hardware_sign', 'zero_offset_rad'):
                text = value.split('#', 1)[0].strip()
                result[active][key] = None if text in ('null', '') else float(text)
    j5 = result.get(5, {})
    if j5.get('can_id') != 5:
        raise ValueError('J5 CAN ID 5 not found in canonical mapping')
    j5['can_id'] = 5
    error = None
    try:
        calibration = load_j5_calibration(cal_path)
    except ValueError as exc:
        calibration = None
        error = str(exc)
    if calibration is not None and calibration.is_usable():
        j5.update(hardware_sign=calibration.sign,
                  zero_offset_rad=calibration.zero_offset_rad,
                  calibrated=True, calibration=calibration,
                  calibration_error=None)
    else:
        j5.update(hardware_sign=1, zero_offset_rad=0., calibrated=False,
                  calibration=None, calibration_error=error)
    return result


def request(motor_id, rid=None):
    if motor_id not in IDS or (rid is not None and rid not in REGS):
        raise ValueError('Only J1-J5 approved reads allowed')
    return bytes([motor_id, 0, 0xCC if rid is None else 0x33,
                  0 if rid is None else rid, 0, 0, 0, 0])


def usb_packet(motor_id, rid=None):
    data = request(motor_id, rid)
    frame = bytearray([0x55, 0xAA, 0x1E, 3, 1, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                       0xFF, 7, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    frame[21:29] = data
    return bytes(frame)


class Stream:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk):
        self.buf.extend(chunk); result = []
        while self.buf:
            if self.buf[0] != 0xAA:
                del self.buf[0]; continue
            if len(self.buf) < 16:
                break
            if self.buf[1] != 0x11 or self.buf[2] != 8 or self.buf[15] != 0x55:
                del self.buf[0]; continue
            raw = bytes(self.buf[:16]); del self.buf[:16]
            result.append((int.from_bytes(raw[3:7], 'little'), raw[7:15]))
        return result


class State5:
    def __init__(self, mapping):
        self.mapping = mapping
        self.regs = {i: {} for i in IDS}
        self.conflicts = {i: set() for i in IDS}
        self.feedback = {}; self.at = {}; self.notice = ''

    def ready(self, i):
        r = self.regs[i]
        return (not self.conflicts[i] and all(k in r for k in REGS)
                and r[8] == i and 0 <= r[7] <= 0x7FF and 0 <= r[9] <= 0xFFFFFFFF
                and all(math.isfinite(r[k]) and r[k] > 0 for k in (21, 22, 23)))

    def feed(self, can_id, data, now):
        if len(data) != 8:
            return
        if data[0] in IDS and data[1] == 0 and data[2] == 0x33 and data[3] in REGS:
            i, rid = data[0], data[3]
            value = struct.unpack('<I' if rid in (7, 8, 9, 10) else '<f', data[4:])[0]
            r = self.regs[i]
            if rid in r and r[rid] != value:
                self.conflicts[i].add(rid)
                self.feedback.pop(i, None); self.at.pop(i, None)
                self.notice = f'J{i} 同一寄存器有不同回复，检查重复编号；不显示旧角度。'
            r[rid] = value
            if self.ready(i) and can_id != r[7]:
                self.conflicts[i].add(rid)
                self.notice = f'J{i} 反馈帧ID与Master ID不符'
            return
        i = data[0] & 15
        if i not in IDS:
            self.notice = f'收到非目标编号 {i} 的报文，已忽略；程序不会向其发送请求。'
            return
        if not self.ready(i) or can_id != self.regs[i][7]:
            return
        f = decode_feedback(data, *(self.regs[i][k] for k in (21, 22, 23)))
        m = self.mapping[i]
        f['model_deg'] = math.degrees(m['hardware_sign']*(f['position_rad']-m['zero_offset_rad']))
        f['calibrated'] = bool(m.get('calibrated', True))
        self.feedback[i] = f; self.at[i] = now

    def fresh(self, i, now):
        return i in self.feedback and self.ready(i) and 0 <= now-self.at[i] <= MAX_AGE


class Reader5:
    def __init__(self, port, mapping, serial_factory=None, clock=time.monotonic,
                 cal_path=J5_PATH):
        self.state = State5(mapping); self.stream = Stream(); self.tx = 0; self.rx = 0
        self.clock = clock; self.cal_path = cal_path
        self.poll = 0; self.next_send = 0
        self.setup = [(i, rid) for _ in range(3) for i in IDS for rid in REGS]
        folder = WORKSPACE/'tmp/five_axis_readonly'; folder.mkdir(parents=True, exist_ok=True)
        self.path = folder/(time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000}.jsonl')
        self.log = self.path.open('x', encoding='utf-8')
        try:
            if serial_factory is None:
                sys.path.insert(0, str(WORKSPACE.parent/'tmp/slai_serial_deps'))
                import serial
                serial_factory = serial.Serial
            self.serial = serial_factory(port, 921600, timeout=0, write_timeout=.05)
            self.serial.reset_input_buffer()
        except Exception:
            self.log.close(); raise

    def event(self, **data):
        self.log.write(json.dumps({'time': time.time(), **data}, ensure_ascii=False)+'\n')
        self.log.flush()

    def tick(self, now):
        for cid, data in self.stream.feed(self.serial.read(self.serial.in_waiting)):
            self.rx += 1
            self.event(direction='rx', can_id=cid, payload=data.hex(' '))
            self.state.feed(cid, data, now)
        if now < self.next_send:
            return
        while self.setup:
            i, rid = self.setup.pop(0)
            if rid not in self.state.regs[i]:
                break
        else:
            i = IDS[self.poll % len(IDS)]; self.poll += 1; rid = None
        packet = usb_packet(i, rid)
        if self.serial.write(packet) != len(packet):
            raise RuntimeError('Incomplete serial write')
        self.tx += 1
        self.event(direction='tx', can_id=0x7FF, payload=packet[21:29].hex(' '))
        self.next_send = now+.05

    def close(self):
        try:
            self.serial.close()
        finally:
            self.log.close()


def main():
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk(); root.title('SLAI 五轴实机只读监视 · J5未标定只显示原始值')
    root.geometry('1200x650'); root.minsize(900, 500)
    ui = ttk.Frame(root, padding=18); ui.pack(fill='both', expand=True)
    ttk.Label(ui, text='J1–J5 实机反馈与软件映射', font=('Microsoft YaHei', 19, 'bold')).pack(anchor='w')
    ttk.Label(ui, text='严格只读：无使能、运动、参数写入、清故障或设零。J5未标定前不参与控制。').pack(anchor='w', pady=8)
    bar = ttk.Frame(ui); bar.pack(fill='x', pady=8)
    port = tk.StringVar(value='COM9')
    ttk.Label(bar, text='USB2CAN 串口：').pack(side='left')
    ttk.Entry(bar, textvariable=port, width=14).pack(side='left')
    reader = [None]; mapping = [None]; closing = [False]
    status = tk.StringVar(value='未连接。先在 DMTool 关闭设备；转接器保持CAN模式。')
    detail = tk.StringVar(value='J5 没有标定记录，只显示原始值；不会猜测方向/零位/限位。')
    columns = ('joint', 'encoder', 'model', 'velocity', 'state', 'temps', 'mode', 'timeout', 'cal')
    table = ttk.Treeview(ui, columns=columns, show='headings', height=5)
    widths = (55, 135, 135, 95, 145, 105, 80, 130, 110)
    names = ('关节', '编码器角度 °', '模型角度 °', '速度 rad/s', '运行状态',
             '驱动/电机 ℃', '模式', 'TIMEOUT', '标定')
    for col, width, title in zip(columns, widths, names):
        table.heading(col, text=title); table.column(col, width=width, minwidth=50, anchor='center')

    def blank():
        for item in table.get_children():
            table.delete(item)
        for i in IDS:
            table.insert('', 'end', iid=str(i),
                         values=(f'J{i}', '—', '—', '—', '未连接', '—', '—', '—', '—'))

    def disconnect():
        if reader[0]:
            reader[0].close(); reader[0] = None
        blank(); status.set('已断开。未发送失能命令；只读窗口不是急停。')

    def connect():
        disconnect()
        try:
            mapping[0] = load_mapping5()
            reader[0] = Reader5(port.get().strip(), mapping[0])
            status.set('只读核对五轴编号、映射、模式和超时，然后轮询 J1–J5…')
            detail.set('日志：'+str(reader[0].path))
        except Exception as exc:
            status.set('连接失败：'+str(exc)+'；检查 DMTool 是否释放串口。')

    ttk.Button(bar, text='连接并读取五轴', command=connect).pack(side='left', padx=12)
    ttk.Button(bar, text='断开', command=disconnect).pack(side='left')
    ttk.Label(ui, textvariable=status, wraplength=1100).pack(anchor='w', pady=10)
    table.pack(fill='x', pady=8); blank()
    ttk.Label(ui, textvariable=detail, wraplength=1100).pack(anchor='w', pady=8)
    ttk.Label(ui, text='角度过期超过1秒会清除显示；仅查看不会保持位置。'
                       '若发现已使能或故障，不要手动掰动，先停止后续操作。',
              wraplength=1100).pack(anchor='w', pady=8)

    def tick():
        if closing[0]:
            return
        try:
            r = reader[0]
            if r:
                now = r.clock(); r.tick(now)
                fresh = 0; warnings = []
                for i in IDS:
                    s = r.state; regs = s.regs[i]
                    mode = {1:'MIT', 2:'位置速度', 3:'速度', 4:'力位混控'}.get(regs.get(10), str(regs.get(10, '—')))
                    timeout = str(regs.get(9, '—'))
                    cal = '已标定' if mapping[0][i].get('calibrated', True) else '未标定/原始'
                    if s.fresh(i, now):
                        f = s.feedback[i]; fresh += 1
                        state = STATES.get(f['state_code'], '未知状态 '+str(f['state_code']))
                        if f['state_code'] != 0:
                            warnings.append(f'J{i}：{state}')
                        row = (f'J{i}', f"{f['position_deg']:+.2f}", f"{f['model_deg']:+.2f}",
                               f"{f['velocity_rad_s']:+.3f}", state,
                               f"{f['driver_temp_c']}/{f['motor_temp_c']}", mode, timeout, cal)
                    else:
                        state = ('参数回复冲突' if s.conflicts[i] else
                                 ('反馈过期/未收到' if s.ready(i) else '参数未齐'))
                        row = (f'J{i}', '—', '—', '—', state, '—', mode, timeout, cal)
                    table.item(str(i), values=row)
                status.set(f'只读收发：请求 {r.tx} / 接收 {r.rx}；新鲜反馈 {fresh}/5；不发送控制。'
                           + (' 注意：'+'；'.join(warnings) if warnings else ''))
                if s.notice:
                    status.set(status.get()+' '+s.notice)
                if mapping[0]:
                    detail.set('J5：'+(mapping[0][5].get('calibration_error') or
                                       ('已标定 sign=%+g zero=%+.6f rad' % (
                                           mapping[0][5]['hardware_sign'],
                                           mapping[0][5]['zero_offset_rad'])
                                        if mapping[0][5].get('calibrated') else
                                        '未标定，只显示原始编码器值')))
        except Exception as exc:
            disconnect(); status.set('读取停止：'+str(exc)+'。未发送控制或失能命令，请检查设备。')
        root.after(20, tick)

    def close():
        closing[0] = True; disconnect(); root.destroy()

    root.protocol('WM_DELETE_WINDOW', close)
    root.after(20, tick)
    root.mainloop()


if __name__ == '__main__':
    main()
