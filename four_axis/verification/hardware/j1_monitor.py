"""J1 USB2CAN status monitor. Transmit whitelist: read registers and query status.

USB framing source: dmBots/motor-sdk, Python例程/u2can/DM_CAN.py (2026-10-10).
No enable, control, parameter-write, zero-save or calibration commands exist here.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import time
import tkinter as tk
from tkinter import ttk

from feedback_decode import decode_feedback

DEPS = Path(r'C:\Users\ASUS\Documents\机械臂\tmp\slai_serial_deps')
sys.path.insert(0, str(DEPS))
import serial
from serial.tools import list_ports

REQUIRED_REGISTERS = (7, 8, 10, 21, 22, 23)
ALLOWED_REGISTERS = REQUIRED_REGISTERS + (1, 3, 9, 20, 80, 81)
SOURCE = 'https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py'


def load_reference():
    """Load J1 reference from the authoritative hardware record."""
    path = Path(__file__).resolve().parents[2] / 'src/slaihand_bringup/config/hardware_mapping.yaml'
    in_j1 = False
    for line in path.read_text(encoding='utf-8').splitlines():
        if line == '  slaihand_joint1:':
            in_j1 = True
        elif in_j1 and line.startswith('  slaihand_joint'):
            break
        elif in_j1 and line.strip().startswith('reference_pose_encoder_rad:'):
            value = float(line.split(':', 1)[1])
            if not math.isfinite(value):
                raise ValueError('Reference is not finite')
            return value
    return None


def request_packet(rid=None):
    # Device is deliberately fixed to J1 ID=1. Never accept arbitrary CAN data.
    if rid is not None and rid not in ALLOWED_REGISTERS:
        raise ValueError('Register is not in the read whitelist')
    data = bytes([1, 0, 0xCC if rid is None else 0x33, 0 if rid is None else rid, 0, 0, 0, 0])
    packet = bytearray([0x55, 0xAA, 0x1E, 0x03, 0x01, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    packet[13:15] = struct.pack('<H', 0x7FF)
    packet[21:29] = data
    return bytes(packet)


class PacketStream:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while self.buffer:
            if self.buffer[0] != 0xAA:
                del self.buffer[0]
                continue
            if len(self.buffer) < 16:
                break
            if self.buffer[15] != 0x55 or self.buffer[1] != 0x11:
                del self.buffer[0]
                continue
            packet = bytes(self.buffer[:16])
            del self.buffer[:16]
            packets.append((int.from_bytes(packet[3:7], 'little'), packet[7:15]))
        return packets


class Reader:
    def __init__(self, port):
        self.port = serial.Serial(port, 921600, timeout=0, write_timeout=0.25)
        self.stream = PacketStream()
        self.registers = {}
        self.latest = None
        self.latest_at = None
        self.tx = self.rx = 0
        self.log_path = Path(r'C:\Users\ASUS\Documents\机械臂\tmp') / time.strftime('J1_readonly_%Y%m%d_%H%M%S.jsonl')
        self.log = self.log_path.open('a', encoding='utf-8')
        for rid in ALLOWED_REGISTERS:
            self.port.write(request_packet(rid))
            self.tx += 1

    @property
    def ready(self):
        r = self.registers
        return all(k in r for k in REQUIRED_REGISTERS) and r[8] == 1 and r[7] == 0 and all(
            math.isfinite(r[k]) and r[k] > 0 for k in (21, 22, 23))

    def query(self):
        self.port.write(request_packet())
        self.tx += 1

    def receive(self):
        results = []
        for can_id, data in self.stream.feed(self.port.read(self.port.in_waiting)):
            self.rx += 1
            event = dict(time=time.time(), can_id=can_id, payload=data.hex(' '))
            if data[:3] == b'\x01\x00\x33' and data[3] in ALLOWED_REGISTERS:
                rid = data[3]
                value = struct.unpack('<I' if rid in (7, 8, 9, 10) else '<f', data[4:])[0]
                if can_id == 0:
                    self.registers[rid] = value
                event.update(kind='register', rid=rid, value=value)
            elif self.ready and can_id == self.registers[7] and (data[0] & 15) == 1:
                self.latest = decode_feedback(data, *(self.registers[k] for k in (21,22,23)))
                self.latest_at = time.monotonic()
                event.update(kind='feedback', decoded=self.latest)
                results.append(self.latest)
            else:
                event['kind'] = 'unclassified'
            self.log.write(json.dumps(event, ensure_ascii=False) + '\n')
            self.log.flush()
        return results

    def close(self):
        # Closing never transmits any command. Monitor has never enabled the motor.
        self.port.close()
        self.log.close()


def self_test():
    assert request_packet()[21:29] == bytes.fromhex('01 00 cc 00 00 00 00 00')
    assert request_packet(21)[21:29] == bytes.fromhex('01 00 33 15 00 00 00 00')
    try:
        request_packet(0x55)
        raise AssertionError('Unsafe RID accepted')
    except ValueError:
        pass
    raw = bytes.fromhex('aa 11 08 00 00 00 00 01 8e 7c 80 07 ff 1c 18 55')
    stream = PacketStream()
    assert stream.feed(b'noise' + raw[:7]) == []
    assert stream.feed(raw[7:] + raw) == [(0, raw[7:15]), (0, raw[7:15])]
    decoded = decode_feedback(raw[7:15])
    assert abs(decoded['position_rad'] - 1.4147020675974673) < 1e-10
    assert decoded['state_code'] == 0 and decoded['driver_temp_c'] == 28 and decoded['motor_temp_c'] == 24
    print('Transport whitelist, fragmented frames, multiple frames, ID=0 feedback and known decode: PASS')


class Monitor:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('SLAI J1 状态监视 - 只读')
        self.root.geometry('800x700')
        self.reader = None
        self.reference = load_reference()
        self.next_query = 0
        main = ttk.Frame(self.root, padding=20)
        main.pack(fill='both', expand=True)
        ttk.Label(main, text='J1 实际反馈', font=('Microsoft YaHei', 22)).pack(anchor='w')
        ttk.Label(main, text='只读参数和状态查询 · 不使能、不运动、不设零', font=('Microsoft YaHei', 11)).pack(anchor='w', pady=(3,18))
        line = ttk.Frame(main); line.pack(fill='x')
        self.port_name = tk.StringVar(value='COM9')
        ttk.Combobox(line, textvariable=self.port_name, values=[p.device for p in list_ports.comports()], width=14).pack(side='left')
        ttk.Button(line, text='连接并读取', command=self.connect).pack(side='left', padx=10)
        ttk.Button(line, text='断开', command=self.disconnect).pack(side='left')
        self.status = tk.StringVar(value='未连接。请先在 DMTool 中关闭设备，释放串口。')
        ttk.Label(main, textvariable=self.status, wraplength=690).pack(anchor='w', pady=15)
        self.values = {}
        for title in ['位置', '参考偏角', '速度', '状态', '温度', '控制模式', '超时设置', '电流参数', '原始反馈']:
            row = ttk.Frame(main); row.pack(fill='x', pady=7)
            ttk.Label(row, text=title, width=12, font=('Microsoft YaHei',11)).pack(side='left')
            val = tk.StringVar(value='— 尚无有效反馈')
            self.values[title] = val
            ttk.Label(row, textvariable=val, font=('Microsoft YaHei',13)).pack(side='left')
        self.count = tk.StringVar(value='查询 0 / 接收 0')
        ttk.Label(main, textvariable=self.count).pack(anchor='w', pady=12)
        ttk.Label(main, text='参考偏角 = 编码器位置 − 照片参考位置；正方向朝人体前方。\n尚未验证与仿真零位对应。断开不改变电机状态。', wraplength=680).pack(anchor='w')
        self.root.protocol('WM_DELETE_WINDOW', self.quit)
        self.root.after(100,self.tick)

    def connect(self):
        self.disconnect()
        try:
            self.reader = Reader(self.port_name.get())
            self.status.set('正在核对电机 ID、Master ID、控制模式和反馈幅值…')
            self.next_query = 0
        except Exception as exc:
            self.status.set(f'连接失败：{exc}。检查 DMTool 是否已关闭设备。')

    def disconnect(self):
        if self.reader:
            self.reader.close()
            self.reader = None
        self.status.set('未连接 / 旧数据不能作为实时状态')
        for val in self.values.values(): val.set('— 尚无有效反馈')

    def tick(self):
        try:
            if self.reader:
                r = self.reader
                r.receive()
                now = time.monotonic()
                if r.ready and now >= self.next_query:
                    r.query(); self.next_query=now+0.2
                self.count.set(f'查询/读参数 {r.tx} / 接收 {r.rx}')
                if r.ready and r.latest_at is not None and now-r.latest_at < 1.0:
                    d=r.latest
                    self.status.set('反馈有效 · 每 200ms 查询一次 · Master ID=0 已正确处理')
                    self.values['位置'].set(f"{d['position_rad']:.6f} rad  /  {d['position_deg']:.2f}°")
                    if self.reference is not None:
                        relative=d['position_rad']-self.reference
                        self.values['参考偏角'].set(f'{relative:+.6f} rad  /  {math.degrees(relative):+.2f}°')
                    else:self.values['参考偏角'].set('未记录参考姿态')
                    self.values['速度'].set(f"{d['velocity_rad_s']:.4f} rad/s")
                    code=d['state_code']
                    self.values['状态'].set({0:'失能（不输出驱动力）',1:'已使能'}.get(code,f'状态码 {code}：需核对故障'))
                    self.values['温度'].set(f"电机 {d['motor_temp_c']}°C  /  驱动 {d['driver_temp_c']}°C")
                    self.values['控制模式'].set({1:'MIT',2:'位置速度',3:'速度',4:'力位混合'}.get(r.registers[10],str(r.registers[10])))
                    timeout = r.registers.get(9)
                    self.values['超时设置'].set('未读到' if timeout is None else f'{timeout} 计数（按说明书 {timeout * .05:g} ms；行为待验证）')
                    oc = r.registers.get(3)
                    self.values['电流参数'].set(f'过流比例 {oc if oc is not None else "未读到"}（保护阈值，非指令限流）')
                    self.values['原始反馈'].set(d['payload'])
                elif r.ready:
                    self.status.set('反馈未到达或已过期。检查电机供电；不要把旧值当实时位置。')
                    for val in self.values.values():val.set('— 反馈已过期')
        except Exception as exc:
            self.disconnect();self.status.set(f'读取中断：{exc}')
        self.root.after(50,self.tick)

    def quit(self):
        self.disconnect();self.root.destroy()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test',action='store_true')
    p.add_argument('--probe',action='store_true')
    p.add_argument('--port',default='COM9')
    p.add_argument('--connect',action='store_true',help='Automatically connect the read-only monitor')
    a=p.parse_args()
    if a.self_test:self_test()
    elif a.probe:
        r=Reader(a.port)
        try:
            deadline=time.monotonic()+4
            count=0
            while time.monotonic()<deadline:
                values=r.receive()
                if values:
                    print(json.dumps(values[-1],ensure_ascii=False));count+=1
                    if count>=3:break
                if r.ready:r.query()
                time.sleep(.2)
            print(json.dumps(dict(registers=r.registers,tx=r.tx,rx=r.rx,log=str(r.log_path)),ensure_ascii=False))
            if count==0:sys.exit(2)
        finally:r.close()
    else:
        app=Monitor()
        app.port_name.set(a.port)
        if a.connect:app.root.after(200,app.connect)
        app.root.mainloop()
