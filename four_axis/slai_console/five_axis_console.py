"""Explicit five-axis console: read-only monitor + native MIT hold/trajectory/velocity.

No auto-connect, auto-enable, parameter write, zero write or auto retry. The
J5 calibration dialog only reads four manually chosen poses; it never commands
a motor. A five-axis control run requires the operator-confirmed config, the
native/five-axis ticket and (for J5) a usable calibration record.
"""
import json
import math
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from .clock import now as clock_now
from .core import ROOT
from .five_axis_config import (DEFAULT_PATH as CONFIG_PATH, build_envelopes,
                               ensure_template, load_settings)
from .five_axis_mit import FiveAxisAcceptance
from .five_axis_monitor import IDS, STATES, Reader5, load_mapping5
from .five_axis_session import build_five_session, fresh_active_feedback
from .j5_calibration import (DEFAULT_PATH as J5_PATH, build_j5_calibration,
                             load_j5_calibration, save_j5_calibration)
from .watchdog_check import make_event_writer

POLICY = ROOT/'verification/hardware/five_axis_20261102/OpenArm五轴功能_设计.md'
AUDIT_DIR = ROOT/'tmp/five_axis_control'


class FiveAxisConsole:
    def __init__(self, root):
        self.root = root
        root.title('SLAI 五轴 OpenArm 风格原生 MIT · 显式使能 · J5 未标定不使能')
        root.geometry('1280x860'); root.minsize(1000, 700)
        self.reader = None
        self.mapping = None
        self.settings = None
        self.busy = False
        self.latched = False
        self.stop_requested = threading.Event()
        self.events = queue.Queue()
        self.closing = False
        self._build_ui()
        self._load_settings()
        self._tick()

    def _build_ui(self):
        root = self.root
        outer = ttk.Frame(root, padding=14); outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='J1–J5 原生 MIT 五轴控制',
                  font=('Microsoft YaHei', 20, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='严格显式：不自动连接/使能/回零/重试；J5 未标定只监视。'
                             '2Nm 类“估计护栏”不是驱动内部持续力矩硬限幅。',
                  foreground='#8a1f1f').pack(anchor='w', pady=4)
        bar = ttk.Frame(outer); bar.pack(fill='x', pady=6)
        self.port = tk.StringVar(value='COM9')
        ttk.Label(bar, text='USB2CAN 串口：').pack(side='left')
        ttk.Entry(bar, textvariable=self.port, width=12).pack(side='left')
        ttk.Button(bar, text='连接·只读', command=self.connect).pack(side='left', padx=8)
        ttk.Button(bar, text='断开', command=self.disconnect).pack(side='left')
        self.status = tk.StringVar(value='未连接。先在 DMTool 关闭设备；J5 未标定前只显示原始值。')
        ttk.Label(outer, textvariable=self.status, wraplength=1220).pack(anchor='w', pady=6)

        columns = ('joint', 'model', 'raw', 'velocity', 'state', 'temps', 'mode', 'cal')
        self.table = ttk.Treeview(outer, columns=columns, show='headings', height=5)
        widths = (60, 120, 120, 100, 150, 110, 80, 120)
        names = ('关节', '模型角 °', '编码器 °', '速度 rad/s', '状态', '驱动/电机 ℃', '模式', 'J5标定')
        for col, width, title in zip(columns, widths, names):
            self.table.heading(col, text=title); self.table.column(col, width=width, anchor='center')
        self.table.pack(fill='x', pady=6)
        self._blank_table()

        cfg = ttk.LabelFrame(outer, text='配置与门控', padding=8); cfg.pack(fill='x', pady=6)
        self.config_label = tk.StringVar(value='配置未加载')
        ttk.Label(cfg, textvariable=self.config_label, wraplength=1180).pack(anchor='w')
        row = ttk.Frame(cfg); row.pack(fill='x', pady=4)
        ttk.Button(row, text='重新加载配置', command=self._load_settings).pack(side='left')
        ttk.Button(row, text='创建配置模板', command=self._make_template).pack(side='left', padx=8)
        ttk.Button(row, text='J5 手动标定…', command=self.open_j5_calibration).pack(side='left')
        self.t_support = tk.BooleanVar(); self.t_clear = tk.BooleanVar()
        self.t_native = tk.BooleanVar(); self.t_five = tk.BooleanVar()
        self.t_vel = tk.BooleanVar()
        for var, text in ((self.t_support, '支撑确认'), (self.t_clear, '净空确认'),
                          (self.t_native, '接受原生MIT输出区别'),
                          (self.t_five, '接受五轴同时运动'),
                          (self.t_vel, '额外接受速度模式')):
            ttk.Checkbutton(row, text=text, variable=var).pack(side='left', padx=6)

        targets = ttk.LabelFrame(outer, text='五轴目标（度，模型坐标）', padding=8)
        targets.pack(fill='x', pady=6)
        self.entries = {}
        for i in IDS:
            ttk.Label(targets, text=f'J{i}').pack(side='left')
            var = tk.StringVar(value='0.0')
            ttk.Entry(targets, textvariable=var, width=8).pack(side='left', padx=2)
            self.entries[i] = var
        ttk.Label(targets, text='时长 s').pack(side='left', padx=(12, 2))
        self.duration = tk.StringVar(value='10')
        ttk.Entry(targets, textvariable=self.duration, width=6).pack(side='left')
        ttk.Button(targets, text='目标=当前+0.5°',
                   command=lambda: self._fill_relative(0.5)).pack(side='left', padx=6)
        ttk.Button(targets, text='目标=当前+10°（18s）',
                   command=lambda: self._fill_relative(10.0)).pack(side='left', padx=6)

        controls = ttk.Frame(outer); controls.pack(fill='x', pady=8)
        ttk.Button(controls, text='原生MIT五轴保持2秒', command=lambda: self._start('hold')).pack(side='left')
        ttk.Button(controls, text='五轴轨迹', command=lambda: self._start('trajectory')).pack(side='left', padx=8)
        ttk.Button(controls, text='回归0位（超限回退）',
                   command=lambda: self._start('recovery')).pack(side='left', padx=8)
        ttk.Button(controls, text='五轴速度', command=lambda: self._start('velocity')).pack(side='left')
        ttk.Button(controls, text='停止（FD全部已激活轴）', command=self.request_stop).pack(side='left', padx=16)
        ttk.Label(controls, text='速度目标 rad/s：').pack(side='left', padx=(16, 2))
        self.velocity_values = {}
        for i in IDS:
            var = tk.StringVar(value='0.0')
            ttk.Entry(controls, textvariable=var, width=7).pack(side='left', padx=1)
            self.velocity_values[i] = var
        ttk.Label(controls, text='时长 s').pack(side='left', padx=(8, 2))
        self.velocity_duration = tk.StringVar(value='2')
        ttk.Entry(controls, textvariable=self.velocity_duration, width=5).pack(side='left')

        self.log = tk.Text(outer, height=10, wrap='none')
        self.log.pack(fill='both', expand=True, pady=6)
        ttk.Label(outer, text='日志与结果：tmp/five_axis_control；本窗口不是急停，异常必须现场切电。',
                  wraplength=1220).pack(anchor='w')

    def _blank_table(self):
        for item in self.table.get_children():
            self.table.delete(item)
        for i in IDS:
            self.table.insert('', 'end', iid=str(i),
                              values=(f'J{i}', '—', '—', '—', '未连接', '—', '—', '—'))

    def _append(self, text):
        self.log.insert('end', text+'\n'); self.log.see('end')

    def _load_settings(self):
        try:
            self.settings = load_settings(CONFIG_PATH)
            self.config_label.set(f'配置 {CONFIG_PATH.name}：user_confirmed={self.settings.user_confirmed}；'
                                  f'J5_enabled={self.settings.j5_enabled}；每轴 guard/kp/kd 已读取')
        except Exception as exc:
            self.settings = None
            self.config_label.set(f'配置未通过：{exc}')

    def _make_template(self):
        try:
            path = ensure_template(CONFIG_PATH)
            self.config_label.set(f'已创建模板 {path}；请逐轴复核并设置 user_confirmed=true')
        except Exception as exc:
            messagebox.showerror('配置模板', str(exc))

    def connect(self):
        if self.busy:
            messagebox.showwarning('五轴控制', '控制运行中不能重连；先停止。')
            return
        self.disconnect()
        self.latched = False
        try:
            self.mapping = load_mapping5()
            self.reader = Reader5(self.port.get().strip(), self.mapping)
            j5 = self.mapping[5]
            self.status.set('只读预检：J1–J5 参数/反馈；J5 '+
                            ('已标定' if j5.get('calibrated') else '未标定，只显示原始值')+
                            '。控制需配置确认+显式票据。')
            self._append(f'只读连接，日志 {self.reader.path}')
        except Exception as exc:
            self.status.set(f'连接失败：{exc}；检查 DMTool 是否释放串口。')

    def disconnect(self):
        if self.busy:
            messagebox.showwarning('五轴控制', '控制运行中不能断开；先停止。')
            return
        if self.reader:
            try:
                self.reader.close()
            except Exception:
                pass
            self.reader = None
        self._blank_table()
        self.status.set('已断开。未发送失能命令；只读窗口不是急停。')

    def _fill_relative(self, delta=0.5):
        if not self.reader:
            messagebox.showwarning('五轴目标', '先连接只读串口。')
            return
        try:
            calibration = load_j5_calibration(J5_PATH)
            _envelopes, active = build_envelopes(self.settings, calibration)
            now = time.monotonic()
            for i in active:
                if not self.reader.state.fresh(i, now):
                    raise ValueError(f'J{i} 反馈不新鲜，不能取当前角')
                current = self.reader.state.feedback[i]['model_deg']
                self.entries[i].set(f'{current+delta:.3f}')
            if 5 not in active:
                self.entries[5].set('0.0')
            if delta >= 10:
                self.duration.set('18')
                self.status.set('已填入当前模型角+10°；时长 18s（10s 会超 0.02rad/s）。再点“五轴轨迹”。')
            else:
                self.status.set(f'已填入当前模型角+{delta:g}°；时长保持当前值，再点“五轴轨迹”。')
        except Exception as exc:
            messagebox.showerror('五轴目标', str(exc))

    def _tick(self):
        if self.closing:
            return
        self._drain_events()
        try:
            if self.reader and not self.busy and not self.latched:
                now = time.monotonic()
                self.reader.tick(now)
                fresh = 0; warnings = []
                for i in IDS:
                    s = self.reader.state; regs = s.regs[i]
                    mode = {1:'MIT', 2:'位置速度', 3:'速度', 4:'力位混控'}.get(regs.get(10), str(regs.get(10, '—')))
                    cal = ('已标定' if self.mapping[i].get('calibrated', True) else '未标定/原始')
                    if s.fresh(i, now):
                        f = s.feedback[i]; fresh += 1
                        state = STATES.get(f['state_code'], '未知状态 '+str(f['state_code']))
                        if f['state_code'] != 0:
                            warnings.append(f'J{i}：{state}')
                        raw_deg = math.degrees(f['position_rad'])
                        row = (f'J{i}', f"{f['model_deg']:+.3f}", f'{raw_deg:+.3f}',
                               f"{f['velocity_rad_s']:+.3f}", state,
                               f"{f['driver_temp_c']}/{f['motor_temp_c']}", mode, cal)
                    else:
                        state = ('冲突' if s.conflicts[i] else
                                 ('过期/未收到' if s.ready(i) else '参数未齐'))
                        row = (f'J{i}', '—', '—', '—', state, '—', mode, cal)
                    self.table.item(str(i), values=row)
                self.status.set(f'只读：请求 {self.reader.tx}/接收 {self.reader.rx}；新鲜 {fresh}/5。'
                                + (' 注意：'+'；'.join(warnings) if warnings else ''))
                if self.reader.state.notice:
                    self.status.set(self.status.get()+' '+self.reader.state.notice)
        except Exception as exc:
            self.disconnect(); self.status.set(f'读取停止：{exc}')
        self.root.after(25, self._tick)

    # ---- J5 calibration -----------------------------------------------------
    def open_j5_calibration(self):
        if self.busy:
            messagebox.showwarning('J5 标定', '控制运行中不能标定；先停止。')
            return
        if not self.reader:
            messagebox.showwarning('J5 标定', '先连接只读串口。')
            return
        win = tk.Toplevel(self.root); win.title('J5 手动标定（只读，不使能）')
        win.geometry('720x560')
        ttk.Label(win, text='J5 手动标定：不发送使能/运动/参数/零位写入。',
                  font=('Microsoft YaHei', 14, 'bold')).pack(anchor='w', padx=12, pady=8)
        ttk.Label(win, text='手动把 J5 放在四个位置，分别记录：零点、正方向、最小安全端、最大安全端。'
                            '每次读数必须新鲜；电机必须已失能，人员离开转动范围。',
                  wraplength=680).pack(anchor='w', padx=12)
        value = tk.StringVar(value='未记录')
        ttk.Label(win, textvariable=value, wraplength=680).pack(anchor='w', padx=12, pady=8)
        points = {}

        def read_pose(label):
            try:
                s = self.reader.state
                if not s.fresh(5, time.monotonic()):
                    raise ValueError('J5 反馈不新鲜；不要继续。')
                f = s.feedback[5]
                points[label] = (f['position_rad'], f['payload'])
                value.set(f'{label}：raw={math.degrees(f["position_rad"]):+.4f}° '
                          f'payload={f["payload"]}；已记录 {len(points)}/4')
                if '正方向' in points and '零点' in points:
                    delta = points['正方向'][0]-points['零点'][0]
                    value.set(value.get()+f'；推断 sign={1 if delta>0 else -1}，'
                                           f'位移 {math.degrees(delta):+.4f}°')
            except Exception as exc:
                messagebox.showerror('J5 标定', str(exc))

        row = ttk.Frame(win); row.pack(fill='x', padx=12, pady=6)
        for label in ('零点', '正方向', '最小安全端', '最大安全端'):
            ttk.Button(row, text=f'记录{label}', command=lambda l=label: read_pose(l)).pack(side='left', padx=3)
        confirm = tk.BooleanVar(value=False)
        ttk.Checkbutton(win, text='我确认以上四个位置由我手动摆放，且 J5 已失能/可反驱',
                        variable=confirm).pack(anchor='w', padx=12, pady=8)
        def save():
            try:
                if not confirm.get():
                    raise ValueError('必须确认手动摆位与失能状态')
                missing = {'零点', '正方向', '最小安全端', '最大安全端'}-set(points)
                if missing:
                    raise ValueError(f'缺少记录：{sorted(missing)}')
                record = build_j5_calibration(
                    zero_position_rad=points['零点'][0],
                    positive_position_rad=points['正方向'][0],
                    travel_min_position_rad=points['最小安全端'][0],
                    travel_max_position_rad=points['最大安全端'][0],
                    zero_payload=points['零点'][1], positive_payload=points['正方向'][1],
                    travel_min_payload=points['最小安全端'][1],
                    travel_max_payload=points['最大安全端'][1],
                    user_zero_confirmed=True, user_direction_confirmed=True,
                    user_limits_confirmed=True,
                    notes='manual four-pose calibration; no motor command')
                save_j5_calibration(record, J5_PATH)
                value.set(f'已保存 {J5_PATH}；sign={record.sign} '
                          f'zero={record.zero_offset_rad:+.6f}rad。请断开并重新连接以加载。')
            except Exception as exc:
                messagebox.showerror('J5 标定', str(exc))
        ttk.Button(win, text='确认并保存 J5 标定', command=save).pack(anchor='w', padx=12, pady=8)
        ttk.Label(win, text='保存后 canonical mapping 未改；J5 标定是独立记录。未标定前五轴控制器只跑 J1–J4。',
                  wraplength=680).pack(anchor='w', padx=12)

    # ---- control ------------------------------------------------------------
    def _ticket(self):
        return FiveAxisAcceptance(self.t_support.get(), self.t_clear.get(),
                                  self.t_native.get(), self.t_five.get(),
                                  f'five-axis-{int(time.time())}',
                                  str(POLICY), str(CONFIG_PATH),
                                  velocity_acknowledged=self.t_vel.get())

    def _start(self, mode):
        if self.busy:
            messagebox.showwarning('五轴控制', '上一次控制尚未结束。')
            return
        if not self.reader:
            messagebox.showwarning('五轴控制', '先连接只读串口。')
            return
        if not self.settings or not self.settings.user_confirmed:
            messagebox.showwarning('五轴控制', '配置未加载或未由操作员确认 user_confirmed=true。')
            return
        if mode == 'velocity' and not self.t_vel.get():
            messagebox.showwarning('五轴控制', '速度模式需要先勾选“额外接受速度模式”；本次未接管、未使能。')
            return
        try:
            ticket = self._ticket(); ticket.validate()
            calibration = load_j5_calibration(J5_PATH)
            recovery_targets = None; recovery_duration = None; recovery_axes = ()
            if mode == 'recovery':
                envelopes, active = build_envelopes(self.settings, calibration)
                now = time.monotonic()
                recovery = []; qs = {}
                for i in active:
                    if not self.reader.state.fresh(i, now):
                        raise ValueError(f'J{i} 反馈不新鲜，不能计算回归0位')
                    qs[i] = math.radians(self.reader.state.feedback[i]['model_deg'])
                    env = envelopes[i]
                    if qs[i] < env.lower or qs[i] > env.upper:
                        recovery.append(i)
                if not recovery:
                    messagebox.showinfo('回归0位', '所有 active 轴都在包络内；不需要超限回退，请用普通轨迹。')
                    return
                duration = max([1.875*abs(qs[i])/0.02 for i in active] + [10.]) + 2.0
                if duration > 170.:
                    raise ValueError(f'回归0位预计 {duration:.1f}s 超过 170s 限制；请先分轴处理')
                lines = '；'.join(f'J{i} {math.degrees(qs[i]):+.2f}°→0°'
                                  for i in recovery)
                if not messagebox.askyesno(
                        '回归0位（超限回退）',
                        (f'{lines}\n其他 active 轴也回 0°；预计 {duration:.0f}s。\n'
                         '只允许向内、低速、单程；确认支撑/净空/可切电，且整臂回零路径无障碍？'),
                        icon='warning'):
                    return
                recovery_targets = tuple(0. for _ in active)
                recovery_duration = duration
                recovery_axes = tuple(recovery)
            self.busy = True; self.stop_requested.clear()
            thread = threading.Thread(target=self._worker,
                                      args=(mode, ticket, calibration, recovery_targets,
                                            recovery_duration, recovery_axes),
                                      daemon=True)
            thread.start()
        except Exception as exc:
            self.busy = False
            messagebox.showerror('五轴控制', str(exc))

    def _confirm_stop(self, session, timeout=1.2):
        try:
            session.stop()
        except Exception:
            pass
        end = time.monotonic()+timeout
        while session.controller.state not in ('DISABLED', 'UNKNOWN') and time.monotonic() < end:
            try:
                session.cycle()
            except Exception:
                break
            time.sleep(.010)
        session.endpoint.closed = True
        return session.controller.state

    def _worker(self, mode, ticket, calibration, recovery_targets=None,
                recovery_duration=None, recovery_axes=()):
        session = None; handle = None
        try:
            AUDIT_DIR.mkdir(parents=True, exist_ok=True)
            handle = (AUDIT_DIR/(time.strftime('%Y%m%d_%H%M%S')+f'_{mode}.jsonl')).open('w', encoding='utf-8')
            audit = make_event_writer(handle, lambda row: self.events.put(('log', row)))
            settings = self.settings
            _envelopes, active_axes = build_envelopes(settings, calibration)
            fresh_active_feedback(self.reader, active_axes,
                                  clock=clock_now, sleep=time.sleep)
            session = build_five_session(self.reader, settings, ticket, audit,
                                         clock=clock_now, calibration=calibration,
                                         recovery_axes=recovery_axes if mode == 'recovery' else (),
                                         max_runtime=180. if mode == 'recovery' else 60.)
            session.start()
            self.events.put(('log', f'开始 {mode}；active={session.controller.active_axes}'))
            while session.cycle() not in ('HOLD', 'DISABLED', 'UNKNOWN'):
                time.sleep(.010)
                if self.stop_requested.is_set():
                    break
            if mode == 'hold':
                end = time.monotonic()+2.
                while time.monotonic() < end and session.controller.state not in ('DISABLED', 'UNKNOWN'):
                    session.cycle(); time.sleep(.010)
            elif mode == 'trajectory':
                if session.controller.state != 'HOLD':
                    raise RuntimeError(f'takeover did not reach HOLD: {session.controller.state} {session.controller.reason}')
                targets = tuple(math.radians(float(self.entries[i].get()))
                                for i in session.controller.active_axes)
                duration = float(self.duration.get())
                session.controller.trajectory((targets,), duration, clock_now())
                while session.controller.state not in ('HOLD', 'DISABLED', 'UNKNOWN'):
                    session.cycle(); time.sleep(.010)
                    if self.stop_requested.is_set():
                        break
            elif mode == 'velocity':
                if session.controller.state != 'HOLD':
                    raise RuntimeError(f'takeover did not reach HOLD: {session.controller.state} {session.controller.reason}')
                targets = tuple(float(self.velocity_values[i].get())
                                for i in session.controller.active_axes)
                duration = float(self.velocity_duration.get())
                session.controller.velocity(targets, duration, clock_now())
                while session.controller.state not in ('HOLD', 'DISABLED', 'UNKNOWN'):
                    session.cycle(); time.sleep(.010)
                    if self.stop_requested.is_set():
                        break
            elif mode == 'recovery':
                if session.controller.state != 'HOLD':
                    raise RuntimeError(f'takeover did not reach HOLD: {session.controller.state} {session.controller.reason}')
                if recovery_targets is None or recovery_duration is None:
                    raise ValueError('recovery targets/duration missing')
                self.events.put(('log', f'回归0位：axes={recovery_axes} 时长={recovery_duration:.1f}s'))
                session.controller.trajectory((tuple(recovery_targets),),
                                              recovery_duration, clock_now())
                while session.controller.state not in ('HOLD', 'DISABLED', 'UNKNOWN'):
                    session.cycle(); time.sleep(.010)
                    if self.stop_requested.is_set():
                        break
            state = self._confirm_stop(session)
            self.events.put(('log', f'结束 {mode}：state={state} '
                                     f'reason={session.controller.reason}'))
            if state != 'DISABLED':
                self.latched = True
                self.events.put(('log', '停止未确认 DISABLED：锁定只读轮询；请断开并现场检查。'))
        except Exception as exc:
            started = bool(session is not None and getattr(session, 'started', False))
            self.events.put(('log', f'控制停止/错误：{exc}'))
            if session is not None:
                if started:
                    state = self._confirm_stop(session)
                    if state != 'DISABLED':
                        self.latched = True
                        self.events.put(('log', '异常后停止未确认 DISABLED：锁定轮询；请现场检查。'))
                    else:
                        self.events.put(('log', '异常后已确认 DISABLED；未锁定，可重连重试。'))
                else:
                    session.endpoint.closed = True
                    self.events.put(('log', '接管前预检失败：未使能、未发送控制帧；不锁定，修正后可重试。'))
        finally:
            if handle:
                handle.close()
            self.busy = False

    def _drain_events(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self._append(str(value))

    def request_stop(self):
        if self.busy:
            self.stop_requested.set()
            self._append('已请求停止：worker 将在当前周期调用 FD；不是急停。')
        else:
            self._append('当前没有控制运行；未发送任何帧。')

    def close(self):
        if self.busy:
            messagebox.showwarning('五轴控制', '控制运行中；先点停止并等待结束，再关闭窗口。')
            return
        self.closing = True
        self.stop_requested.set()
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = FiveAxisConsole(root)
    root.protocol('WM_DELETE_WINDOW', app.close)
    root.mainloop()


if __name__ == '__main__':
    main()
