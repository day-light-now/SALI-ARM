"""Native Qt workstation. Run: python -m slai_console.app (from workspace)."""
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen, QPixmap, QShortcut, QKeySequence
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFileDialog,
    QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QListWidget,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QSlider, QSplitter,
    QStackedWidget, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)
from .core import (ROOT, ACTIVE, LIMITS, Audit, CommandGateway, Context,
                   PreviewSlider, Rejected, validate_waypoints)
from .commissioning import capture, readiness
from .shadow_worker import ShadowWorker
from .clock import now as clock_now, CLOCK_NAME
from .adapters import SimAdapter, ReadOnlyWorker, RenderWorker, load_mapping, STATES, mujoco

STYLE = '''
QWidget { background:#0A0E13; color:#CED8E3; font:10pt "Microsoft YaHei UI"; }
QMainWindow {background:#0A0E13;} QFrame#panel {background:#111821; border:1px solid #243142; border-radius:6px;}
QLabel {background:transparent;} QLabel#title {font-size:16pt; font-weight:600; color:#EAF2FC;}
QLabel#brand {font-size:20pt; font-weight:700; color:#1FD3C1;}
QLabel#muted {color:#8A9AAE; font-size:9pt;} QLabel#actual {color:#3ED598; font:12pt Consolas;}
QPushButton {background:#182330; border:1px solid #2C3D50; border-radius:4px; padding:7px 12px;}
QPushButton:hover {background:#22374A; border-color:#4E9BFF;} QPushButton:focus {border:1px solid #4E9BFF;}
QPushButton:disabled {color:#667487; background:#111821; border-color:#1E2937;}
QPushButton#primary {background:#073D39; border-color:#1FD3C1; color:#59E9D9;}
QPushButton#danger {background:#321D24; border-color:#F0554B; color:#FF938E;}
QPushButton#primary:disabled {background:#111821; border-color:#243142; color:#667487;}
QPushButton#danger:disabled {background:#111821; border-color:#243142; color:#667487;}
QComboBox,QLineEdit,QDoubleSpinBox {background:#111B26; border:1px solid #2B3B4D; padding:5px; border-radius:4px;}
QComboBox QAbstractItemView {background:#172331; selection-background-color:#25445A;}
QListWidget {border:0; background:#0D141D; outline:0; padding:8px;}
QListWidget::item {padding:16px 9px; margin:2px 0; border-radius:4px;}
QListWidget::item:selected {background:#19313D; color:#59E9D9; border-left:3px solid #1FD3C1;}
QTableWidget {background:#101720; alternate-background-color:#121D28; gridline-color:#1E2937; border:1px solid #243142; selection-background-color:#224055;}
QHeaderView::section {background:#16212D; border:0; padding:7px 4px; color:#8FA5BD;}
QTextEdit {background:#0D141D; border:1px solid #243142; font:9pt Consolas;}
QSlider::groove:horizontal {height:4px; background:#293A4D;} QSlider::handle:horizontal {width:12px; margin:-5px 0; background:#4E9BFF; border-radius:5px;}
QSplitter::handle {background:#1E2937;} QScrollArea {border:0;} QStatusBar {color:#8298AC;}
'''


def label(text='', kind=None):
    w = QLabel(text)
    w.setWordWrap(True)
    if kind:
        w.setObjectName(kind)
    return w


def button(text, fn=None, kind=None):
    w = QPushButton(text)
    if fn:
        w.clicked.connect(fn)
    if kind:
        w.setObjectName(kind)
    return w


def panel(title=None):
    w = QFrame()
    w.setObjectName('panel')
    layout = QVBoxLayout(w)
    layout.setContentsMargins(14, 12, 14, 12)
    layout.setSpacing(9)
    if title:
        layout.addWidget(label(title, 'title'))
    return w, layout


def table(headers, rows=0):
    w = QTableWidget(rows, len(headers))
    w.setHorizontalHeaderLabels(headers)
    w.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    w.verticalHeader().hide()
    w.setAlternatingRowColors(True)
    w.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
    w.verticalHeader().setDefaultSectionSize(35)
    return w


def cells(w, row, values):
    for col, value in enumerate(values):
        item = QTableWidgetItem(str(value))
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        if values and values[0] == 'J5':
            item.setForeground(QColor('#6E7C8B'))
        w.setItem(row, col, item)


class Curve(QWidget):
    def __init__(self):
        super().__init__()
        self.history = deque(maxlen=900)
        self.axis = 0
        self.torque = False
        self.setMinimumHeight(125)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(42, 20, self.width()-60, self.height()-43)
        p.setPen(QColor('#273749'))
        for i in range(5):
            y = r.top()+r.height()*i/4
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
        if not self.history:
            p.setPen(QColor('#8298AC'))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, '等待新鲜反馈 · 不生成示例波形')
            return
        vals = []
        for _, actual, target, torque in self.history:
            vals.extend([torque[self.axis]] if self.torque else [actual[self.axis], target[self.axis]])
        finite = [v for v in vals if v is not None and math.isfinite(v)]
        lo, hi = (min(finite)-2, max(finite)+2) if finite else (-1, 1)
        p.setPen(QColor('#8FA5BD'))
        p.drawText(0, 25, f'{hi:.1f}')
        p.drawText(0, int(r.bottom()), f'{lo:.1f}')
        p.drawText(44, self.height()-4, '最近 30 秒  /  '+('仿真补偿力矩 Nm' if self.torque else '实际 —  /  目标 - -   °'))
        now = self.history[-1][0]
        for index, color, dashed in ((3, '#C67CFF', False),) if self.torque else ((1,'#3ED598',False),(2,'#4E9BFF',True)):
            pen = QPen(QColor(color), 1.5)
            if dashed:
                pen.setStyle(Qt.PenStyle.DashLine)
            p.setPen(pen)
            path = QPainterPath()
            active = False
            for row in self.history:
                value = row[index][self.axis]
                if value is None or not math.isfinite(value):
                    active = False
                    continue
                x = r.right()-(now-row[0])*r.width()/30
                y = r.bottom()-(value-lo)/(hi-lo)*r.height()
                if active:
                    path.lineTo(x,y)
                else:
                    path.moveTo(x,y)
                    active = True
            p.drawPath(path)


class View(QLabel):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim
        self.last = None
        self.setMinimumSize(300, 240)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet('background:#101822; border:1px solid #243142;')
        self.setToolTip('左键拖动旋转；滚轮缩放。蓝色半透明模型是未执行目标。')

    def mousePressEvent(self, event):
        self.last = event.position()

    def mouseMoveEvent(self, event):
        if self.last is not None and event.buttons() & Qt.MouseButton.LeftButton:
            d = event.position()-self.last
            self.sim.cam.azimuth -= d.x()*.4
            self.sim.cam.elevation = np.clip(self.sim.cam.elevation+d.y()*.4, -85, 85)
            self.last = event.position()

    def mouseReleaseEvent(self, event):
        self.last = None

    def wheelEvent(self, event):
        self.sim.cam.distance = float(np.clip(self.sim.cam.distance*(.9 if event.angleDelta().y()>0 else 1.1), .25, 4))

    def frame(self, rgb):
        h,w,_ = rgb.shape
        q = QImage(rgb.data,w,h,w*3,QImage.Format.Format_RGB888).copy()
        self.setPixmap(QPixmap.fromImage(q).scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))


class Console(QMainWindow):
    PAGES = ['设备概览','关节控制','软件零位与方向','动作轨迹  ·  仿真','动力学实验  ·  仅仿真','日志诊断']

    def __init__(self, audit=None):
        super().__init__()
        self.setWindowTitle('SLAI 机械臂桌面控制台 · 仿真 / 实机只读')
        self.resize(1440, 930)
        self.setMinimumSize(1140, 740)
        self.audit = audit if audit is not None else Audit()
        self.mapping = load_mapping()
        self.preview = PreviewSlider()
        self.gateway = CommandGateway(self.preview, self.log)
        self.sim = SimAdapter()
        self.worker = None
        self.shadow_worker = None
        self.live = dict(rows={}, tx=0, rx=0, at=0)
        self.mode = 'sim'
        self.last_tick = clock_now()
        self.accumulator = 0.
        self.frame_counter = 0
        self.render_ms = 0.
        self.recording = False
        self.records = deque(maxlen=30000)
        self.drafts = {}
        self.old_log_count = 0
        self.active_command_id = None
        self.settle_start = None
        self.sim_fault = False
        self.sim_feedback_at = clock_now()
        self.last_refresh_error = 0.
        self.build_ui()
        self.render_worker = RenderWorker(self.sim.sim.model)
        self.render_worker.frame.connect(self.on_frame)
        self.render_worker.failed.connect(self.on_render_error)
        self.render_worker.start()
        self.log('SAFE','启动为仿真；串口未打开；J5 无发送入口；实机运动被网关拒绝')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)
        self.cancel_shortcut = QShortcut(QKeySequence('Esc'),self)
        self.cancel_shortcut.activated.connect(self.cancel_preview)

    def log(self, level, message, **fields):
        if self.audit.file.closed:
            return
        self.audit(level,message,**fields)
        if hasattr(self, 'toast'):
            self.toast.setText(message)

    def build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(10,8,10,6)
        top = QHBoxLayout()
        top.addWidget(label('SLAI', 'brand'))
        top.addWidget(label('机械臂控制台\n本机桌面版', 'muted'))
        self.mode_box = QComboBox()
        self.mode_box.addItems(['仿真 · 可执行','实机 · 只读'])
        self.mode_box.currentIndexChanged.connect(self.change_mode)
        top.addWidget(self.mode_box)
        self.port = QComboBox()
        self.port.setEditable(True)
        from serial.tools import list_ports
        self.port.addItems([p.device for p in list_ports.comports()] or ['COM9'])
        self.port.setCurrentText('COM9')
        top.addWidget(self.port)
        self.connection = button('连接只读', self.connect_live)
        self.connection.setEnabled(False)
        top.addWidget(self.connection)
        self.health = label('仿真运行 · 物理设备未连接')
        top.addWidget(self.health, 1)
        top.addWidget(label('硬件急停\n未接入 / 未知', 'muted'))
        self.stop_button = button('软件停止', self.stop, 'danger')
        self.stop_button.setToolTip('仅停止当前仿真。实机只读时不会发送停机或失能报文；不能替代物理急停。')
        top.addWidget(self.stop_button)
        outer.addLayout(top)
        self.toast = label('所有角度采用已确认的本机模型坐标系；软件零位不写入电机。', 'muted')
        outer.addWidget(self.toast)
        body = QSplitter()
        outer.addWidget(body,1)
        nav_area = QWidget()
        nav_layout = QVBoxLayout(nav_area)
        nav_layout.setContentsMargins(0,0,0,0)
        self.nav = QListWidget()
        self.nav.addItems(self.PAGES)
        nav_layout.addWidget(self.nav,1)
        nav_layout.addWidget(label('角度与力矩\n绿实线：实际反馈\n蓝虚线：目标\n紫色：估算力矩\n\nJ5 预留\n不查询／不使能／不控制', 'muted'))
        nav_area.setMinimumWidth(168)
        nav_area.setMaximumWidth(215)
        body.addWidget(nav_area)
        middle = QWidget()
        ml = QVBoxLayout(middle)
        ml.setContentsMargins(5,0,5,0)
        self.heading = label('', 'title')
        ml.addWidget(self.heading)
        self.view_bar = label('MuJoCo · 你的 SLAI CAD 模型 · 肩部安装版 · 蓝色为未执行预览', 'muted')
        ml.addWidget(self.view_bar)
        self.view = View(self.sim)
        ml.addWidget(self.view,3)
        self.pages = QStackedWidget()
        ml.addWidget(self.pages,2)
        self.inspector = QStackedWidget()
        self.inspector.setMinimumWidth(315)
        self.inspector.setMaximumWidth(400)
        self.make_overview()
        self.make_joints()
        self.make_zero()
        self.make_trajectory()
        self.make_dynamics()
        self.make_logs()
        self.dock = QWidget()
        dl = QVBoxLayout(self.dock)
        dl.setContentsMargins(0,0,0,0)
        chartbar = QHBoxLayout()
        chartbar.addWidget(label('反馈记录', 'muted'))
        axis = QComboBox()
        axis.addItems(['J1','J2','J3','J4'])
        axis.currentIndexChanged.connect(lambda i:setattr(self.curve,'axis',i))
        chartbar.addWidget(axis)
        self.chart_kind = QComboBox()
        self.chart_kind.addItems(['实际 / 目标角度','仿真重力力矩'])
        self.chart_kind.currentIndexChanged.connect(lambda i:setattr(self.curve,'torque',bool(i)))
        chartbar.addWidget(self.chart_kind)
        chartbar.addStretch()
        dl.addLayout(chartbar)
        dock_split = QSplitter()
        self.curve = Curve()
        dock_split.addWidget(self.curve)
        self.log_tail = QTextEdit()
        self.log_tail.setReadOnly(True)
        self.log_tail.setMaximumHeight(165)
        dock_split.addWidget(self.log_tail)
        dock_split.setSizes([400,400])
        dl.addWidget(dock_split)
        ml.addWidget(self.dock,1)
        body.addWidget(middle)
        body.addWidget(self.inspector)
        body.setSizes([185,860,360])
        self.nav.currentRowChanged.connect(self.change_page)
        self.nav.setCurrentRow(1)
        self.statusBar().showMessage(str(self.audit.path))

    def add_inspector(self, title):
        content, layout = panel(title)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        self.inspector.addWidget(scroll)
        return layout

    def make_overview(self):
        w,l = panel('关节健康与能力')
        self.overview = table(['关节','实际 °','目标 °','编码器 raw16','温度 ℃','状态'],5)
        self.overview.setFixedHeight(215)
        l.addWidget(self.overview)
        l.addWidget(label('已实现：四轴仿真 / 实机只读反馈 / 本地映射导出\n待验收：实机运动与多轴联动　　仅仿真：重力补偿 / 阻抗实验　　J5：未启用','muted'))
        l.addStretch()
        self.pages.addWidget(w)
        il = self.add_inspector('连接与自检')
        il.addWidget(label('实机：USB2CAN · CAN 模式\n921600 baud · 仅查询 J1–J4\n先释放其他上位机占用的串口。'))
        self.selfcheck_text = label('尚未进行连接检查')
        il.addWidget(self.selfcheck_text)
        il.addWidget(button('检查当前状态',self.selfcheck,'primary'))
        self.read_rate=QComboBox()
        self.read_rate.addItem('标准只读 · 每50ms查询一轴',.05)
        self.read_rate.addItem('快速只读诊断 · 每10ms查询一轴',.01)
        self.read_rate.currentIndexChanged.connect(self.change_read_rate)
        il.addWidget(self.read_rate)
        il.addWidget(label('快速诊断仅增加查询，不使能；影子记录本身不改变查询速率。','muted'))
        il.addWidget(button('导出当前参数快照',self.export_parameter_snapshot))
        il.addWidget(button('运行 20 秒影子记录 · 不下发',self.start_shadow))
        il.addWidget(button('停止影子记录',self.stop_shadow))
        self.shadow_status=label('影子模式：未运行。100 Hz 是主机计算目标，不代表总线控制周期。','muted')
        il.addWidget(self.shadow_status)
        il.addWidget(label('型号来自本地映射：\nJ1 / J2：DM-J10010L-2EC\nJ3 / J4：DM-J8009P-2EC\nJ5：DM-J4340-2EC（锁定）','muted'))
        il.addWidget(label('未接入：硬件急停、碰撞检测、负载辨识。\nMuJoCo 的惯量仍是暂估值。','muted'))
        il.addStretch()

    def make_joints(self):
        w,l = panel('J1–J4 目标预览')
        l.addWidget(label('拖动只改蓝色预览。点击右侧“执行到仿真”才会运动；Esc 取消。','muted'))
        self.joint_summary = label('', 'actual')
        l.addWidget(self.joint_summary)
        l.addWidget(label('鼠标在模型上拖动可旋转视角，滚轮缩放。实机模式只映射反馈姿态，不输出控制指令。','muted'))
        l.addStretch()
        w.setMaximumHeight(160)
        self.pages.addWidget(w)
        il = self.add_inspector('关节操作')
        self.enable_button = button('暂停仿真控制',self.toggle_enabled)
        il.addWidget(self.enable_button)
        self.sliders, self.spins, self.axis_values = [], [], []
        for i,(lo,hi) in enumerate(LIMITS,1):
            il.addWidget(label(f'J{i}　{lo:g}° ～ {hi:g}°','muted'))
            val = label('实际 —　目标 —','actual')
            self.axis_values.append(val)
            il.addWidget(val)
            row = QHBoxLayout()
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(int(lo*100),int(hi*100))
            slider.setValue(0)
            spin = QDoubleSpinBox()
            spin.setRange(lo,hi)
            spin.setDecimals(2)
            spin.setSingleStep(.1)
            spin.setSuffix(' °')
            slider.valueChanged.connect(lambda v,j=i:self.edit_target(j,v/100))
            spin.valueChanged.connect(lambda v,j=i:self.edit_target(j,v))
            self.sliders.append(slider)
            self.spins.append(spin)
            row.addWidget(slider,1)
            row.addWidget(spin)
            il.addLayout(row)
        il.addWidget(label('J5　🔒 预留\n不查询／不使能／不控制','muted'))
        self.pending = label('无待执行预览')
        il.addWidget(self.pending)
        self.execute = button('执行到仿真',self.execute_preview,'primary')
        il.addWidget(self.execute)
        il.addWidget(button('取消预览 · Esc',self.cancel_preview))
        self.zero_preview_button = button('预览全部零位',self.preview_zero)
        il.addWidget(self.zero_preview_button)
        il.addWidget(label('限位来自当前 URDF；仍须按实物核验。\n实机只读；仿真暂停不等于电机失能。','muted'))
        il.addStretch()

    def make_zero(self):
        w,l = panel('软件零位与方向 · 本地记录')
        self.zero_table = table(['关节','零位偏移 rad','方向','本地记录'],5)
        self.zero_table.setFixedHeight(215)
        for i in ACTIVE:
            m = self.mapping[i]
            cells(self.zero_table,i-1,[f'J{i}',f"{m['zero_offset_rad']:+.9f}",f"{m['hardware_sign']:+g}",'沿用已确认映射'])
        cells(self.zero_table,4,['J5','—','—','锁定'])
        l.addWidget(self.zero_table)
        l.addWidget(label('encoder_rad = raw16 × 2 × PMAX / 65535 − PMAX\nmodel_rad = direction × (encoder_rad − zero_offset_rad)\nPMAX 来自电机回复；不按 16384 计数换算。','muted'))
        l.addStretch()
        self.pages.addWidget(w)
        il = self.add_inspector('标定链与运动分离')
        self.zero_axis = QComboBox()
        self.zero_axis.addItems(['J1','J2','J3','J4'])
        il.addWidget(self.zero_axis)
        self.chain = label('请在实机只读模式查看编码器 → 偏移 → 方向 → 软件角度。')
        il.addWidget(self.chain)
        il.addWidget(button('采集当前位置到草稿',self.capture_zero))
        il.addWidget(button('导出映射 / 草稿 JSON',self.export_mapping,'primary'))
        il.addWidget(label('草稿不会自动生效，不覆盖已确认映射；只在静止、失能且反馈新鲜时采集。','muted'))
        il.addWidget(label('驱动回零：属于运动命令，当前实机不开放。\n仿真回零：先预览，再到关节页执行。'))
        il.addWidget(button('预览仿真零位',self.zero_to_joint))
        il.addStretch()

    def make_trajectory(self):
        w,l = panel('关节空间路点 · J5 不参与')
        self.waypoints = table(['路点','J1 °','J2 °','J3 °','J4 °','J5'],0)
        self.waypoints.setMaximumHeight(220)
        self.waypoints.setEditTriggers(QTableWidget.EditTrigger.DoubleClicked|QTableWidget.EditTrigger.EditKeyPressed)
        l.addWidget(self.waypoints)
        row = QHBoxLayout()
        row.addWidget(button('添加当前仿真姿态',self.add_waypoint))
        row.addWidget(button('删除选中路点',self.remove_waypoint))
        l.addLayout(row)
        self.pages.addWidget(w)
        il = self.add_inspector('轨迹规划')
        il.addWidget(label('五次最小跃度关节插补\n端点速度与加速度为 0。'))
        self.duration = QDoubleSpinBox()
        self.duration.setRange(1,60)
        self.duration.setValue(4)
        self.duration.setSuffix(' 秒 / 段')
        il.addWidget(self.duration)
        il.addWidget(button('校验路点与速度估算',self.check_trajectory))
        self.plan_info = label('未规划。限位检查可用；碰撞 / 奇异点检查未接入。')
        il.addWidget(self.plan_info)
        self.trajectory_execute = button('执行仿真轨迹',self.run_trajectory,'primary')
        il.addWidget(self.trajectory_execute)
        il.addWidget(button('导出路点 CSV',self.export_trajectory))
        il.addWidget(label('实机联动：待验收，网关拒绝。\n速度为几何规划估算，未施加实机额定约束。','muted'))
        il.addStretch()

    def make_dynamics(self):
        w,l = panel('重力补偿与阻抗 · 仅仿真')
        self.dynamics_text = label('')
        l.addWidget(self.dynamics_text)
        l.addWidget(label('τ = g(q) + Kp(q目标 − q) − Kd·q̇\n浮动模式 Kp=0；关闭补偿仍保留演示阻尼。\n惯量和质量为当前暂估模型；无碰撞，不含线缆、负载、摩擦辨识。','muted'))
        l.addStretch()
        w.setMaximumHeight(200)
        self.pages.addWidget(w)
        il = self.add_inspector('动力学实验')
        self.dyn_mode = QComboBox()
        self.dyn_mode.addItems(['弹簧保持 + 重力补偿','浮动 + 重力补偿','关闭补偿 / 阻尼'])
        self.dyn_mode.currentIndexChanged.connect(self.set_dynamics)
        il.addWidget(self.dyn_mode)
        il.addWidget(button('捕获当前姿态为保持目标',self.capture_sim))
        il.addWidget(button('末端 +Z 扰动 8 N / 0.4 秒',self.push_sim,'primary'))
        il.addWidget(label('这是仿真外力，不会向电机发报文。\n内部积分 1 ms；显示与采样以实际 UI 周期计时。','muted'))
        self.record_button = button('开始记录',self.toggle_record)
        il.addWidget(self.record_button)
        il.addWidget(button('导出仿真记录 CSV',self.export_records))
        self.record_label = label('未记录')
        il.addWidget(self.record_label)
        il.addStretch()

    def make_logs(self):
        w,l = panel('日志诊断')
        row = QHBoxLayout()
        self.log_filter = QComboBox()
        self.log_filter.addItems(['全部','INFO','WARN','CMD','SAFE','ERROR'])
        self.search = QLineEdit()
        self.search.setPlaceholderText('搜索消息 / 关联 ID')
        self.log_filter.currentTextChanged.connect(self.refresh_logs)
        self.search.textChanged.connect(self.refresh_logs)
        row.addWidget(self.log_filter)
        row.addWidget(self.search)
        l.addLayout(row)
        self.log_table = table(['时间','级别','消息 / 关联 ID'])
        self.log_table.horizontalHeader().setSectionResizeMode(0,QHeaderView.ResizeMode.ResizeToContents)
        self.log_table.horizontalHeader().setSectionResizeMode(1,QHeaderView.ResizeMode.ResizeToContents)
        l.addWidget(self.log_table)
        self.pages.addWidget(w)
        il = self.add_inspector('诊断与导出')
        self.diagnostics = label('')
        il.addWidget(self.diagnostics)
        il.addWidget(button('导出本次 JSONL 日志',self.export_logs,'primary'))
        il.addWidget(label('每次仿真执行都有 command_id；\n真实 CAN 帧另存在只读通信日志。\n未测量的往返延迟 / 丢帧率不伪造。\n软件停止与断开不等于实机急停。','muted'))
        il.addStretch()

    def context(self):
        if self.mode=='sim':
            return Context(mode='sim',connected=self.sim.sim.model.nq==4,
                enabled=self.sim.enabled,fault=self.sim_fault,busy=self.sim.busy,
                fresh=0<=clock_now()-self.sim_feedback_at<=.5
                      and bool(np.isfinite(self.sim.sim.data.qpos).all()))
        rows = self.fresh_rows()
        return Context(mode=self.mode,connected=bool(self.worker and self.worker.isRunning()),
            fresh=len(rows)==4,enabled=len(rows)==4 and all(f['state_code']==1 for f in rows.values()),
            fault=any(f['state_code'] not in (0,1) for f in rows.values()))

    def change_page(self, page):
        self.cancel_preview()
        self.pages.setCurrentIndex(page)
        self.pages.setMaximumHeight(160 if page==1 else 200 if page==4 else 16777215)
        self.inspector.setCurrentIndex(page)
        self.heading.setText(self.PAGES[page])
        self.view.setVisible(page in (1,3,4))
        self.view_bar.setVisible(page in (1,3,4))
        self.dock.setVisible(page != 5)
        self.log('INFO','切换页面：'+self.PAGES[page])

    def cancel_preview(self):
        self.preview.clear()
        if hasattr(self,'pending'):
            self.pending.setText('无待执行预览')
        if hasattr(self,'spins'):
            values = np.rad2deg(self.sim.sim.data.qpos) if self.mode=='sim' else [self.fresh_rows().get(i,{}).get('model_deg',0.) for i in ACTIVE]
            for i in range(4):
                self.spins[i].blockSignals(True)
                self.sliders[i].blockSignals(True)
                self.spins[i].setValue(float(values[i]))
                self.sliders[i].setValue(round(float(values[i])*100))
                self.spins[i].blockSignals(False)
                self.sliders[i].blockSignals(False)

    def edit_target(self,j,v):
        if self.mode!='sim':
            self.log('SAFE','实机只读：不创建运动预览')
            return
        try:
            self.preview.edit(j,v)
            for w,value in ((self.spins[j-1],v),(self.sliders[j-1],round(v*100))):
                w.blockSignals(True)
                w.setValue(value)
                w.blockSignals(False)
        except ValueError as exc:
            self.log('SAFE',str(exc))

    def execute_preview(self):
        try:
            ticket = self.preview.ticket
            targets = self.gateway.commit(ticket,self.context())
            self.sim.motion(targets,ticket.duration)
            self.active_command_id = ticket.nonce
            self.settle_start = None
        except ValueError as exc:
            self.log('SAFE',str(exc))

    def preview_zero(self):
        if self.mode!='sim':
            self.log('SAFE','实机只读：不创建回零预览')
            return
        self.preview.clear()
        for i in ACTIVE:
            self.edit_target(i,0.)

    def zero_to_joint(self):
        self.nav.setCurrentRow(1)
        self.preview_zero()

    def change_mode(self,index):
        self.stop()
        if not self.disconnect_live():
            self.mode_box.blockSignals(True)
            self.mode_box.setCurrentIndex(0 if self.mode=='sim' else 1)
            self.mode_box.blockSignals(False)
            self.log('ERROR','通信尚未退出；模式切换已取消，保持原模式')
            return
        self.mode = 'sim' if index == 0 else 'live'
        self.sim.paused = self.mode != 'sim'
        self.sim.sim.data.qvel[:] = 0
        self.sim.sim.capture()
        self.curve.history.clear()
        self.recording = False
        self.record_button.setText('开始记录')
        self.log('SAFE','切换到 '+('仿真' if self.mode=='sim' else '实机只读；不会自动连接'))
        self.connection.setEnabled(self.mode=='live')
        self.enable_button.setEnabled(self.mode=='sim')
        self.stop_button.setText('软件停止' if self.mode=='sim' else '只读 · 无停机权限')
        for w in [*self.sliders,*self.spins,self.zero_preview_button,self.trajectory_execute,self.dyn_mode]:
            w.setEnabled(self.mode=='sim')

    def connect_live(self):
        if self.mode != 'live':
            self.log('SAFE','先选择“实机 · 只读”，连接不会自动进行')
            return
        self.cancel_preview()
        if self.worker is not None:
            self.disconnect_live()
            return
        worker = ReadOnlyWorker(self.port.currentText().strip(),self.mapping,self.read_rate.currentData())
        self.worker = worker
        worker.sample.connect(self.on_sample)
        worker.status.connect(lambda s:self.log('INFO',s))
        worker.finished.connect(lambda w=worker:self.worker_finished(w))
        worker.start()
        self.connection.setText('断开只读')

    def change_read_rate(self,index):
        if self.shadow_worker is not None:
            self.log('SAFE','先停止影子记录再改采样速率，避免同一记录混用两种计划')
            self.read_rate.blockSignals(True)
            self.read_rate.setCurrentIndex(getattr(self,'accepted_read_rate_index',0))
            self.read_rate.blockSignals(False)
            return
        self.accepted_read_rate_index=index
        if self.worker is not None:
            self.worker.set_interval(self.read_rate.currentData())
        self.log('INFO','只读查询计划：'+self.read_rate.currentText()+'；不是运动控制频率')

    def on_sample(self,sample):
        try:
            if not isinstance(sample,dict) or not math.isfinite(sample['at']):
                raise ValueError('无效采样时间')
            for i,f in sample['rows'].items():
                if i not in ACTIVE:
                    raise ValueError('非目标编号')
                for key in ('position_rad','model_deg','velocity_rad_s','driver_temp_c','motor_temp_c','age'):
                    if not math.isfinite(f[key]):
                        raise ValueError('非有限反馈字段：'+key)
                if not 0<=f['state_code']<=15 or f['age']<0:
                    raise ValueError('状态码或反馈年龄错误')
                if len(bytes.fromhex(f['payload']))!=8:
                    raise ValueError('报文不是八字节')
                for key in (21,22,23):
                    if not math.isfinite(f['regs'][key]) or f['regs'][key]<=0:
                        raise ValueError('无效映射范围')
                for key in (9,10):
                    if not isinstance(f['regs'][key],int):
                        raise ValueError('缺少模式或超时参数')
            self.live = sample
            if self.shadow_worker is not None:
                self.shadow_worker.submit(sample)
        except (KeyError,ValueError,TypeError,AttributeError) as exc:
            self.live = dict(rows={},tx=0,rx=0,at=0)
            if self.shadow_worker is not None:
                self.shadow_worker.submit(self.live)
            self.cancel_preview()
            self.log('ERROR','丢弃无效遥测：'+str(exc))

    def worker_finished(self, worker):
        if self.worker is worker:
            self.worker = None
            self.connection.setText('连接只读')
            self.cancel_preview()

    def disconnect_live(self):
        if not self.stop_shadow():
            return False
        if self.worker is not None:
            worker = self.worker
            worker.requestInterruption()
            if not worker.wait(2000):
                self.log('ERROR','通信线程尚未退出，保留连接对象等待，不切换模式')
                return False
            self.worker = None
        self.live = dict(rows={},tx=0,rx=0,at=0)
        self.connection.setText('连接只读')
        self.cancel_preview()
        return True

    def commissioning_snapshot(self):
        if self.mode!='live':
            raise ValueError('请先手动连接实机只读；不从仿真冒充参数快照')
        if self.live.get('clock')!=CLOCK_NAME:
            raise ValueError('会话计时域不符，请断开后重连新版读取线程')
        return capture(self.live,self.mapping,clock_now())

    def export_parameter_snapshot(self):
        try:
            if self.mode!='live':
                raise ValueError('只在实机只读模式导出；不从仿真生成硬件快照')
            data=dict(schema=1,captured_unix_s=time.time(),source='existing_readonly_worker',
                sample=self.live,local_software_mapping=self.mapping,
                model_limits_only_deg=LIMITS,hardware_writes=0,actuation_tx=0,
                fresh_four_axes=len(self.fresh_rows())==4,motion_permitted=False)
            try:
                snapshot=self.commissioning_snapshot()
                data['validated_shadow_snapshot']=snapshot.serializable()
                data['motion_readiness']=readiness(snapshot)
            except Exception as exc:
                data['shadow_preparation_error']=str(exc)
            self.save('导出实机只读参数快照','parameter_snapshot.json',
                lambda p:p.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8'))
        except Exception as exc:
            self.log('SAFE','快照未导出：'+str(exc))

    def start_shadow(self):
        if self.shadow_worker is not None:
            self.log('SAFE','影子记录已在运行')
            return
        try:
            snapshot=self.commissioning_snapshot()
            worker=ShadowWorker(snapshot,self.mapping,self.live,sample_provider=self.worker.peek)
            self.shadow_worker=worker
            worker.report.connect(self.on_shadow_report)
            worker.finished.connect(lambda w=worker:self.shadow_finished(w))
            worker.start()
            self.log('INFO','影子模式开始：冻结当前反馈为候选目标；不使能、不下发、不写参数')
        except Exception as exc:
            self.log('SAFE','影子模式未启动：'+str(exc))

    def on_shadow_report(self,report):
        host=report.get('host_cycle_ms',{})
        self.shadow_status.setText(f"影子周期 {report.get('cycles',0)} / 运动发送 0\n"
            f"主机周期 p95：{host.get('p95','—')} ms\n"
            f"超出80ms反馈年龄的周期：{report.get('cycles_exceeding_80ms_feedback_age','—')}\n"
            f"全局锁存：{report.get('global_latch') or report.get('error') or '无'}\n"
            '总线控制周期：未测；实机运动：锁定。\n日志：'+Path(report['path']).name)
        if report.get('finished'):
            self.log('INFO','影子记录结束：'+report['path'])

    def shadow_finished(self,worker):
        if self.shadow_worker is worker:
            self.shadow_worker=None

    def stop_shadow(self):
        worker=self.shadow_worker
        if worker is not None:
            worker.requestInterruption()
            if not worker.wait(2000):
                self.log('ERROR','等待影子线程退出；连接/模式保持原状')
                return False
            self.shadow_worker=None
        return True

    def stop(self):
        self.cancel_preview()
        self.sim.stop()
        if self.mode=='sim':
            self.log('SAFE','仿真运动已停止，保持当前姿态')
        else:
            self.log('WARN','实机只读：未发送停机命令；请使用物理急停或已验收工具')

    def toggle_enabled(self):
        if self.mode!='sim':
            return
        self.stop()
        self.sim.enabled = not self.sim.enabled
        self.enable_button.setText('暂停仿真控制' if self.sim.enabled else '恢复仿真控制')

    def selfcheck(self):
        if self.mode=='sim':
            text = '仿真模型：4 个关节 ✓\n初始零位：已确认模型坐标系\n碰撞检测：未接入\nJ5：跳过 ✓'
        else:
            rows = self.fresh_rows()
            text = f'新鲜反馈：{len(rows)}/4\nJ5：未查询 ✓\n'
            for i in ACTIVE:
                f = rows.get(i)
                text += f'J{i}：'+(f"{STATES.get(f['state_code'],'未知')} / 模式 {f['regs'][10]} / TIMEOUT {f['regs'][9]}" if f else '无新鲜反馈')+'\n'
        self.selfcheck_text.setText(text)
        self.log('INFO',text.replace('\n','；'))

    def fresh_rows(self):
        age = clock_now()-self.live['at']
        return {i:f for i,f in self.live['rows'].items() if 0<=age+f['age']<=1.0}

    def capture_zero(self):
        i = self.zero_axis.currentIndex()+1
        f = self.fresh_rows().get(i) if self.mode=='live' else None
        if not f or f['state_code']!=0 or abs(f['velocity_rad_s'])>.02:
            self.log('SAFE','采集要求实机新鲜反馈、失能且速度 < 0.02 rad/s')
            return
        self.drafts[i] = dict(zero_offset_rad=f['position_rad'],hardware_sign=self.mapping[i]['hardware_sign'],time=time.time(),applied=False)
        cells(self.zero_table,i-1,[f'J{i}',f"{f['position_rad']:+.9f}",self.mapping[i]['hardware_sign'],'草稿 · 未应用'])
        self.log('SAFE',f'J{i} 零位采集到本地草稿；未修改电机或生效映射')

    def save(self,title,suffix,writer):
        path,_ = QFileDialog.getSaveFileName(self,title,str(ROOT/'tmp/desktop_console'/suffix))
        if path:
            try:
                writer(Path(path))
                self.log('INFO','已导出：'+path)
            except Exception as exc:
                self.log('ERROR','导出失败：'+str(exc))

    def export_mapping(self):
        self.save('导出软件映射','software_mapping.json',lambda p:p.write_text(json.dumps(dict(schema=1,units='rad',active=self.mapping,drafts=self.drafts,drafts_applied=False,j5='forbidden'),ensure_ascii=False,indent=2),encoding='utf-8'))

    def add_waypoint(self):
        if self.mode!='sim':
            self.log('SAFE','仅从仿真姿态添加路点')
            return
        row = self.waypoints.rowCount()
        self.waypoints.insertRow(row)
        cells(self.waypoints,row,[str(row+1),*[f'{v:.2f}' for v in np.rad2deg(self.sim.sim.data.qpos[:4])],'—'])
        for col in (0,5):
            item = self.waypoints.item(row,col)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)

    def remove_waypoint(self):
        row = self.waypoints.currentRow()
        if row>=0:
            self.waypoints.removeRow(row)

    def read_waypoints(self):
        points = [[float(self.waypoints.item(r,c).text()) for c in range(1,5)] for r in range(self.waypoints.rowCount())]
        return validate_waypoints(points,self.duration.value())

    def check_trajectory(self):
        try:
            points = self.read_waypoints()
            seq = np.vstack([np.rad2deg(self.sim.sim.data.qpos),points])
            peak = np.max(np.abs(np.diff(seq,axis=0)),axis=0)*1.875/self.duration.value()
            self.plan_info.setText(f'{len(points)} 段 / 总时长 {len(points)*self.duration.value():.1f} 秒\n各轴峰值 °/s：'+', '.join(f'{v:.1f}' for v in peak)+'\n模型限位通过；碰撞与奇异点未检查。')
            return points
        except (ValueError,AttributeError) as exc:
            self.log('SAFE','轨迹校验失败：'+str(exc))
            return None

    def run_trajectory(self):
        points = self.check_trajectory()
        if points is None:
            return
        try:
            self.cancel_preview()
            ticket = self.preview.trajectory(points,self.duration.value())
            self.gateway.commit(ticket,self.context())
            self.sim.trajectory(ticket.plan,ticket.duration)
            self.active_command_id = ticket.nonce
            self.settle_start = None
            self.log('CMD','仿真轨迹已开始',command_id=ticket.nonce,waypoints=points,duration=self.duration.value())
        except ValueError as exc:
            self.log('SAFE',str(exc))

    @staticmethod
    def write_csv(path,headers,rows):
        with path.open('w',encoding='utf-8-sig',newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(rows)

    def export_trajectory(self):
        points = self.check_trajectory()
        if points:
            self.save('导出路点','waypoints.csv',lambda p:self.write_csv(p,['J1_deg','J2_deg','J3_deg','J4_deg'],points))

    def set_dynamics(self,index):
        if self.mode!='sim':
            self.log('SAFE','动力学仅在仿真生效')
            return
        self.stop()
        self.sim.sim.set_mode(('spring','float','off')[index])
        self.log('INFO','仿真模式：'+self.dyn_mode.currentText())

    def capture_sim(self):
        if self.mode=='sim':
            self.stop()
            self.sim.sim.capture()

    def push_sim(self):
        if self.mode=='sim' and self.sim.enabled:
            self.sim.sim.push()
            self.log('CMD','仿真末端 +Z 扰动 8 N / 0.4 秒')
        else:
            self.log('SAFE','扰动仅在运行的仿真中可用')

    def toggle_record(self):
        if self.mode!='sim':
            self.log('SAFE','动力学记录仅接受仿真数据')
            return
        self.recording = not self.recording
        self.record_button.setText('停止记录' if self.recording else '开始记录')

    def export_records(self):
        self.save('导出仿真采样','simulation.csv',lambda p:self.write_csv(p,['sim_time',*[f'q{i}_rad' for i in ACTIVE],*[f'g{i}_Nm' for i in ACTIVE]],self.records))

    def export_logs(self):
        self.save('导出本次日志','console.jsonl',lambda p:p.write_text(self.audit.path.read_text(encoding='utf-8'),encoding='utf-8'))

    def refresh_logs(self,*args):
        if not hasattr(self,'log_table'):
            return
        level = self.log_filter.currentText()
        query = self.search.text().lower()
        rows = [r for r in self.audit.rows if (level=='全部' or r['level']==level) and query in json.dumps(r,ensure_ascii=False).lower()]
        self.log_table.setRowCount(len(rows))
        for i,r in enumerate(rows):
            cells(self.log_table,i,[time.strftime('%H:%M:%S',time.localtime(r['time'])),r['level'],r['message']+' '+r.get('command_id','')])
        self.log_table.scrollToBottom()

    def tick(self):
        try:
            self._tick()
        except Exception as exc:
            self.sim.enabled = False
            self.sim_fault = True
            self.live = dict(rows={},tx=0,rx=0,at=0)
            self.preview.clear()
            self.curve.history.clear()
            for i in ACTIVE:
                cells(self.overview,i-1,[f'J{i}','—','—','—','—','刷新错误'])
                self.axis_values[i-1].setText('反馈无效 · 控制已暂停')
            if clock_now()-self.last_refresh_error>1:
                self.last_refresh_error = clock_now()
                self.log('ERROR','界面刷新已降级；控制暂停：'+str(exc))

    def _tick(self):
        now = clock_now()
        elapsed = min(.1,now-self.last_tick)
        self.last_tick = now
        self.accumulator += elapsed
        count = int(self.accumulator/.001)
        self.accumulator -= count*.001
        try:
            self.sim.step(count)
            self.sim_feedback_at = now
        except Exception as exc:
            self.sim.enabled = False
            self.sim_fault = True
            self.log('ERROR','仿真已暂停：'+str(exc))
        rows = self.fresh_rows() if self.mode=='live' else {}
        if self.mode=='live':
            for i,f in rows.items():
                self.sim.sim.data.qpos[i-1] = math.radians(f['model_deg'])
            mujoco.mj_forward(self.sim.sim.model,self.sim.sim.data)
        q = np.rad2deg(self.sim.sim.data.qpos)
        g = self.sim.sim.gravity_torque()
        actual,target = [],[]
        for i in ACTIVE:
            f = rows.get(i)
            a = float(q[i-1]) if self.mode=='sim' else (f['model_deg'] if f else None)
            t = self.preview.values.get(i)
            if t is None and self.mode=='sim':
                t = float(np.rad2deg(self.sim.sim.target[i-1]))
            actual.append(a)
            target.append(t)
            av = '—' if a is None else f'{a:+.2f}'
            tv = '—' if t is None else f'{t:+.2f}'
            state = ('运行' if self.sim.enabled else '暂停') if self.mode=='sim' else (STATES.get(f['state_code'],'未知') if f else '反馈缺失 / 过期')
            raw = '—' if not f else int.from_bytes(bytes.fromhex(f['payload'])[1:3],'big')
            temp = '—' if not f else f"{f['driver_temp_c']}/{f['motor_temp_c']}"
            cells(self.overview,i-1,[f'J{i}',av,tv,raw,temp,state])
            self.axis_values[i-1].setText(f'<span style="color:#3ED598">实际 {av}°</span>　<span style="color:#4E9BFF">目标 {tv}°</span>')
            self.overview.item(i-1,1).setForeground(QColor('#3ED598'))
            self.overview.item(i-1,2).setForeground(QColor('#4E9BFF'))
            self.overview.item(i-1,3).setForeground(QColor('#C67CFF'))
        cells(self.overview,4,['J5','—','—','—','—','禁发送'])
        self.joint_summary.setText('　 '.join(f'J{i}  '+('—' if actual[i-1] is None else f'{actual[i-1]:+.2f}°') for i in ACTIVE))
        if self.preview.state=='EXECUTING' and not self.sim.busy:
            if self.settle_start is None:
                self.settle_start = self.sim.sim.data.time
            err = np.max(np.abs(np.rad2deg(self.sim.sim.target-self.sim.sim.data.qpos)))
            if err<.3:
                self.preview.state = 'DONE'
                self.log('INFO',f'仿真到达目标；最大误差 {err:.3f}°',command_id=self.active_command_id)
            elif self.sim.sim.data.time-self.settle_start>10:
                self.preview.state = 'FAILED'
                self.sim.stop()
                self.log('WARN',f'仿真未在 10 秒内收敛；误差 {err:.3f}°，已停止',command_id=self.active_command_id)
        reason = self.gateway.reason(self.preview.ticket,self.context())
        self.execute.setEnabled(not reason)
        self.execute.setToolTip(reason or '执行本次仿真预览')
        states={'IDLE':'无待执行预览','PREVIEW':'预览中','EXECUTING':'仿真执行中','DONE':'已到达仿真目标','FAILED':'仿真未收敛，已停止'}
        self.pending.setText((f'待执行 {len(self.preview.values)} 轴 · 仅本地预览' if self.preview.values else states.get(self.preview.state,self.preview.state))+'\n'+(reason if self.mode!='sim' else ''))
        self.health.setText(f'仿真 {self.sim.sim.data.time:.2f} s · '+('运行' if self.sim.enabled else '暂停') if self.mode=='sim' else f"实机只读 · 新鲜 {len(rows)}/4 · TX {self.live['tx']} / RX {self.live['rx']}")
        self.view_bar.setText('MuJoCo · SLAI 原模型 · 肩部安装 · 蓝色为预览' if self.mode=='sim' else '实机只读姿态映射 · 缺失轴保留上次可视姿态，不代表当前状态')
        self.curve.history.append((now,actual,target,list(g) if self.mode=='sim' else [None]*4))
        self.curve.update()
        if self.recording:
            self.records.append([self.sim.sim.data.time,*self.sim.sim.data.qpos.tolist(),*g.tolist()])
        self.record_label.setText(f'{"记录中" if self.recording else "记录已停止"} · {len(self.records)} / 30000 样本（环形缓存）')
        self.dynamics_text.setText('重力补偿力矩 g(q) / Nm（仿真暂估）\n'+'　'.join(f'J{i}: {g[i-1]:+.3f}' for i in ACTIVE)+f'\n当前模式：{self.sim.sim.mode}；仿真积分时间 {self.sim.sim.data.time:.3f} s')
        self.dyn_mode.blockSignals(True)
        self.dyn_mode.setCurrentIndex(('spring','float','off').index(self.sim.sim.mode))
        self.dyn_mode.blockSignals(False)
        i = self.zero_axis.currentIndex()+1
        f = rows.get(i)
        m = self.mapping[i]
        self.chain.setText(f"J{i}　偏移 {m['zero_offset_rad']:+.9f} rad\n方向 {m['hardware_sign']:+g}\n"+(f"编码器 {f['position_rad']:+.6f} rad\n软件角度 {f['model_deg']:+.2f}°\nPMAX {f['regs'][21]:g}" if f else '暂无实机新鲜反馈'))
        self.frame_counter += 1
        if self.view.isVisible() and self.frame_counter%3==0:
            self.render_worker.submit(self.sim.sim.data.qpos,self.preview.values,self.sim.cam)
        if self.old_log_count!=len(self.audit.rows):
            self.old_log_count = len(self.audit.rows)
            self.refresh_logs()
            self.log_tail.setPlainText('\n'.join(time.strftime('%H:%M:%S',time.localtime(r['time']))+' '+r['level']+' '+r['message'] for r in self.audit.rows[-20:]))
            self.log_tail.verticalScrollBar().setValue(self.log_tail.verticalScrollBar().maximum())
        self.diagnostics.setText(f'UI 定时目标：约 30 Hz\n渲染耗时：{self.render_ms:.1f} ms（独立线程）\n仿真步长：1 ms\n串口线程：{"运行" if self.worker and self.worker.isRunning() else "未连接"}\n硬件输出权限：只读\nJ5 TX：由报文白名单排除\n\n日志位于工作区：\ntmp / desktop_console\n'+self.audit.path.name.replace('_','_\u200b'))
        self.statusBar().showMessage(f'模型：slaihand_four_axis.urdf　|　渲染 {self.render_ms:.1f} ms　|　J5 锁定　|　{self.mode}　|　{self.audit.path.name}')

    def on_frame(self,rgb,milliseconds):
        self.render_ms = milliseconds
        self.view.frame(rgb)

    def on_render_error(self,message):
        self.view.setText('三维渲染不可用；详情见日志。\n'+message[:300])
        self.log('ERROR','三维渲染失败：'+message)

    def keyPressEvent(self,event):
        if event.key()==Qt.Key.Key_Escape:
            self.cancel_preview()
        else:
            super().keyPressEvent(event)

    def closeEvent(self,event):
        self.timer.stop()
        try:
            if not self.disconnect_live():
                self.timer.start(33)
                event.ignore()
                return
        except Exception as exc:
            self.log('ERROR',str(exc))
            self.timer.start(33)
            event.ignore()
            return
        self.sim.stop()
        self.render_worker.requestInterruption()
        self.render_worker.wake.set()
        if not self.render_worker.wait(5000):
            self.log('ERROR','渲染线程正在退出，请稍后关闭')
            self.timer.start(33)
            event.ignore()
            return
        self.sim.close()
        self.log('SAFE','控制台退出；未发送使能、失能、写寄存器或 J5 命令')
        self.audit.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--smoke',action='store_true',help='Offline six-page capture and exit; never opens serial')
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    with Audit() as audit:
        window = Console(audit=audit)
        window.show()
        if args.smoke:
            folder = ROOT/'tmp/desktop_console/screenshots'
            folder.mkdir(parents=True,exist_ok=True)
            def capture_page(index=0):
                if index==6:
                    window.close()
                    app.quit()
                    return
                window.nav.setCurrentRow(index)
                QTimer.singleShot(500,lambda:(window.grab().save(str(folder/f'{index+1:02d}.png')),capture_page(index+1)))
            QTimer.singleShot(1500,capture_page)
        return app.exec()


if __name__=='__main__':
    raise SystemExit(main())
