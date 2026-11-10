"""SLAI desktop host: one window, one COM9 owner, explicit backend switch.

This host does NOT open a serial port, enable a motor or send any frame. It only
starts one selected backend as a child process. While that child is alive the
other modes are disabled, so two control windows cannot fight over COM9.

Modes:
  DeepSeek 五轴      slai_console.five_axis_console   J1-J4 native MIT, J5 off
  Codex J4 原生     slai_console.native_mit_console   validated J4-only baseline
  PySide6 上位机    slai_console.app                 simulation / four-axis read-only
  Host-PD 受限接管  slai_console.live_trial          diagnostic only
  J4 保护测试       slai_console.watchdog_check      diagnostic only
"""
from pathlib import Path
import os
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]

MODES = [
    dict(id='deepseek', title='DeepSeek 五轴',
         tag='实验 · J1–J4 原生 MIT · J5 关闭', color='#1FD3C1',
         module='slai_console.five_axis_console',
         desc='J1–J4 同时使能；J4=Kp160/Kd5/1.0（Codex 已测），'
              'J1–J3=Kp32/Kd2/0.2（模板）；2Nm 观测点估计护栏；'
              '保持/轨迹/速度；J5 不接、标定前不使能。',
         evidence='2026-11-02 10:48 J1–J4 保持 2s 通过；'
                  '下一步 0.5° 协调轨迹（目标=当前+0.5°）。'),
    dict(id='codex', title='Codex J4 原生 MIT',
         tag='已实机 · J4 单轴 · 已知良好基线', color='#4E9BFF',
         module='slai_console.native_mit_console',
         desc='Codex 的 J4 原生 MIT 入口；Kp160/Kd5、反馈速度中止 1.0rad/s、'
              '2Nm 观测点估计护栏；J5 排除；不适用 J1–J3。',
         evidence='2026-11-02 10:19 受支撑往返：+9.857°，回差 0.306°，'
                  '四轴新反馈 DISABLED；单次试验，不是全臂验收。'),
    dict(id='app', title='PySide6 桌面上位机',
         tag='仿真 / 四轴只读', color='#8FA5BD',
         module='slai_console.app',
         desc='原桌面上位机：MuJoCo 仿真、轨迹/动力学实验、四轴只读监视；'
              '实机运动网关仍关闭；J5 不查询。',
         evidence='原有仿真与只读交付；不是实机运动入口。'),
    dict(id='hostpd', title='主机 PD 受限接管',
         tag='诊断 · 旧控制路径', color='#C7A35A',
         module='slai_console.live_trial',
         desc='旧四轴主机 PD 诊断入口；不是驱动内部原生 MIT；'
              '仅用于原有主机 PD 路径对照。',
         evidence='既有交付路径；与新五轴原生 MIT 分开。'),
    dict(id='watchdog', title='J4 保护测试',
         tag='诊断 · 看门狗/保护', color='#C7A35A',
         module='slai_console.watchdog_check',
         desc='J4 保护链路/看门狗诊断；不用于正常运动。',
         evidence='既有保护验证入口。'),
]

STYLE = '''
QWidget { background:#0A0E13; color:#CED8E3;
          font:10pt "Microsoft YaHei UI"; }
QMainWindow { background:#0A0E13; }
QLabel { background:transparent; }
QLabel#title { font-size:18pt; font-weight:700; color:#EAF2FC; }
QLabel#sub { color:#8A9AAE; font-size:9pt; }
QLabel#tag { font-size:9pt; font-weight:600; }
QLabel#modeTitle { font-size:15pt; font-weight:700; color:#EAF2FC; }
QLabel#section { font-size:10pt; font-weight:600; color:#8FA5BD; }
QLabel#evidence { color:#59E9D9; font-size:9pt; }
QListWidget { background:#0D141D; border:1px solid #243142; outline:0; padding:6px; }
QListWidget::item { padding:14px 10px; margin:3px 0; border-radius:5px;
                    border-left:3px solid transparent; }
QListWidget::item:selected { background:#19313D; color:#59E9D9;
                             border-left:3px solid #1FD3C1; }
QPushButton { background:#182330; border:1px solid #2C3D50; border-radius:4px;
              padding:8px 14px; }
QPushButton:hover { background:#22374A; border-color:#4E9BFF; }
QPushButton:disabled { color:#667487; background:#111821; border-color:#1E2937; }
QPushButton#primary { background:#073D39; border-color:#1FD3C1; color:#59E9D9;
                      font-weight:600; }
QPushButton#primary:disabled { background:#111821; border-color:#243142; color:#667487; }
QFrame#panel { background:#111821; border:1px solid #243142; border-radius:7px; }
QTextEdit { background:#0D141D; border:1px solid #243142; font:9pt Consolas; }
'''


def _child_exe():
    """Prefer pythonw so backend windows do not get an extra console."""
    exe = Path(sys.executable)
    if exe.name.lower() == 'python.exe':
        sibling = exe.with_name('pythonw.exe')
        if sibling.exists():
            return str(sibling)
    return str(exe)


def _qt_main():
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
                                   QListWidget, QListWidgetItem, QMainWindow,
                                   QMessageBox, QPushButton, QTextEdit,
                                   QVBoxLayout, QWidget)

    class Host(QMainWindow):
        def __init__(self):
            super().__init__()
            self.child = None
            self.running_mode = None
            self.setWindowTitle('SLAI 机械臂上位机 · 模式切换')
            self.resize(1060, 720)
            self.setStyleSheet(STYLE)
            central = QWidget()
            self.setCentralWidget(central)
            outer = QVBoxLayout(central)
            outer.setContentsMargins(18, 16, 18, 14)
            outer.setSpacing(10)

            title = QLabel('SLAI 机械臂上位机')
            title.setObjectName('title')
            outer.addWidget(title)
            sub = QLabel('同一时间只允许一个模式占用 COM9。切换前先关闭当前模式窗口。'
                         '本窗口不连接串口、不使能、不发送任何帧。')
            sub.setObjectName('sub')
            sub.setWordWrap(True)
            outer.addWidget(sub)

            body = QHBoxLayout()
            body.setSpacing(14)
            self.list = QListWidget()
            self.list.setMinimumWidth(310)
            self.list.setMaximumWidth(360)
            for index, mode in enumerate(MODES):
                item = QListWidgetItem(mode['title'])
                item.setData(Qt.UserRole, index)
                self.list.addItem(item)
            self.list.currentRowChanged.connect(self._select)
            body.addWidget(self.list)

            panel = QFrame()
            panel.setObjectName('panel')
            right = QVBoxLayout(panel)
            right.setContentsMargins(18, 16, 18, 16)
            right.setSpacing(9)
            self.mode_title = QLabel()
            self.mode_title.setObjectName('modeTitle')
            self.mode_tag = QLabel()
            self.mode_tag.setObjectName('tag')
            self.mode_desc = QLabel()
            self.mode_desc.setWordWrap(True)
            self.mode_evidence = QLabel()
            self.mode_evidence.setObjectName('evidence')
            self.mode_evidence.setWordWrap(True)
            right.addWidget(self.mode_title)
            right.addWidget(self.mode_tag)
            right.addWidget(self.mode_desc)
            right.addWidget(QLabel('证据/边界'))
            right.addWidget(self.mode_evidence)
            right.addStretch(1)

            self.open_btn = QPushButton('打开选中模式')
            self.open_btn.setObjectName('primary')
            self.open_btn.clicked.connect(self._open)
            self.log_btn = QPushButton('打开日志目录')
            self.log_btn.clicked.connect(self._open_logs)
            row = QHBoxLayout()
            row.addWidget(self.open_btn)
            row.addWidget(self.log_btn)
            right.addLayout(row)

            self.status = QLabel('未启动子程序。选择模式后点“打开选中模式”。')
            self.status.setObjectName('section')
            self.status.setWordWrap(True)
            right.addWidget(self.status)
            self.detail = QTextEdit()
            self.detail.setReadOnly(True)
            self.detail.setMaximumHeight(120)
            self.detail.setPlainText('切换规则：\n'
                                     '1. 一个模式运行时，其他模式按钮禁用。\n'
                                     '2. 关闭子程序窗口后，本窗口才会重新允许选择。\n'
                                     '3. Codex J4 是已测基线；DeepSeek 五轴是实验路径；'
                                     '两者不要同时连接 COM9。')
            right.addWidget(self.detail)
            body.addWidget(panel, 1)
            outer.addLayout(body)

            goal = QLabel('最终目标：一个上位机内安全切换 Codex J4 基线与 DeepSeek 五轴；'
                          '先把 J1–J4 协调运动做实，再手动标定 J5 并入；'
                          '不追求 OpenArm 整机 / ROS2 / 750Hz 等价。')
            goal.setObjectName('sub')
            goal.setWordWrap(True)
            outer.addWidget(goal)

            self.timer = QTimer(self)
            self.timer.timeout.connect(self._poll)
            self.timer.start(500)
            self.list.setCurrentRow(0)

        def _select(self, row):
            if row < 0:
                return
            mode = MODES[row]
            color = mode['color']
            self.mode_title.setText(mode['title'])
            self.mode_tag.setText(mode['tag'])
            self.mode_tag.setStyleSheet(f'color:{color};')
            self.mode_desc.setText(mode['desc'])
            self.mode_evidence.setText(mode['evidence'])

        def _selected(self):
            row = self.list.currentRow()
            return MODES[row] if 0 <= row < len(MODES) else None

        def _open(self):
            if self.child is not None and self.child.poll() is None:
                QMessageBox.warning(self, '模式切换', '已有子程序在运行；先关闭它再切换。')
                return
            mode = self._selected()
            if mode is None:
                return
            try:
                flags = 0x08000000 if os.name == 'nt' else 0  # CREATE_NO_WINDOW
                self.child = subprocess.Popen(
                    [_child_exe(), '-m', mode['module']],
                    cwd=str(ROOT), creationflags=flags)
                self.running_mode = mode
                self.open_btn.setEnabled(False)
                self.list.setEnabled(False)
                self.status.setText(f'已启动：{mode["title"]}（pid {self.child.pid}）。'
                                     '关闭该子程序窗口后，本窗口自动恢复切换。')
                self.detail.append(f'\n[{time.strftime("%H:%M:%S")}] 启动 {mode["module"]} '
                                   f'pid={self.child.pid}')
            except Exception as exc:
                self.child = None
                self.running_mode = None
                QMessageBox.critical(self, '启动失败', str(exc))
                self.detail.append(f'\n启动失败：{exc}')

        def _poll(self):
            if self.child is None:
                return
            code = self.child.poll()
            if code is None:
                return
            mode = self.running_mode
            self.child = None
            self.running_mode = None
            self.open_btn.setEnabled(True)
            self.list.setEnabled(True)
            self.status.setText(f'子程序已退出（code={code}）。可以切换模式。')
            self.detail.append(f'\n[{time.strftime("%H:%M:%S")}] '
                               f'{mode["title"] if mode else "子程序"} 退出 code={code}')

        def _open_logs(self):
            folder = ROOT/'tmp'
            folder.mkdir(parents=True, exist_ok=True)
            os.startfile(str(folder))

        def closeEvent(self, event):
            if self.child is not None and self.child.poll() is None:
                answer = QMessageBox.question(
                    self, '退出上位机',
                    '子程序仍在运行。退出本窗口不会关闭它；请先在子程序里停止/关闭。\n'
                    '仍要退出本上位机壳吗？',
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if answer != QMessageBox.Yes:
                    event.ignore()
                    return
            event.accept()

    app = QApplication.instance() or QApplication(sys.argv)
    host = Host()
    host.show()
    return app.exec()


def main():
    try:
        return _qt_main()
    except Exception:
        folder = ROOT/'tmp/desktop_console'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/'startup_error.txt'
        path.write_text(traceback.format_exc(), encoding='utf-8')
        try:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
            root.withdraw()
            messagebox.showerror('SLAI 上位机启动失败',
                                 '错误详情已保存：\n'+str(path)+'\n\n'
                                 +traceback.format_exc()[-1800:])
            root.destroy()
        except Exception:
            pass
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
