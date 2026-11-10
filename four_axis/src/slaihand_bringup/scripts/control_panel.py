#!/usr/bin/env python3
"""四轴 ROS 统一控制台：本地中文网页，仅监听 127.0.0.1。

观察只走既有话题/服务；运动只通过受保护的 commission_supported_trial.py
子进程执行预览已冻结的绝对计划。本程序不打开串口/CAN，不改增益/限位/门禁，
不做回零、自动重试或自动重新使能。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import queue
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from std_msgs.msg import Float64MultiArray, String
from control_panel_http import PanelServer
from control_panel_connection import DriverConnection
from control_panel_gravity import DEFAULT_GRAVITY_BASE, KNOWN_J3_PLACEMENT_OFFSET, GravityShadow, gravity_profile_filename
from control_panel_model import (CLIENT_RESERVE_S, EXEC_HOLD_S, FEEDBACK_FRESH_S, HEARTBEAT_TIMEOUT_S,
                                 HOLD_SECONDS, JOINT_NAMES, MAX_WAYPOINTS, POSE_DRIFT_DEG, PREVIEW_TTL_S,
                                 REFERENCE_SPEED_RAD_S, SESSION_BUDGET_S, STOP_CONFIRM_TIMEOUT_S, PanelError,
                                 TicketStore, clean_for_json, degrees_limits, digest, load_limits,
                                 normalize_document, number, preview_payload, project_result, stage_label,
                                 validate_confirmations)
from control_panel_model import execution_mode as validate_execution_mode
import supported_capture as capture_flow
from drag_teaching import (DEFAULT_SPEED_FACTOR, MAX_SPEED_FACTOR, MIN_SPEED_FACTOR,
                           PLAYBACK_LIMIT_S, SPEED_LIMIT_RAD_S, TeachingError, TeachingRecorder,
                           TeachingStore, compile_playback, identity_compatibility,
                           identity_fingerprint, json_sha256, normalize_identity,
                           teaching_summary, validate_teaching, normalize_float_settings,
                           content_sha256, FLOAT_MAX_SECONDS)

PLAN_ID = re.compile(r"^[0-9a-f]{16}$")
JOINT_STATE_TOPIC = "/slaihand/joint_states"
DYNAMIC_STATE_TOPIC = "/slaihand/dynamic_joint_states"
CONTROLLER_STATE_TOPIC = "/slaihand/slaihand_joint_trajectory_controller/controller_state"
SERVICE_PREFIX = "/slaihand"
CONTROLLER_NAME = "slaihand_joint_trajectory_controller"


class Operation:
    def __init__(self, kind, operation_id, directory):
        self.kind = kind
        self.id = operation_id
        self.directory = Path(directory)
        self.created_at = time.time()
        self.started_at = None
        self.finished_at = None
        self.proc = None
        self.pid = None
        self.argv = []
        self.exit_code = None
        self.phase = "starting"
        self.stop_reason = None
        self.cancel_requested = False
        self.lease_expired = False
        self.lines = deque(maxlen=400)
        self.result = None
        self.result_error = None
        self.projection = None
        self.source_sha256 = None
        self.plan_sha256 = None
        self.ticket_id = None
        self.frozen = None
        self.final_target_deg = None
        self.finished = False
        self.recording_id = None
        self.float_settings = None
        self.capture_ticket = None
        self.capture_preview = None
        self.capture_state = None
        self.capture_command_id = None
        self.capture_identity_sha256 = None
        self.capture_boot_id = None
        self.capture_stop_generation = None
        self.capture_ready = False
        self.capture_ready_at = None
        self.capture_ack_sent_at = None
        self.capture_ack_nonce = None
        self.capture_ack_path = None
        self.capture_alpha = None


class PanelBackend:
    def __init__(self, node, archive_dir, preview_ttl, heartbeat_timeout, token,
                 gravity_base=None, gravity_j3_offset=None, gravity_enabled=True):
        self.node = node
        self.archive = Path(archive_dir)
        self.archive.mkdir(parents=True, exist_ok=True)
        (self.archive / "plans").mkdir(exist_ok=True)
        (self.archive / "operations").mkdir(exist_ok=True)
        self._heartbeat_timeout = float(heartbeat_timeout)
        self.token = token
        self.tickets = TicketStore(ttl=preview_ttl)

        self.static_dir = self._resolve_static_dir()
        self.limits, self.limits_source, self.limits_error = self._resolve_limits()
        self.client_script, self.client_sha256, self.client_error = self._resolve_client()
        self.gravity_base = list(gravity_base) if gravity_base else list(DEFAULT_GRAVITY_BASE)
        self.gravity_j3_offset = float(gravity_j3_offset) if gravity_j3_offset is not None \
            else KNOWN_J3_PLACEMENT_OFFSET
        self.gravity_shadow = None
        self.gravity_error = None
        if gravity_enabled:
            self.gravity_shadow, self.gravity_error = self._resolve_gravity()

        self._lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._joints = None
        self._dynamic = None
        self._controller = None

        self._operation = None
        self._last_operation = None
        self._service = {"stop_all": {}, "reset_disabled": {}, "switch_controller": {}}
        self._pending = {}
        self._events = deque(maxlen=200)
        self._lease_last = time.monotonic()
        self._closing = threading.Event()
        self._ros_stop = threading.Event()
        self._ros_thread = None
        self._spin_epoch = 0.0

        self._stop_generation = 0
        self._stop_state = {"state": "not_requested", "requested_at": None, "requested_wall": None,
                            "reason": None, "generation": 0, "confirmed_at": None, "confirmed_wall": None}
        # Stop epoch gate: once a stop is accepted, preview/execute/hold stay
        # locked until a manual reset was requested AND a causally fresh,
        # four-axis-consistent DISARMED observation arrived afterwards.
        self._stop_locked = False
        self._stop_locked_wall = None
        self._stop_locked_reason = None
        self._stop_locked_generation = 0
        self._reset_after_stop_at = None
        self._reset_after_stop_wall = None
        self._stop_gate_unlocked_wall = None
        self._connection_manager = DriverConnection(self.archive / 'connection')
        self._connection_cancel = threading.Event()
        self._connection_state = {'phase': 'idle', 'message': '可检查连接或尝试重连'}
        self._connection_recovered = False
        self._stop_queue = queue.SimpleQueue()
        self._reset_queue = queue.SimpleQueue()
        self._deactivate_queue = queue.SimpleQueue()
        # 拖动示教：独占文件存储、当前记录器、最近一次录制与驱动运行身份。
        self.teachings = TeachingStore(self.archive / "teachings")
        self._teaching_recorder = None
        self._teaching_last = None
        self._teaching_finalizing = False
        self._teaching_last_error = None
        self._dynamic_sequence = 0
        self._identity = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(JointState, JOINT_STATE_TOPIC, self._joint_callback, qos)
        node.create_subscription(DynamicJointState, DYNAMIC_STATE_TOPIC, self._dynamic_callback, qos)
        node.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE_TOPIC,
                                 self._controller_callback, qos)
        node.create_subscription(Float64MultiArray, SERVICE_PREFIX + "/gravity_ff_state",
                                 self._ff_callback, qos)
        identity_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, SERVICE_PREFIX + "/runtime_identity",
                                 self._identity_callback, identity_qos)
        self._stop_client = node.create_client(Trigger, SERVICE_PREFIX + "/stop_all")
        self._reset_client = node.create_client(Trigger, SERVICE_PREFIX + "/reset_disabled")
        self._switch_client = node.create_client(SwitchController,
                                                 SERVICE_PREFIX + "/controller_manager/switch_controller")
        self.record_event("面板启动；只读观察，运动仅经受保护客户端子进程")
        if self.limits_error:
            self.record_event("限位不可用：%s" % self.limits_error, "error")
        if self.client_error:
            self.record_event("客户端不可用：%s" % self.client_error, "error")
        if self.gravity_shadow is not None:
            self.record_event("重力 shadow 已加载（只显示、不改命令）：质量档案 %s，模型 %s"
                              % (self.gravity_shadow.model.mass_status,
                                 self.gravity_shadow.model.metadata.get("urdf_sha256", "")[:12]))
        elif gravity_enabled:
            self.record_event("重力 shadow 不可用：%s" % self.gravity_error, "warn")

    # ------------------------------------------------------------------ setup
    @staticmethod
    def _resolve_static_dir():
        try:
            candidate = Path(get_package_share_directory("slaihand_bringup")) / "web" / "control_panel"
            if (candidate / "index.html").is_file():
                return candidate
        except Exception:
            pass
        candidate = Path(__file__).resolve().parent.parent / "web" / "control_panel"
        if (candidate / "index.html").is_file():
            return candidate
        raise SystemExit("找不到 web/control_panel 静态资源")

    @staticmethod
    def _resolve_limits():
        try:
            path = Path(get_package_share_directory("slaihand_description")) / "urdf" / "slaihand_four_axis.urdf"
        except Exception as error:
            return None, None, "无法定位 slaihand_description：%s" % error
        if not path.is_file():
            return None, str(path), "URDF 不存在：%s" % path
        try:
            return load_limits(path), str(path), None
        except Exception as error:
            return None, str(path), "URDF 限位读取失败：%s" % error

    @staticmethod
    def _resolve_client():
        candidates = [Path(__file__).resolve().parent / "commission_supported_trial.py"]
        try:
            candidates.append(Path(get_package_prefix("slaihand_bringup")) / "lib" / "slaihand_bringup"
                              / "commission_supported_trial.py")
        except Exception:
            pass
        for path in candidates:
            try:
                if path.is_file() and "--plan" in path.read_text(encoding="utf-8", errors="replace"):
                    return path, hashlib.sha256(path.read_bytes()).hexdigest(), None
            except OSError:
                continue
        return None, None, "找不到受保护客户端 commission_supported_trial.py（或版本不支持 --plan）"

    def _resolve_gravity(self):
        """定位重力模型所需文件；失败只禁用 shadow 显示，不阻塞面板。"""
        try:
            try:
                bringup = Path(get_package_share_directory("slaihand_bringup"))
            except Exception:
                bringup = Path(__file__).resolve().parent.parent
            try:
                description = Path(get_package_share_directory("slaihand_description"))
            except Exception:
                description = Path(__file__).resolve().parent.parent.parent / "slaihand_description"

            def pick(*candidates):
                for candidate in candidates:
                    if candidate.is_file():
                        return candidate
                return None

            urdf = pick(description / "urdf" / "slaihand_four_axis.urdf",
                        Path(__file__).resolve().parent.parent.parent / "slaihand_description" / "urdf"
                        / "slaihand_four_axis.urdf")
            mass = pick(bringup / "config" / "gravity_provisional.json",
                        Path(__file__).resolve().parent.parent / "config" / "gravity_provisional.json")
            mapping = pick(bringup / "config" / "hardware_mapping.yaml",
                           Path(__file__).resolve().parent.parent / "config" / "hardware_mapping.yaml")
            policy = pick(bringup / "config" / "canusb_motion_policy.yaml",
                          Path(__file__).resolve().parent.parent / "config" / "canusb_motion_policy.yaml")
            if policy is not None:
                name = gravity_profile_filename(policy)
                mass = pick(bringup / 'config' / name,
                            Path(__file__).resolve().parent.parent / 'config' / name)
            missing = [label for label, path in (("URDF", urdf), ("质量档案", mass),
                                                 ("坐标映射", mapping), ("policy", policy)) if path is None]
            if missing:
                raise ValueError("缺少 %s" % "、".join(missing))
            fixed = {"slaihand_joint3": self.gravity_j3_offset} if abs(self.gravity_j3_offset) > 0 else None
            shadow = GravityShadow(urdf, mass, mapping, policy, gravity_base=self.gravity_base,
                                   fixed_joint_offsets=fixed)
            return shadow, None
        except Exception as error:
            return None, "%s: %s" % (type(error).__name__, error)


    # ------------------------------------------------------------- ROS inputs
    def _store(self, key, value):
        with self._state_lock:
            setattr(self, "_" + key, value)

    def _joint_callback(self, msg):
        row = dict(zip(msg.name, msg.position))
        positions = None
        if all(name in row and math.isfinite(row[name]) for name in JOINT_NAMES):
            positions = [float(row[name]) for name in JOINT_NAMES]
        self._store("joints", {"at": time.monotonic(), "positions": positions})

    def _dynamic_callback(self, msg):
        axes = {}
        for name, values in zip(msg.joint_names, msg.interface_values):
            if name in JOINT_NAMES:
                axes[name] = {iface: float(value) for iface, value in zip(values.interface_names, values.values)}
        self._store("dynamic", {"at": time.monotonic(), "axes": axes})
        # Keep the publisher stamp: it is the identity of a source frame and the
        # only reliable key for dropping duplicate deliveries.
        source_stamp = None
        try:
            stamp = msg.header.stamp
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            source_stamp = value if value > 0 else None
        except (AttributeError, TypeError, ValueError):
            source_stamp = None
        self._teaching_frame(axes, source_stamp)

    def _identity_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict) or payload.get("schema") != "slaihand_runtime_identity/1":
            return
        self._store("identity", {"at": time.monotonic(), "payload": payload})

    def _identity_view(self):
        with self._state_lock:
            identity = getattr(self, "_identity", None)
        if identity is None:
            return {"available": False, "fresh": False, "reason": "未收到驱动运行身份"}
        age = max(0.0, time.monotonic() - identity["at"])
        view = {"available": True, "fresh": age <= 5.0, "age_s": round(age, 3),
                "payload": identity["payload"]}
        try:
            normalize_identity(identity["payload"])
        except TeachingError as error:
            view["fresh"] = False
            view["reason"] = error.message
        return view

    def _teaching_frame(self, axes, source_stamp=None):
        """把一帧四轴状态原样交给记录器；任何异常只记事件，不打断 ROS 回调。"""
        recorder = self._teaching_recorder
        if recorder is None or recorder.window_open is False:
            return
        try:
            self._dynamic_sequence += 1
            frame = None
            if all(name in axes for name in JOINT_NAMES):
                rows = [axes[name] for name in JOINT_NAMES]
                def row_value(key):
                    values = [row.get(key) for row in rows]
                    return None if any(value is None for value in values) else values
                def integer_row(key):
                    values = [number(row.get(key)) for row in rows]
                    return [int(value) if value is not None and value.is_integer() else None
                            for value in values]
                positions = row_value("position")
                velocities = row_value("velocity")
                ages = row_value("feedback_age")
                valid = row_value("feedback_valid")
                motor = integer_row("motor_state")
                stages = integer_row("control_state")
                command = integer_row("command_id")
                hold_ready = row_value("hold_ready")
                if None not in (positions, velocities, ages, valid, motor, stages, command):
                    frame = {"q": positions, "dq": velocities, "age": ages,
                             "source_stamp": source_stamp,
                             "valid": [value == 1 for value in valid],
                             "motor": motor,
                             "stage": stages[0] if len(set(stages)) == 1 else None,
                             "command_id": command[0] if len(set(command)) == 1 else None,
                             "hold_ready": bool(hold_ready and all(value == 1 for value in hold_ready)),
                             "float": bool(self._ff_view().get("gravity_float"))}
            identity = self._identity_view()
            if not identity.get("fresh"):
                return
            recorder.record(self._dynamic_sequence,
                            time.monotonic(), frame or {}, identity["payload"])
        except Exception as error:  # never let teaching capture break the state tap
            self._teaching_last_error = "%s: %s" % (type(error).__name__, error)

    def _controller_callback(self, msg):
        self._store("controller", {"at": time.monotonic(), "reference": [float(v) for v in msg.reference.positions],
                                  'scaling':float(msg.speed_scaling_factor)})

    def _ff_callback(self, msg):
        values = [float(value) for value in msg.data]
        if len(values) not in (21, 22, 24):
            return
        self._store("ff", {"at": time.monotonic(), "alpha": values[0], "ramp": values[1],
                           "active": values[2] >= 0.5, "stage": int(values[3]),
                           "command_id": int(values[4]), "requested": values[5:9],
                           "encoded": values[9:13], "applied": values[13:17],
                           "last_written": values[17:21],
                           "float_available": len(values) >= 22,
                           "gravity_float": len(values) >= 22 and values[21] >= 0.5,
                           "float_kd": values[22] if len(values) >= 24 else 5.0,
                           "float_seconds": values[23] if len(values) >= 24 else 5.0,
                           "float_configurable": len(values) >= 24})

    def _read_state(self):
        with self._state_lock:
            return self._joints, self._dynamic, self._controller

    # ------------------------------------------------------------- core state
    def _core(self):
        now = time.monotonic()
        joints, dynamic, controller = self._read_state()
        dynamic_age = None if dynamic is None else max(0.0, now - dynamic["at"])
        joint_age = None if joints is None else max(0.0, now - joints["at"])
        controller_age = None if controller is None else max(0.0, now - controller["at"])
        axes = dict(dynamic["axes"]) if dynamic else {}

        def value(name, key):
            return number(axes.get(name, {}).get(key))

        stages = [value(name, "control_state") for name in JOINT_NAMES]
        stage = int(stages[0]) if stages[0] is not None else None
        consistent = all(item is not None for item in stages) and len({int(item) for item in stages}) == 1
        feedback_valid = [bool(value(name, "feedback_valid") == 1) and dynamic_age is not None
                          and dynamic_age <= FEEDBACK_FRESH_S for name in JOINT_NAMES]
        positions = None
        if joints is not None and joint_age is not None and joint_age <= FEEDBACK_FRESH_S:
            positions = joints["positions"]
        if positions is None and all(feedback_valid):
            positions = [value(name, "position") for name in JOINT_NAMES]
            if any(item is None for item in positions):
                positions = None
        reference = None
        if controller is not None and controller_age is not None and controller_age <= 1.0:
            if len(controller["reference"]) == 4:
                reference = controller["reference"]
        return {"now": now, "axes": axes, "dynamic_age": dynamic_age, "joint_age": joint_age,
                "controller_age": controller_age, "stage": stage, "stage_fresh": dynamic_age is not None
                and dynamic_age <= FEEDBACK_FRESH_S, "consistent": consistent,
                "hold_ready": all(value(name, "hold_ready") == 1 for name in JOINT_NAMES),
                "disabled_observed": all(value(name, "disabled_observed") == 1 for name in JOINT_NAMES),
                "owned": all(value(name, "command_owner") == 1 for name in JOINT_NAMES),
                "audit_ok": all(value(name, "audit_ok") == 1 for name in JOINT_NAMES),
                "command_id": value(JOINT_NAMES[0], "command_id"),
                "feedback_valid": feedback_valid, "positions": positions, "reference": reference,
                'speed_scaling_factor':controller.get('scaling') if controller and controller_age<=1. else None,
                "dynamic_at": None if dynamic is None else dynamic["at"]}

    # ---------------------------------------------------------------- helpers
    def record_event(self, message, level="info"):
        with self._lock:
            self._events.append({"at": time.time(), "level": level, "message": str(message)[:400]})

    def _event(self, message, level="info"):
        self.record_event(message, level)

    def _operation_running(self):
        with self._lock:
            operation = self._operation
            return operation is not None and not operation.finished

    def _require_open(self):
        if self._closing.is_set():
            raise PanelError(503, "CLOSING", "面板正在退出，不接受新请求")

    def _require_lease(self):
        if time.monotonic() - self._lease_last > self._heartbeat_timeout:
            raise PanelError(403, "LEASE", "操作页心跳过期（%.1f 秒）；请刷新页面后再执行" % self._heartbeat_timeout)

    def _require_disarmed(self, core):
        if self._closing.is_set():
            raise PanelError(503, "CLOSING", "面板正在退出")
        if self.limits is None:
            raise PanelError(503, "LIMITS", "URDF 限位不可用：%s" % self.limits_error)
        if core["dynamic_age"] is None:
            raise PanelError(409, "FEEDBACK_STALE", "没有收到四轴动态状态（失联），拒绝预览或执行")
        if core["dynamic_age"] > FEEDBACK_FRESH_S:
            raise PanelError(409, "FEEDBACK_STALE", "四轴反馈陈旧（%.2f 秒），拒绝预览或执行" % core["dynamic_age"])
        if not all(core["feedback_valid"]):
            raise PanelError(409, "FEEDBACK_INVALID", "四轴反馈无效或未完整，拒绝预览或执行")
        if core["positions"] is None:
            raise PanelError(409, "FEEDBACK_INVALID", "没有完整四轴位置，拒绝预览或执行")
        if core["stage"] == 9:
            raise PanelError(409, "UNKNOWN", "控制端 UNKNOWN：保持机械支撑并现场切电，不要重试使能")
        if core["stage"] != 0 or not core["stage_fresh"]:
            raise PanelError(409, "NOT_DISARMED", "当前控制阶段为%s；需要先观测到未使能（手动复位）"
                             % stage_label(core["stage"]))

    def _capability_reason(self, core, busy, live_preview):
        if self._closing.is_set():
            return "面板正在退出"
        with self._lock:
            if self._stop_locked:
                return "停止已受理：等待手动复位与新的四轴一致未使能观测"
        if busy:
            return "有操作正在进行"
        if self.limits is None:
            return "URDF 限位不可用"
        if self.client_script is None:
            return "受保护客户端不可用"
        if core["dynamic_age"] is None:
            return "未收到动态关节状态（失联）"
        if core["dynamic_age"] > FEEDBACK_FRESH_S:
            return "反馈陈旧（%.1f 秒）" % core["dynamic_age"]
        if not all(core["feedback_valid"]):
            return "四轴反馈无效"
        if core["stage"] is None:
            return "控制阶段未知"
        if core["stage"] == 9:
            return "UNKNOWN：保持支撑并现场切电，不要重试使能"
        if core["stage"] != 0 or not core["stage_fresh"]:
            return "需要手动复位到未使能（当前：%s）" % stage_label(core["stage"])
        if live_preview is not None and not live_preview["valid"]:
            return "预览已失效：%s" % "；".join(live_preview["reasons"])
        return None

    def _start_positions(self):
        return self._core()["positions"]

    def _require_stop_gate(self):
        """Refuse new motion/preview while a stop epoch is still locked."""
        with self._lock:
            locked = self._stop_locked
            reason = self._stop_locked_reason
        if locked:
            raise PanelError(409, "STOP_LOCKED",
                             "停止已受理（%s）：在手动复位并观测到新的四轴一致未使能之前，"
                             "预览/执行/保持保持锁定" % (reason or "全轴停止"))

    # --------------------------------------------------------------- preview
    def preview(self, document, mode='trajectory'):
        self._require_open()
        mode=validate_execution_mode(mode)
        self._require_stop_gate()
        if self._operation_running():
            raise PanelError(409, "BUSY", "有操作正在进行，不能预览")
        core = self._core()
        self._require_disarmed(core)
        try:
            payload = preview_payload(document, core["positions"], self.limits, self.limits_source,mode)
        except PanelError:
            raise
        except ValueError as error:
            raise PanelError(400, "PLAN_INVALID", str(error))
        ticket = self.tickets.issue({"mode": "absolute_deg", "execution_mode":mode, "frozen": payload["frozen"],
                                     "frozen_sha256": payload["frozen_sha256"],
                                     "source_sha256": payload["source_sha256"], "preview": payload,
                                     "start_rad": list(core["positions"]), "start_deg": payload["start_deg"]})
        self.record_event("预览已生成：%d 点、%.1f 秒，票根 %s…（未使能）"
                          % (len(payload["waypoints"]), payload["total_s"], ticket["id"][:8]))
        return {"ok": True, "accepted": True, "completed": True,
                "message": "服务器校验通过；这是预览，没有请求使能或轨迹。",
                "ticket": ticket, "preview": payload}

    def _preview_validity(self, ticket, core, busy):
        reasons = []
        if ticket["expired"]:
            reasons.append("预览已过期")
        if ticket["used"]:
            reasons.append(ticket["invalid_reason"] or "预览票根已使用")
        if busy:
            reasons.append("有操作正在进行")
        if core["dynamic_age"] is None or core["dynamic_age"] > FEEDBACK_FRESH_S:
            reasons.append("反馈陈旧或失联")
        elif not all(core["feedback_valid"]):
            reasons.append("四轴反馈无效")
        if core["stage"] != 0:
            reasons.append("控制阶段不是未使能（%s）" % stage_label(core["stage"]))
        drift = None
        if core["positions"] is None:
            reasons.append("没有完整四轴位置")
        else:
            drift = max(abs(a - b) for a, b in zip(core["positions"], ticket["start_rad"]))
            if drift > math.radians(POSE_DRIFT_DEG):
                reasons.append("预览后姿态变化 %.2f°（上限 %.1f°）" % (math.degrees(drift), POSE_DRIFT_DEG))
        return {"valid": not reasons, "reasons": reasons,
                "drift_deg": None if drift is None else round(math.degrees(drift), 3)}

    # --------------------------------------------------------------- execute
    def execute(self, ticket_id, document, confirmations, mode='trajectory'):
        mode=validate_execution_mode(mode)
        confirmations = validate_confirmations(confirmations)
        if mode=='calibration' and not confirmations.get('free_static'):
            raise PanelError(403,'FREE_STATIC_REQUIRED','重力采样需确认自由承重、无接触；防坠支撑不能承担重量')
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发执行")
            baseline = self._stop_generation
        core = self._core()
        self._require_disarmed(core)
        ticket = self.tickets.peek(ticket_id)
        if ticket is None:
            raise PanelError(409, "TICKET_UNKNOWN", "预览票根不存在，请重新预览")
        if ticket.get('execution_mode','trajectory')!=mode:
            raise PanelError(409,'MODE_CHANGED','执行模式已改变，请重新预览')
        if ticket["used"]:
            raise PanelError(409, "TICKET_USED", ticket["invalid_reason"] or "预览票根已使用，请重新预览")
        if ticket["expired"]:
            raise PanelError(409, "TICKET_EXPIRED", "预览已过期，请重新预览")
        try:
            submitted = normalize_document(document)
        except ValueError as error:
            raise PanelError(400, "PLAN_INVALID", str(error))
        if digest(submitted) != ticket["source_sha256"]:
            raise PanelError(409, "PLAN_CHANGED", "计划在预览后已被修改，请重新预览")
        drift = max(abs(a - b) for a, b in zip(core["positions"], ticket["start_rad"]))
        if drift > math.radians(POSE_DRIFT_DEG):
            raise PanelError(409, "POSE_DRIFT", "预览后姿态变化 %.2f°，请重新预览" % math.degrees(drift))
        ticket = self.tickets.consume(ticket_id)
        self.record_event("执行受理：票根 %s… 源计划 %s…" % (ticket_id[:8], ticket["source_sha256"][:8]))
        return self._launch("execute", ticket, confirmations, baseline)

    def _float_settings(self, settings):
        try:
            return normalize_float_settings(settings)
        except TeachingError as error:
            raise PanelError(400, error.code, error.message)

    def hold(self, confirmations, gravity_float=False, float_settings=None):
        settings = self._float_settings(float_settings) if gravity_float else None
        if gravity_float:
            driver = self._ff_view()
            if not driver.get('connected') or not driver.get('float_available'):
                raise PanelError(409, 'FLOAT_UNAVAILABLE', '当前驱动未开放自由拖动模式')
            if not isinstance(confirmations, dict) or confirmations.get('gravity_float') is not True:
                raise PanelError(400, 'FLOAT_ACK', '请确认托住机械臂等待渐入，并在结束失能时承接重量')
        confirmations = validate_confirmations(confirmations)
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发保持")
            baseline = self._stop_generation
        core = self._core()
        self._require_disarmed(core)
        self.record_event("有限保持受理：%g 秒" % (settings["seconds"] if settings else HOLD_SECONDS))
        return self._launch("gravity_float" if gravity_float else "hold", None, confirmations, baseline,
                            start_positions=core["positions"], float_settings=settings)

    def _launch(self, kind, ticket, confirmations, baseline, start_positions=None, recording_id=None,
                float_settings=None):
        operation_id = uuid.uuid4().hex[:16]
        directory = self.archive / "operations" / operation_id
        directory.mkdir(parents=True, exist_ok=False)
        client_dir = directory / "client"
        argv = [sys.executable, str(self.client_script), "--execute", "--ack-supported", "--ack-clearance",
                "--ack-native-output"]
        operation = Operation(kind, operation_id, directory)
        operation.recording_id = recording_id
        operation.float_settings = float_settings
        if kind == "execute":
            plan_path = directory / "plan.json"
            plan_path.write_text(json.dumps(ticket["frozen"], ensure_ascii=False, indent=2), encoding="utf-8")
            (directory / "source.json").write_text(json.dumps(ticket["preview"]["source"], ensure_ascii=False,
                                                             indent=2), encoding="utf-8")
            (directory / "ticket.json").write_text(json.dumps(
                {"ticket": ticket["id"], "source_sha256": ticket["source_sha256"],
                 "frozen_sha256": ticket["frozen_sha256"], "start_rad": ticket["start_rad"],
                 "confirmations": confirmations}, ensure_ascii=False, indent=2), encoding="utf-8")
            argv += ["--plan", str(plan_path), "--hold-seconds", str(EXEC_HOLD_S)]
            if ticket.get('execution_mode')=='point_to_point':argv+=['--point-to-point']
            if ticket.get('execution_mode')=='calibration':argv+=['--calibrate','--ack-free-static']
            operation.source_sha256 = ticket["source_sha256"]
            operation.plan_sha256 = ticket["frozen_sha256"]
            operation.ticket_id = ticket["id"]
            operation.frozen = ticket["frozen"]
            operation.final_target_deg = list(ticket["frozen"]["waypoints"][-1]["degrees"])
        elif kind == "supported_capture":
            preview = dict(ticket["preview"])
            (directory / "capture_preview.json").write_text(
                json.dumps(preview, ensure_ascii=False, indent=2), encoding="utf-8")
            ack_path = directory / capture_flow.ACK_FILENAME
            (directory / "capture_ticket.json").write_text(json.dumps(
                {"ticket": ticket["id"], "source_sha256": ticket["source_sha256"],
                 "frozen_sha256": ticket["frozen_sha256"], "start_rad": ticket["start_rad"],
                 "identity_sha256": ticket["identity_sha256"], "boot_id": ticket["boot_id"],
                 "stop_generation": ticket["stop_generation"],
                 "confirmations": confirmations}, ensure_ascii=False, indent=2), encoding="utf-8")
            argv += ["--supported-static-capture", "--ack-file", str(ack_path),
                     "--ack-timeout", str(capture_flow.ACK_TIMEOUT_S),
                     "--hold-ready-timeout", str(capture_flow.HOLD_READY_TIMEOUT_S),
                     "--capture-seconds", str(capture_flow.CAPTURE_SECONDS),
                     "--capture-samples", str(capture_flow.CAPTURE_SAMPLES),
                     "--capture-output", str(client_dir / "calibration"),
                     "--panel-stop-epoch", str(int(ticket["stop_generation"])),
                     "--operation-id", operation_id,
                     "--support", "free", "--contact", "none"]
            operation.source_sha256 = ticket["source_sha256"]
            operation.plan_sha256 = ticket["frozen_sha256"]
            operation.ticket_id = ticket["id"]
            operation.frozen = ticket["frozen"]
            operation.capture_ticket = ticket
            operation.capture_preview = preview
            operation.capture_identity_sha256 = ticket["identity_sha256"]
            operation.capture_boot_id = ticket["boot_id"]
            operation.capture_stop_generation = ticket["stop_generation"]
            operation.capture_ack_path = ack_path
            operation.final_target_deg = list(ticket["start_deg"])
        elif kind == "teaching_playback":
            playback_path = directory / "playback.json"
            playback_path.write_text(json.dumps(ticket["frozen"], ensure_ascii=False, indent=2),
                                     encoding="utf-8")
            (directory / "teaching.json").write_text(json.dumps(
                {"teaching_id": ticket.get("teaching_id"),
                 "speed_factor": ticket.get("speed_factor"),
                 "source_sha256": ticket["source_sha256"],
                 "frozen_sha256": ticket["frozen_sha256"],
                 "start_rad": ticket["start_rad"],
                 "confirmations": confirmations}, ensure_ascii=False, indent=2), encoding="utf-8")
            argv += ["--teaching-plan", str(playback_path), "--hold-seconds", str(EXEC_HOLD_S)]
            operation.source_sha256 = ticket["source_sha256"]
            operation.plan_sha256 = ticket["frozen_sha256"]
            operation.ticket_id = ticket["id"]
            operation.frozen = ticket["frozen"]
            last = ticket["frozen"]["samples"][-1]
            operation.final_target_deg = [round(math.degrees(value), 3) for value in last[1:]]
        else:
            argv += ["--hold-seconds", str(float_settings["seconds"] if float_settings else HOLD_SECONDS)]
            if kind in ('gravity_float', 'drag_teaching'):
                argv += ['--gravity-float', '--ack-gravity-float']
                argv += ['--float-kd', str(float_settings["kd"] if float_settings else 5.0)]
            operation.final_target_deg = [round(math.degrees(q), 3) for q in start_positions]
        argv += ["--output-directory", str(client_dir)]
        operation.argv = argv
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发启动")
            if self._closing.is_set():
                raise PanelError(503, "CLOSING", "面板正在退出")
            if self._stop_locked:
                raise PanelError(409, "STOP_LOCKED",
                                 "停止已受理：在手动复位并观测到新的四轴一致未使能之前，不能启动客户端")
            if self._stop_generation != baseline:
                raise PanelError(409, "STOP_RACE", "停止请求与执行并发，已拒绝本次执行")
            self._operation = operation
            generation = self._stop_generation
        try:
            proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
        except Exception as error:
            with self._lock:
                self._operation = None
            operation.phase = "failed"
            operation.finished = True
            operation.result_error = "无法启动受保护客户端：%s" % error
            self.record_event(operation.result_error, "error")
            raise PanelError(500, "SPAWN", "无法启动受保护客户端：%s" % error)
        with self._lock:
            operation.proc = proc
            operation.pid = proc.pid
            operation.started_at = time.time()
            operation.phase = "running"
            cancel = operation.cancel_requested or self._stop_generation != generation or self._closing.is_set()
        threading.Thread(target=self._read_output, args=(operation,), daemon=True).start()
        if cancel:
            operation.cancel_requested = True
            self._terminate(proc)
        self.record_event("已启动%s客户端 pid=%d（固定 argv，无 shell）"
                          % ({"execute": "执行", "teaching_playback": "拖动示教回放",
                              "drag_teaching": "拖动示教记录"}.get(kind, "有限保持"), proc.pid))
        return {"ok": True, "accepted": True, "completed": False, "phase": "accepted",
                "message": "请求已受理；执行结果以操作状态和原始错误为准，不是已完成的运动。",
                "operation": self._operation_display()}

    def _read_output(self, operation):
        try:
            for line in operation.proc.stdout:
                line = line.rstrip("\r\n")
                if line:
                    with self._lock:
                        operation.lines.append(line[:500])
        except Exception:
            pass

    def _terminate(self, proc):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError):
            return
        def escalate():
            try:
                proc.wait(timeout=2.5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
        threading.Thread(target=escalate, daemon=True).start()

    # ------------------------------------------------------------------ stop
    def _request_stop(self, reason, operator=True):
        self._connection_cancel.set()
        now = time.monotonic()
        with self._lock:
            self._stop_generation += 1
            self._stop_state = {"state": "requested", "requested_at": now, "requested_wall": time.time(),
                                "reason": reason, "generation": self._stop_generation,
                                "confirmed_at": None, "confirmed_wall": None}
            self._stop_locked = True
            self._stop_locked_wall = time.time()
            self._stop_locked_reason = reason
            self._stop_locked_generation = self._stop_generation
            self._reset_after_stop_at = None
            operation = self._operation
            if operation is not None and not operation.finished:
                operation.cancel_requested = True
                operation.stop_reason = reason
                operation.phase = "stopping"
                proc = operation.proc
            else:
                proc = None
        self._stop_queue.put(reason)
        if proc is not None:
            self._terminate(proc)
        self.record_event("停止请求已受理：%s（预览/执行/保持锁定至手动复位后的新 DISARMED 观测）" % reason,
                          "warn")
        return {"ok": True, "accepted": True, "completed": False, "phase": "stop_requested",
                "message": "停止已请求；这不是物理急停。新失能确认以状态中的 disabled_observed 为准。",
                "operator": operator, "stop": self._stop_display(),
                "stop_gate": self._stop_gate_display(), "ros_thread_alive": self._ros_alive()}

    def stop(self, reason=None):
        label = str(reason or "操作员全轴停止").strip()[:60] or "操作员全轴停止"
        return self._request_stop(label)

    def _stop_display(self):
        with self._lock:
            state = dict(self._stop_state)
        state["stage_label"] = {"not_requested": "未请求", "requested": "停止已请求",
                                "confirmed": "新失能已确认", "unknown": "未知（保持支撑并现场切电）"
                                }.get(state["state"], state["state"])
        return state

    def _stop_gate_display(self):
        with self._lock:
            return {"locked": self._stop_locked, "reason": self._stop_locked_reason,
                    "locked_at": self._stop_locked_wall,
                    "generation": self._stop_locked_generation,
                    "reset_requested_at": self._reset_after_stop_wall,
                    "unlocked_at": self._stop_gate_unlocked_wall,
                    "requires": "手动复位后、因果上更新的四轴一致 DISARMED 观测"}

    def _check_stop_gate(self, now):
        """Unlock only on a manual reset followed by a fresh consistent DISARMED."""
        with self._lock:
            if not self._stop_locked or self._reset_after_stop_at is None:
                return
            reset_at = self._reset_after_stop_at
            generation = self._stop_locked_generation
        core = self._core()
        fresh_after_reset = core["dynamic_at"] is not None and core["dynamic_at"] > reset_at
        if not (fresh_after_reset and core["stage"] == 0 and core["stage_fresh"] and core["consistent"]
                and core["positions"] is not None and all(core["feedback_valid"])):
            return
        with self._lock:
            if (self._stop_locked and self._stop_locked_generation == generation
                    and self._reset_after_stop_at == reset_at):
                self._stop_locked = False
                self._reset_after_stop_at = None
                self._stop_gate_unlocked_wall = time.time()
                self.record_event("停止门禁解除：已观测手动复位后的新鲜四轴一致 DISARMED", "warn")

    # ------------------------------------------------------------ connection
    def connection(self):
        details = self._connection_manager.inspect()
        core = self._core()
        return {'ok': True, 'stage': core['stage'],
                'feedback_valid_all': all(core['feedback_valid']),
                'state': dict(self._connection_state), **details}

    def reconnect(self, ack_power_cycle=False):
        self._require_open()
        with self._lock:
            if self._operation_running() or self._connection_state['phase'] == 'connecting':
                raise PanelError(409, 'BUSY', '操作或连接恢复正在进行')
            core = self._core()
            known_disabled = (core['stage'] in (0, 8) and core['stage_fresh']
                              and core['consistent'] and all(core['feedback_valid']))
            if core['stage'] in (1, 2, 3, 4, 5, 6, 7):
                raise PanelError(409, 'ACTIVE', '驱动仍在控制或停止中，请先全轴停止并确认失能')
            if not known_disabled and ack_power_cycle is not True:
                raise PanelError(409, 'POWER_REVIEW', '状态未知：先支撑并完成断电检查，恢复供电后勾选确认再重连')
            self._stop_generation += 1
            generation = self._stop_generation
            self._stop_locked = True
            self._stop_locked_generation = generation
            self._stop_locked_wall = time.time()
            self._stop_locked_reason = '正在恢复连接；不会自动使能或续跑'
            self._reset_after_stop_at = None
            self._connection_cancel.clear()
            self._connection_recovered = False
            self._connection_state = {'phase': 'connecting', 'message': '开始恢复连接'}
        # Discard old session snapshots. Only subsequent observations can prove recovery.
        with self._state_lock:
            self._dynamic = self._joints = self._controller = None
            self._ff = None
        self.tickets.drop('连接已变化，请重新预览')
        self.record_event('操作员请求恢复连接；动作入口锁定，完成后需手动确认新连接')
        def progress(message):
            with self._lock:self._connection_state['message'] = message
        def work():
            try:
                proc, log = self._connection_manager.restart(True, self._connection_cancel, progress)
                started = time.monotonic()
                previous = None
                observations = 0
                while time.monotonic() - started < 15:
                    if self._connection_cancel.wait(.1):raise RuntimeError('连接操作已取消')
                    if proc.poll() is not None:raise RuntimeError('驱动启动退出，请检查供电与USB连接；日志：'+log.name)
                    current = self._core()
                    fresh = (current['dynamic_at'] is not None and current['dynamic_at'] > started
                             and current['stage'] == 0 and current['stage_fresh'] and current['consistent']
                             and all(current['feedback_valid']) and current['positions'] is not None
                             and all(current['axes'].get(name, {}).get('motor_state') == 0 for name in JOINT_NAMES))
                    if fresh and current['dynamic_at'] != previous:
                        observations += 1
                        previous = current['dynamic_at']
                    elif not fresh:observations = 0
                    if observations >= 3:
                        with self._lock:
                            if self._connection_cancel.is_set() or self._stop_generation != generation:
                                raise RuntimeError('连接操作已被停止')
                            self._connection_recovered = True
                            self._connection_state = {'phase': 'connected', 'message': '连接恢复，四轴未使能；点击软件复位确认后重新预览'}
                            self._stop_state = {'state': 'not_requested', 'reason': '新连接未使能，等待手动确认',
                                'requested_at': None, 'requested_wall': None, 'generation': generation,
                                'confirmed_at': None, 'confirmed_wall': None}
                        self.record_event('已观测新连接四轴有效且未使能；未自动复位/使能/恢复轨迹')
                        return
                raise RuntimeError('未取得新鲜四轴未使能反馈，请检查电机供电和CAN连接')
            except Exception as error:
                self._connection_manager.cancel_owned_start()
                with self._lock:self._connection_state = {'phase': 'failed', 'message': str(error)}
                self.record_event('连接恢复失败：'+str(error), 'error')
        threading.Thread(target=work, name='panel-connection', daemon=True).start()
        return {'ok': True, 'accepted': True, 'completed': False, 'message': '正在尝试重连，不会使能电机'}

    # ----------------------------------------------------------------- reset
    def reset(self):
        self._require_open()
        if self._operation_running():
            raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发复位")
        core = self._core()
        if self._connection_recovered and core['stage'] == 0 and core['stage_fresh'] \
                and core['consistent'] and all(core['feedback_valid']) and not core['owned']:
            with self._lock:
                self._reset_after_stop_at = time.monotonic()
                self._reset_after_stop_wall = time.time()
                self._connection_recovered = False
            self.record_event('操作员确认新连接；等待随后新鲜四轴未使能反馈解锁，不请求使能')
            return {'ok': True, 'accepted': True, 'completed': False,
                    'message': '已确认新连接；等待新未使能观测解锁'}
        if core["stage"] != 8:
            raise PanelError(409, "NOT_DISABLED", "软件复位需要先观测到已失能（当前：%s）"
                             % stage_label(core["stage"]))
        if core["owned"]:
            raise PanelError(409, "OWNED", "轨迹控制器仍接管接口，先停止并释放控制器")
        with self._lock:
            reset_at = time.monotonic()
            reset_wall = time.time()
            self._reset_queue.put(reset_at)
            if self._stop_locked:
                self._reset_after_stop_at = reset_at
                self._reset_after_stop_wall = reset_wall
        self.record_event("已排队软件复位请求；复位不会自动重新使能")
        return {"ok": True, "accepted": True, "completed": False, "phase": "reset_requested",
                "message": "软件复位请求已排队；是否成功以控制阶段变为未使能为准，不会自动使能。",
                "stop_gate": self._stop_gate_display()}

    # ------------------------------------------------------------ teaching
    def _teaching_config(self):
        """当前驱动运行身份；记录与回放都要求它是新鲜且可解析的。"""
        identity = self._identity_view()
        if not identity.get("fresh"):
            raise PanelError(409, "RUNTIME_IDENTITY",
                             "驱动运行身份不可用或已过期（%s），不能记录或回放拖动示教"
                             % identity.get("reason", "未就绪"))
        try:
            return normalize_identity(identity["payload"])
        except TeachingError as error:
            raise PanelError(409, "RUNTIME_IDENTITY", error.message)

    def _teaching_driver(self):
        driver = self._ff_view()
        return {"connected": bool(driver.get("connected")),
                "float_available": bool(driver.get("float_available")),
                "gravity_float": bool(driver.get("gravity_float")),
                "float_kd": driver.get("float_kd"),
                "float_seconds": driver.get("float_seconds"),
                "float_configurable": bool(driver.get("float_configurable")),
                "reason": driver.get("reason")}

    def _teaching_record_capability(self, core, busy):
        if self._closing.is_set():
            return "面板正在退出"
        with self._lock:
            if self._stop_locked:
                return "停止已受理：等待手动复位与新的四轴一致未使能观测"
            if self._teaching_recorder is not None:
                return "已有拖动记录正在进行"
        if busy:
            return "有操作正在进行"
        if not self._teaching_driver()["float_available"]:
            return "当前驱动未开放自由拖动模式"
        return self._capability_reason(core, busy, None)

    def start_teaching_record(self, confirmations, float_settings=None):
        settings = self._float_settings(float_settings)
        driver = self._ff_view()
        if not driver.get("connected") or not driver.get("float_available"):
            raise PanelError(409, "FLOAT_UNAVAILABLE", "当前驱动未开放自由拖动模式，不能开始拖动示教")
        if not isinstance(confirmations, dict) or confirmations.get("drag_teaching") is not True:
            raise PanelError(400, "TEACHING_ACK",
                             "请确认已托住机械臂，并在渐入和结束失能时承接重量")
        if confirmations.get("gravity_float") is not True:
            raise PanelError(400, "FLOAT_ACK", "拖动示教复用自由拖动，需要自由拖动确认")
        confirmations = validate_confirmations(confirmations)
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，不能开始拖动记录")
            if self._teaching_recorder is not None:
                raise PanelError(409, "RECORDING", "已有拖动记录正在进行")
            baseline = self._stop_generation
        core = self._core()
        self._require_disarmed(core)
        identity = self._teaching_config()
        limits_deg = degrees_limits(self.limits)
        recorder = TeachingRecorder(self.limits, limits_deg)
        with self._lock:
            if self._teaching_recorder is not None:
                raise PanelError(409, "RECORDING", "已有拖动记录正在进行")
            self._teaching_recorder = recorder
            self._teaching_last_error = None
        self.record_event("拖动示教开始记录：Kd=%g，持续 %g 秒" % (settings["kd"], settings["seconds"]))
        try:
            launched = self._launch("drag_teaching", None, confirmations, baseline,
                                    start_positions=core["positions"], recording_id=recorder.id,
                                    float_settings=settings)
        except Exception:
            with self._lock:
                self._teaching_recorder = None
            raise
        response = dict(launched)
        response["recording"] = recorder.display()
        response["expected_seconds"] = settings["seconds"]
        response["identity_boot_id"] = identity["boot_id"]
        response["message"] = ("拖动示教已受理：托住机械臂等待渐入；记录只跟随自由拖动有效窗口，"
                               "结束走既有停止流程。")
        return response

    def stop_teaching_record(self):
        # Stop is a global epoch: even an idle "stop recording" request bumps the
        # generation and locks the gate, so an in-flight start can never become a
        # late start after the operator pressed stop.
        with self._lock:
            recorder = self._teaching_recorder
        response = self._request_stop("拖动示教结束（停止优先）")
        response["recording"] = recorder.display() if recorder is not None else None
        return response

    def _finalize_teaching_record(self, operation, projection):
        with self._lock:
            recorder = self._teaching_recorder
            self._teaching_recorder = None
            if recorder is None:
                return
            self._teaching_finalizing = True
        try:
            stop_confirmed = bool(projection and projection.get("stop_observations_confirmed"))
            stop_reason = operation.stop_reason
            interrupted = None
            if stop_reason not in (None, "", "拖动示教结束（停止优先）"):
                interrupted = "停止原因：%s" % stop_reason
            identity = self._identity_view()
            normalized = None
            if identity.get("fresh"):
                try:
                    normalized = normalize_identity(identity["payload"])
                except TeachingError:
                    normalized = None
            reference_speed = (normalized or {}).get("reference_speed_rad_s", REFERENCE_SPEED_RAD_S)
            capture = recorder.finalize(
                stop_confirmed=stop_confirmed, interrupted_reason=interrupted,
                operation_id=operation.id, reference_speed_rad_s=reference_speed,
                driver_kp=(normalized or {}).get("kp"), driver_kd=(normalized or {}).get("kd"),
                driver_alpha=(normalized or {}).get("gravity_ff_alpha"),
                driver_ramp=(normalized or {}).get("gravity_ff_ramp_s"))
            capture["source"]["float_settings"] = getattr(operation, "float_settings", None)
            capture["content_sha256"] = content_sha256(capture)
            self.teachings.write_raw(capture)
            summary = {"recording_id": capture["recording_id"], "complete": capture["complete"],
                       "reasons": list(capture["reasons"]),
                       "quality": dict(capture["quality"]),
                       "boot_id": capture["source"].get("boot_id"),
                       "command_id": capture["source"].get("command_id"),
                       "load_id": (capture["source"].get("identity") or {}).get("load_id"),
                       "phase": "ready" if capture["complete"] else "incomplete",
                       "saved": False, "teaching": None}
            with self._lock:
                self._teaching_last = summary
            self.record_event(
                "拖动示教记录结束：%s，%d 帧 / %.2f 秒%s"
                % ("完整" if capture["complete"] else "不完整",
                   capture["quality"]["frames"], capture["quality"]["duration_s"],
                   "" if capture["complete"] else "（" + "；".join(capture["reasons"][:2]) + "）"),
                "info" if capture["complete"] else "warn")
        except Exception as error:
            with self._lock:
                self._teaching_last_error = "%s: %s" % (type(error).__name__, error)
            self.record_event("拖动示教落盘失败：%s" % error, "error")
        finally:
            with self._lock:
                self._teaching_finalizing = False

    def save_teaching(self, recording_id, name):
        self._require_open()
        try:
            capture = self.teachings.load_raw(recording_id)
        except TeachingError as error:
            status = 404 if error.code == "NOT_FOUND" else 400
            raise PanelError(status, error.code, error.message)
        if capture["complete"] is not True:
            raise PanelError(409, "INCOMPLETE", "不完整的记录不能命名保存：%s"
                             % "；".join(capture["reasons"][:3]))
        try:
            payload = self.teachings.save(capture, name)
        except TeachingError as error:
            raise PanelError(400, error.code, error.message)
        summary = teaching_summary(payload)
        with self._lock:
            if self._teaching_last and self._teaching_last.get("recording_id") == recording_id:
                self._teaching_last.update({"saved": True, "teaching": summary, "phase": "saved"})
        self.record_event("拖动示教已保存：%s（%d 帧 / %.2f 秒）"
                          % (summary["name"], summary.get("frames") or 0,
                             summary.get("duration_s") or 0.0))
        return {"ok": True, "accepted": True, "completed": True, "teaching": summary}

    def list_teachings(self):
        return {"ok": True, "teachings": self.teachings.list(),
                "default_speed_factor": DEFAULT_SPEED_FACTOR,
                "speed_factor_min": MIN_SPEED_FACTOR, "speed_factor_max": MAX_SPEED_FACTOR,
                "speed_limit_rad_s": SPEED_LIMIT_RAD_S}

    def load_teaching(self, teaching_id):
        try:
            payload = self.teachings.load(teaching_id)
        except TeachingError as error:
            status = 404 if error.code == "NOT_FOUND" else 400
            raise PanelError(status, error.code, error.message)
        return {"ok": True, "kind": "teaching", "id": payload["id"],
                "name": payload["name"], "saved_at": payload["saved_at"],
                "teaching": teaching_summary(payload), "source": payload["source"],
                "quality": payload["quality"]}

    def preview_teaching(self, teaching_id, speed_factor=DEFAULT_SPEED_FACTOR):
        self._require_open()
        self._require_stop_gate()
        if self._operation_running():
            raise PanelError(409, "BUSY", "有操作正在进行，不能预览回放")
        core = self._core()
        self._require_disarmed(core)
        identity = self._teaching_config()
        try:
            payload = self.teachings.load(teaching_id)
        except TeachingError as error:
            status = 404 if error.code == "NOT_FOUND" else 400
            raise PanelError(status, error.code, error.message)
        reasons = identity_compatibility(payload["source"]["identity"], identity)
        if reasons:
            raise PanelError(409, "TEACHING_VERSION",
                             "教学记录的运行配置与当前驱动不一致（%s）；请在新的配置下重新采集，"
                             "不做静默重映射" % "、".join(reasons))
        limit = min(SPEED_LIMIT_RAD_S, float(identity.get("reference_speed_rad_s") or SPEED_LIMIT_RAD_S))
        try:
            compiled = compile_playback(payload, core["positions"], self.limits, speed_factor,
                                        budget_s=PLAYBACK_LIMIT_S, speed_limit_rad_s=limit,
                                        identity=identity)
        except TeachingError as error:
            status = 400 if error.code in ("SPEED_FACTOR", "SPEED", "LIMIT", "BUDGET") else 409
            raise PanelError(status, error.code, error.message)
        preview = dict(compiled["preview"])
        preview["limits_deg"] = degrees_limits(self.limits)
        preview["limits_source"] = self.limits_source
        ticket = self.tickets.issue({"mode": "teaching", "execution_mode": "drag_teaching",
                                     "frozen": compiled["frozen"],
                                     "frozen_sha256": compiled["frozen_sha256"],
                                     "source_sha256": payload["content_sha256"],
                                     "preview": preview, "start_rad": list(core["positions"]),
                                     "start_deg": preview["start_deg"],
                                     "teaching_id": payload["id"],
                                     "speed_factor": preview["speed_factor"],
                                     "identity_sha256": identity_fingerprint(identity)})
        self.record_event("拖动示教预览：%s；接近 %.2f 秒，回放总 %.2f 秒，速度比例 %.2f，票根 %s…"
                          % (payload["name"], preview["approach_s"],
                             preview["retimed_duration_s"], preview["speed_factor"],
                             ticket["id"][:8]))
        return {"ok": True, "accepted": True, "completed": True,
                "message": "服务器校验通过；这是预览，没有请求使能或轨迹。",
                "ticket": ticket, "preview": preview}

    def execute_teaching(self, ticket_id, confirmations):
        confirmations = validate_confirmations(confirmations)
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发执行")
            baseline = self._stop_generation
        core = self._core()
        self._require_disarmed(core)
        ticket = self.tickets.peek(ticket_id)
        if ticket is None:
            raise PanelError(409, "TICKET_UNKNOWN", "预览票根不存在，请重新预览")
        if ticket.get("mode") != "teaching" or ticket.get("execution_mode") != "drag_teaching":
            raise PanelError(409, "MODE_CHANGED", "票根不是拖动示教回放，请重新预览")
        if ticket["used"]:
            raise PanelError(409, "TICKET_USED", "预览票根已使用，请重新预览")
        if ticket["expired"]:
            raise PanelError(409, "TICKET_EXPIRED", "预览已过期，请重新预览")
        # Bind the ticket to the runtime identity observed at preview time; a
        # mapping/policy/URDF change after preview must never reach the launcher.
        current_identity = self._teaching_config()
        if ticket.get("identity_sha256") != identity_fingerprint(current_identity):
            raise PanelError(409, "IDENTITY_CHANGED",
                             "预览后驱动运行身份已变化（配置/连接指纹不一致），请重新预览")
        drift = max(abs(a - b) for a, b in zip(core["positions"], ticket["start_rad"]))
        if drift > math.radians(POSE_DRIFT_DEG):
            raise PanelError(409, "POSE_DRIFT", "预览后姿态变化 %.2f°，请重新预览" % math.degrees(drift))
        ticket = self.tickets.consume(ticket_id)
        self.record_event("拖动示教回放受理：票根 %s… 速度比例 %.2f（正常位置控制，非自由拖动）"
                          % (ticket_id[:8], ticket.get("speed_factor") or DEFAULT_SPEED_FACTOR))
        return self._launch("teaching_playback", ticket, confirmations, baseline)

    def _teaching_display(self):
        with self._lock:
            recorder = self._teaching_recorder
            last = dict(self._teaching_last) if self._teaching_last else None
            finalizing = self._teaching_finalizing
            last_error = self._teaching_last_error
        driver = self._teaching_driver()
        identity = self._identity_view()
        reference = REFERENCE_SPEED_RAD_S
        if identity.get("fresh"):
            try:
                reference = normalize_identity(identity["payload"])["reference_speed_rad_s"]
            except TeachingError:
                pass
        view = {"available": driver["float_available"], "driver": driver,
                "default_speed_factor": DEFAULT_SPEED_FACTOR,
                "speed_factor_min": MIN_SPEED_FACTOR, "speed_factor_max": MAX_SPEED_FACTOR,
                "speed_limit_rad_s": round(min(SPEED_LIMIT_RAD_S, reference), 3),
                "max_record_seconds": FLOAT_MAX_SECONDS,
                "sample_age_limit_s": 0.06,
                "identity": {"fresh": identity.get("fresh", False),
                             "boot_id": (identity.get("payload") or {}).get("boot_id")},
                "phase": "idle", "recording": None, "last": last,
                "finalizing": finalizing, "last_error": last_error}
        if recorder is not None:
            display = recorder.display()
            view["recording"] = display
            if display["phase"] == "recording":
                view["phase"] = "recording"
            elif driver["gravity_float"] or display["phase"] in ("ramp_in", "waiting"):
                view["phase"] = "ramp_in"
            else:
                view["phase"] = "starting"
        elif last is not None:
            view["phase"] = last.get("phase", "ready")
        elif finalizing:
            view["phase"] = "finalizing"
        return view

    # ------------------------------------------------------- supported capture
    def preview_supported_capture(self):
        self._require_open()
        self._require_stop_gate()
        if self._operation_running():
            raise PanelError(409, "BUSY", "有操作正在进行，不能预览静态采样")
        core = self._core()
        self._require_disarmed(core)
        identity = self._teaching_config()
        driver = self._ff_view()
        alpha = driver.get("alpha") if driver.get("connected") else None
        preview = capture_flow.preview_payload(
            core["positions"], identity, alpha,
            [list(row) for row in degrees_limits(self.limits)], self.limits_source)
        payload = {"mode": "supported_capture", "execution_mode": "supported_capture",
                   "frozen": preview, "frozen_sha256": capture_flow.json_sha256(preview),
                   "source_sha256": capture_flow.json_sha256(preview),
                   "preview": preview, "start_rad": list(core["positions"]),
                   "start_deg": preview["start_deg"],
                   "identity_sha256": identity_fingerprint(identity),
                   "boot_id": identity["boot_id"],
                   "stop_generation": self._stop_generation}
        ticket = self.tickets.issue(payload)
        self.record_event("静态采样预览：%s，α=%s（%s），票根 %s…（未使能）"
                          % (preview["start_deg"], preview["alpha"],
                             "基线满足" if preview["baseline"]["satisfied"] else "仅诊断",
                             ticket["id"][:8]))
        return {"ok": True, "accepted": True, "completed": True,
                "message": "服务器校验通过；这是预览，没有请求使能或轨迹。",
                "ticket": ticket, "preview": preview}

    def start_supported_capture(self, ticket_id, confirmations):
        try:
            capture_flow.validate_start_confirmations(confirmations)
        except capture_flow.CaptureError as error:
            raise PanelError(403, "CONFIRMATION", error.message)
        confirmations = validate_confirmations(confirmations)
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            if self._operation_running():
                raise PanelError(409, "BUSY", "有操作正在进行，拒绝并发启动")
            baseline = self._stop_generation
        core = self._core()
        self._require_disarmed(core)
        ticket = self.tickets.peek(ticket_id)
        if ticket is None:
            raise PanelError(409, "TICKET_UNKNOWN", "预览票根不存在，请重新预览")
        if ticket.get("mode") != "supported_capture":
            raise PanelError(409, "MODE_CHANGED", "票根不是当前位置静态采样，请重新预览")
        if ticket["used"]:
            raise PanelError(409, "TICKET_USED", "预览票根已使用，请重新预览")
        if ticket["expired"]:
            raise PanelError(409, "TICKET_EXPIRED", "预览已过期，请重新预览")
        identity = self._teaching_config()
        if ticket.get("identity_sha256") != identity_fingerprint(identity):
            raise PanelError(409, "IDENTITY_CHANGED",
                             "预览后驱动运行身份已变化，请重新预览")
        if ticket.get("stop_generation") != self._stop_generation:
            raise PanelError(409, "STOP_EPOCH_CHANGED", "预览后停止纪元已变化，请重新预览")
        drift = max(abs(a - b) for a, b in zip(core["positions"], ticket["start_rad"]))
        if drift > math.radians(POSE_DRIFT_DEG):
            raise PanelError(409, "POSE_DRIFT", "预览后姿态变化 %.2f°，请重新预览"
                             % math.degrees(drift))
        ticket = self.tickets.consume(ticket_id)
        self.record_event("当前位置静态采样受理：票根 %s…，起点 %s°，α=%s"
                          % (ticket_id[:8], ticket["preview"]["start_deg"],
                             ticket["preview"]["alpha"]))
        return self._launch("supported_capture", ticket, confirmations, baseline)

    def _capture_ready_reasons(self, core, driver, identity, operation):
        reasons = []
        if core["stage"] != 5:
            reasons.append("控制阶段不是保持（%s）" % stage_label(core["stage"]))
        if not core["stage_fresh"]:
            reasons.append("反馈陈旧")
        if not core["consistent"]:
            reasons.append("四轴阶段不一致")
        if not all(core["feedback_valid"]):
            reasons.append("四轴反馈无效")
        if not core["hold_ready"]:
            reasons.append("驱动尚未就绪（hold_ready=0）")
        if core["command_id"] is None:
            reasons.append("缺少 command_id")
        if not driver.get("connected"):
            reasons.append("驱动前馈状态不可用")
        elif driver.get("gravity_float"):
            reasons.append("驱动仍处于自由拖动模式，不是位置保持")
        if not identity.get("fresh"):
            reasons.append("驱动运行身份不可用")
        return reasons

    def _check_supported_capture(self, now):
        with self._lock:
            operation = self._operation
        if operation is None or operation.finished or operation.kind != "supported_capture":
            return
        core = self._core()
        driver = self._ff_view()
        identity = self._identity_view()
        reasons = self._capture_ready_reasons(core, driver, identity, operation)
        ready = not reasons
        command_id = int(core["command_id"]) if core["command_id"] is not None else None
        identity_sha = None
        if identity.get("fresh"):
            try:
                identity_sha = identity_fingerprint(normalize_identity(identity["payload"]))
            except TeachingError:
                identity_sha = None
        with self._lock:
            operation.capture_state = {
                "stage": core["stage"], "stage_label": stage_label(core["stage"]),
                "hold_ready": core["hold_ready"], "command_id": command_id,
                "feedback_fresh": bool(core["dynamic_age"] is not None
                                       and core["dynamic_age"] <= FEEDBACK_FRESH_S),
                "mode": ("position_hold" if driver.get("connected")
                         and not driver.get("gravity_float") else "unknown"),
                "alpha": driver.get("alpha"),
                "identity_sha256": identity_sha,
                "reasons": reasons, "ready": ready,
                "ack_sent": operation.capture_ack_sent_at is not None}
            if operation.capture_command_id is None and ready:
                operation.capture_command_id = command_id
                operation.capture_identity_sha256 = identity_sha
                operation.capture_boot_id = (identity.get("payload") or {}).get("boot_id")
                operation.capture_ready_at = time.time()
                operation.capture_alpha = driver.get("alpha")
            if operation.capture_command_id is not None and command_id is not None \
                    and command_id != operation.capture_command_id:
                operation.capture_ready = False
                operation.capture_state["reasons"] = list(reasons) + ["command_id 已变化"]
            else:
                operation.capture_ready = ready

    def ack_supported_capture(self, operation_id, confirmations):
        try:
            capture_flow.validate_ack_confirmations(confirmations)
        except capture_flow.CaptureError as error:
            raise PanelError(400, "DISENGAGE_ACK", error.message)
        self._require_open()
        self._require_lease()
        self._require_stop_gate()
        with self._lock:
            operation = self._operation
            generation = self._stop_generation
        if operation is None or operation.finished or operation.kind != "supported_capture":
            raise PanelError(409, "NO_CAPTURE", "当前没有进行中的当前位置静态采样")
        if operation.id != operation_id:
            raise PanelError(409, "STALE_OPERATION", "操作已变化，拒绝旧撤手确认")
        if operation.cancel_requested or operation.stop_reason:
            raise PanelError(409, "STOPPING", "停止已受理，拒绝撤手确认")
        if operation.capture_stop_generation != generation:
            raise PanelError(409, "STOP_EPOCH_CHANGED", "停止纪元已变化，拒绝撤手确认")
        if operation.capture_ack_sent_at is not None:
            raise PanelError(409, "DUPLICATE_ACK", "撤手确认已发送，不能重复确认")
        if operation.capture_command_id is None:
            raise PanelError(409, "NOT_READY", "位置保持尚未就绪，不能确认撤手")
        core = self._core()
        driver = self._ff_view()
        identity_view = self._identity_view()
        identity = self._teaching_config()
        if identity_fingerprint(identity) != operation.capture_identity_sha256:
            raise PanelError(409, "IDENTITY_CHANGED", "运行身份已变化，拒绝撤手确认")
        if core["command_id"] is None or int(core["command_id"]) != operation.capture_command_id:
            raise PanelError(409, "COMMAND_CHANGED", "command_id 已变化，拒绝撤手确认")
        reasons = self._capture_ready_reasons(core, driver, identity_view, operation)
        if reasons:
            raise PanelError(409, "NOT_READY", "不能确认撤手：" + "；".join(reasons))
        ack = capture_flow.build_ack(
            operation.id, operation.capture_command_id, operation.capture_boot_id,
            operation.capture_identity_sha256, generation,
            alpha=driver.get("alpha"), stage=core["stage"], hold_ready=core["hold_ready"])
        try:
            capture_flow.write_ack_file(operation.capture_ack_path, ack)
        except OSError as error:
            raise PanelError(500, "ACK_WRITE", "撤手确认写入失败：%s" % error)
        with self._lock:
            operation.capture_ack_sent_at = time.time()
            operation.capture_ack_nonce = ack["nonce"]
            operation.capture_ready = False
        self.record_event("撤手确认已写入：command_id=%s，boot=%s，nonce=%s"
                          % (ack["command_id"], ack["boot_id"], ack["nonce"]))
        return {"ok": True, "accepted": True, "completed": True,
                "ack": {"schema": ack["schema"], "operation_id": ack["operation_id"],
                        "command_id": ack["command_id"], "nonce": ack["nonce"],
                        "requested_at": ack["requested_at"]},
                "message": "撤手确认已记录；客户端将采集静态窗口，随后停止并保存。"}

    def _capture_display(self, core, operation, preview, busy, live_preview):
        driver = self._ff_view()
        identity = self._identity_view()
        alpha = driver.get("alpha") if driver.get("connected") else None
        baseline = capture_flow.fit_baseline(alpha)
        preview_payload = None
        if (preview or {}).get("mode") == "supported_capture":
            preview_payload = dict(preview.get("preview") or {})
            preview_payload["validity"] = live_preview
        current = None
        if operation is not None and not operation.finished and operation.kind == "supported_capture":
            state = dict(operation.capture_state or {})
            phase = "holding"
            if operation.capture_ack_sent_at is not None:
                phase = "capturing"
            elif state.get("ready"):
                phase = "ready"
            current = {"id": operation.id, "phase": phase,
                       "ready": bool(state.get("ready") and operation.capture_ack_sent_at is None),
                       "ack_sent": operation.capture_ack_sent_at is not None,
                       "command_id": operation.capture_command_id,
                       "boot_id": operation.capture_boot_id,
                       "ack_nonce": operation.capture_ack_nonce,
                       "state": state,
                       "target_deg": operation.final_target_deg,
                       "started_at": operation.started_at}
        last = None
        if operation is not None and operation.kind == "supported_capture" and operation.projection:
            last = {"id": operation.id, "phase": operation.phase,
                    "result": operation.projection}
        readiness = self._capture_ready_reasons(core, driver, identity, operation)
        capabilities = {
            "preview": {"ok": self._capability_reason(core, busy, None) is None,
                        "message": self._capability_reason(core, busy, None) or "可预览当前姿态"},
            "start": {"ok": False, "message": "需要先预览当前姿态"},
            "ack": {"ok": False, "message": "没有进行中的位置保持"},
            "stop": {"ok": True, "message": "始终可用：任何阶段的停止都优先于采样与写文件"},
        }
        if preview_payload is not None:
            validity = live_preview or {}
            if validity.get("valid"):
                capabilities["start"] = {"ok": True, "message": "可启动位置保持"}
            else:
                capabilities["start"] = {"ok": False,
                                         "message": "预览已失效：" + "；".join(validity.get("reasons") or [])}
        if current is not None:
            if current["ack_sent"]:
                capabilities["ack"] = {"ok": False, "message": "撤手确认已发送，等待采样完成"}
            elif operation is None or operation.capture_command_id is None or readiness:
                capabilities["ack"] = {"ok": False,
                                       "message": "等待位置保持就绪：" + "；".join(readiness or ["正在建立保持"])}
            else:
                capabilities["ack"] = {"ok": True, "message": "位置保持就绪：托稳后缓慢撤手，再确认采样"}
        return {"phase": (current or {}).get("phase", "idle"),
                "preview": preview_payload, "current": current, "last": last,
                "baseline": baseline, "readiness_reasons": readiness,
                "timing": {"hold_ready_timeout_s": capture_flow.HOLD_READY_TIMEOUT_S,
                           "ack_timeout_s": capture_flow.ACK_TIMEOUT_S,
                           "capture_seconds": capture_flow.CAPTURE_SECONDS,
                           "capture_samples": capture_flow.CAPTURE_SAMPLES},
                "capabilities": capabilities}

    # ------------------------------------------------------------------ plan
    def save_plan(self, name, document):
        self._require_open()
        try:
            normalised = normalize_document(document)
        except ValueError as error:
            raise PanelError(400, "PLAN_INVALID", str(error))
        plan_id = uuid.uuid4().hex[:16]
        payload = {"id": plan_id, "name": str(name or "未命名")[:80], "saved_at": time.time(),
                   "document": normalised, "sha256": digest(normalised)}
        (self.archive / "plans" / (plan_id + ".json")).write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        self.record_event("计划已保存：%s（%d 点）" % (payload["name"], len(normalised["waypoints"])))
        return {"ok": True, "accepted": True, "completed": True, "plan": payload}

    def list_plans(self):
        plans = []
        for path in sorted((self.archive / "plans").glob("*.json")):
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            document = payload.get("document") or {}
            plans.append({"kind": "plan", "id": payload.get("id"), "name": payload.get("name"),
                          "saved_at": payload.get("saved_at"),
                          "mode": document.get("coordinates"),
                          "waypoints": len(document.get("waypoints") or [])})
        try:
            plans.extend(self.teachings.list())
        except Exception as error:
            self.record_event("拖动示教列表读取失败：%s" % error, "warn")
        plans.sort(key=lambda item: item.get("saved_at") or 0, reverse=True)
        return plans

    def load_plan(self, plan_id):
        if not isinstance(plan_id, str) or not PLAN_ID.match(plan_id):
            raise PanelError(400, "PLAN_ID", "计划 ID 无效")
        teaching_path = self.teachings.root / (plan_id + ".json")
        if teaching_path.is_file():
            return self.load_teaching(plan_id)
        path = self.archive / "plans" / (plan_id + ".json")
        if not path.is_file():
            raise PanelError(404, "PLAN_NOT_FOUND", "计划不存在")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            document = normalize_document(payload["document"])
        except Exception as error:
            raise PanelError(500, "PLAN_CORRUPT", "计划文件无法使用：%s" % error)
        return {"ok": True, "id": payload.get("id"), "name": payload.get("name"),
                "saved_at": payload.get("saved_at"), "document": document}

    # --------------------------------------------------------------- heartbeat
    def heartbeat(self):
        self._lease_last = time.monotonic()
        return {"ok": True, "lease": {"last_heartbeat_age_s": 0.0,
                                      "timeout_s": self._heartbeat_timeout,
                                      "accepted": True}}

    def abandon_lease(self):
        if not self._operation_running():
            return {"ok": True, "accepted": False, "completed": True,
                    "message": "当前没有进行中的操作，无需停止。"}
        self.record_event("页面关闭/租约放弃：触发有界停止", "warn")
        return self._request_stop("浏览器页面关闭（租约放弃）")

    def _lease_display(self):
        age = max(0.0, time.monotonic() - self._lease_last)
        return {"last_heartbeat_age_s": round(age, 2), "timeout_s": self._heartbeat_timeout,
                "fresh": age <= self._heartbeat_timeout}

    # ----------------------------------------------------------------- status
    def status(self):
        core = self._core()
        with self._lock:
            operation = self._operation
            stop = dict(self._stop_state)
            services = {key: dict(value) for key, value in self._service.items()}
            events = list(self._events)[-80:]
            closing = self._closing.is_set()
            busy = operation is not None and not operation.finished
            lease_last = self._lease_last
            stop_generation = self._stop_generation
            stop_gate = self._stop_gate_display()
        preview = self.tickets.describe()
        live_preview = None
        if preview is not None:
            live_preview = {"valid": False, "reasons": ["预览票根已使用"], "drift_deg": None}
            if not preview["used"] and not preview["expired"]:
                live_preview = self._preview_validity(preview, core, busy)
            else:
                live_preview = {"valid": False,
                                "reasons": (["预览已过期"] if preview["expired"] else [])
                                + ([preview["invalid_reason"] or "预览票根已使用"] if preview["used"] else []),
                                "drift_deg": None}
            preview["validity"] = live_preview
        targets, target_source = self._target_frame(operation, core, preview)
        axes = self._axes_status(core, targets, target_source)
        stop_view = self._stop_view(stop, core)
        capability_preview = self._capability_reason(core, busy, None)
        capability_execute = self._capability_reason(core, busy, live_preview)
        if (preview or {}).get("mode") == "teaching" and capability_execute is None:
            capability_execute = "当前票根是拖动示教回放，请用拖动示教卡片执行"
        if not self._lease_display()["fresh"]:
            capability_execute = capability_execute or "操作页心跳过期，请刷新页面"
        capability_hold = self._capability_reason(core, busy, None)
        capability_reset = None
        if busy:
            capability_reset = "有操作正在进行"
        elif core["stage"] != 8 and not (self._connection_recovered and core['stage'] == 0):
            capability_reset = "需要先观测到已失能（当前：%s）" % stage_label(core["stage"])
        elif core["owned"]:
            capability_reset = "轨迹控制器仍接管接口"
        teaching = self._teaching_display()
        teaching_start = self._teaching_record_capability(core, busy)
        teaching_preview = self._capability_reason(core, busy, None)
        if teaching_preview is None and not teaching["identity"]["fresh"]:
            teaching_preview = "驱动运行身份未就绪或已过期，不能预览回放"
        teaching_execute = self._capability_reason(core, busy, live_preview)
        if teaching_execute is None:
            live_teaching = bool(live_preview and live_preview["valid"]
                                 and (preview or {}).get("mode") == "teaching")
            teaching_execute = None if live_teaching else "请先预览拖动示教回放以取得一次性票根"
        if not self._lease_display()["fresh"]:
            teaching_execute = teaching_execute or "操作页心跳过期，请刷新页面"
        teaching["capabilities"] = {
            "start": {"ok": teaching_start is None,
                      "message": teaching_start or "可按设置开始自由拖动记录"},
            "stop": {"ok": True, "message": "始终可用：停止记录走既有全轴停止流程"},
            "preview": {"ok": teaching_preview is None,
                        "message": teaching_preview or "可预览回放"},
            "execute": {"ok": teaching_execute is None,
                        "message": teaching_execute or "可执行回放（正常位置控制）"},
        }
        capture = self._capture_display(core, operation, preview, busy, live_preview)
        return {"ok": True, 'connection': dict(self._connection_state), "server": {
                    "version": "control_panel/1.0", "now": time.time(), "closing": closing,
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                    "archive_dir": str(self.archive), "ros_thread_alive": self._ros_alive(),
                    "session_budget_seconds": SESSION_BUDGET_S, "reserve_seconds": CLIENT_RESERVE_S,
                    "heartbeat_timeout_s": self._heartbeat_timeout, "preview_ttl_s": self.tickets.ttl,
                    "reference_speed_rad_s": REFERENCE_SPEED_RAD_S,
                    "pose_drift_tolerance_deg": POSE_DRIFT_DEG, "feedback_fresh_s": FEEDBACK_FRESH_S,
                    "confirmations_required": ["supported", "clearance_and_local_power_cut",
                                               "native_output_not_hardware_clamp"],
                    "max_waypoints": MAX_WAYPOINTS},
                "ros": {"connected": core["dynamic_age"] is not None and core["dynamic_age"] <= FEEDBACK_FRESH_S,
                        "dynamic_age_s": None if core["dynamic_age"] is None else round(core["dynamic_age"], 3),
                        "joint_age_s": None if core["joint_age"] is None else round(core["joint_age"], 3),
                        "controller_age_s": None if core["controller_age"] is None
                        else round(core["controller_age"], 3),
                        "stage": core["stage"], "stage_label": stage_label(core["stage"]),
                        "stage_fresh": core["stage_fresh"], "consistent": core["consistent"],
                        "hold_ready": core["hold_ready"], "disabled_observed": core["disabled_observed"],
                        'speed_scaling_factor':core.get('speed_scaling_factor'),
                        "audit_ok": core["audit_ok"], "command_owner": core["owned"],
                        "command_id": None if core["command_id"] is None else int(core["command_id"]),
                        "feedback_valid_all": all(core["feedback_valid"]),
                        "limits_deg": None if self.limits is None else degrees_limits(self.limits),
                        "limits_source": self.limits_source, "limits_error": self.limits_error,
                        "client_script": None if self.client_script is None else str(self.client_script),
                        "client_sha256": self.client_sha256, "client_error": self.client_error},
                "axes": axes,
                "gravity_shadow": self._gravity_status(core),
                "teaching": teaching,
                "supported_capture": capture,
                "operation": self._operation_display(operation),
                "preview": preview,
                "stop": stop_view,
                "stop_gate": stop_gate,
                "lease": {"last_heartbeat_age_s": round(max(0.0, time.monotonic() - lease_last), 2),
                          "timeout_s": self._heartbeat_timeout,
                          "fresh": max(0.0, time.monotonic() - lease_last) <= self._heartbeat_timeout,
                          "enforced_while_running": True},
                "services": services,
                "events": events,
                "stop_generation": stop_generation,
                "capabilities": {
                    "preview": {"ok": capability_preview is None, "message": capability_preview or "可预览"},
                    "execute": {"ok": capability_execute is None and bool(live_preview and live_preview["valid"]),
                                "message": capability_execute or ("可执行" if live_preview and live_preview["valid"]
                                                                   else "需要先在服务器生成有效预览")},
                    "hold": {"ok": capability_hold is None, "message": capability_hold or "可有限保持 5 秒"},
                    "reset": {"ok": capability_reset is None, "message": capability_reset or "可请求软件复位"},
                    "stop": {"ok": True, "message": "始终可用；请求全轴停止，不是物理急停"},
                }}

    def _target_frame(self, operation, core, preview):
        if operation is not None and not operation.finished and operation.kind == "supported_capture":
            if operation.final_target_deg:
                return [math.radians(value) for value in operation.final_target_deg], "静态采样保持起点"
        if preview is not None and preview.get("live") and not preview["used"] \
                and (preview or {}).get("mode") == "supported_capture":
            return [math.radians(value) for value in preview["frozen"]["start_deg"]], "静态采样预览起点"
        if operation is not None and not operation.finished and core["reference"] is not None:
            return core["reference"], "控制器参考（当前动作目标）"
        if preview is not None and preview.get("live") and not preview["used"]:
            frozen = preview["frozen"]
            if frozen.get("schema") == "slaihand_drag_teaching_playback/1":
                return list(frozen["samples"][-1][1:]), "拖动示教回放终点"
            return [math.radians(v) for v in frozen["waypoints"][-1]["degrees"]], "预览终点"
        if operation is not None and operation.final_target_deg:
            label = {"hold": "当前保持起点", "drag_teaching": "拖动示教起点",
                     "teaching_playback": "拖动示教回放终点"}.get(operation.kind, "上次计划终点")
            return [math.radians(v) for v in operation.final_target_deg], label
        return None, None

    def _axes_status(self, core, targets, target_source):
        rows = []
        limits = self.limits
        for index, name in enumerate(JOINT_NAMES):
            values = core["axes"].get(name, {})
            actual = core["positions"][index] if core["positions"] else None
            target = targets[index] if targets else None
            rows.append({
                "name": name, "label": "J%d" % (index + 1),
                "actual_deg": None if actual is None else round(math.degrees(actual), 3),
                "target_deg": None if target is None else round(math.degrees(target), 3),
                "error_deg": None if actual is None or target is None else round(math.degrees(target - actual), 3),
                "target_source": target_source,
                "velocity_deg_s": None if number(values.get("velocity")) is None
                else round(math.degrees(number(values.get("velocity"))), 2),
                "effort_nm": None if number(values.get("effort")) is None else round(number(values.get("effort")), 3),
                "driver_temperature_c": number(values.get("driver_temperature")),
                "motor_temperature_c": number(values.get("motor_temperature")),
                "feedback_age_s": None if number(values.get("feedback_age")) is None
                else round(number(values.get("feedback_age")), 3),
                "feedback_valid": bool(core["feedback_valid"][index]),
                "motor_state": None if number(values.get("motor_state")) is None
                else int(number(values.get("motor_state"))),
                "control_mode": None if number(values.get("control_mode")) is None
                else int(number(values.get("control_mode"))),
                "timeout_count": None if number(values.get("timeout_count")) is None
                else int(number(values.get("timeout_count"))),
                "configuration_conflict": None if number(values.get("configuration_conflict")) is None
                else int(number(values.get("configuration_conflict"))),
                "limits_deg": None if limits is None else [round(math.degrees(limits[index][0]), 3),
                                                            round(math.degrees(limits[index][1]), 3)],
            })
        return rows

    def _ff_view(self):
        with self._state_lock:
            ff = getattr(self, "_ff", None)
        if not ff:
            return {"connected": False, "reason": "未收到驱动前馈状态（驱动较旧或未发布）"}
        age = time.monotonic() - ff["at"]
        if age > 2.0:
            return {"connected": False, "reason": "驱动前馈状态已过期 %.1f s" % age,
                    "active": False, "alpha": ff["alpha"], "applied": ff["applied"]}
        return {"connected": True, "age_s": round(age, 3), "alpha": ff["alpha"], "ramp": ff["ramp"],
                "active": bool(ff["active"]), "stage": ff["stage"], "command_id": ff["command_id"],
                "requested": ff["requested"], "encoded": ff["encoded"],
                "applied": ff["applied"], "last_written": ff["last_written"],
                "float_available": ff.get('float_available', False),
                "gravity_float": ff.get('gravity_float', False),
                "float_kd": ff.get('float_kd', 5.0),
                "float_seconds": ff.get('float_seconds', 5.0),
                "float_configurable": ff.get('float_configurable', False)}

    def _gravity_status(self, core):
        """只按实测姿态估算重力并计算门禁余量；不产生任何命令。"""
        if self.gravity_shadow is None:
            return {"available": False, "mode": "shadow", "ff_applied": False,
                    "reason": self.gravity_error or "重力 shadow 未启用",
                    "driver": self._ff_view()}
        if core["positions"] is None:
            return {"available": False, "mode": "shadow", "ff_applied": False,
                    "reason": "没有完整四轴反馈，无法估算重力",
                    "driver": self._ff_view()}
        velocities = [number(core["axes"].get(name, {}).get("velocity")) for name in JOINT_NAMES]
        if any(value is None for value in velocities):
            velocities = None
        driver = self._ff_view()
        applied = driver.get("applied") if driver.get("connected") and driver.get("active") else None
        try:
            floating = driver.get('connected') and driver.get('gravity_float')
            reference = core['positions'] if floating else core['reference']
            estimate = self.gravity_shadow.estimate(core["positions"], velocities, reference,
                                                    applied, gravity_float=floating)
        except Exception as error:
            return {"available": False, "mode": "shadow", "ff_applied": False,
                    "reason": "%s: %s" % (type(error).__name__, error),
                    "driver": driver}
        estimate["config"] = {"gravity_base_m_s2": list(self.gravity_base),
                              "j3_offset_rad": self.gravity_j3_offset}
        estimate["driver"] = self._ff_view()
        return estimate

    def _stop_view(self, stop, core):
        state = stop["state"]
        view = {"state": state, "reason": stop.get("reason"),
                "requested_at": stop.get("requested_wall"), "confirmed_at": stop.get("confirmed_wall"),
                "disabled_observed": core["disabled_observed"],
                "label": {"not_requested": "未请求", "requested": "停止已请求",
                          "confirmed": "新失能已确认", "unknown": "未知（保持支撑并现场切电）"}[state]}
        if state == "requested" and stop.get("requested_at") is not None:
            view["elapsed_s"] = round(max(0.0, time.monotonic() - stop["requested_at"]), 2)
        return view

    def _operation_display(self, operation=None):
        if operation is None:
            with self._lock:
                operation = self._operation
        if operation is None:
            return None
        with self._lock:
            return {"id": operation.id, "kind": operation.kind, "phase": operation.phase,
                    "created_at": operation.created_at, "started_at": operation.started_at,
                    "finished_at": operation.finished_at, "pid": operation.pid,
                    "exit_code": operation.exit_code, "stop_reason": operation.stop_reason,
                    "lease_expired": operation.lease_expired,
                    "cancel_requested": operation.cancel_requested,
                    "source_sha256": operation.source_sha256, "plan_sha256": operation.plan_sha256,
                    "ticket": operation.ticket_id, "final_target_deg": operation.final_target_deg,
                    "recording_id": operation.recording_id,
                    "capture": {"command_id": operation.capture_command_id,
                                "boot_id": operation.capture_boot_id,
                                "ready": operation.capture_ready,
                                "ack_sent_at": operation.capture_ack_sent_at,
                                "ack_nonce": operation.capture_ack_nonce,
                                "state": operation.capture_state},
                    "float_settings": operation.float_settings,
                    "result_available": operation.result is not None,
                    "result_error": operation.result_error, "result": operation.projection,
                    "stdout_tail": list(operation.lines)[-40:],
                    "directory": str(operation.directory), "running": not operation.finished,
                    "argv": [str(part) for part in operation.argv]}

    def _ros_alive(self):
        return self._ros_thread is not None and self._ros_thread.is_alive() \
            and time.monotonic() - self._spin_epoch < 1.0

    # ------------------------------------------------------------- ROS thread
    def start(self):
        self._ros_thread = threading.Thread(target=self._ros_loop, name="panel-ros", daemon=True)
        self._ros_thread.start()

    def _ros_loop(self):
        self._spin_epoch = time.monotonic()
        while not self._ros_stop.is_set():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            except Exception as error:
                self.record_event("ROS 线程异常：%s" % error, "error")
                time.sleep(0.05)
            now = time.monotonic()
            self._spin_epoch = now
            try:
                self._drain_queues()
                self._check_pending(now)
                self._check_operation(now)
                self._check_supported_capture(now)
                self._check_stop_confirmation(now)
                self._check_stop_gate(now)
                self._check_lease(now)
            except Exception as error:
                self.record_event("看门狗异常：%s" % error, "error")

    def _drain_queues(self):
        while True:
            try:
                reason = self._stop_queue.get_nowait()
            except queue.Empty:
                break
            self._call_service("stop_all", self._stop_client, Trigger.Request())
            self.record_event("已调用 /slaihand/stop_all：%s" % reason)
        while True:
            try:
                self._reset_queue.get_nowait()
            except queue.Empty:
                break
            self._call_service("reset_disabled", self._reset_client, Trigger.Request())
            self.record_event("已调用 /slaihand/reset_disabled")
        while True:
            try:
                self._deactivate_queue.get_nowait()
            except queue.Empty:
                break
            request = SwitchController.Request()
            request.strictness = 2
            request.activate_asap = True
            request.timeout.sec = 3
            request.deactivate_controllers = [CONTROLLER_NAME]
            self._call_service("switch_controller", self._switch_client, request)
            self.record_event("已请求停用轨迹控制器（仅释放所有权，不激活）")

    def _call_service(self, key, client, request):
        try:
            future = client.call_async(request)
        except Exception as error:
            self._record_service(key, False, "调用失败：%s" % error)
            return
        with self._lock:
            self._pending[key] = (future, time.monotonic() + 3.0)
        future.add_done_callback(lambda done, key=key: self._service_done(key, done))

    def _service_done(self, key, future):
        try:
            response = future.result()
            ok = bool(getattr(response, "success", getattr(response, "ok", False)))
            message = str(getattr(response, "message", "") or "")
            if not message:
                message = "ok=%s" % getattr(response, "ok", None)
        except Exception as error:
            ok, message = False, "服务结果异常：%s" % error
        self._record_service(key, ok, message)
        self.record_event("服务 %s：%s（%s）" % (key, "接受" if ok else "失败/拒绝", message),
                          "info" if ok else "warn")

    def _record_service(self, key, ok, message):
        with self._lock:
            self._service[key] = {"at": time.time(), "ok": bool(ok), "message": str(message)[:400]}
            self._pending.pop(key, None)

    def _check_pending(self, now):
        with self._lock:
            stale = [key for key, (future, deadline) in self._pending.items()
                     if now > deadline and not future.done()]
        for key in stale:
            with self._lock:
                self._pending.pop(key, None)
            self._record_service(key, False, "服务响应超时（服务未发现或驱动未响应）")
            self.record_event("服务 %s 响应超时" % key, "warn")

    def _check_operation(self, now):
        with self._lock:
            operation = self._operation
            if operation is None or operation.finished or operation.proc is None:
                return
            proc = operation.proc
        code = proc.poll()
        if code is None:
            return
        self._finish_operation(operation, code)

    def _read_result(self, client_dir):
        try:
            files = sorted(Path(client_dir).glob("*.json"))
        except OSError as error:
            return None, "结果目录不可读：%s" % error
        if not files:
            return None, "客户端未产生结果文件（未执行或提前退出）"
        try:
            return json.loads(files[-1].read_text(encoding="utf-8")), None
        except Exception as error:
            return None, "结果文件无法解析：%s" % error

    def _finish_operation(self, operation, code):
        with self._lock:
            if operation.finished:
                return
            operation.exit_code = code
            operation.finished_at = time.time()
            operation.finished = True
            stop_reason = operation.stop_reason
            cancelled = operation.cancel_requested
            directory = operation.directory
        result, error = self._read_result(directory / "client")
        projection = project_result(result)
        with self._lock:
            operation.result = result
            operation.result_error = error
            operation.projection = projection
            if stop_reason:
                operation.phase = "stopped"
            elif code == 0 and projection is not None and projection["overall_ok"]:
                operation.phase = "finished"
            else:
                operation.phase = "failed"
            self._last_operation = operation
        if projection is not None:
            self.record_event("客户端结束 code=%s reason=%s 精度=%s 停止确认=%s"
                              % (code, projection["reason"], projection["accuracy_passed"],
                                 projection["stop_observations_confirmed"]),
                              "info" if operation.phase == "finished" else "warn")
        else:
            self.record_event("客户端结束 code=%s，%s" % (code, error), "warn")
        if operation.kind == "drag_teaching":
            # Stop-first: the stop flow is already complete when the client exits;
            # writing the recording happens afterwards on its own thread.
            threading.Thread(target=self._finalize_teaching_record,
                             args=(operation, projection), name="teaching-finalize",
                             daemon=True).start()
        self._maybe_cleanup_after_operation(operation, projection, stop_reason, cancelled, directory)

    def _maybe_cleanup_after_operation(self, operation, projection, stop_reason, cancelled, directory):
        core = self._core()
        confirmed = bool(projection and projection["stop_observations_confirmed"])
        disabled = core["stage"] == 8 and core["disabled_observed"]
        if confirmed or disabled:
            needs_stop = False
        elif core["stage"] == 0 and not cancelled and stop_reason is None:
            needs_stop = False  # never armed: driver proves it stayed DISARMED
        else:
            needs_stop = True
        with self._lock:
            stop_pending = self._stop_state["state"] in ("requested", "unknown")
        if needs_stop and stop_pending:
            # Keep the operator's stop record; a real UNKNOWN is not retried or
            # downgraded, and the epoch gate stays locked until manual recovery.
            self.record_event("操作结束未确认新失能：已有停止请求/UNKNOWN，保留原停止记录，不重复请求", "warn")
        elif needs_stop and not (stop_reason is not None and core["stage"] == 8):
            self.record_event("操作结束但未确认新失能：补发停止请求", "warn")
            self._request_stop("操作结束未确认新失能", operator=False)
        if core["owned"]:
            self._deactivate_queue.put(True)
        with self._lock:
            record = self._operation_display(operation)
            record["argv"] = [str(part) for part in operation.argv]
            record["final_target_deg"] = operation.final_target_deg
        try:
            (directory / "panel.json").write_text(json.dumps(clean_for_json(record), ensure_ascii=False,
                                                             indent=2), encoding="utf-8")
        except Exception as error:
            self.record_event("操作记录写入失败：%s" % error, "error")

    def _check_stop_confirmation(self, now):
        with self._lock:
            stop = dict(self._stop_state)
        if stop["state"] != "requested" or stop["requested_at"] is None:
            return
        core = self._core()
        fresh_after = core["dynamic_at"] is not None and core["dynamic_at"] > stop["requested_at"]
        if fresh_after and core["disabled_observed"]:
            with self._lock:
                if self._stop_state["generation"] == stop["generation"] \
                        and self._stop_state["state"] == "requested":
                    self._stop_state["state"] = "confirmed"
                    self._stop_state["confirmed_at"] = now
                    self._stop_state["confirmed_wall"] = time.time()
            self.record_event("已观测四轴新反馈失能（不是物理切力矩证明）", "warn")
        elif fresh_after and core["stage"] == 9:
            with self._lock:
                if self._stop_state["generation"] == stop["generation"]:
                    self._stop_state["state"] = "unknown"
            self.record_event("停止后观测到 UNKNOWN：保持支撑并现场切电，不重试使能", "error")
        elif now - stop["requested_at"] > STOP_CONFIRM_TIMEOUT_S:
            with self._lock:
                if self._stop_state["generation"] == stop["generation"] \
                        and self._stop_state["state"] == "requested":
                    self._stop_state["state"] = "unknown"
            self.record_event("停止后 %.1f 秒内未确认新失能：状态未知，保持支撑并现场切电"
                              % STOP_CONFIRM_TIMEOUT_S, "error")

    def _check_lease(self, now):
        with self._lock:
            operation = self._operation
            if operation is None or operation.finished or operation.lease_expired:
                return
            if now - self._lease_last <= self._heartbeat_timeout:
                return
            operation.lease_expired = True
        self.record_event("操作页心跳丢失超过 %.1f 秒：有界停止" % self._heartbeat_timeout, "error")
        self._request_stop("浏览器心跳丢失（有界停止）", operator=False)

    # -------------------------------------------------------------- lifecycle
    def shutdown(self):
        self._connection_cancel.set()
        self._closing.set()
        with self._lock:
            operation = self._operation
            running = operation is not None and not operation.finished
        if running:
            self._request_stop("面板退出（有在途操作）", operator=False)
        elif self._stop_state["state"] == "requested":
            pass
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            with self._lock:
                operation = self._operation
                if operation is None or operation.finished:
                    break
            time.sleep(0.05)
        with self._lock:
            operation = self._operation
            proc = operation.proc if operation is not None and not operation.finished else None
        if proc is not None:
            self._terminate(proc)
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                with self._lock:
                    op = self._operation
                    if op is None or op.finished:
                        break
                time.sleep(0.05)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            state = self._stop_display()["state"]
            if state in ("confirmed", "unknown", "not_requested"):
                break
            time.sleep(0.05)
        self._ros_stop.set()
        thread = self._ros_thread
        if thread is not None:
            thread.join(timeout=3)
        try:
            payload = {"closed_at": time.time(), "stop": self._stop_display(),
                       "status": self.status()}
            path = self.archive / ("session_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
            path.write_text(json.dumps(clean_for_json(payload), ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as error:
            print("退出记录写入失败：%s" % error, file=sys.stderr, flush=True)
        print("控制台已退出；存档：%s" % self.archive, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description="四轴 ROS 统一控制台（仅监听 127.0.0.1）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="0 表示自动分配端口")
    parser.add_argument("--archive-dir", type=Path,
                        default=Path.home() / ".ros" / "slaihand" / "control_panel")
    parser.add_argument("--preview-ttl-seconds", type=float, default=PREVIEW_TTL_S)
    parser.add_argument("--heartbeat-timeout-seconds", type=float, default=HEARTBEAT_TIMEOUT_S)
    parser.add_argument("--gravity-shadow", choices=("on", "off"), default="on",
                        help="重力估算只显示、不改命令；off 时完全关闭")
    parser.add_argument("--gravity-base", type=float, nargs=3, default=list(DEFAULT_GRAVITY_BASE),
                        help="基础系重力向量 m/s^2（默认 +Y 向下 9.80665，待现场确认）")
    parser.add_argument("--gravity-j3-offset-rad", type=float, default=KNOWN_J3_PLACEMENT_OFFSET,
                        help="现有 MuJoCo 模型采用的 J3 固定放置偏置，只应用一次")
    args = parser.parse_args(argv)
    if args.host != "127.0.0.1":
        parser.error("只允许监听 127.0.0.1")
    if not all(math.isfinite(v) for v in args.gravity_base) or not math.isfinite(args.gravity_j3_offset_rad):
        parser.error("重力向量与 J3 偏置必须为有限数值")
    if not 1.0 <= args.preview_ttl_seconds <= 300.0:
        parser.error("预览票根有效期必须在 1～300 秒")
    if not 1.0 <= args.heartbeat_timeout_seconds <= 30.0:
        parser.error("心跳超时必须 1～30 秒")
    token = secrets.token_urlsafe(24)
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("slaihand_control_panel")
    backend = PanelBackend(node, args.archive_dir, args.preview_ttl_seconds,
                           args.heartbeat_timeout_seconds, token,
                           gravity_base=args.gravity_base,
                           gravity_j3_offset=args.gravity_j3_offset_rad,
                           gravity_enabled=args.gravity_shadow == "on")
    backend.start()
    server = None
    code = 0
    stopping = threading.Event()

    def request_exit(signum, _frame):
        stopping.set()

    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)
    try:
        try:
            server = PanelServer(backend, token, backend.static_dir, args.port)
            server.start()
        except OSError as error:
            print("无法监听 127.0.0.1:%s：%s（端口被占用？）" % (args.port, error), file=sys.stderr, flush=True)
            code = 2
            return code
        print("四轴统一控制台：http://127.0.0.1:%d/?token=%s" % (server.port, token), flush=True)
        print("ROS_DOMAIN_ID=%s  存档=%s" % (os.environ.get("ROS_DOMAIN_ID", "0"), backend.archive), flush=True)
        while not stopping.wait(0.5):
            pass
        print("收到退出信号：终止在途客户端并请求停止…", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        backend.shutdown()
        if server is not None:
            server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
