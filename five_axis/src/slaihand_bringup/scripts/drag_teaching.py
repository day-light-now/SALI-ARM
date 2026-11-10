"""拖动示教：记录、严格校验、重定时与回放轨迹编译。

纯 Python，无 ROS 导入，供面板后端、受保护客户端与离线测试共用。

数据流：面板在唯一的 DynamicJointState 回调里把每一帧交给 TeachingRecorder；
自由拖动有效窗口内的帧（单调时间、位置/速度/年龄/使能/阶段/命令 ID）原样保存。
回放 = 从当前姿态到记录首帧的五次接近段 + 原始采样点按速度比例重定时，
经由既有受保护客户端/正常位置控制/JTC 执行，不做有损降采样。
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import secrets
import time
from pathlib import Path

from joint_waypoint_plan import REFERENCE_SPEED_RAD_S

# ---------------------------------------------------------------- 版本与常量
SCHEMA_CAPTURE = "slaihand_drag_teaching_capture/1"
SCHEMA_TEACHING = "slaihand_drag_teaching/1"
SCHEMA_PLAYBACK = "slaihand_drag_teaching_playback/1"
JOINT_COUNT = 4
SUPPORTED_AXES = (4, 5)
FLOAT_MAX_SECONDS = 120.0
MAX_FRAMES = 15000
MAX_RECORD_S = FLOAT_MAX_SECONDS + 1.5
MIN_SAMPLES = 10
MIN_DURATION_S = 0.5
MAX_GAP_S = 0.05
MAX_SAMPLE_AGE_S = 0.06
DEFAULT_SPEED_FACTOR = 0.5
MIN_SPEED_FACTOR = 0.1
MAX_SPEED_FACTOR = 1.0
# 回放峰值参考速度的硬上限来自保护规划器（当前 1 rad/s），并与驱动实际
# reference_speed_rad_s 取较小者；不要在别处硬编码。
SPEED_LIMIT_RAD_S = float(REFERENCE_SPEED_RAD_S)
APPROACH_PEAK_RAD_S = 0.5
APPROACH_MIN_S = 0.6
APPROACH_SAMPLE_S = 0.05
PLAYBACK_LIMIT_S = 169.0  # 178 会话预算 − 8 秒余量 − 1 秒保持
MAX_TEACHING_BYTES = 10_000_000
NAME_MAX = 40
NAME_MIN = 1
ID_RE = re.compile(r"^[0-9a-f]{16}$")
HEX64_RE = re.compile(r"^[0-9a-f]{64}$")

IDENTITY_KEYS = ("schema", "boot_id", "started_at", "mapping_sha256", "model_sha256",
                 "policy_sha256", "mass_profile_sha256", "policy_profile", "load_id",
                 "gravity_base_m_s2", "j3_offset_rad", "kp", "kd", "reference_speed_rad_s",
                 "gravity_ff_alpha", "gravity_ff_ramp_s")
HASH_KEYS = ("mapping_sha256", "model_sha256", "policy_sha256", "mass_profile_sha256")
SAMPLE_KEYS = {"t", "mono_at", "q", "dq", "age", "stage", "command_id", "motor"}


class TeachingError(ValueError):
    """带稳定错误码的拒绝，供面板映射成 HTTP 响应。"""

    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


def configure_axes(count):
    """进程级选择4轴/5轴及对应版本 schema；默认4轴 /1 保持不变。

    面板必须在处理记录/回放前按 --joint-count 调用一次；五轴模式拒绝 /1
    旧记录而不是把它补成五轴。
    """
    if isinstance(count, bool) or count not in SUPPORTED_AXES:
        raise TeachingError("AXES", "只支持4轴或5轴，收到 %r" % (count,))
    global JOINT_COUNT, SCHEMA_CAPTURE, SCHEMA_TEACHING, SCHEMA_PLAYBACK
    JOINT_COUNT = int(count)
    version = "1" if JOINT_COUNT == 4 else "2"
    SCHEMA_CAPTURE = "slaihand_drag_teaching_capture/" + version
    SCHEMA_TEACHING = "slaihand_drag_teaching/" + version
    SCHEMA_PLAYBACK = "slaihand_drag_teaching_playback/" + version


def normalize_float_settings(settings=None):
    if settings is None:
        return {"kd": 5.0, "seconds": 5.0}
    if not isinstance(settings, dict) or set(settings) != {"kd", "seconds"}:
        raise TeachingError("FLOAT_SETTINGS", "自由拖动参数必须包含 kd 和 seconds")
    kd, seconds = settings["kd"], settings["seconds"]
    if not _finite(kd) or not 0 <= kd <= 5:
        raise TeachingError("FLOAT_SETTINGS", "Kd 必须在 0～5 之间")
    if not _finite(seconds) or not 1 <= seconds <= FLOAT_MAX_SECONDS:
        raise TeachingError("FLOAT_SETTINGS", "持续时间必须在 1～120 秒之间")
    return {"kd": float(kd), "seconds": float(seconds)}


# ---------------------------------------------------------------- 通用工具
def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _vector(value, length, label, finite=True):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise TeachingError("SHAPE", "%s 必须是 %d 维数组" % (label, length))
    rows = list(value)
    if finite and any(not _finite(item) for item in rows):
        raise TeachingError("NUMBER", "%s 必须是有限数值" % label)
    return [float(item) for item in rows]


def canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def json_sha256(payload):
    return hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()


def _exclusive_write(path, text):
    """独占创建；绝不覆盖既有文件。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            path.unlink()
        except OSError:
            pass
        raise


def normalize_identity(identity):
    """校验并固化驱动运行身份；缺项直接拒绝，不用文件哈希冒充运行身份。"""
    if not isinstance(identity, dict):
        raise TeachingError("IDENTITY", "缺少驱动运行身份")
    result = {}
    for key in IDENTITY_KEYS:
        if key not in identity:
            raise TeachingError("IDENTITY", "驱动运行身份缺少 %s" % key)
    if identity["schema"] != "slaihand_runtime_identity/1":
        raise TeachingError("IDENTITY", "驱动运行身份版本不匹配")
    boot = identity["boot_id"]
    if not isinstance(boot, str) or not boot:
        raise TeachingError("IDENTITY", "驱动 boot_id 无效")
    for key in HASH_KEYS:
        value = identity[key]
        if not isinstance(value, str) or not HEX64_RE.match(value):
            raise TeachingError("IDENTITY", "驱动身份 %s 不是 64 位十六进制" % key)
    if not isinstance(identity["policy_profile"], str):
        raise TeachingError("IDENTITY", "policy_profile 无效")
    if not isinstance(identity["load_id"], str):
        raise TeachingError("IDENTITY", "load_id 无效")
    result["schema"] = identity["schema"]
    result["boot_id"] = boot
    result["started_at"] = float(identity["started_at"]) if _finite(identity["started_at"]) else None
    for key in HASH_KEYS:
        result[key] = identity[key]
    result["policy_profile"] = identity["policy_profile"]
    result["load_id"] = identity["load_id"]
    result["gravity_base_m_s2"] = _vector(identity["gravity_base_m_s2"], 3, "gravity_base_m_s2")
    for key in ("j3_offset_rad", "kp", "kd"):
        if not _finite(identity[key]):
            raise TeachingError("IDENTITY", "驱动身份 %s 不是有限数值" % key)
        result[key] = float(identity[key])
    for key in ("reference_speed_rad_s",):
        if not _finite(identity[key]) or identity[key] <= 0:
            raise TeachingError("IDENTITY", "驱动身份 %s 无效" % key)
        result[key] = float(identity[key])
    for key in ("gravity_ff_alpha", "gravity_ff_ramp_s"):
        if not _finite(identity[key]):
            raise TeachingError("IDENTITY", "驱动身份 %s 不是有限数值" % key)
        result[key] = float(identity[key])
    return result


def identity_fingerprint(identity):
    return json_sha256(normalize_identity(identity))


def identity_compatibility(teaching_source, current_identity):
    """记录身份 vs 当前运行身份：允许换 boot_id，拒绝配置/负载不一致。"""
    recorded = normalize_identity(teaching_source)
    current = normalize_identity(current_identity)
    reasons = []
    for key in HASH_KEYS:
        if recorded[key] != current[key]:
            reasons.append(key)
    if recorded["load_id"] != current["load_id"]:
        reasons.append("load_id")
    if abs(recorded["j3_offset_rad"] - current["j3_offset_rad"]) > 1e-9:
        reasons.append("j3_offset_rad")
    return reasons


def _clean_name(name):
    if not isinstance(name, str):
        raise TeachingError("NAME", "名称必须是文本")
    if any(ord(character) < 32 or character == "\x7f" for character in name):
        raise TeachingError("NAME", "名称不能包含控制字符")
    cleaned = " ".join(name.strip().split())
    if not (NAME_MIN <= len(cleaned) <= NAME_MAX):
        raise TeachingError("NAME", "名称长度必须是 %d～%d 个字符" % (NAME_MIN, NAME_MAX))
    return cleaned


def content_sha256(capture):
    return json_sha256({"schema": capture["schema"], "source": capture["source"],
                        "quality": capture["quality"], "samples": capture["samples"],
                        "complete": capture["complete"], "reasons": capture["reasons"]})


# ---------------------------------------------------------------- 记录器
class TeachingRecorder:
    """把自由拖动有效窗口内的帧原样收集；窗口外帧只计数、不入文件。"""

    def __init__(self, bounds, limits_deg=None, clock=time.monotonic):
        self.id = secrets.token_hex(8)
        self.bounds = [(float(lo), float(hi)) for lo, hi in bounds]
        if len(self.bounds) != JOINT_COUNT:
            raise TeachingError("SHAPE", "限位必须是 %d 维" % JOINT_COUNT)
        self.limits_deg = [list(row) for row in (limits_deg or [])]
        if self.limits_deg and len(self.limits_deg) != JOINT_COUNT:
            raise TeachingError("SHAPE", "限位显示必须是 %d 维" % JOINT_COUNT)
        self.clock = clock
        self.created_at = time.time()
        self.opened_at = clock()
        self.frames = []
        self.raw_frames = 0
        self.duplicate_frames = 0
        self.time_backwards = 0
        self.invalid_frames = 0
        self.waiting = 0
        self.last_seq = None
        self.last_at = None
        self.first_at = None
        self.window_open = None  # None=未开始, True=记录中, False=已关闭
        self.closed_reason = None
        self.normal_end = None
        self.last_invalid = None
        self.identity = None
        self.identity_sha = None
        self.boot_id = None
        self.command_id = None
        self.max_age_seen = 0.0
        self.max_gap_seen = 0.0
        self.min_gap_seen = None
        self.last_source_stamp = None
        self.first_stamp = None
        self.last_t = None

    # -- 单帧入口 -----------------------------------------------------
    def record(self, seq, at, frame, identity):
        if self.window_open is False:
            return False
        if not _int(seq):
            return False
        if self.last_seq is not None and seq <= self.last_seq:
            self.duplicate_frames += 1
            return False
        source_stamp = frame.get("source_stamp") if isinstance(frame, dict) else None
        stamped = _finite(source_stamp) and source_stamp > 0
        if stamped and self.last_source_stamp is not None:
            if abs(source_stamp - self.last_source_stamp) <= 1e-9:
                # The same source frame delivered again must not become a second
                # sample; dedupe on the original timestamp, not on arrival order.
                self.duplicate_frames += 1
                return False
            if source_stamp < self.last_source_stamp:
                self.time_backwards += 1
                self._close("源时间戳倒退：%.6f → %.6f"
                            % (self.last_source_stamp, source_stamp))
                return False
        if self.last_at is not None and not (at > self.last_at):
            self.time_backwards += 1
            self._close("接收单调时钟未前进或倒退（%.6f → %.6f）" % (self.last_at, at))
            return False
        self.last_seq = seq
        self.last_at = at
        if stamped:
            self.last_source_stamp = source_stamp
        self.raw_frames += 1
        if self.window_open is True:
            candidate = self._time_of(at, source_stamp)
            gap = candidate - self.last_t
            if not (gap > 0):
                self.time_backwards += 1
                self._close("采样时间未前进")
                return False
            self.max_gap_seen = max(self.max_gap_seen, gap)
            self.min_gap_seen = gap if self.min_gap_seen is None else min(self.min_gap_seen, gap)
            if gap > MAX_GAP_S:
                self._close("采样间隙 %.3f s 超过 %.3f s" % (gap, MAX_GAP_S))
                return False
        valid, reason = self._evaluate(frame, identity)
        if self.window_open is not True:
            if not valid:
                self.waiting += 1
                self.last_invalid = reason
                return False
            self.window_open = True
            self.first_at = at
            self.first_stamp = source_stamp if stamped else None
            self.identity = normalize_identity(identity)
            self.identity_sha = identity_fingerprint(self.identity)
            self.boot_id = self.identity["boot_id"]
            self.command_id = frame["command_id"]
            self.last_t = 0.0
            self.frames.append(self._sample(at, frame, 0.0))
            return True
        if not valid:
            # Only the intentional stop sequence is a normal window end.
            self._close(reason, normal_end=self._is_normal_stop(frame, identity))
            return False
        if self.identity_sha != identity_fingerprint(identity):
            self._close("驱动运行身份在记录中变化")
            return False
        if frame.get("command_id") != self.command_id:
            self._close("command_id 在记录中变化")
            return False
        if len(self.frames) >= MAX_FRAMES:
            self._close("记录达到帧数上限")
            return False
        t = self._time_of(at, source_stamp)
        if t > MAX_RECORD_S:
            self._close("记录超过 %.1f 秒上限" % MAX_RECORD_S)
            return False
        self.last_t = t
        self.frames.append(self._sample(at, frame, t))
        return True

    def _time_of(self, at, source_stamp):
        """优先原始时间戳，缺失时退回接收单调时钟。"""
        if self.first_stamp is not None and _finite(source_stamp) and source_stamp > 0:
            return source_stamp - self.first_stamp
        return at - self.first_at

    def _is_normal_stop(self, frame, identity):
        """只有停止流程（stage 7/8）且反馈仍健康才是正常结束。"""
        if not isinstance(frame, dict) or frame.get("stage") not in (7, 8):
            return False
        try:
            q = _vector(frame.get("q"), JOINT_COUNT, "位置")
            dq = _vector(frame.get("dq"), JOINT_COUNT, "速度")
            ages = _vector(frame.get("age"), JOINT_COUNT, "反馈年龄")
            valid = frame.get("valid")
            if not isinstance(valid, (list, tuple)) or len(valid) != JOINT_COUNT:
                return False
            normalized = normalize_identity(identity)
        except TeachingError:
            return False
        if any(bool(item) is not True for item in valid):
            return False
        if any(age < 0 or age > MAX_SAMPLE_AGE_S for age in ages):
            return False
        if any(not math.isfinite(value) for value in q + dq):
            return False
        if any(not lo - 1e-9 <= value <= hi + 1e-9 for value, (lo, hi) in zip(q, self.bounds)):
            return False
        if identity_fingerprint(normalized) != self.identity_sha:
            return False
        if frame.get("command_id") != self.command_id:
            return False
        return True

    def _evaluate(self, frame, identity):
        """返回 (是否满足有效窗口, 原因)。打开窗口与窗口内复用同一判据。"""
        if not isinstance(frame, dict):
            return False, "帧格式无效"
        try:
            q = _vector(frame.get("q"), JOINT_COUNT, "位置")
            dq = _vector(frame.get("dq"), JOINT_COUNT, "速度")
            ages = _vector(frame.get("age"), JOINT_COUNT, "反馈年龄")
            valid = frame.get("valid")
            motor = frame.get("motor")
            if not isinstance(valid, (list, tuple)) or len(valid) != JOINT_COUNT:
                raise TeachingError("SHAPE", "反馈有效位维度错误")
            if not isinstance(motor, (list, tuple)) or len(motor) != JOINT_COUNT:
                raise TeachingError("SHAPE", "使能状态维度错误")
            normalize_identity(identity)
        except TeachingError as error:
            return False, error.message
        problems = []
        if not _int(frame.get("stage")) or frame.get("stage") != 5:
            problems.append("控制阶段不是保持（%s）" % frame.get("stage"))
        if any(bool(item) is not True for item in valid):
            problems.append("%d轴反馈无效" % JOINT_COUNT)
        if any(not _int(item) or item != 1 for item in motor):
            problems.append("%d轴未全部使能" % JOINT_COUNT)
        if frame.get("float") is not True:
            problems.append("自由拖动模式未生效")
        # hold_ready includes a stillness test, which naturally drops during
        # dragging. It qualifies the initial ramp, not every moving sample.
        if self.window_open is not True and frame.get("hold_ready") is not True:
            problems.append("增益/前馈渐入未完成")
        if not _int(frame.get("command_id")):
            problems.append("缺少 command_id")
        if any(age < 0 or age > MAX_SAMPLE_AGE_S for age in ages):
            problems.append("反馈年龄超过 %.2f s" % MAX_SAMPLE_AGE_S)
        if any(not math.isfinite(value) for value in q + dq):
            problems.append("存在非有限数值")
        if any(not lo - 1e-9 <= value <= hi + 1e-9 for value, (lo, hi) in zip(q, self.bounds)):
            problems.append("记录含越限帧")
        if problems:
            return False, "；".join(problems)
        if self.window_open is True:
            if identity_fingerprint(identity) != self.identity_sha:
                return False, "驱动运行身份在记录中变化"
            if frame.get("command_id") != self.command_id:
                return False, "command_id 在记录中变化"
        return True, None

    def _sample(self, at, frame, t):
        q = _vector(frame["q"], JOINT_COUNT, "位置")
        ages = _vector(frame["age"], JOINT_COUNT, "反馈年龄")
        self.max_age_seen = max(self.max_age_seen, max(ages))
        return {"t": round(t, 6), "mono_at": round(at, 6),
                "source_stamp": frame.get("source_stamp"),
                "q": [round(value, 9) for value in q],
                "dq": [round(value, 9) for value in _vector(frame["dq"], JOINT_COUNT, "速度")],
                "age": [round(value, 6) for value in ages],
                "stage": 5, "command_id": int(frame["command_id"]),
                "motor": [1] * JOINT_COUNT}

    def _close(self, reason, normal_end=False):
        if self.window_open is False:
            return
        self.window_open = False
        self.closed_reason = str(reason)
        self.normal_end = bool(normal_end)

    # -- 结束落盘 -----------------------------------------------------
    def finalize(self, stop_confirmed, interrupted_reason, operation_id, reference_speed_rad_s,
                 driver_kp=None, driver_kd=None, driver_alpha=None, driver_ramp=None):
        if self.window_open is True:
            self._close("操作结束时记录窗口未由失能/停止关闭")
        reasons = []
        samples = list(self.frames)
        duration = samples[-1]["t"] if samples else 0.0
        gaps = [round(samples[index + 1]["t"] - samples[index]["t"], 6)
                for index in range(len(samples) - 1)] if samples else []
        max_gap = max(gaps) if gaps else 0.0
        if self.window_open is None:
            reasons.append("未进入自由拖动有效窗口：" + (self.last_invalid or "无有效帧"))
        if len(samples) < MIN_SAMPLES:
            reasons.append("有效样本 %d 少于 %d 帧" % (len(samples), MIN_SAMPLES))
        if duration < MIN_DURATION_S:
            reasons.append("有效记录时长 %.2f s 少于 %.1f s" % (duration, MIN_DURATION_S))
        if max_gap > MAX_GAP_S:
            reasons.append("采样间隙 %.3f s 超过 %.3f s" % (max_gap, MAX_GAP_S))
        if self.closed_reason and not self.normal_end:
            reasons.append("窗口关闭原因：" + self.closed_reason)
        if not stop_confirmed:
            reasons.append("客户端未确认结束后的新失能")
        if interrupted_reason:
            reasons.append("记录被中断：" + str(interrupted_reason))
        if self.identity is None:
            reasons.append("缺少驱动运行身份")
        quality = {"frames": len(samples), "raw_frames": self.raw_frames,
                   "duplicate_frames": self.duplicate_frames,
                   "time_backwards": self.time_backwards,
                   "waiting_frames": self.waiting,
                   "duration_s": round(duration, 6),
                   "sample_rate_hz": round(len(samples) / duration, 3) if duration > 0 else None,
                   "min_gap_s": round(min(gaps), 6) if gaps else None,
                   "max_gap_s": round(max_gap, 6),
                   "mean_gap_s": round(sum(gaps) / len(gaps), 6) if gaps else None,
                   "max_age_s": round(self.max_age_seen, 6),
                   "window_close_reason": self.closed_reason,
                   "window_normal_end": self.normal_end,
                   "interrupted": interrupted_reason}
        source = {"operation_id": str(operation_id or "")[:32],
                  "identity": dict(self.identity) if self.identity else None,
                  "identity_sha256": self.identity_sha,
                  "boot_id": self.boot_id, "command_id": self.command_id,
                  "started_at": round(self.opened_at, 6),
                  "limits_deg": self.limits_deg,
                  "reference_speed_rad_s": float(reference_speed_rad_s),
                  "driver_kp": None if driver_kp is None else float(driver_kp),
                  "driver_kd": None if driver_kd is None else float(driver_kd),
                  "driver_gravity_ff_alpha": None if driver_alpha is None else float(driver_alpha),
                  "driver_gravity_ff_ramp_s": None if driver_ramp is None else float(driver_ramp)}
        capture = {"schema": SCHEMA_CAPTURE, "recording_id": self.id,
                   "created_at": self.created_at, "source": source, "quality": quality,
                   "samples": samples, "complete": not reasons, "reasons": reasons}
        capture["content_sha256"] = content_sha256(capture)
        return capture

    def display(self):
        samples = self.frames
        duration = samples[-1]["t"] if samples else 0.0
        return {"recording_id": self.id, "phase": self._phase(), "axes": JOINT_COUNT,
                "frames": len(samples), "raw_frames": self.raw_frames,
                "duplicate_frames": self.duplicate_frames,
                "duration_s": round(duration, 3),
                "max_age_s": round(self.max_age_seen, 4),
                "boot_id": self.boot_id, "command_id": self.command_id,
                "closed_reason": self.closed_reason,
                "last_invalid": self.last_invalid,
                "waiting_frames": self.waiting}

    def _phase(self):
        if self.window_open is True:
            return "recording"
        if self.window_open is False:
            return "closed"
        if self.raw_frames:
            return "ramp_in"
        return "waiting"


def _sample_shape(index, sample):
    if not isinstance(sample, dict) or set(sample) - {"source_stamp"} != SAMPLE_KEYS:
        raise TeachingError("SAMPLE", "第 %d 帧字段不完整" % index)
    # Earlier v1 files omit this metadata. Preserve their shape and digest while
    # retaining the original publisher timestamp in newly recorded files.
    source_metadata = {}
    if "source_stamp" in sample:
        stamp = sample["source_stamp"]
        if stamp is not None and (not _finite(stamp) or stamp <= 0):
            raise TeachingError("TIME", "第 %d 帧源时间戳无效" % index)
        source_metadata["source_stamp"] = stamp
    t = sample["t"]
    if not _finite(t) or t < 0:
        raise TeachingError("TIME", "第 %d 帧时间无效" % index)
    motor = sample["motor"]
    if not isinstance(motor, (list, tuple)) or len(motor) != JOINT_COUNT \
            or any(not _int(value) or value not in (0, 1) for value in motor):
        raise TeachingError("SAMPLE", "第 %d 帧使能状态无效" % index)
    if not _int(sample["stage"]) or not _int(sample["command_id"]):
        raise TeachingError("SAMPLE", "第 %d 帧阶段/命令 ID 无效" % index)
    return {"t": float(t), "mono_at": float(sample["mono_at"]) if _finite(sample["mono_at"]) else 0.0,
            "q": _vector(sample["q"], JOINT_COUNT, "第 %d 帧位置" % index),
            "dq": _vector(sample["dq"], JOINT_COUNT, "第 %d 帧速度" % index),
            "age": _vector(sample["age"], JOINT_COUNT, "第 %d 帧反馈年龄" % index),
            "stage": int(sample["stage"]), "command_id": int(sample["command_id"]),
            "motor": [int(value) for value in motor], **source_metadata}


def validate_capture(capture):
    if not isinstance(capture, dict):
        raise TeachingError("SHAPE", "记录必须是 JSON 对象")
    expected = {"schema", "recording_id", "created_at", "source", "quality", "samples",
                "complete", "reasons", "content_sha256"}
    if set(capture) != expected:
        raise TeachingError("SCHEMA", "记录字段与版本 %s 不匹配" % SCHEMA_CAPTURE)
    if capture["schema"] != SCHEMA_CAPTURE:
        raise TeachingError("SCHEMA", "记录 schema 不匹配")
    if not isinstance(capture["recording_id"], str) or not ID_RE.match(capture["recording_id"]):
        raise TeachingError("ID", "记录 ID 无效")
    samples = capture["samples"]
    if not isinstance(samples, list) or not 0 <= len(samples) <= MAX_FRAMES:
        raise TeachingError("SAMPLES", "样本数量无效")
    previous = None
    cleaned = []
    for index, sample in enumerate(samples):
        row = _sample_shape(index, sample)
        if previous is not None and row["t"] <= previous:
            raise TeachingError("TIME", "样本时间必须严格递增")
        previous = row["t"]
        cleaned.append(row)
    if len(cleaned) >= 2 and abs(cleaned[0]["t"]) > 1e-6:
        raise TeachingError("TIME", "首帧时间必须为 0")
    source = capture["source"]
    if not isinstance(source, dict) or "identity" not in source:
        raise TeachingError("SOURCE", "来源身份字段缺失")
    if source["identity"] is not None:
        normalize_identity(source["identity"])
    elif capture["complete"] is True:
        raise TeachingError("SOURCE", "完整记录必须包含驱动运行身份")
    for key in ("boot_id", "command_id", "operation_id", "identity_sha256", "limits_deg"):
        if key not in source:
            raise TeachingError("SOURCE", "来源字段 %s 缺失" % key)
    if not isinstance(source["limits_deg"], list) or len(source["limits_deg"]) != JOINT_COUNT:
        raise TeachingError("SOURCE", "来源限位必须是 %d 维；不补0、不截断" % JOINT_COUNT)
    if not isinstance(capture["quality"], dict):
        raise TeachingError("QUALITY", "质量字段无效")
    if not isinstance(capture["complete"], bool) or not isinstance(capture["reasons"], list):
        raise TeachingError("QUALITY", "完整性字段无效")
    recomputed = content_sha256({"schema": capture["schema"], "source": source,
                                 "quality": capture["quality"], "samples": capture["samples"],
                                 "complete": capture["complete"], "reasons": capture["reasons"]})
    if capture["content_sha256"] != recomputed:
        raise TeachingError("INTEGRITY", "记录内容哈希不匹配（文件被修改或损坏）")
    result = dict(capture)
    result["samples"] = cleaned
    return result


def validate_teaching(payload, require_complete=True):
    if not isinstance(payload, dict):
        raise TeachingError("SHAPE", "教学必须是 JSON 对象")
    expected = {"schema", "id", "recording_id", "name", "created_at", "saved_at", "source",
                "quality", "samples", "complete", "reasons", "content_sha256"}
    if set(payload) != expected:
        raise TeachingError("SCHEMA", "教学字段与版本 %s 不匹配" % SCHEMA_TEACHING)
    if payload["schema"] != SCHEMA_TEACHING:
        raise TeachingError("SCHEMA", "教学 schema 不匹配")
    if not isinstance(payload["id"], str) or not ID_RE.match(payload["id"]):
        raise TeachingError("ID", "教学 ID 无效")
    if not isinstance(payload["recording_id"], str) or not ID_RE.match(payload["recording_id"]):
        raise TeachingError("ID", "记录 ID 无效")
    if not isinstance(payload["name"], str) or not NAME_MIN <= len(payload["name"]) <= NAME_MAX:
        raise TeachingError("NAME", "名称长度必须是 %d～%d 个字符" % (NAME_MIN, NAME_MAX))
    capture_view = {"schema": SCHEMA_CAPTURE, "recording_id": payload["recording_id"],
                    "created_at": payload["created_at"], "source": payload["source"],
                    "quality": payload["quality"], "samples": payload["samples"],
                    "complete": payload["complete"], "reasons": payload["reasons"],
                    "content_sha256": payload["content_sha256"]}
    capture = validate_capture(capture_view)
    if require_complete and capture["complete"] is not True:
        raise TeachingError("INCOMPLETE", "不完整的记录不能回放：" + "；".join(capture["reasons"][:3]))
    return {"schema": payload["schema"], "id": payload["id"],
            "recording_id": capture["recording_id"], "name": payload["name"],
            "created_at": capture["created_at"], "saved_at": payload["saved_at"],
            "source": capture["source"], "quality": capture["quality"],
            "samples": capture["samples"], "complete": capture["complete"],
            "reasons": capture["reasons"], "content_sha256": capture["content_sha256"]}


# ---------------------------------------------------------------- 存储
class TeachingStore:
    """独占创建的原始记录与命名教学文件；名称与路径分离。"""

    def __init__(self, root):
        self.root = Path(root)
        self.raw_dir = self.root / "raw"
        self.root.mkdir(parents=True, exist_ok=True)
        self.raw_dir.mkdir(parents=True, exist_ok=True)

    def _raw_path(self, recording_id):
        if not isinstance(recording_id, str) or not ID_RE.match(recording_id):
            raise TeachingError("ID", "记录 ID 无效")
        return self.raw_dir / (recording_id + ".json")

    def _path(self, teaching_id):
        if not isinstance(teaching_id, str) or not ID_RE.match(teaching_id):
            raise TeachingError("ID", "教学 ID 无效")
        return self.root / (teaching_id + ".json")

    def write_raw(self, capture):
        capture = validate_capture(capture)
        path = self._raw_path(capture["recording_id"])
        _exclusive_write(path, json.dumps(capture, ensure_ascii=False, indent=2))
        return capture["recording_id"]

    def load_raw(self, recording_id):
        path = self._raw_path(recording_id)
        if not path.is_file():
            raise TeachingError("NOT_FOUND", "记录不存在")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise TeachingError("CORRUPT", "记录文件无法解析：%s" % error)
        return validate_capture(payload)

    def save(self, capture, name, now=None):
        capture = validate_capture(capture)
        if capture["complete"] is not True:
            raise TeachingError("INCOMPLETE", "不完整的记录不能命名保存")
        cleaned = _clean_name(name)
        teaching_id = secrets.token_hex(8)
        payload = {"schema": SCHEMA_TEACHING, "id": teaching_id,
                   "recording_id": capture["recording_id"], "name": cleaned,
                   "created_at": capture["created_at"],
                   "saved_at": time.time() if now is None else float(now),
                   "source": capture["source"], "quality": capture["quality"],
                   "samples": capture["samples"], "complete": True,
                   "reasons": [], "content_sha256": capture["content_sha256"]}
        _exclusive_write(self._path(teaching_id), json.dumps(payload, ensure_ascii=False, indent=2))
        return payload

    def save_raw_id(self, recording_id, name, now=None):
        return self.save(self.load_raw(recording_id), name, now=now)

    def list(self):
        rows = []
        for path in sorted(self.root.glob("*.json")):
            try:
                if path.stat().st_size > MAX_TEACHING_BYTES:
                    continue
                payload = validate_teaching(json.loads(path.read_text(encoding="utf-8")),
                                            require_complete=False)
            except (OSError, ValueError, TeachingError):
                continue
            rows.append(teaching_summary(payload))
        rows.sort(key=lambda item: item.get("saved_at") or 0, reverse=True)
        return rows

    def load(self, teaching_id):
        path = self._path(teaching_id)
        if not path.is_file():
            raise TeachingError("NOT_FOUND", "教学不存在")
        try:
            if path.stat().st_size > MAX_TEACHING_BYTES:
                raise TeachingError("SIZE", "教学文件超过大小限制")
            payload = json.loads(path.read_text(encoding="utf-8"))
        except TeachingError:
            raise
        except (OSError, ValueError) as error:
            raise TeachingError("CORRUPT", "教学文件无法解析：%s" % error)
        return validate_teaching(payload)


def teaching_summary(payload):
    source = payload.get("source") or {}
    identity = source.get("identity") or {}
    quality = payload.get("quality") or {}
    return {"kind": "teaching", "id": payload.get("id"), "recording_id": payload.get("recording_id"),
            "name": payload.get("name"), "saved_at": payload.get("saved_at"),
            "created_at": payload.get("created_at"), "axes": JOINT_COUNT,
            "frames": quality.get("frames"), "duration_s": quality.get("duration_s"),
            "sample_rate_hz": quality.get("sample_rate_hz"),
            "max_gap_s": quality.get("max_gap_s"),
            "boot_id": source.get("boot_id"), "command_id": source.get("command_id"),
            "load_id": identity.get("load_id"),
            "limits_deg": source.get("limits_deg"),
            "complete": payload.get("complete"),
            "content_sha256": payload.get("content_sha256")}


# ---------------------------------------------------------------- 回放编译
def _samples_from_teaching(payload):
    payload = validate_teaching(payload)
    times = [row["t"] for row in payload["samples"]]
    positions = [row["q"] for row in payload["samples"]]
    return times, positions


def _peak_speed(times, positions):
    peak = 0.0
    for index in range(1, len(times)):
        delta = times[index] - times[index - 1]
        if delta <= 0:
            raise TeachingError("TIME", "样本时间必须严格递增")
        peak = max(peak, max(abs(a - b) for a, b in zip(positions[index], positions[index - 1])) / delta)
    return peak


def _approach_seconds(start, first):
    delta = max(abs(a - b) for a, b in zip(start, first)) if first else 0.0
    return max(APPROACH_MIN_S, 1.875 * delta / APPROACH_PEAK_RAD_S)


def _quintic(start, target, ratio):
    s = max(0.0, min(1.0, ratio))
    blend = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return [a + (b - a) * blend for a, b in zip(start, target)]


def _interp_bound_deg(times, positions):
    """相邻采样线性插值的局部弯曲界（各关节 二阶差分/8 的最大值，度）。"""
    bound = 0.0
    for index in range(1, len(positions) - 1):
        for axis in range(JOINT_COUNT):
            second = positions[index + 1][axis] - 2 * positions[index][axis] + positions[index - 1][axis]
            bound = max(bound, abs(second) / 8.0)
    return math.degrees(bound)


def compile_playback(teaching, start_rad, bounds, speed_factor=DEFAULT_SPEED_FACTOR,
                     budget_s=PLAYBACK_LIMIT_S, speed_limit_rad_s=SPEED_LIMIT_RAD_S,
                     identity=None):
    payload = validate_teaching(teaching)
    if identity is None:
        raise TeachingError("IDENTITY", "回放规格必须绑定当前驱动运行身份")
    bound_identity = normalize_identity(identity)
    bound_identity_sha = identity_fingerprint(bound_identity)
    start = _vector(start_rad, JOINT_COUNT, "回放起点")
    bounds = [(float(lo), float(hi)) for lo, hi in bounds]
    if len(bounds) != JOINT_COUNT:
        raise TeachingError("LIMIT", "限位源必须是 %d 维；不补0、不截断" % JOINT_COUNT)
    if not _finite(speed_factor) or not MIN_SPEED_FACTOR - 1e-9 <= speed_factor <= MAX_SPEED_FACTOR + 1e-9:
        raise TeachingError("SPEED_FACTOR",
                            "速度比例必须在 %.1f～%.1f 之间" % (MIN_SPEED_FACTOR, MAX_SPEED_FACTOR))
    factor = float(speed_factor)
    if not _finite(speed_limit_rad_s) or speed_limit_rad_s <= 0:
        raise TeachingError("SPEED_LIMIT", "回放速度上限不可用")
    limit = min(float(speed_limit_rad_s), SPEED_LIMIT_RAD_S)
    times, positions = _samples_from_teaching(payload)
    original_duration = times[-1]
    peak_original = _peak_speed(times, positions)
    first = positions[0]
    approach = _approach_seconds(start, first)
    retimed_duration = approach + original_duration / factor
    if retimed_duration > budget_s:
        raise TeachingError("BUDGET", "回放总时长 %.1f s 超过 %.1f s 会话预算"
                            % (retimed_duration, budget_s))
    for index, position in enumerate([start] + positions):
        if any(not lo - 1e-9 <= value <= hi + 1e-9 for value, (lo, hi) in zip(position, bounds)):
            label = "起点" if index == 0 else "第 %d 帧" % index
            raise TeachingError("LIMIT", "%s超出当前完整 URDF 限位；不裁剪" % label)
    retimed_peak = peak_original * factor
    if retimed_peak > limit + 1e-9:
        suggested = max(MIN_SPEED_FACTOR, math.floor((limit / peak_original) * 100) / 100)
        raise TeachingError("SPEED", "记录峰值 %.3f rad/s，按 %.2f 比例回放会达到 %.3f rad/s，"
                            "超过 %.2f rad/s；请把速度比例降到 %.2f 或更低"
                            % (peak_original, factor, retimed_peak, limit, suggested))
    ranges = [[round(math.degrees(min(row[axis] for row in [start] + positions)), 3),
               round(math.degrees(max(row[axis] for row in [start] + positions)), 3)]
              for axis in range(JOINT_COUNT)]
    approach_delta_deg = [round(math.degrees(first[axis] - start[axis]), 3) for axis in range(JOINT_COUNT)]
    warnings = []
    warnings.append("接近段：从当前姿态以峰值约 %.2f rad/s 移动到记录首帧（%.1f s）"
                    % (APPROACH_PEAK_RAD_S, approach))
    warnings.append("回放为正常位置控制，不是自由拖动；结束后按既有流程失能，需要外部承接重量。")
    warnings.append("回放精度不是 0.5° 位置验收；跟踪误差与终点误差单独记录。")
    if peak_original > limit:
        warnings.append("原始记录峰值 %.2f rad/s 超过硬上限 %.2f rad/s，已按比例 %.2f 降速回放。"
                        % (peak_original, limit, factor))
    if max(abs(value) for value in approach_delta_deg) > 0.5:
        warnings.append("记录首帧与当前姿态相差 %s°，执行前请确认接近路径净空。"
                        % [round(value, 2) for value in approach_delta_deg])
    frozen = {"schema": SCHEMA_PLAYBACK, "source_id": payload["id"],
              "source_sha256": payload["content_sha256"], "speed_factor": factor,
              "original_duration_s": round(original_duration, 6),
              "peak_original_speed_rad_s": round(peak_original, 9),
              "identity": bound_identity, "identity_sha256": bound_identity_sha,
              "samples": [[round(t, 6)] + [round(value, 9) for value in row]
                          for t, row in zip(times, positions)]}
    preview = {"kind": "teaching", "execution_mode": "drag_teaching",
               "teaching": teaching_summary(payload), "speed_factor": factor,
               "speed_factor_default": DEFAULT_SPEED_FACTOR,
               "speed_factor_min": MIN_SPEED_FACTOR, "speed_factor_max": MAX_SPEED_FACTOR,
               "speed_limit_rad_s": round(limit, 3),
               "runtime_identity": {"boot_id": bound_identity["boot_id"],
                                    "identity_sha256": bound_identity_sha},
               "approach_s": round(approach, 3),
               "approach_delta_deg": approach_delta_deg,
               "target_start_deg": [round(math.degrees(value), 3) for value in first],
               "start_deg": [round(math.degrees(value), 3) for value in start],
               "original_duration_s": round(original_duration, 3),
               "retimed_duration_s": round(retimed_duration, 3),
               "record_retimed_s": round(original_duration / factor, 3),
               "ranges_deg": ranges,
               "peak_speed_rad_s": round(max(retimed_peak, APPROACH_PEAK_RAD_S), 6),
               "original_peak_speed_rad_s": round(peak_original, 6),
               "peak_approach_speed_rad_s": APPROACH_PEAK_RAD_S,
               "interp_error_bound_deg": round(_interp_bound_deg(times, positions), 4),
               "confidence": "recorded samples are replayed exactly; only inter-sample "
                             "linear interpolation introduces bounded geometric deviation",
               "warnings": warnings}
    return {"frozen": frozen, "frozen_sha256": json_sha256(frozen), "preview": preview,
            "payload": payload}


def validate_frozen(frozen):
    if not isinstance(frozen, dict):
        raise TeachingError("SHAPE", "回放规格必须是 JSON 对象")
    expected = {"schema", "source_id", "source_sha256", "speed_factor",
                "original_duration_s", "peak_original_speed_rad_s",
                "identity", "identity_sha256", "samples"}
    if set(frozen) != expected or frozen["schema"] != SCHEMA_PLAYBACK:
        raise TeachingError("SCHEMA", "回放规格版本不匹配")
    if not isinstance(frozen["source_id"], str) or not ID_RE.match(frozen["source_id"]):
        raise TeachingError("ID", "回放来源 ID 无效")
    if not isinstance(frozen["source_sha256"], str) or not HEX64_RE.match(frozen["source_sha256"]):
        raise TeachingError("ID", "回放来源哈希无效")
    factor = frozen["speed_factor"]
    if not _finite(factor) or not MIN_SPEED_FACTOR - 1e-9 <= factor <= MAX_SPEED_FACTOR + 1e-9:
        raise TeachingError("SPEED_FACTOR", "回放速度比例无效")
    bound_identity = normalize_identity(frozen["identity"])
    if frozen["identity_sha256"] != identity_fingerprint(bound_identity):
        raise TeachingError("IDENTITY", "回放规格运行身份哈希不一致")
    rows = frozen["samples"]
    if not isinstance(rows, list) or not 2 <= len(rows) <= MAX_FRAMES:
        raise TeachingError("SAMPLES", "回放样本数量无效")
    cleaned = []
    previous = None
    for index, row in enumerate(rows):
        if not isinstance(row, (list, tuple)) or len(row) != JOINT_COUNT + 1:
            raise TeachingError("SHAPE", "第 %d 个回放点必须是 1+%d 个数值" % (index, JOINT_COUNT))
        values = list(row)
        if any(not _finite(value) for value in values):
            raise TeachingError("NUMBER", "回放点必须是有限数值")
        if previous is not None and values[0] <= previous:
            raise TeachingError("TIME", "回放样本时间必须严格递增")
        previous = values[0]
        cleaned.append([float(value) for value in values])
    if abs(cleaned[0][0]) > 1e-6:
        raise TeachingError("TIME", "回放首帧时间必须为 0")
    return {"schema": frozen["schema"], "source_id": frozen["source_id"],
            "source_sha256": frozen["source_sha256"], "speed_factor": float(factor),
            "original_duration_s": float(frozen["original_duration_s"]),
            "peak_original_speed_rad_s": float(frozen["peak_original_speed_rad_s"]),
            "identity": bound_identity, "identity_sha256": frozen["identity_sha256"],
            "samples": cleaned}


def load_playback(path):
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise TeachingError("CORRUPT", "回放规格无法解析：%s" % error)
    return validate_frozen(payload)


def build_trajectory(frozen, start_rad, bounds, budget_s=PLAYBACK_LIMIT_S,
                     speed_limit_rad_s=SPEED_LIMIT_RAD_S):
    """从冻结的原始采样生成 JTC 点：接近段 + 重定时记录段。"""
    spec = validate_frozen(frozen)
    start = _vector(start_rad, JOINT_COUNT, "回放起点")
    bounds = [(float(lo), float(hi)) for lo, hi in bounds]
    if len(bounds) != JOINT_COUNT:
        raise TeachingError("LIMIT", "限位源必须是 %d 维；不补0、不截断" % JOINT_COUNT)
    limit = min(float(speed_limit_rad_s), SPEED_LIMIT_RAD_S)
    factor = spec["speed_factor"]
    samples = spec["samples"]
    times = [row[0] for row in samples]
    positions = [row[1:] for row in samples]
    original_peak = _peak_speed(times, positions)
    if abs(original_peak - spec["peak_original_speed_rad_s"]) > 1e-6:
        raise TeachingError("INTEGRITY", "回放规格峰值速度与样本不一致")
    first = positions[0]
    approach = _approach_seconds(start, first)
    retimed_peak = original_peak * factor
    if retimed_peak > limit + 1e-9:
        raise TeachingError("SPEED", "回放峰值 %.3f rad/s 超过上限 %.2f rad/s"
                            % (retimed_peak, limit))
    total = approach + times[-1] / factor
    if total > budget_s:
        raise TeachingError("BUDGET", "回放总时长 %.1f s 超过 %.1f s 会话预算" % (total, budget_s))
    for index, position in enumerate([start] + positions):
        if any(not lo - 1e-9 <= value <= hi + 1e-9 for value, (lo, hi) in zip(position, bounds)):
            raise TeachingError("LIMIT", "回放%s超出当前完整 URDF 限位；不裁剪"
                                % ("起点" if index == 0 else "第 %d 帧" % index))
    steps = max(2, int(math.ceil(approach / APPROACH_SAMPLE_S)))
    points = [{"t": 0.0, "q": list(start)}]
    for step in range(1, steps + 1):
        ratio = step / steps
        points.append({"t": round(approach * ratio, 6),
                       "q": _quintic(start, first, ratio)})
    for index in range(1, len(times)):
        points.append({"t": round(approach + times[index] / factor, 6),
                       "q": list(positions[index])})
    peak = 0.0
    for index in range(1, len(points)):
        delta = points[index]["t"] - points[index - 1]["t"]
        if not (delta > 0):
            raise TeachingError("TIME", "回放点时间必须严格递增")
        peak = max(peak, max(abs(a - b) for a, b in zip(points[index]["q"],
                                                        points[index - 1]["q"])) / delta)
    if peak > limit + 1e-9:
        raise TeachingError("SPEED", "回放轨迹峰值 %.3f rad/s 超过上限 %.2f rad/s" % (peak, limit))
    ranges = [[round(math.degrees(min(row[axis] for row in [start] + positions)), 3),
               round(math.degrees(max(row[axis] for row in [start] + positions)), 3)]
              for axis in range(JOINT_COUNT)]
    meta = {"speed_factor": factor, "source_id": spec["source_id"],
            "source_sha256": spec["source_sha256"],
            "original_duration_s": round(times[-1], 3),
            "approach_s": round(approach, 3),
            "retimed_duration_s": round(total, 3),
            "peak_speed_rad_s": round(peak, 6),
            "peak_record_speed_rad_s": round(retimed_peak, 6),
            "original_peak_speed_rad_s": round(original_peak, 6),
            "ranges_deg": ranges,
            "samples": len(samples), "points": len(points),
            "interp_error_bound_deg": round(_interp_bound_deg(times, positions), 4),
            "limit_rad_s": round(limit, 3)}
    return points, meta


def tracking_error_deg(points, position_samples, movement_at):
    """position_samples: [(monotonic_at, [q1..qN])]；返回跟踪误差与终点误差。"""
    rows = [(at - movement_at, q) for at, q in position_samples if at >= movement_at]
    if not rows or len(points) < 2:
        return None
    times = [point["t"] for point in points]
    max_error = [0.0] * JOINT_COUNT
    final_error = None
    for at, q in rows:
        if at > times[-1]:
            continue
        index = 1
        while index < len(times) - 1 and times[index] < at:
            index += 1
        left, right = times[index - 1], times[index]
        ratio = 0.0 if right <= left else max(0.0, min(1.0, (at - left) / (right - left)))
        target = [a + (b - a) * ratio for a, b in zip(points[index - 1]["q"], points[index]["q"])]
        errors = [abs(math.degrees(value - wanted)) for value, wanted in zip(q, target)]
        max_error = [max(a, b) for a, b in zip(max_error, errors)]
        final_error = errors
    return {"max_tracking_error_deg": [round(value, 3) for value in max_error],
            "final_tracking_error_deg": None if final_error is None
            else [round(value, 3) for value in final_error],
            "samples": len(rows)}
