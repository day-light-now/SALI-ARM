"""当前位置静态采样：撤手确认协议、预览与基线资格（纯 Python，无 ROS）。

单一所有者流程：面板冻结当前姿态 → 受保护客户端做正常位置保持 → 面板校验
就绪/身份/命令/停止纪元后写一次性撤手确认文件 → 客户端校验后采集静态窗口 →
先停止确认再落盘。本模块只放协议与纯校验，运动仍由受保护客户端执行。
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import secrets
import time
from pathlib import Path

from drag_teaching import identity_fingerprint, normalize_identity

SCHEMA_ACK = "slaihand_supported_capture_ack/1"
SCHEMA_PREVIEW = "slaihand_supported_capture_preview/1"
ACK_FILENAME = "operator_disengage_ack.json"
ACK_TIMEOUT_S = 60.0
ACK_MAX_AGE_S = 10.0
HOLD_READY_TIMEOUT_S = 10.0
CAPTURE_SECONDS = 2.0
CAPTURE_SAMPLES = 10
CAPTURE_MAX_SECONDS = 10.0
CAPTURE_MIN_SAMPLES = 5
CAPTURE_MAX_SAMPLES = 100
CAPTURE_WINDOW_MARGIN_S = 3.0
SUPPORT = "free"
CONTACT = "none"
STAGE_HOLD = 5
STAGE_TRACK = 6
ACK_FIELDS = ("schema", "operation_id", "command_id", "boot_id", "identity_sha256",
              "panel_stop_generation", "nonce", "requested_at", "alpha", "stage",
              "hold_ready", "mode", "support", "contact", "operator_disengaged")
HEX64_RE = __import__("re").compile(r"^[0-9a-f]{64}$")
NONCE_RE = __import__("re").compile(r"^[0-9a-f]{16}$")
CONFIRMATION_KEY = "supported_static_capture"
START_REQUIRED = ("supported", "clearance_and_local_power_cut",
                  "native_output_not_hardware_clamp", CONFIRMATION_KEY)


class CaptureError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def json_sha256(payload):
    return hashlib.sha256(canonical(payload).encode("utf-8")).hexdigest()


def validate_start_confirmations(confirmations):
    if not isinstance(confirmations, dict):
        raise CaptureError("CONFIRMATION", "启动前必须提交确认")
    for key in START_REQUIRED:
        if confirmations.get(key) is not True:
            raise CaptureError("CONFIRMATION", "确认缺失：%s" % key)
    return dict(confirmations)


def validate_ack_confirmations(confirmations):
    if not isinstance(confirmations, dict) or confirmations.get("operator_disengaged") is not True:
        raise CaptureError("DISENGAGE_ACK", "需要操作员明确确认已撤手且防坠支撑不承重")
    return {"operator_disengaged": True}


def fit_baseline(alpha):
    value = float(alpha) if _finite(alpha) else None
    satisfied = value is not None and abs(value) < 1e-12
    if value is None:
        note = "未收到驱动前馈状态：基线未知，样本只能作诊断"
    elif satisfied:
        note = "α=0：普通位置保持基线满足，样本可进入既有静态拟合资格"
    else:
        note = ("α=%.3f：驱动普通位置保持已写入重力前馈，反馈力矩不是纯重力；"
                "样本可保存为诊断数据，但不进入拟合" % value)
    return {"alpha": value, "satisfied": bool(satisfied), "note": note}


def _normalize_ack(payload):
    if not isinstance(payload, dict):
        raise CaptureError("SHAPE", "撤手确认必须是 JSON 对象")
    if set(payload) != set(ACK_FIELDS):
        raise CaptureError("SCHEMA", "撤手确认字段与版本 %s 不匹配" % SCHEMA_ACK)
    if payload.get("schema") != SCHEMA_ACK:
        raise CaptureError("SCHEMA", "撤手确认 schema 不匹配")
    if not isinstance(payload.get("operation_id"), str) or not payload["operation_id"]:
        raise CaptureError("SCHEMA", "operation_id 无效")
    if not _int(payload.get("command_id")) or payload["command_id"] < 0:
        raise CaptureError("SCHEMA", "command_id 无效")
    if not isinstance(payload.get("boot_id"), str) or not payload["boot_id"]:
        raise CaptureError("SCHEMA", "boot_id 无效")
    if not isinstance(payload.get("identity_sha256"), str) or \
            not HEX64_RE.match(payload["identity_sha256"]):
        raise CaptureError("SCHEMA", "identity_sha256 无效")
    if not _int(payload.get("panel_stop_generation")) or payload["panel_stop_generation"] < 0:
        raise CaptureError("SCHEMA", "panel_stop_generation 无效")
    if not isinstance(payload.get("nonce"), str) or not NONCE_RE.match(payload["nonce"]):
        raise CaptureError("SCHEMA", "nonce 无效")
    if not _finite(payload.get("requested_at")):
        raise CaptureError("SCHEMA", "requested_at 无效")
    if not _finite(payload.get("alpha")):
        raise CaptureError("SCHEMA", "alpha 无效")
    if payload.get("stage") not in (STAGE_HOLD, STAGE_TRACK):
        raise CaptureError("STAGE", "撤手确认阶段的控制状态不是位置保持")
    if payload.get("hold_ready") is not True:
        raise CaptureError("NOT_READY", "撤手确认时四轴并非保持就绪")
    if payload.get("mode") != "position_hold":
        raise CaptureError("MODE", "撤手确认不是普通位置保持")
    if payload.get("support") != SUPPORT or payload.get("contact") != CONTACT:
        raise CaptureError("SUPPORT", "撤手确认必须声明自由、无接触")
    if payload.get("operator_disengaged") is not True:
        raise CaptureError("DISENGAGE_ACK", "缺少操作员已撤手确认")
    return dict(payload)


def build_ack(operation_id, command_id, boot_id, identity_sha256, panel_stop_generation,
              alpha, stage, hold_ready, nonce=None, requested_at=None):
    if not isinstance(nonce, str) or not NONCE_RE.match(nonce):
        nonce = secrets.token_hex(8)
    payload = {"schema": SCHEMA_ACK, "operation_id": str(operation_id),
               "command_id": int(command_id), "boot_id": str(boot_id),
               "identity_sha256": str(identity_sha256),
               "panel_stop_generation": int(panel_stop_generation),
               "nonce": nonce,
               "requested_at": float(time.time() if requested_at is None else requested_at),
               "alpha": float(alpha), "stage": int(stage),
               "hold_ready": hold_ready is True, "mode": "position_hold",
               "support": SUPPORT, "contact": CONTACT, "operator_disengaged": True}
    return _normalize_ack(payload)


def validate_ack(payload, expected):
    ack = _normalize_ack(payload)
    for key in ("operation_id", "command_id", "boot_id", "identity_sha256",
                "panel_stop_generation"):
        if expected.get(key) is None:
            raise CaptureError("EXPECTED", "内部缺少核对项 %s" % key)
        if ack[key] != expected[key]:
            raise CaptureError("IDENTITY", "撤手确认的 %s 与当前操作不一致" % key)
    age = abs(time.time() - ack["requested_at"])
    if age > ACK_MAX_AGE_S:
        raise CaptureError("STALE", "撤手确认已过期 %.1f 秒（上限 %.1f）" % (age, ACK_MAX_AGE_S))
    return ack


def read_ack_file(path):
    path = Path(path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise CaptureError("NOT_FOUND", "撤手确认文件不存在")
    except (OSError, ValueError) as error:
        raise CaptureError("CORRUPT", "撤手确认文件无法解析：%s" % error)
    return _normalize_ack(payload)


def write_ack_file(path, payload):
    """一次性写：同目录唯一临时文件 + fsync + rename，绝不部分可见。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    temporary = path.parent / (".%s.%s.tmp" % (path.name, secrets.token_hex(4)))
    descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise
    return path


def preview_payload(start_rad, identity, alpha, limits_deg, limits_source=None,
                   start_deg=None):
    normalized = normalize_identity(identity)
    baseline = fit_baseline(alpha)
    if start_deg is None:
        start_deg = [round(math.degrees(value), 3) for value in start_rad]
    session_estimate = HOLD_READY_TIMEOUT_S + ACK_TIMEOUT_S + CAPTURE_SECONDS + 4.0
    return {"schema": SCHEMA_PREVIEW, "execution_mode": "supported_capture",
            "start_rad": [round(float(value), 9) for value in start_rad],
            "start_deg": start_deg, "limits_deg": limits_deg,
            "limits_source": limits_source,
            "identity": {"boot_id": normalized["boot_id"],
                         "identity_sha256": identity_fingerprint(normalized),
                         "mapping_sha256": normalized["mapping_sha256"],
                         "model_sha256": normalized["model_sha256"],
                         "policy_sha256": normalized["policy_sha256"],
                         "mass_profile_sha256": normalized["mass_profile_sha256"],
                         "load_id": normalized["load_id"]},
            "gains": {"kp": normalized["kp"], "kd": normalized["kd"],
                      "reference_speed_rad_s": normalized["reference_speed_rad_s"]},
            "alpha": baseline["alpha"], "baseline": baseline,
            "timing": {"hold_ready_timeout_s": HOLD_READY_TIMEOUT_S,
                       "ack_timeout_s": ACK_TIMEOUT_S,
                       "capture_seconds": CAPTURE_SECONDS,
                       "capture_samples": CAPTURE_SAMPLES,
                       "session_estimate_s": round(session_estimate, 1)},
            "steps": [
                "结束自由拖动/示教并等待新失能确认，再用软件复位进入未使能",
                "预览冻结当前实际姿态，核对角度、运行身份、参数和 α 基线",
                "确认后启动普通位置保持（Kp/Kd），等待“保持已就绪”",
                "托稳后缓慢撤手；防坠支撑不承重后勾选并点击“已撤手，开始采样”",
                "客户端采集 2 秒静态窗口，先停止并确认新失能，再保存原始数据",
            ],
            "warnings": [baseline["note"],
                         "采样窗口要求四轴逐帧使能、新鲜、速度/位置稳定，命令 ID 不变；"
                         "不静态或身份变化会拒绝",
                         "本操作不是实机部署授权；现场由操作员确认支撑、净空与断电手段"]}


def validate_preview_payload(preview):
    if not isinstance(preview, dict) or preview.get("schema") != SCHEMA_PREVIEW:
        raise CaptureError("SCHEMA", "采样预览版本不匹配")
    return preview
