"""Pure helpers for the unified control panel.

No ROS imports here so the plan/ticket/authorisation logic can be exercised
offline.  The single planning authority stays ``joint_waypoint_plan.compile_plan``;
this module only normalises, freezes and describes its output.  Both the
historical four-axis plan version and the unified five-axis version are
understood, and a document is never silently promoted between them.
"""
from __future__ import annotations

import hashlib
import json
import math
import secrets
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from joint_waypoint_plan import compile_plan, joint_names
import joint_waypoint_plan as _planner
from point_to_point import effective_document,EXTRA_SECONDS

# The peak reference speed belongs to the protected planner; never hard-code it
# here, otherwise the UI/tests drift when the commissioning baseline changes.
REFERENCE_SPEED_RAD_S = float(getattr(_planner, "REFERENCE_SPEED_RAD_S", 0.02))

# Legacy four-axis names stay importable for old callers; the production panel
# picks its list from ``joint_count`` and never assumes four here.
JOINT_NAMES = _planner.FOUR_AXIS_JOINT_NAMES
FIVE_AXIS_JOINT_NAMES = _planner.FIVE_AXIS_JOINT_NAMES
FOUR_AXES = _planner.FOUR_AXES
FIVE_AXES = _planner.FIVE_AXES
PLAN_SCHEMA = _planner.SCHEMA
LEGACY_PLAN_SCHEMA = _planner.LEGACY_SCHEMA
J5_JOINT_NAME = _planner.FIVE_AXIS_JOINT_NAMES[-1]
STAGE_LABELS = {
    0: "未使能",
    1: "启动前检查",
    2: "预载",
    3: "使能中",
    4: "增益斜坡",
    5: "保持",
    6: "轨迹跟踪",
    7: "停止中",
    8: "已失能",
    9: "UNKNOWN（锁定）",
}
# Two seconds below the driver's finite 180s session watchdog.
SESSION_BUDGET_S = 178.0
CLIENT_RESERVE_S = 8.0
EXEC_HOLD_S = 1.0
HOLD_SECONDS = 5.0
PREVIEW_TTL_S = 60.0
POSE_DRIFT_DEG = 0.5
FEEDBACK_FRESH_S = 0.5
HEARTBEAT_TIMEOUT_S = 3.0
STOP_CONFIRM_TIMEOUT_S = 3.0
MAX_PLAN_BYTES = 65536
MAX_WAYPOINTS = 12
CONFIRMATION_KEYS = ("supported", "clearance_and_local_power_cut", "native_output_not_hardware_clamp")
COORDINATE_MODES = ("absolute_deg", "relative_start_deg")

# --------------------------------------------------- gravity FF wire protocol
# The driver publishes ``gravity_ff_state`` as one flat Float64MultiArray.  A
# new driver may attach an explicit ``key=value`` protocol marker; an old driver
# sends data only.  Exactly the documented 4/5-axis shapes are accepted, and a
# wrong dimension is reported explicitly instead of being silently dropped.
FF_STATE_SCHEMA = "slaihand_gravity_ff_state"
FF_PROTOCOL_VERSION = 2
FF_PROTOCOL = "slaihand_gravity_ff_state/2"
FF_LEGACY_PROTOCOL = "slaihand_gravity_ff_state/legacy"
FF_HEADER_FIELDS = 5
FF_FLOAT_FIELDS = 3

# ------------------------------------------- unified axis calibration workflow
# Panel-side, deliberately non-formal: manual readings only.  No motor zero is
# written, no motor command is produced, and hardware_mapping.yaml / URDF are
# never touched.  The workflow derives provisional candidates that a human must
# review and write through the normal authorised path.
AXIS_CALIBRATION_SCHEMA = "slaihand_axis_calibration_workflow/1"
AXIS_CAL_PHASES = ("zero", "positive", "lower", "upper")
AXIS_CAL_PHASE_LABELS = {"zero": "零位读数", "positive": "正方向移动",
                         "lower": "下限端点", "upper": "上限端点"}
AXIS_CAL_CONFIRMATION_KEYS = ("axis_calibration_supported", "axis_calibration_clearance")
AXIS_CAL_POSITIVE_MIN_DEG = 1.0
# A sign capture is a small manual move; a larger delta means the reading is a
# single-turn wrap/other axis movement and must not be turned into a sign.
AXIS_CAL_POSITIVE_MAX_DEG = 90.0


class PanelError(Exception):
    """Rejection with an explicit HTTP status and stable machine code."""

    def __init__(self, status, code, message, **extra):
        super().__init__(message)
        self.status = int(status)
        self.code = str(code)
        self.message = str(message)
        self.extra = extra

    def payload(self):
        body = {"ok": False, "accepted": False, "completed": False,
                "code": self.code, "message": self.message}
        body.update(self.extra)
        return body


def number(value):
    """Finite float or None; never 0 for a missing observation."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def clean_for_json(value):
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean_for_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_for_json(v) for v in value]
    if isinstance(value, int):
        return value
    return value


def stage_label(stage):
    return STAGE_LABELS.get(stage, "状态未知")


def gravity_ff_layout_metadata(layout=None, header=None):
    """Read the optional ``gravity_ff_state`` protocol marker, defensively.

    The C++ publisher labels ``layout.dim[0]`` with the schema string and puts
    the controlled axis count in ``size``; ``layout.dim[1]`` is labelled
    ``float`` with size 0/1.  Fake/older messages may instead carry
    ``key=value`` tokens in layout data, dimension labels or the header frame.
    Absence of every marker returns ``{}`` so the caller falls back to inferring
    the legacy/new shape from the payload length.
    """
    markers = {}

    def absorb(text):
        text = str(text or "").strip()
        if not text:
            return
        if text == FF_STATE_SCHEMA or text == FF_PROTOCOL:
            markers.setdefault("schema", FF_STATE_SCHEMA)
            markers.setdefault("version", str(FF_PROTOCOL_VERSION))
            return
        base = FF_STATE_SCHEMA.rsplit("/", 1)[0]
        if text.startswith(base + "/"):
            suffix = text[len(base) + 1:].strip()
            if suffix.isdigit():
                markers.setdefault("schema", FF_STATE_SCHEMA)
                markers.setdefault("version", suffix)
                return
            # An explicit but malformed schema is a rejection, not a hint to
            # fall back to length inference.
            markers.setdefault("invalid_schema", text)
            return
        for token in text.replace(";", ",").split(","):
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            key = key.strip().lower()
            if key:
                markers.setdefault(key, value.strip())

    dims = list(getattr(layout, "dim", None) or ()) if layout is not None else []
    for dim in dims:
        label = getattr(dim, "label", None)
        if not isinstance(label, str):
            continue
        stripped = label.strip()
        if stripped.lower() == "float":
            size = getattr(dim, "size", None)
            if isinstance(size, int) and not isinstance(size, bool) and size in (0, 1):
                markers.setdefault("float", str(size))
            else:
                markers.setdefault("invalid_float", str(size))
            continue
        if stripped == FF_STATE_SCHEMA or stripped == FF_PROTOCOL \
                or stripped.startswith(FF_STATE_SCHEMA.rsplit("/", 1)[0] + "/"):
            absorb(stripped)
            size = getattr(dim, "size", None)
            if isinstance(size, int) and not isinstance(size, bool) and size > 0:
                markers.setdefault("axes", str(size))
            continue
        absorb(stripped)
    if layout is not None:
        data = getattr(layout, "data", None)
        if isinstance(data, str):
            absorb(data)
        elif isinstance(data, (list, tuple)):
            for item in data:
                absorb(item)
    frame = getattr(header, "frame_id", None) if header is not None else None
    if isinstance(frame, str):
        absorb(frame)
    return markers


def _gravity_ff_shape(length, markers, joint_count=None):
    """Resolve one payload length to (axes, float block, hold extension)."""
    if "invalid_schema" in markers:
        return None, {"code": "FF_SCHEMA",
                      "reason": "前馈状态显式 schema=%s 非法；拒绝按长度回退"
                                % markers["invalid_schema"]}
    if "invalid_float" in markers:
        return None, {"code": "FF_PROTOCOL_LAYOUT",
                      "reason": "前馈状态 float 维度 size=%s 非法；只接受 0/1"
                                % markers["invalid_float"]}
    declared_axes = None
    if "axes" in markers:
        try:
            declared_axes = int(str(markers["axes"]).strip())
        except (TypeError, ValueError):
            return None, {"code": "FF_PROTOCOL_LAYOUT",
                          "reason": "前馈状态 layout axes 标记无效：%r" % markers["axes"]}
        if declared_axes not in (FOUR_AXES, FIVE_AXES):
            return None, {"code": "FF_PROTOCOL_LAYOUT",
                          "reason": "前馈状态 layout axes=%d 不是受控轴数 4/5" % declared_axes}
        if joint_count in (FOUR_AXES, FIVE_AXES) and declared_axes != joint_count:
            return None, {"code": "FF_AXIS_MISMATCH",
                          "reason": "前馈消息 axes=%d 与当前受控轴数 %d 不一致"
                                    % (declared_axes, joint_count)}
    declared_float = None
    if "float" in markers:
        value = str(markers["float"]).strip().lower()
        if value in ("1", "true", "yes", "on"):
            declared_float = True
        elif value in ("0", "false", "no", "off"):
            declared_float = False
        else:
            return None, {"code": "FF_PROTOCOL_LAYOUT",
                          "reason": "前馈状态 layout float 标记无效：%r" % markers["float"]}
    schema = markers.get("schema")
    if schema is not None and schema != FF_STATE_SCHEMA:
        return None, {"code": "FF_SCHEMA",
                      "reason": "前馈状态 schema=%s，期望 %s" % (schema, FF_STATE_SCHEMA)}
    version = markers.get("version")
    if version is not None and str(version).strip() != str(FF_PROTOCOL_VERSION):
        return None, {"code": "FF_VERSION",
                      "reason": "前馈状态协议版本=%s，只支持 %d"
                                % (version, FF_PROTOCOL_VERSION)}
    options = []
    for axes in (FOUR_AXES, FIVE_AXES):
        base = FF_HEADER_FIELDS + 4 * axes
        # hold extension: lock + gain + remaining + N target + N error + reason + command
        extension = 2 * axes + 5
        rest = length - base
        if rest == 0:
            options.append({"axes": axes, "float": False, "float_flag_only": False, "hold": False})
        elif rest == FF_FLOAT_FIELDS:
            options.append({"axes": axes, "float": True, "float_flag_only": False, "hold": False})
        elif rest == extension:
            options.append({"axes": axes, "float": False, "float_flag_only": False, "hold": True})
        elif rest == FF_FLOAT_FIELDS + extension:
            options.append({"axes": axes, "float": True, "float_flag_only": False, "hold": True})
        if axes == FOUR_AXES and rest == 1:
            # Historical 22-field payload: a lone gravity-float flag without the
            # configurable Kd/seconds block.  Still a four-axis message only.
            options.append({"axes": axes, "float": False, "float_flag_only": True, "hold": False})
    options = [option for option in options
               if (declared_axes is None or option["axes"] == declared_axes)
               and (declared_float is None or option["float"] == declared_float)]
    if not options:
        detail = "" if declared_axes is None else "（layout axes=%d）" % declared_axes
        return None, {"code": "FF_PROTOCOL_LENGTH",
                      "reason": "前馈状态长度 %d 不是已知的 4/5 轴布局%s；拒绝静默忽略"
                                % (length, detail)}
    if len(options) > 1:
        if joint_count in (FOUR_AXES, FIVE_AXES):
            hinted = [option for option in options if option["axes"] == joint_count]
            if len(hinted) == 1:
                options = hinted
        if len(options) > 1:
            return None, {"code": "FF_AMBIGUOUS",
                          "reason": "前馈状态长度 %d 可解释为多种布局且缺少协议标记" % length}
    return options[0], None


def parse_gravity_ff_state(values, joint_count=None, layout=None, header=None):
    """Decode one ``gravity_ff_state`` payload without padding or truncation.

    Returns ``(state, error)``.  ``error`` is a stable ``{"code", "reason"}``
    dict when the dimension or protocol marker is wrong; the caller must store
    that as an explicit rejection instead of keeping an older value.  Four-axis
    payloads (21/22/24/34/37 fields) stay readable for compatibility while the
    five-axis payloads (40/43 fields) expose exactly five hold target/error
    values.  No ROS types are required here.
    """
    try:
        data = [float(value) for value in values]
    except (TypeError, ValueError):
        return None, {"code": "FF_PROTOCOL_DATA",
                      "reason": "驱动前馈状态包含非数值，拒绝解析", "length": None}
    if any(not math.isfinite(value) for value in data):
        return None, {"code": "FF_PROTOCOL_DATA",
                      "reason": "驱动前馈状态包含 NaN/Inf，拒绝解析", "length": len(data)}
    markers = gravity_ff_layout_metadata(layout, header)
    shape, error = _gravity_ff_shape(len(data), markers, joint_count)
    if error is not None:
        error = dict(error)
        error.setdefault("length", len(data))
        error.setdefault("axes", None)
        return None, error
    axes = shape["axes"]
    base = FF_HEADER_FIELDS + 4 * axes
    cursor = base
    float_full = shape["float"]
    float_flag_only = shape["float_flag_only"]
    has_float = float_full or float_flag_only
    gravity_float = False
    float_kd = 5.0
    float_seconds = 5.0
    if has_float:
        gravity_float = data[cursor] >= 0.5
        if float_full:
            float_kd = data[cursor + 1]
            float_seconds = data[cursor + 2]
            cursor += FF_FLOAT_FIELDS
        else:
            cursor += 1
    state = {
        "axes": axes, "length": len(data),
        "protocol": FF_PROTOCOL if (markers.get("schema") == FF_STATE_SCHEMA
                                    or axes == FIVE_AXES) else FF_LEGACY_PROTOCOL,
        "protocol_source": "layout" if markers else "length",
        "alpha": data[0], "ramp": data[1], "active": data[2] >= 0.5,
        "stage": int(data[3]), "command_id": int(data[4]),
        "requested": data[5:5 + axes],
        "encoded": data[5 + axes:5 + 2 * axes],
        "applied": data[5 + 2 * axes:5 + 3 * axes],
        "last_written": data[5 + 3 * axes:base],
        "float_available": has_float, "gravity_float": gravity_float,
        "float_kd": float_kd, "float_seconds": float_seconds,
        "float_configurable": float_full,
        "gravity_hold": False, "lock_state": 0, "hold_gain_scale": 0.0,
        "remaining_s": None, "hold_target": None, "hold_error": None,
        "lock_command_id": None, "lock_reason_code": 0,
    }
    if shape["hold"]:
        offset = cursor
        lock_state = int(data[offset])
        state.update({
            "lock_state": lock_state,
            "hold_gain_scale": data[offset + 1],
            "remaining_s": data[offset + 2],
            "hold_target": data[offset + 3:offset + 3 + axes],
            "hold_error": data[offset + 3 + axes:offset + 3 + 2 * axes],
            "lock_reason_code": int(data[offset + 3 + 2 * axes]),
            "lock_command_id": int(data[offset + 4 + 2 * axes]),
            "gravity_hold": lock_state in (2, 3),
        })
    return state, None


def validate_axis_calibration_confirmations(value):
    """Server-side support/clearance acknowledgement for the axis workflow."""
    if not isinstance(value, dict):
        raise PanelError(403, "AXIS_CAL_CONFIRMATION",
                         "轴标定流程必须在服务端提交支撑与净空确认")
    result = {}
    for key in AXIS_CAL_CONFIRMATION_KEYS:
        if value.get(key) is not True:
            raise PanelError(403, "AXIS_CAL_CONFIRMATION",
                             "确认缺失或未勾选：%s（服务端强制校验，不能只禁用按钮）" % key)
        result[key] = True
    return result


def axis_calibration_direction_candidate(zero_rad, positive_rad):
    """Candidate sign from one manual positive-direction move; never formal."""
    zero, positive = number(zero_rad), number(positive_rad)
    if zero is None or positive is None:
        raise PanelError(400, "CAL_ANGLE", "零位或正方向读数不是有限数值，不能推导符号")
    delta = positive - zero
    magnitude = abs(delta)
    if magnitude < math.radians(AXIS_CAL_POSITIVE_MIN_DEG):
        raise PanelError(409, "CAL_DIRECTION_AMBIGUOUS",
                         "正方向移动 %.3f° 小于 %.1f°；无法判断方向，不编造符号"
                         % (math.degrees(magnitude), AXIS_CAL_POSITIVE_MIN_DEG))
    if magnitude > math.radians(AXIS_CAL_POSITIVE_MAX_DEG):
        raise PanelError(409, "CAL_SINGLE_TURN_JUMP",
                         "正方向移动 %.3f° 超过 %.0f°；疑似单圈跳变/换向，拒绝作为符号证据"
                         % (math.degrees(magnitude), AXIS_CAL_POSITIVE_MAX_DEG))
    return {"delta_rad": delta, "delta_deg": math.degrees(delta),
            "candidate_sign": 1 if delta > 0 else -1,
            "provisional": True, "formal_mapping_written": False,
            "note": "候选符号只来自一次人工正方向移动；未写 hardware_mapping.yaml，不是正式标定"}


def axis_calibration_limits_candidate(lower_rad, upper_rad, zero_rad=None):
    """Finite ordered endpoint candidate from manual readings; never formal."""
    lower, upper = number(lower_rad), number(upper_rad)
    if lower is None or upper is None:
        raise PanelError(400, "CAL_ANGLE", "下限或上限读数不是有限数值，不能推导区间")
    if lower >= upper:
        raise PanelError(409, "CAL_LIMIT_ORDER",
                         "限位端点顺序无效：下限 %.3f° 必须小于上限 %.3f°；拒绝伪造区间"
                         % (math.degrees(lower), math.degrees(upper)))
    zero = number(zero_rad)
    return {"lower_rad": lower, "upper_rad": upper,
            "span_rad": upper - lower, "span_deg": math.degrees(upper - lower),
            "zero_inside": None if zero is None else lower <= zero <= upper,
            "provisional": True, "formal_mapping_written": False,
            "note": "端点只是本次人工读数；未写 hardware_mapping.yaml 或 URDF，不是正式有限限位"}


def load_limits(path, names=None):
    """Bounded revolute joints from the installed description package.

    ``names`` defaults to the historical J1-J4 list so existing four-axis
    callers keep their exact limits source.  A five-axis caller must pass the
    five names explicitly and provide J5 bounds from an authorised source.
    """
    path = Path(path)
    root = ET.parse(path).getroot()
    bounds = []
    for name in tuple(names or JOINT_NAMES):
        joint = root.find("joint[@name='%s']" % name)
        if joint is None or joint.get("type") != "revolute":
            raise ValueError("URDF 缺少受限转动关节 %s" % name)
        limit = joint.find("limit")
        if limit is None:
            raise ValueError("URDF 关节 %s 缺少 limit" % name)
        lower, upper = number(limit.get("lower")), number(limit.get("upper"))
        if lower is None or upper is None or lower >= upper:
            raise ValueError("URDF 关节 %s 限位无效" % name)
        bounds.append((lower, upper))
    return bounds


# --------------------------------------------------------------------- J5
# ``hardware_mapping.yaml`` is a hardware record, not a calibration generator.
# These readers only report what is actually written there; missing/null values
# stay missing and are never replaced by zeros or old no_end_load data.
def _mapping_scalar(value):
    if value is None:
        return None
    text = str(value).strip()
    if not text or text.lower() in ("null", "~", "none"):
        return None
    return text


def parse_j5_mapping(text):
    """Parse only the ``slaihand_joint5`` record from the mapping YAML text.

    A deliberately tiny reader: the file is a flat hardware ledger and using a
    full YAML dependency here would add no authority.
    """
    record = {}
    inside = False
    for raw in str(text or "").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line.startswith("  slaihand_joint") and line.strip().endswith(":"):
            inside = line.strip()[:-1] == J5_JOINT_NAME
            continue
        if not inside or ":" not in line:
            continue
        key, value = line.strip().split(":", 1)
        key, value = key.strip(), _mapping_scalar(value)
        if value is None:
            record[key] = None
            continue
        if value.startswith("[") and value.endswith("]"):
            parts = [part.strip() for part in value[1:-1].split(",") if part.strip()]
            record[key] = parts
        elif value.lower() in ("true", "false"):
            record[key] = value.lower() == "true"
        else:
            try:
                record[key] = int(value) if value.lstrip("+-").isdigit() else float(value)
            except ValueError:
                record[key] = value.strip("'\"")
    return record


def _first_number(record, keys):
    for key in keys:
        if key in record:
            value = number(record.get(key))
            if value is not None:
                return value
    return None


def _j5_limits(record):
    values = record.get("limits_rad")
    if isinstance(values, (list, tuple)) and len(values) == 2:
        lower, upper = number(values[0]), number(values[1])
        if lower is not None and upper is not None and lower < upper:
            return (lower, upper)
    lower = _first_number(record, ("lower_rad", "min_rad", "limit_min_rad"))
    upper = _first_number(record, ("upper_rad", "max_rad", "limit_max_rad"))
    if lower is not None and upper is not None and lower < upper:
        return (lower, upper)
    return None


def j5_motion_qualification(record, *, identity_payload=None, identity_fresh=False,
                            mapping_sha256=None):
    """Judge whether J5 may join a position plan, from observable evidence only.

    All checks are explicit and additive.  Nothing here writes or invents a
    calibration value: a null sign/zero/limit, TIMEOUT0, a stale identity or a
    missing profile each produce a stable reason code.
    """
    reasons = []
    observations = {"joint": J5_JOINT_NAME,
                    "mapping_sha256": mapping_sha256,
                    "identity_present": bool(identity_payload),
                    "calibration_status": (record or {}).get("calibration_status"),
                    "calibration_qualified": False, "feedback_required": True}
    record = record if isinstance(record, dict) else {}
    identity = identity_payload if isinstance(identity_payload, dict) else {}

    if not identity_fresh or not identity:
        reasons.append("J5_IDENTITY_NOT_READY: 驱动运行身份未就绪/已过期，无法确认 J5 属于本次受控轴")
    else:
        # Contract for the C++ owner: publish j5_motion_qualified=true (or
        # j5_motion_authorized=true) and, when present, active_axes=5.  A
        # four-axis driver identity must not pass.  Geometry is deliberately a
        # separate flag because position motion needs calibration, not CAD.
        declared = any(identity.get(key) is True for key in
                       ("j5_motion_qualified", "j5_motion_authorized"))
        active = number(identity.get("active_axes"))
        observations["active_axes"] = None if active is None else int(active)
        observations["j5_declared"] = bool(declared)
        if not declared:
            reasons.append("J5_IDENTITY_MISSING: 运行身份未声明 J5 运动资格"
                           "（需要 j5_motion_qualified=true）")
        elif active is not None and int(active) != FIVE_AXES:
            reasons.append("J5_IDENTITY_MISSING: 运行身份 active_axes=%d，不是五轴" % int(active))
        if mapping_sha256 is not None and identity.get("mapping_sha256") != mapping_sha256:
            reasons.append("J5_MAPPING_MISMATCH: 运行身份 mapping_sha256 与当前 hardware_mapping.yaml 不一致")

    sign = number(record.get("hardware_sign"))
    if sign not in (1, -1):
        reasons.append("J5_SIGN_MISSING: hardware_mapping.yaml 的 J5 hardware_sign 未标定")
    zero = number(record.get("zero_offset_rad"))
    if zero is None:
        reasons.append("J5_ZERO_MISSING: hardware_mapping.yaml 的 J5 zero_offset_rad 未标定")
    limits = _j5_limits(record)
    if limits is None:
        reasons.append("J5_LIMIT_MISSING: J5 有限限位未标定，不能规划位置运动")

    pmax = _first_number(record, ("commissioning_mit_pmax_rad", "pmax_rad"))
    vmax = _first_number(record, ("commissioning_mit_vmax_rad_s", "vmax_rad_s"))
    tmax = _first_number(record, ("commissioning_mit_tmax_nm", "tmax_nm"))
    if pmax is None or vmax is None or tmax is None or min(pmax, vmax, tmax) <= 0:
        reasons.append("J5_PROFILE_MISSING: J5 PMAX/VMAX/TMAX 未记录或无效")
    mode = _first_number(record, ("commissioning_control_mode", "mode"))
    if mode is None or int(mode) != 1:
        reasons.append("J5_MODE_INVALID: J5 控制模式不是 MIT(mode=1)")
    timeout = _first_number(record, ("commissioning_timeout_count", "timeout_count", "timeout"))
    if timeout is None or timeout <= 0:
        reasons.append("J5_TIMEOUT_INVALID: J5 驱动看门狗 TIMEOUT 未设置（TIMEOUT0 不合格）")

    observations.update({
        "calibration_qualified": sign in (1, -1) and zero is not None and limits is not None,
        "hardware_sign": sign, "zero_offset_rad": zero,
        "limits_rad": list(limits) if limits is not None else None,
        "profile": {"pmax_rad": pmax, "vmax_rad_s": vmax, "tmax_nm": tmax},
        "control_mode": int(mode) if mode is not None else None,
        "timeout_count": timeout,
        "identity_sha256": identity.get("j5_geometry_sha256"),
    })
    return {"joint": J5_JOINT_NAME, "required": True, "qualified": not reasons,
            "reasons": list(reasons), "observations": observations,
            "limits_rad": list(limits) if limits is not None else None}


def j5_feedback_observation(name, axes, dynamic_age, feedback_valid, positions):
    """Only fresh, valid, finite J5 feedback counts; missing is never 0°."""
    reasons = []
    values = axes.get(name) if isinstance(axes, dict) else None
    values = values if isinstance(values, dict) else {}
    if dynamic_age is None:
        reasons.append("J5_FEEDBACK_STALE: 没有收到动态关节状态（失联），J5 不能显示为 0/就绪")
    elif dynamic_age > FEEDBACK_FRESH_S:
        reasons.append("J5_FEEDBACK_STALE: J5 反馈陈旧（%.2f 秒）" % dynamic_age)
    if not values:
        reasons.append("J5_FEEDBACK_MISSING: 同一帧动态状态中没有 J5，不能用 0 补齐")
    if not isinstance(feedback_valid, (list, tuple)) or len(feedback_valid) < FIVE_AXES \
            or feedback_valid[FOUR_AXES] is not True:
        reasons.append("J5_FEEDBACK_INVALID: J5 feedback_valid 不是 1 或整体帧不完整")
    position = None
    if isinstance(positions, (list, tuple)) and len(positions) >= FIVE_AXES:
        position = number(positions[FOUR_AXES])
    if position is None:
        reasons.append("J5_FEEDBACK_INVALID: 没有有限 J5 位置，不能用 0 代替")
    return {"joint": name, "position_rad": position,
            "feedback_age_s": number(values.get("feedback_age")),
            "feedback_valid": values.get("feedback_valid"),
            "motor_state": number(values.get("motor_state")),
            "reasons": reasons}


def unified_motion_preflight(*, joint_count, names, axes, dynamic_age, feedback_valid,
                             positions, identity_payload=None, identity_fresh=False,
                             mapping_text=None, mapping_sha256=None):
    """One gate for every motion entry in the panel.

    Four-axis mode keeps its historical behaviour.  Five-axis mode refuses
    *all* motion unless J5 calibration, finite limits, MIT mode, a non-zero
    watchdog, a J5-bearing runtime identity and fresh valid J5 feedback are all
    present.  The returned reason list is what the UI must show.
    """
    names = list(names)
    if joint_count == FOUR_AXES:
        return {"ok": True, "axes": FOUR_AXES, "joint_names": names, "reasons": [],
                "j5": {"required": False, "qualified": True, "reasons": [],
                       "observations": {"joint": J5_JOINT_NAME, "not_required_in_four_axis": True}},
                "feedback": None,
                "gravity": gravity_model_identity(identity_payload, identity_fresh,
                                                  load_id=(identity_payload or {}).get("load_id"),
                                                  j5_qualified=True,
                                                  j5_geometry_qualified=True)}
    if joint_count != FIVE_AXES:
        raise ValueError("只支持4轴或5轴，收到 %r" % (joint_count,))
    record = parse_j5_mapping(mapping_text)
    j5 = j5_motion_qualification(record, identity_payload=identity_payload,
                                 identity_fresh=identity_fresh, mapping_sha256=mapping_sha256)
    feedback = j5_feedback_observation(names[FOUR_AXES], axes, dynamic_age,
                                       feedback_valid, positions)
    reasons = list(j5["reasons"]) + list(feedback["reasons"])
    identity = identity_payload if isinstance(identity_payload, dict) else {}
    geometry = (identity.get("j5_geometry") is True
                and isinstance(identity.get("j5_geometry_sha256"), str)
                and len(identity.get("j5_geometry_sha256") or "") == 64
                and not record.get("virtual_placeholder")
                and "placeholder" not in str(record.get("geometry_status") or ""))
    load_id = identity.get("load_id")
    gravity = gravity_model_identity(identity_payload, identity_fresh, load_id=load_id,
                                     j5_qualified=j5["qualified"],
                                     j5_geometry_qualified=geometry)
    return {"ok": not reasons, "axes": FIVE_AXES, "joint_names": names,
            "reasons": reasons, "j5": j5, "feedback": feedback, "gravity": gravity}


def gravity_model_identity(identity_payload=None, identity_fresh=False, load_id=None,
                           j5_qualified=False, j5_geometry_qualified=False):
    """Formal gravity is allowed only for a qualified five-axis/load identity.

    ``j5_qualified`` means J5 position calibration/profile passed.  Formal
    gravity additionally needs J5 geometry/COM evidence; a frame-only
    placeholder or a new 3.5 kg load without a measured profile stays
    PROVISIONAL.  The old ``no_end_load`` declaration is reported as stale, not
    reused as if it described the new assembly.
    """
    reasons = []
    if not identity_fresh:
        reasons.append("IDENTITY_NOT_READY: 运行身份未就绪，不能确认负载档案")
    if not isinstance(load_id, str) or not load_id:
        reasons.append("LOAD_ID_MISSING: 运行身份缺少 load_id")
    elif load_id == "no_end_load":
        reasons.append("LOAD_ID_STALE: no_end_load 是旧四轴无负载档案，不能冒充 J5+3.5kg 五轴负载")
    elif "provisional" in load_id.lower():
        reasons.append("LOAD_ID_PROVISIONAL: load_id=%s 仍是候选声明，几何/质心实测前不能做正式重力"
                       % load_id)
    if not j5_qualified:
        reasons.append("J5_MOTION_UNQUALIFIED: J5 标定/限位/profile 未全部合格")
    if not j5_geometry_qualified:
        reasons.append("J5_GEOMETRY_UNQUALIFIED: J5 几何/安装变换/质心未合格"
                       "（frame-only placeholder 或缺少 j5_geometry_sha256）")
    status = "PROVISIONAL" if reasons else "QUALIFIED"
    return {"status": status, "formal_gravity_allowed": not reasons,
            "provisional": bool(reasons), "load_id": load_id, "reasons": reasons}



def degrees_limits(bounds):
    return [[round(math.degrees(lo), 3), round(math.degrees(hi), 3)] for lo, hi in bounds]


def normalize_document(document):
    """Shape-only normalisation, with an explicit plan version.

    Range/speed/timing rules stay in compile_plan.  A missing ``schema`` is the
    legacy four-axis version and is written back explicitly so later stages and
    saved files can never mistake it for a five-axis plan.
    """
    schema, axes = _planner.document_shape(document)
    mode = document["coordinates"]
    if mode not in COORDINATE_MODES:
        raise ValueError("coordinates 必须是 absolute_deg 或 relative_start_deg")
    rows = document["waypoints"]
    if not isinstance(rows, list) or not 1 <= len(rows) <= MAX_WAYPOINTS:
        raise ValueError("需要1～%d个路点" % MAX_WAYPOINTS)
    waypoints = []
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict) or set(row) != {"degrees", "move_s", "dwell_s"}:
            raise ValueError("第%d点必须且仅包含 degrees、move_s、dwell_s" % index)
        values = row["degrees"]
        if not isinstance(values, list) or len(values) != axes:
            if axes == FOUR_AXES:
                raise ValueError("第%d点需要J1～J4四个角度；五轴计划必须显式声明 schema=%s"
                                 "并提供五个角度" % (index, PLAN_SCHEMA))
            raise ValueError("第%d点五轴计划需要J1～J5五个角度（schema=%s）" % (index, PLAN_SCHEMA))
        numbers = []
        for value in list(values) + [row["move_s"], row["dwell_s"]]:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("第%d点角度与时间必须为有限数值" % index)
            numbers.append(float(value))
        waypoints.append({"degrees": numbers[:axes], "move_s": numbers[axes],
                          "dwell_s": numbers[axes + 1]})
    return {"schema": schema, "coordinates": mode, "waypoints": waypoints}


def canonical(document):
    return json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def digest(document):
    return hashlib.sha256(canonical(document).encode("utf-8")).hexdigest()


def document_axes(document):
    """Axis count demanded by a normalised (or raw) plan document."""
    return _planner.document_shape(document)[1]


def compile_document(document, start_rad, bounds):
    """Normalise then hand the plan to the one production validator."""
    normalised = normalize_document(document)
    axes = document_axes(normalised)
    if len(start_rad) != axes or any(number(q) is None for q in start_rad):
        raise ValueError("没有完整%d轴反馈，不能预览或执行" % axes)
    if len(bounds) != axes:
        raise ValueError("限位源有%d轴，计划需要%d轴；拒绝执行" % (len(bounds), axes))
    points, windows = compile_plan(normalised, [float(q) for q in start_rad],
                                   [tuple(b) for b in bounds])
    return normalised, points, windows


def freeze_absolute(document, start_rad, bounds):
    """Freeze a plan to absolute software degrees (what will actually execute)."""
    normalised, _, windows = compile_document(document, start_rad, bounds)
    rows = []
    for row, window in zip(normalised["waypoints"], windows):
        rows.append({"degrees": [math.degrees(q) for q in window["target_rad"]],
                     "move_s": row["move_s"], "dwell_s": row["dwell_s"]})
    frozen = {"schema": normalised["schema"], "coordinates": "absolute_deg", "waypoints": rows}
    frozen_points, frozen_windows = compile_plan(frozen, [float(q) for q in start_rad],
                                                 [tuple(b) for b in bounds])
    return frozen, frozen_points, frozen_windows


def budget(plan_seconds, hold_seconds=EXEC_HOLD_S):
    limit = SESSION_BUDGET_S - CLIENT_RESERVE_S - float(hold_seconds)
    return {"plan_seconds": round(float(plan_seconds), 3), "hold_seconds": float(hold_seconds),
            "reserve_seconds": CLIENT_RESERVE_S, "session_budget_seconds": SESSION_BUDGET_S,
            "plan_limit_seconds": round(limit, 3),
            "ok": float(plan_seconds) <= limit + 1e-9}


def execution_mode(value):
    if value not in ('trajectory','point_to_point','calibration'):
        raise PanelError(400,'EXECUTION_MODE','未知执行模式')
    return value


def preview_payload(document, start_rad, bounds, limits_source=None, mode='trajectory'):
    mode=execution_mode(mode)
    normalised=normalize_document(document)
    axes=document_axes(normalised)
    if len(start_rad) != axes:
        raise ValueError("计划是%d轴，但当前反馈是%d轴；不补0、不截断" % (axes, len(start_rad)))
    if len(bounds) != axes:
        raise ValueError("计划是%d轴，但限位源是%d轴；拒绝预览" % (axes, len(bounds)))
    effective=effective_document(normalised) if mode=='point_to_point' else normalised
    if mode=='calibration':
        # Legacy calibration groups import the heavy capture/analysis modules;
        # keep that work off the normal five-axis panel startup path.
        from calibration_session import effective_document as calibration_document
        effective=calibration_document(normalised)
    frozen, points, windows = freeze_absolute(effective, start_rad, bounds)
    total = windows[-1]["end_s"]
    plan_budget = budget(total+(EXTRA_SECONDS if mode=='point_to_point' else 0))
    plan_budget['retiming_allowance_s']=EXTRA_SECONDS if mode=='point_to_point' else 0
    plan_budget['nominal_plan_seconds']=total
    if not plan_budget["ok"]:
        raise PanelError(400, "PLAN_BUDGET", "计划与保持及8秒余量超过178秒会话预算（上限 %.1f 秒）"
                         % plan_budget["plan_limit_seconds"])
    target_deg = [[round(math.degrees(q), 3) for q in window["target_rad"]] for window in windows]
    returns = all(abs(t - s) <= 1e-9 for t, s in zip(windows[-1]["target_rad"], start_rad))
    warnings = []
    if normalised["schema"] == LEGACY_PLAN_SCHEMA:
        warnings.append("这是旧四轴计划（schema=%s），只在四轴模式执行；它不是五轴计划，不会补J5。"
                        % LEGACY_PLAN_SCHEMA)
    if axes == FIVE_AXES:
        warnings.append("五轴计划：J1～J5 必须来自同一批新鲜反馈；任一轴资格/反馈不满足时整臂拒绝运动。")
    if mode=='calibration':
        warnings.append('重力采样：每段移动至少7.5秒、停留至少6秒；逐点采集至少2秒自由静态窗口，按实际角度记录。α必须为0；0.5°精度另行评分，不修改质量档案。')
    if mode=='point_to_point':
        warnings.append('点到点：停留至少1秒；终点误差≤0.5°且速度≤0.05rad/s，连续稳定0.5秒。自适应最多额外6秒，完成后失能。')
    if not returns:
        warnings.append("计划终点没有回到本次起点：动作完成即失能，不会在终点持续保持。")
    if normalised["coordinates"] == "relative_start_deg":
        warnings.append("relative_start_deg 每个点都相对本次起点（预览起点），不逐点累加；执行时按实际起点重新校验。")
    rows = []
    for index, (window, deg) in enumerate(zip(windows, target_deg), 1):
        rows.append({"index": index, "arrival_s": round(window["arrival_s"], 3),
                     "end_s": round(window["end_s"], 3),
                     "move_s": effective["waypoints"][index - 1]["move_s"],
                     "dwell_s": effective["waypoints"][index - 1]["dwell_s"],
                     "target_deg": deg})
    trajectory = [{"at_s": round(t, 3), "degrees": [round(math.degrees(q), 3) for q in qs]}
                  for t, qs in points]
    return {"schema": normalised["schema"], "axes": axes,
            "joint_names": list(joint_names(axes)),
            "mode": "absolute_deg", "execution_mode":mode, "source_mode": normalised["coordinates"],
            "start_deg": [round(math.degrees(q), 3) for q in start_rad],
            "limits_deg": degrees_limits(bounds), "limits_source": limits_source,
            "frozen": frozen, "frozen_sha256": digest(frozen),
            "source_sha256": digest(normalised), "source": normalised,
            "waypoints": rows, "trajectory": trajectory,
            "total_s": round(total, 3), "budget": plan_budget,
            "returns_to_start": returns, "warnings": warnings}


def validate_confirmations(value):
    if not isinstance(value, dict):
        raise PanelError(400, "CONFIRMATION_REQUIRED", "执行前必须在服务端提交三项显式确认")
    result = {}
    if value.get('free_static') is True:
        result['free_static'] = True
    for key in CONFIRMATION_KEYS:
        if value.get(key) is not True:
            raise PanelError(403, "CONFIRMATION_REQUIRED",
                             "确认缺失或未勾选：%s（必须由服务端校验，不能只禁用按钮）" % key)
        result[key] = True
    return result


class TicketStore:
    """One short-lived, single-use preview ticket at a time."""

    def __init__(self, ttl=PREVIEW_TTL_S, clock=time.monotonic, wall=time.time):
        self._lock = threading.Lock()
        self._ttl = float(ttl)
        self._clock = clock
        self._wall = wall
        self._ticket = None

    def issue(self, payload):
        now, wall = self._clock(), self._wall()
        ticket = dict(payload)
        ticket.update(id=secrets.token_hex(16), created_at=now, created_wall=wall,
                      expires_at=now + self._ttl, expires_wall=wall + self._ttl,
                      used=False, invalid_reason=None)
        with self._lock:
            self._ticket = ticket
        return self.describe()

    def peek(self, ticket_id=None):
        with self._lock:
            ticket = self._ticket
            if ticket is None or (ticket_id is not None and ticket["id"] != ticket_id):
                return None
            return self._describe_locked(ticket)

    def consume(self, ticket_id):
        with self._lock:
            ticket = self._ticket
            if ticket is None or ticket["id"] != ticket_id:
                raise PanelError(409, "TICKET_UNKNOWN", "预览票根不存在，请重新预览")
            if ticket["used"]:
                raise PanelError(409, "TICKET_USED", "预览票根已使用或已失效，请重新预览")
            if self._clock() >= ticket["expires_at"]:
                ticket["used"] = True
                ticket["invalid_reason"] = "预览已过期"
                raise PanelError(409, "TICKET_EXPIRED", "预览已过期，请重新预览")
            ticket["used"] = True
            return self._describe_locked(ticket)

    def drop(self, reason="预览已失效"):
        with self._lock:
            if self._ticket is not None and not self._ticket["used"]:
                self._ticket["used"] = True
                self._ticket["invalid_reason"] = reason

    def is_live(self, ticket_id=None):
        with self._lock:
            ticket = self._ticket
            if ticket is None or ticket["used"]:
                return False
            if ticket_id is not None and ticket["id"] != ticket_id:
                return False
            return self._clock() < ticket["expires_at"]

    @property
    def ttl(self):
        return self._ttl

    def describe(self, ticket_id=None):
        with self._lock:
            ticket = self._ticket
            if ticket is None or (ticket_id is not None and ticket["id"] != ticket_id):
                return None
            return self._describe_locked(ticket)

    def _describe_locked(self, ticket):
        remaining = max(0.0, ticket["expires_at"] - self._clock())
        return {"id": ticket["id"], "source_sha256": ticket["source_sha256"],
                "execution_mode":ticket.get('execution_mode','trajectory'),
                "frozen_sha256": ticket["frozen_sha256"], "mode": ticket["mode"],
                "identity_sha256": ticket.get("identity_sha256"),
                "teaching_id": ticket.get("teaching_id"),
                "speed_factor": ticket.get("speed_factor"),
                "stop_generation": ticket.get("stop_generation"),
                "boot_id": ticket.get("boot_id"),
                "frozen": ticket["frozen"], "preview": ticket["preview"],
                "start_rad": list(ticket["start_rad"]), "start_deg": ticket["start_deg"],
                "created_at": ticket["created_wall"], "expires_at": ticket["expires_wall"],
                "remaining_s": round(remaining, 1), "expired": remaining <= 0.0,
                "used": bool(ticket["used"]), "invalid_reason": ticket["invalid_reason"],
                "live": (not ticket["used"]) and remaining > 0.0}


def project_result(result):
    """Display projection of the protected client's result file."""
    if not isinstance(result, dict):
        return None
    rows = result.get("waypoint_results")
    waypoints = []
    if isinstance(rows, list):
        for row in rows:
            if not isinstance(row, dict):
                continue
            target = row.get("target_rad") or []
            waypoints.append({
                "arrival_s": number(row.get("arrival_s")), "end_s": number(row.get("end_s")),
                "target_deg": [round(math.degrees(q), 3) for q in target if number(q) is not None],
                "samples": row.get("samples"), "sampled_span_s": number(row.get("sampled_span_s")),
                "max_error_deg": [number(v) for v in (row.get("max_error_deg") or [])],
                "passed": row.get("passed") is True,
                'max_velocity_rad_s':row.get('max_velocity_rad_s')})
    waypoints_observed = result.get("waypoints_observed")
    if waypoints:
        accuracy = bool(waypoints_observed is True and all(row["passed"] for row in waypoints))
    else:
        accuracy = None
    error_code = result.get("trajectory_error_code")
    action_passed = result.get("standard_jtc_action_succeeded") is True and error_code == 0
    stop_confirmed = result.get("stop_observations_confirmed") is True
    calibration=result.get('calibration') if result.get('mode')=='calibration' else None
    capture_ok = (isinstance(calibration,dict) and calibration.get('campaign_complete') is True
                  and action_passed)
    teaching_playback = result.get('teaching_playback') if result.get('mode')=='drag_teaching' else None
    playback_tracking = result.get('playback_tracking') if result.get('mode')=='drag_teaching' else None
    supported_capture = result.get('supported_capture') if result.get('mode')=='supported_static_capture' else None
    capture_sample_ok = bool(isinstance(supported_capture, dict)
                             and supported_capture.get('sample_valid') is True)
    overall = bool(result.get("reason") == "finite_trial_complete" and stop_confirmed
                   and (capture_ok if result.get('mode')=='calibration'
                        else (capture_sample_ok if result.get('mode')=='supported_static_capture'
                              else accuracy is not False)))
    return {"request_id": result.get("request_id"), "mode": result.get("mode"),
            "reason": result.get("reason"), "command_id": result.get("command_id"),
            "trajectory_error_code": error_code,
            "trajectory_error_string": result.get("trajectory_error_string"),
            "concurrent_control_error": result.get("concurrent_control_error"),
            "point_to_point":result.get('point_to_point'),
            "calibration":clean_for_json(calibration),
            "calibration_error":result.get('calibration_error'),
            "teaching_playback":clean_for_json(teaching_playback),
            "playback_tracking":clean_for_json(playback_tracking),
            "speed_factor":number(result.get('speed_factor')),
            "runtime_identity_sha256": result.get("runtime_identity_sha256"),
            "supported_capture": clean_for_json(supported_capture),
            "capture_sample_ok": capture_sample_ok,
            "capture_alpha": (supported_capture or {}).get('alpha')
            if isinstance(supported_capture, dict) else None,
            "capture_fit_eligible": (supported_capture or {}).get('fit_eligible')
            if isinstance(supported_capture, dict) else None,
            "capture_raw_file": (supported_capture or {}).get('raw_file')
            if isinstance(supported_capture, dict) else None,
            "float_settings": clean_for_json(result.get("float_settings")),
            "phase": result.get("phase"),
            "arm_requested": result.get("arm_requested") is True,
            "trajectory_requested": result.get("trajectory_requested") is True,
            "trajectory_started": result.get("trajectory_started") is True,
            "stop_requested": result.get("stop_requested") is True,
            "phase_result": result.get("phase_result"),
            "stop_result": result.get("stop_result"),
            "prearm_timeout": result.get("prearm_timeout") is True,
            "prearm_reasons": clean_for_json(result.get("prearm_reasons") or []),
            "prearm_observations": clean_for_json(result.get("prearm_observations") or {}),
            "unknown_observed": result.get("unknown_observed") is True,
            "stop_error": result.get("stop_error"),
            "backend_arm_receipt": result.get("backend_arm_receipt"),
            "standard_jtc_action_succeeded": result.get("standard_jtc_action_succeeded") is True,
            "action_passed": action_passed,
            "waypoints_observed": waypoints_observed,
            "waypoint_results": waypoints,
            "accuracy_passed": accuracy,
            "stop_observations_confirmed": stop_confirmed,
            "observed_delta_range_deg": clean_for_json(result.get("observed_delta_range_deg")),
            "final_delta_from_start_deg": clean_for_json(result.get("final_delta_from_start_deg")),
            "actual_excursion_deg": clean_for_json(result.get("actual_excursion_deg")),
            "return_error_deg": clean_for_json(result.get("return_error_deg")),
            "requested_excursion_observed": result.get("requested_excursion_observed"),
            "initial_deg": [round(math.degrees(v), 3) for v in (result.get("initial_model_rad") or [])
                            if number(v) is not None],
            "plan_sha256": result.get("plan_sha256"),
            "operator_acknowledgements": result.get("operator_acknowledgements"),
            "trial_parameters": result.get("trial_parameters"),
            "overall_ok": overall}
