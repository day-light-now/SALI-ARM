"""Pure helpers for the four-axis control panel.

No ROS imports here so the plan/ticket/authorisation logic can be exercised
offline.  The single planning authority stays ``joint_waypoint_plan.compile_plan``;
this module only normalises, freezes and describes its output.
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

from joint_waypoint_plan import compile_plan
import joint_waypoint_plan as _planner
from point_to_point import effective_document,EXTRA_SECONDS
from calibration_session import effective_document as calibration_document

# The peak reference speed belongs to the protected planner; never hard-code it
# here, otherwise the UI/tests drift when the commissioning baseline changes.
REFERENCE_SPEED_RAD_S = float(getattr(_planner, "REFERENCE_SPEED_RAD_S", 0.02))

JOINT_NAMES = tuple("slaihand_joint%d" % i for i in range(1, 5))
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


def load_limits(path):
    """Four bounded revolute joints from the installed description package."""
    path = Path(path)
    root = ET.parse(path).getroot()
    bounds = []
    for name in JOINT_NAMES:
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


def degrees_limits(bounds):
    return [[round(math.degrees(lo), 3), round(math.degrees(hi), 3)] for lo, hi in bounds]


def normalize_document(document):
    """Shape-only normalisation.  Range/speed/timing rules stay in compile_plan."""
    if not isinstance(document, dict):
        raise ValueError("计划必须是 JSON 对象")
    if set(document) != {"coordinates", "waypoints"}:
        raise ValueError("计划必须且仅包含 coordinates、waypoints")
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
        if not isinstance(values, list) or len(values) != 4:
            raise ValueError("第%d点需要J1～J4四个角度，不支持J5" % index)
        numbers = []
        for value in list(values) + [row["move_s"], row["dwell_s"]]:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError("第%d点角度与时间必须为有限数值" % index)
            numbers.append(float(value))
        waypoints.append({"degrees": numbers[:4], "move_s": numbers[4], "dwell_s": numbers[5]})
    return {"coordinates": mode, "waypoints": waypoints}


def canonical(document):
    return json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def digest(document):
    return hashlib.sha256(canonical(document).encode("utf-8")).hexdigest()


def compile_document(document, start_rad, bounds):
    """Normalise then hand the plan to the one production validator."""
    if len(start_rad) != 4 or any(number(q) is None for q in start_rad):
        raise ValueError("没有完整四轴反馈，不能预览或执行")
    normalised = normalize_document(document)
    points, windows = compile_plan(normalised, [float(q) for q in start_rad], [tuple(b) for b in bounds])
    return normalised, points, windows


def freeze_absolute(document, start_rad, bounds):
    """Freeze a plan to absolute software degrees (what will actually execute)."""
    normalised, _, windows = compile_document(document, start_rad, bounds)
    rows = []
    for row, window in zip(normalised["waypoints"], windows):
        rows.append({"degrees": [math.degrees(q) for q in window["target_rad"]],
                     "move_s": row["move_s"], "dwell_s": row["dwell_s"]})
    frozen = {"coordinates": "absolute_deg", "waypoints": rows}
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
    effective=effective_document(normalised) if mode=='point_to_point' else normalised
    if mode=='calibration':
        effective=calibration_document(normalised)
    frozen, points, windows = freeze_absolute(effective, start_rad, bounds)
    normalised = normalize_document(document)
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
    return {"mode": "absolute_deg", "execution_mode":mode, "source_mode": normalised["coordinates"],
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
