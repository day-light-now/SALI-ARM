"""Five-axis output-margin evaluator: pure model, no ROS and no motion.

The operator requirement is:

    allowed_output / demand >= 1.5        (equivalently demand <= (2/3)*allowed)

for every axis J1-J5 and for the *same* operating condition.  This module is
deliberately strict and fail-closed:

* the allowed value must come from a verified capability source with an
  explicit unit, a continuous/peak duration statement and an operating
  condition; a protocol TMAX register (200/54/28 N*m) is a protocol range, not
  a verified capability and is rejected;
* the demand must name its source, measurement type and operating condition.
  A control/friction request estimate (``request_estimate``) or a torque
  feedback estimate is only a pre-screen, never a measured acceptance;
* missing / non-finite / unverifiable evidence is ``UNKNOWN`` or
  ``NOT_QUALIFIED`` and never PASS; unknown or insufficient margin never grants
  tuning / amplitude-extension qualification;
* PASS requires ``allowed / demand >= 1.5`` with the equality boundary accepted,
  measured demand, complete conditions, and all five axes passing.

No value in this module is a physical 1.5x qualification claim.  The evidence
file is expected at ``config/five_axis_margin_evidence.json`` (see the
``EVIDENCE_SCHEMA`` docstring below); this round the file is intentionally
absent so the panel reports UNKNOWN for every axis.
"""
from __future__ import annotations

import json
import math

# --------------------------------------------------------------------- schema
# Evidence file (all values are supplied by an operator and recorded with a
# source; this module never invents a number):
#
# {
#   "schema": "slaihand_five_axis_margin_evidence/1",
#   "axes": [
#     {
#       "axis": 1,
#       "capability": {
#         "allowed_output_nm": 16.65,
#         "source": "motor test report 2026-11-??, locked-rotor at 25C",
#         "source_kind": "verified_measurement",
#         "unit": "Nm",
#         "verified": true,
#         "duration": {"kind": "continuous"}          # or {"kind": "peak", "duration_s": 2.0}
#         "condition": "static hold, J2=40deg, load 3.5kg, 48V, 25C"
#       },
#       "demand": {
#         "demand_nm": 11.1,
#         "source": "external torque sensor ??, sample window 2s",
#         "measurement_type": "measured_torque",
#         "condition": "static hold, J2=40deg, load 3.5kg, 48V, 25C",
#         "duration_s": 2.0,
#         "kind": "peak"
#       }
#     }
#   ]
# }
#
# ``source_kind`` accepted as verified: "verified_measurement",
# "rated_datasheet", "documented_limit", "thermal_test".
# ``source_kind`` prohibited: "protocol_tmax", "protocol_register",
# "control_estimate", "tau_feedback_estimate", "request_estimate".
EVIDENCE_SCHEMA = "slaihand_five_axis_margin_evidence/1"
REPORT_SCHEMA = "slaihand_five_axis_margin_report/1"
REQUIRED_SAFETY_FACTOR = 1.5
MAX_DEMAND_FRACTION = 2.0 / 3.0
MARGIN_RULE = "allowed / demand >= 1.5  <=>  demand <= (2/3) * allowed"
EVIDENCE_FILENAME = "five_axis_margin_evidence.json"

STATUS_PASS = "PASS"
STATUS_FAIL = "FAIL"
STATUS_UNKNOWN = "UNKNOWN"
STATUS_NOT_QUALIFIED = "NOT_QUALIFIED"
STATUS_PRESCREEN_PASS = "PRESCREEN_PASS"
STATUS_PRESCREEN_FAIL = "PRESCREEN_FAIL"
_AXIS_STATUSES = (STATUS_PASS, STATUS_FAIL, STATUS_UNKNOWN, STATUS_NOT_QUALIFIED,
                  STATUS_PRESCREEN_PASS, STATUS_PRESCREEN_FAIL)

VERIFIED_CAPABILITY_SOURCES = ("verified_measurement", "rated_datasheet",
                               "documented_limit", "thermal_test")
PROHIBITED_CAPABILITY_SOURCES = ("protocol_tmax", "protocol_register",
                                 "control_estimate", "tau_feedback_estimate",
                                 "request_estimate")
DEMAND_MEASUREMENT_TYPES = ("measured_torque", "external_sensor",
                            "verified_current_estimate")
PRESCREEN_MEASUREMENT_TYPES = ("request_estimate", "control_estimate",
                               "tau_feedback_estimate")
_UNIT_ALIASES = {"nm": "Nm", "n*m": "Nm", "n·m": "Nm", "newtonmeter": "Nm",
                 "newton_meter": "Nm", "n.m": "Nm"}
_DURATION_KINDS = ("continuous", "peak")

_CAPABILITY_VALUE_KEYS = ("allowed_output_nm", "allowed_nm", "capacity_nm",
                          "verified_allowed_output_nm")
_DEMAND_VALUE_KEYS = ("demand_nm", "demand_output_nm", "required_output_nm",
                      "value_nm")

_DISCLAIMER = ("裕量按 allowed/demand>=1.5 计算；请求估算/力矩反馈估算只能预筛查，"
               "不是实测验收；协议 TMAX 不是能力；本轮无实测能力数据时保持 UNKNOWN。"
               "本表不构成五轴物理 1.5 倍实机验收。")


# ------------------------------------------------------------------- helpers
def _finite(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _text(value):
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _mapping(value):
    return value if isinstance(value, dict) else None


def _pick(record, keys):
    for key in keys:
        if key in record and record[key] is not None:
            return record[key]
    return None


def _normalise_unit(value):
    text = _text(value)
    if text is None:
        return None, "CAPABILITY_UNIT_MISSING"
    mapped = _UNIT_ALIASES.get(text.lower().replace(" ", ""))
    if mapped is None:
        return None, "CAPABILITY_UNIT_INVALID"
    return mapped, None


def _normalise_condition(value):
    text = _text(value)
    if text is None:
        return None
    return " ".join(text.lower().split())


def _duration(capability, *, prefix="CAPABILITY"):
    """Return (kind, duration_s, reason_code) from explicit duration fields.

    Accepted shapes: ``duration: {kind, duration_s}`` or flat
    ``duration_kind`` / ``duration_s``.  Missing kind or a peak without a finite
    positive duration is insufficient evidence (UNKNOWN), never a guess.
    """
    duration = _mapping(capability.get("duration")) or {}
    kind = _text(duration.get("kind") or capability.get("duration_kind"))
    if kind is None:
        return None, None, "%s_DURATION_MISSING" % prefix
    kind = kind.strip().lower()
    if kind not in _DURATION_KINDS:
        return None, None, "%s_DURATION_INVALID" % prefix
    seconds = _finite(duration.get("duration_s", capability.get("duration_s")))
    if kind == "continuous":
        return kind, None, None
    if seconds is None or seconds <= 0:
        return None, None, "%s_DURATION_MISSING" % prefix
    return kind, seconds, None


# --------------------------------------------------------------- capability
def normalise_capability(axis, record):
    """Normalise one allowed-output record; fail closed with stable codes."""
    reasons = []
    qualified = []
    label = "J%d" % axis if isinstance(axis, int) else str(axis)
    record = _mapping(record)
    if record is None:
        return {"axis": axis, "label": label, "value_nm": None, "status": STATUS_UNKNOWN,
                "reasons": ["CAPABILITY_MISSING"], "qualified_reasons": [],
                "source": None, "source_kind": None, "unit": None,
                "duration_kind": None, "duration_s": None, "condition": None,
                "verified": None}

    raw_value = _pick(record, _CAPABILITY_VALUE_KEYS)
    value = _finite(raw_value)
    if raw_value is None:
        reasons.append("CAPABILITY_VALUE_MISSING")
    elif value is None:
        reasons.append("CAPABILITY_VALUE_NONFINITE")
    elif value <= 0.0:
        qualified.append("CAPABILITY_VALUE_INVALID")

    unit, unit_reason = _normalise_unit(record.get("unit"))
    if unit_reason == "CAPABILITY_UNIT_MISSING":
        reasons.append(unit_reason)
    elif unit_reason:
        # A present but wrong unit is an invalid claim, not missing evidence.
        qualified.append(unit_reason)

    source = _text(record.get("source"))
    if source is None:
        reasons.append("CAPABILITY_SOURCE_MISSING")
    source_kind = _text(record.get("source_kind"))
    source_kind_norm = source_kind.strip().lower().replace("-", "_") if source_kind else None
    if source_kind_norm is None:
        reasons.append("CAPABILITY_SOURCE_KIND_MISSING")
    elif source_kind_norm in PROHIBITED_CAPABILITY_SOURCES:
        qualified.append("TMAX_NOT_CAPABILITY" if "tmax" in source_kind_norm
                         else "CAPABILITY_SOURCE_NOT_VERIFIED")
    elif source_kind_norm not in VERIFIED_CAPABILITY_SOURCES:
        qualified.append("CAPABILITY_SOURCE_KIND_UNKNOWN")
    # A source string that itself admits it is the protocol TMAX range must
    # never become a capability, even if someone labels it "verified".
    haystack = ("%s %s" % (source or "", source_kind or "")).lower()
    if "tmax" in haystack or "protocol_tmax" in haystack:
        qualified.append("TMAX_NOT_CAPABILITY")

    verified = record.get("verified")
    if verified is not True:
        qualified.append("CAPABILITY_NOT_VERIFIED")

    duration_kind, duration_s, duration_reason = _duration(record)
    if duration_reason:
        reasons.append(duration_reason)

    condition = _normalise_condition(record.get("condition"))
    if condition is None:
        reasons.append("CAPABILITY_CONDITION_MISSING")

    status = _side_status(qualified, reasons)
    result = {
        "axis": axis, "label": label,
        "value_nm": value,
        "source": source, "source_kind": source_kind_norm,
        "unit": unit, "verified": verified is True,
        "duration_kind": duration_kind, "duration_s": duration_s,
        "condition": condition,
        "status": status, "reasons": reasons,
        "qualified_reasons": qualified,
        "display": _capability_display(value, unit, duration_kind, duration_s,
                                       condition, source),
    }
    return result


def _capability_display(value, unit, duration_kind, duration_s, condition, source):
    if value is None:
        return "能力未提供（UNKNOWN）"
    duration = "持续" if duration_kind == "continuous" else (
        "峰值 %.2f s" % duration_s if duration_kind == "peak" else "时长未知")
    condition_text = condition if condition else "工况未知"
    return "%s %s（%s；%s；来源：%s）" % (("%.3f" % value).rstrip("0").rstrip("."),
                                        unit or "单位?", duration, condition_text,
                                        source or "来源未知")


# ------------------------------------------------------------------- demand
def normalise_demand(axis, record):
    reasons = []
    qualified = []
    label = "J%d" % axis if isinstance(axis, int) else str(axis)
    record = _mapping(record)
    if record is None:
        return {"axis": axis, "label": label, "value_nm": None, "status": STATUS_UNKNOWN,
                "reasons": ["DEMAND_MISSING"], "qualified_reasons": [],
                "source": None, "measurement_type": None, "condition": None,
                "duration_kind": None, "duration_s": None,
                "pre_screen_only": True, "measured": False}

    raw_value = _pick(record, _DEMAND_VALUE_KEYS)
    value = _finite(raw_value)
    if raw_value is None:
        reasons.append("DEMAND_VALUE_MISSING")
    elif value is None:
        reasons.append("DEMAND_VALUE_NONFINITE")
    elif value <= 0.0:
        qualified.append("DEMAND_VALUE_INVALID")

    source = _text(record.get("source"))
    if source is None:
        reasons.append("DEMAND_SOURCE_MISSING")

    measurement_type = _text(record.get("measurement_type"))
    measurement_norm = measurement_type.strip().lower().replace("-", "_") \
        if measurement_type else None
    pre_screen_only = True
    if measurement_norm is None:
        reasons.append("DEMAND_MEASUREMENT_TYPE_MISSING")
    elif measurement_norm in DEMAND_MEASUREMENT_TYPES:
        pre_screen_only = False
    elif measurement_norm in PRESCREEN_MEASUREMENT_TYPES:
        pre_screen_only = True
    else:
        reasons.append("DEMAND_MEASUREMENT_TYPE_UNKNOWN")
        pre_screen_only = True

    condition = _normalise_condition(record.get("condition"))
    if condition is None:
        reasons.append("DEMAND_CONDITION_MISSING")

    duration = _mapping(record.get("duration")) or {}
    demand_kind = _text(duration.get("kind") or record.get("kind"))
    demand_kind = demand_kind.strip().lower() if demand_kind else None
    if demand_kind is not None and demand_kind not in _DURATION_KINDS:
        reasons.append("DEMAND_DURATION_INVALID")
        demand_kind = None
    demand_seconds = _finite(duration.get("duration_s", record.get("duration_s")))

    status = _side_status(qualified, reasons)
    return {
        "axis": axis, "label": label,
        "value_nm": value,
        "source": source, "measurement_type": measurement_norm,
        "condition": condition,
        "duration_kind": demand_kind, "duration_s": demand_seconds,
        "status": status, "reasons": reasons, "qualified_reasons": qualified,
        "pre_screen_only": pre_screen_only,
        "measured": (not pre_screen_only) and status != STATUS_NOT_QUALIFIED,
    }


def _side_status(qualified, reasons):
    if qualified:
        return STATUS_NOT_QUALIFIED
    if reasons:
        return STATUS_UNKNOWN
    return "OK"


# ------------------------------------------------------------------- compare
def _compare(capability, demand):
    """Compare one normalised pair; returns (status, reasons, safety_factor)."""
    reasons = []
    if capability["status"] == STATUS_NOT_QUALIFIED:
        return STATUS_NOT_QUALIFIED, list(capability["qualified_reasons"]), None
    if capability["status"] == STATUS_UNKNOWN:
        return STATUS_UNKNOWN, list(capability["reasons"]), None
    if demand["status"] == STATUS_NOT_QUALIFIED:
        return STATUS_NOT_QUALIFIED, list(demand["qualified_reasons"]), None
    if demand["status"] == STATUS_UNKNOWN:
        return STATUS_UNKNOWN, list(demand["reasons"]), None

    allowed = capability["value_nm"]
    need = demand["value_nm"]
    if allowed is None or need is None or need <= 0.0:
        return STATUS_UNKNOWN, ["MARGIN_VALUE_MISSING"], None
    safety_factor = allowed / need

    if capability["condition"] != demand["condition"]:
        return STATUS_NOT_QUALIFIED, ["CONDITION_MISMATCH"], safety_factor

    if capability["duration_kind"] == "peak":
        if demand["duration_kind"] == "continuous":
            return STATUS_NOT_QUALIFIED, ["DURATION_NOT_COVERED"], safety_factor
        if demand["duration_s"] is None:
            return STATUS_UNKNOWN, ["DEMAND_DURATION_MISSING"], safety_factor
        if demand["duration_s"] > capability["duration_s"] + 1e-12:
            return STATUS_NOT_QUALIFIED, ["DURATION_NOT_COVERED"], safety_factor
    elif capability["duration_kind"] is None:
        return STATUS_UNKNOWN, ["CAPABILITY_DURATION_MISSING"], safety_factor

    sufficient = safety_factor >= REQUIRED_SAFETY_FACTOR - 1e-9
    if demand["pre_screen_only"]:
        return (STATUS_PRESCREEN_PASS if sufficient else STATUS_PRESCREEN_FAIL,
                [] if sufficient else ["INSUFFICIENT_MARGIN"], safety_factor)
    if not sufficient:
        return STATUS_FAIL, ["INSUFFICIENT_MARGIN"], safety_factor
    return STATUS_PASS, [], safety_factor


def evaluate_axis(axis, capability_record, demand_record):
    """Evaluate one axis.  Never fabricates a missing operand."""
    capability = normalise_capability(axis, capability_record)
    demand = normalise_demand(axis, demand_record)
    status, reasons, safety_factor = _compare(capability, demand)
    reasons = list(dict.fromkeys(list(reasons)))
    if status == STATUS_UNKNOWN and not reasons:
        reasons = ["MARGIN_EVIDENCE_INCOMPLETE"]
    measured_acceptance = status in (STATUS_PASS, STATUS_FAIL)
    return {
        "axis": axis, "label": capability["label"],
        "status": status, "status_label": _STATUS_LABELS.get(status, status),
        "reasons": reasons,
        "safety_factor": None if safety_factor is None else round(safety_factor, 6),
        "demand_fraction_of_allowed": None if safety_factor in (None, 0.0)
        else round(1.0 / safety_factor, 6),
        "required_safety_factor": REQUIRED_SAFETY_FACTOR,
        "rule": MARGIN_RULE,
        "measured_acceptance": measured_acceptance,
        "pre_screen_only": bool(demand.get("pre_screen_only")),
        "qualification_granted": status == STATUS_PASS,
        "capability": capability, "demand": demand,
        "display": _axis_display(status, safety_factor, reasons, capability, demand),
    }


_STATUS_LABELS = {
    STATUS_PASS: "通过（>=1.5）",
    STATUS_FAIL: "不足（<1.5）",
    STATUS_UNKNOWN: "UNKNOWN（证据不足）",
    STATUS_NOT_QUALIFIED: "NOT_QUALIFIED（证据不合格）",
    STATUS_PRESCREEN_PASS: "预筛查通过（未实测）",
    STATUS_PRESCREEN_FAIL: "预筛查不足（未实测）",
}


def _axis_display(status, safety_factor, reasons, capability, demand):
    if status in (STATUS_UNKNOWN, STATUS_NOT_QUALIFIED):
        detail = "、".join(reasons) if reasons else "证据不足"
        return "%s：%s" % (_STATUS_LABELS[status], detail)
    factor = "%.3f" % safety_factor if safety_factor is not None else "—"
    return "%s：allowed/demand=%s" % (_STATUS_LABELS[status], factor)


# -------------------------------------------------------------------- report
def _evidence_axis_map(axes):
    mapping = {}
    duplicates = set()
    if isinstance(axes, dict):
        for key, value in axes.items():
            try:
                axis = int(key)
            except (TypeError, ValueError):
                continue
            if axis in mapping:
                duplicates.add(axis)
            mapping[axis] = value
    elif isinstance(axes, (list, tuple)):
        for item in axes:
            if not isinstance(item, dict):
                continue
            try:
                axis = int(item.get("axis"))
            except (TypeError, ValueError):
                continue
            if axis in mapping:
                duplicates.add(axis)
            mapping[axis] = item
    return mapping, duplicates


def evaluate_five_axis(axes=None, *, axis_count=5, joint_names=None, source=None,
                       evidence_loaded=False, evidence_reasons=None):
    """Evaluate all controlled axes and return the panel report.

    ``axes`` is the raw evidence list/dict; an axis absent from it is UNKNOWN.
    ``overall`` is PASS only when every controlled axis is PASS with measured
    acceptance; pre-screen-only or partial evidence is never overall PASS.
    """
    try:
        axis_count = int(axis_count)
    except (TypeError, ValueError):
        axis_count = 5
    if axis_count < 1:
        axis_count = 1
    names = list(joint_names or [])
    evidence_map, duplicates = _evidence_axis_map(axes)
    rows = []
    for axis in range(1, axis_count + 1):
        entry = evidence_map.get(axis) if axis not in duplicates else None
        entry = entry if isinstance(entry, dict) else {}
        row = evaluate_axis(axis, entry.get("capability"), entry.get("demand"))
        if axis in duplicates:
            row = _unknown_axis(axis, ["DUPLICATE_AXIS_EVIDENCE"])
        if axis < len(names) and names[axis - 1]:
            row["name"] = names[axis - 1]
        rows.append(row)

    resolved = [row for row in rows if row["status"] in
                (STATUS_PASS, STATUS_FAIL, STATUS_NOT_QUALIFIED,
                 STATUS_PRESCREEN_PASS, STATUS_PRESCREEN_FAIL)]
    counts = {status: sum(1 for row in rows if row["status"] == status)
              for status in _AXIS_STATUSES}

    if not evidence_map:
        data_status = STATUS_UNKNOWN
        overall = STATUS_UNKNOWN
        aggregate = ["MARGIN_EVIDENCE_MISSING"] if not evidence_loaded \
            else ["MARGIN_EVIDENCE_NO_AXES"]
    else:
        if any(row["status"] in (STATUS_FAIL, STATUS_PRESCREEN_FAIL) for row in rows):
            overall = STATUS_FAIL
        elif any(row["status"] == STATUS_NOT_QUALIFIED for row in rows):
            overall = STATUS_NOT_QUALIFIED
        elif any(row["status"] == STATUS_UNKNOWN for row in rows):
            overall = STATUS_UNKNOWN
        elif all(row["status"] == STATUS_PASS for row in rows):
            overall = STATUS_PASS
        elif any(row["status"] == STATUS_PRESCREEN_PASS for row in rows):
            # Nothing measured: pre-screen evidence can never be overall PASS.
            overall = STATUS_PRESCREEN_PASS
        else:  # pragma: no cover - defensive
            overall = STATUS_UNKNOWN
        data_status = "COMPLETE" if len(resolved) == axis_count else "PARTIAL"
        aggregate = []
    if evidence_reasons:
        aggregate = list(dict.fromkeys(list(evidence_reasons) + aggregate))

    passes = counts.get(STATUS_PASS, 0)
    measured_axes = sum(1 for row in rows if row["measured_acceptance"])
    report = {
        "schema": REPORT_SCHEMA,
        "required_safety_factor": REQUIRED_SAFETY_FACTOR,
        "max_demand_fraction_of_allowed": round(MAX_DEMAND_FRACTION, 6),
        "rule": MARGIN_RULE,
        "axis_count": axis_count,
        "overall": overall,
        "overall_label": {
            STATUS_PASS: "全部轴 PASS（软件评估）",
            STATUS_FAIL: "存在裕量不足",
            STATUS_NOT_QUALIFIED: "存在不合格证据（TMAX/工况不符等）",
            STATUS_UNKNOWN: "证据未知/不足（本轮默认）",
            STATUS_PRESCREEN_PASS: "仅请求估算预筛查，不是实测通过",
        }.get(overall, overall),
        "data_status": data_status,
        "evidence_loaded": bool(evidence_loaded),
        "source": source,
        "axes": rows,
        "counts": counts,
        "pass_axes": passes,
        "measured_axes": measured_axes,
        "pre_screen_axes": counts.get(STATUS_PRESCREEN_PASS, 0)
        + counts.get(STATUS_PRESCREEN_FAIL, 0),
        "unknown_axes": counts.get(STATUS_UNKNOWN, 0),
        "fail_axes": counts.get(STATUS_FAIL, 0),
        "not_qualified_axes": counts.get(STATUS_NOT_QUALIFIED, 0),
        "reasons": aggregate,
        "disclaimer": _DISCLAIMER,
        "physical_acceptance": False,
    }
    report["qualification"] = qualification_gate(report)
    return report


def _unknown_axis(axis, reasons):
    row = evaluate_axis(axis, None, None)
    row["status"] = STATUS_UNKNOWN
    row["status_label"] = _STATUS_LABELS[STATUS_UNKNOWN]
    row["reasons"] = list(reasons)
    row["display"] = "%s：%s" % (_STATUS_LABELS[STATUS_UNKNOWN], "、".join(reasons))
    row["qualification_granted"] = False
    return row


def qualification_gate(report):
    """Tuning/amplitude qualification may be granted only by a complete PASS.

    Unknown, insufficient, condition-mismatched or pre-screen-only evidence
    never grants qualification.  Even a PASS is a software evidence evaluation,
    not a physical 1.5x acceptance: ``physical_acceptance`` is always False.
    """
    reasons = []
    overall = (report or {}).get("overall")
    data_status = (report or {}).get("data_status")
    if overall != STATUS_PASS:
        reasons.append("MARGIN_NOT_PASS: overall=%s" % (overall or STATUS_UNKNOWN))
    if data_status != "COMPLETE":
        reasons.append("MARGIN_DATA_INCOMPLETE: data_status=%s" % (data_status or STATUS_UNKNOWN))
    for row in (report or {}).get("axes") or []:
        if row.get("status") != STATUS_PASS:
            reasons.append("AXIS_%s_%s: %s" % (row.get("label"), row.get("status"),
                                               "、".join(row.get("reasons") or []) or "未通过"))
    allowed = not reasons
    return {
        "tuning_allowed": allowed,
        "amplitude_extension_allowed": allowed,
        "physical_acceptance": False,
        "label": "软件证据通过；仍需现场独立验收，不是物理 1.5 倍验收"
        if allowed else "拒绝授予调参/扩幅资格：未知或不足裕量",
        "reasons": list(dict.fromkeys(reasons)),
    }


# ------------------------------------------------------------------ evidence
def load_evidence(path=None):
    """Load the evidence JSON fail-closed.  Never raises, never invents data."""
    result = {"loaded": False, "axes": None, "source": None,
              "reasons": [], "schema": None}
    if path is None:
        result["reasons"] = ["MARGIN_EVIDENCE_MISSING"]
        return result
    result["source"] = str(path)
    try:
        from pathlib import Path
        evidence_path = Path(path)
        if not evidence_path.is_file():
            result["reasons"] = ["MARGIN_EVIDENCE_MISSING"]
            return result
        payload = json.loads(evidence_path.read_text(encoding="utf-8"))
    except Exception:
        result["reasons"] = ["MARGIN_EVIDENCE_INVALID"]
        return result
    if not isinstance(payload, dict):
        result["reasons"] = ["MARGIN_EVIDENCE_INVALID"]
        return result
    result["schema"] = payload.get("schema")
    if payload.get("schema") != EVIDENCE_SCHEMA:
        result["reasons"] = ["MARGIN_EVIDENCE_SCHEMA"]
        return result
    axes = payload.get("axes")
    if not isinstance(axes, (list, dict)):
        result["reasons"] = ["MARGIN_EVIDENCE_NO_AXES"]
        return result
    result["loaded"] = True
    result["axes"] = axes
    return result


def build_report(evidence=None, *, axis_count=5, joint_names=None):
    """Build the report from an ``load_evidence`` result (or a raw axes value)."""
    if isinstance(evidence, dict) and "loaded" in evidence:
        return evaluate_five_axis(evidence.get("axes"), axis_count=axis_count,
                                  joint_names=joint_names,
                                  source=evidence.get("source"),
                                  evidence_loaded=bool(evidence.get("loaded")),
                                  evidence_reasons=evidence.get("reasons"))
    return evaluate_five_axis(evidence, axis_count=axis_count,
                              joint_names=joint_names)


def default_unknown_report(*, axis_count=5, joint_names=None, source=None,
                           reason="MARGIN_EVIDENCE_MISSING"):
    """All-UNKNOWN report used when this round has no verified capability data."""
    return evaluate_five_axis(None, axis_count=axis_count, joint_names=joint_names,
                              source=source, evidence_loaded=False,
                              evidence_reasons=[reason])


def margin_for_axes(axes_records, *, axis_count=5, joint_names=None, source="inline"):
    """Convenience for tests/API callers that already hold per-axis records."""
    return evaluate_five_axis(axes_records, axis_count=axis_count,
                              joint_names=joint_names, source=source,
                              evidence_loaded=True)
