"""Five-axis diagnostics: pure parsers for per-axis gains and first fault.

No ROS imports.  The panel feeds this module the payloads it already receives:

* ``/slaihand/runtime_identity`` (``std_msgs/String`` JSON, schema
  ``slaihand_runtime_identity/1``) - legacy scalar ``kp``/``kd`` plus the
  provisional per-axis contract ``kp_axis``/``kd_axis`` (+ ``*_axis_source``),
  ``following_error_deg`` and the frozen driver ``first_fault`` object;
* ``/slaihand/dynamic_joint_states`` - 17 state interfaces per axis (the driver
  does *not* add gain interfaces; per-axis interfaces are accepted defensively
  for older/alternate publishers);
* ``/slaihand/gravity_ff_state`` - feed-forward payload (not used for gains);
* ``config/canusb_five_axis_controllers.yaml`` - JTC per-joint ``trajectory``
  tolerance (currently 10 deg) and ``goal`` tolerance (1 deg);
* ``config/canusb_five_axis_motion_policy.yaml`` - driver
  ``following_error_deg`` guard (currently 4 deg).

Contract assumptions recorded for review (C++ owner ``cpp-gain-diagnostics``):

1. Per-axis applied gains are ``kp_axis`` / ``kd_axis`` arrays of length
   ``active_axes`` in J1..Jn order, with ``kp_axis_source`` in
   ``{"per_axis","scalar_legacy"}``.  A present array is authoritative; the old
   scalar is *policy fallback only* and must never be substituted when the
   array is present (wrong length is rejected, never padded/truncated).
2. Driver first cause is one frozen object: ``first_fault`` inside
   ``runtime_identity`` (also mirrored into the audit STAGE record).  Fields:
   ``schema/layer/axis/joint/code/detected_at/error_deg/limit_deg/
   target_deg/actual_deg/feedback_age_s/feedback_valid/encoded_kp/encoded_kd/
   encoded_tau_ff_nm/request_estimate_nm/request_estimate_at/command_id/
   cycle_id/boot_id/last_complete_write_at/frozen``.  ``layer`` must be
   ``DRIVER``; a JTC error is never relabelled as the driver first cause.
   ``axis`` is 1-based, 0 means "not axis-specific" (do not paint it as J1).
3. ``request_estimate_state`` is one of ``UNAVAILABLE`` / ``CURRENT`` /
   ``LAST_ACTIVE_STALE`` (the frozen first-fault object names the last one
   ``LAST_ACTIVE``; both mean "not current output").  A request estimate is
   never a measured output; after STOPPING/DISABLED/UNKNOWN/DISARMED it is
   shown as last-active/stale.  Negative ``-1`` time sentinels are rendered as
   finite JSON by the driver, so they must be treated as unavailable.
4. JTC following-error tolerance (10 deg today) is display-only from the YAML;
   the panel recommends but does not deploy a candidate restore to the driver
   4 deg.  That restore does not remove the load deviation itself.

Defensive aliases are accepted when they cost nothing (see
``GAIN_ALIASES``/``FAULT_ALIASES``) but the canonical names above are what the
fixtures and tests pin.
"""
from __future__ import annotations

import json
import math
import re

DIAGNOSTICS_SCHEMA = "slaihand_five_axis_diagnostics/1"
GAIN_VIEW_SCHEMA = "slaihand_axis_gains_view/1"
FIRST_FAULT_SCHEMA = "slaihand_first_fault/1"
RUNTIME_IDENTITY_SCHEMA = "slaihand_runtime_identity/1"

POLICY_KP_MAX = 160.0
POLICY_KD_MAX = 5.0
PROTOCOL_KP_MAX = 500.0
PROTOCOL_KD_MAX = 5.0
# The protected client sends an explicit per-goal goal_tolerance for plan and
# teaching goals: 0.5 deg for a normal (non-calibration) plan document and
# 4 deg for teaching playback/calibration.  A plain delta goal sets none, so the
# YAML goal tolerance (1 deg) would apply.  Only the YAML *trajectory/path*
# tolerance is a config-only candidate; editing the YAML goal alone is not
# enough for these explicit client goals.
CLIENT_GOAL_TOLERANCE_DEG = 0.5
CLIENT_TEACHING_GOAL_TOLERANCE_DEG = 4.0
CLIENT_CALIBRATION_GOAL_TOLERANCE_DEG = 4.0
CLIENT_GOAL_TOLERANCE_SOURCE = "commission_supported_trial.py goal_tolerance"

DRIVER_LAYER = "DRIVER"
JTC_LAYER = "JTC"

REQUEST_ESTIMATE_UNAVAILABLE = "UNAVAILABLE"
REQUEST_ESTIMATE_CURRENT = "CURRENT"
REQUEST_ESTIMATE_LAST_ACTIVE_STALE = "LAST_ACTIVE_STALE"
_REQUEST_ESTIMATE_STATES = (REQUEST_ESTIMATE_UNAVAILABLE, REQUEST_ESTIMATE_CURRENT,
                            REQUEST_ESTIMATE_LAST_ACTIVE_STALE)
# The C++ first_fault JSON names the frozen estimate "LAST_ACTIVE" (inside the
# fault object); it means exactly the panel's LAST_ACTIVE_STALE. Accept both.
_REQUEST_ESTIMATE_STATE_ALIASES = {
    "LAST_ACTIVE": REQUEST_ESTIMATE_LAST_ACTIVE_STALE,
    "LAST_ACTIVE_STALE": REQUEST_ESTIMATE_LAST_ACTIVE_STALE,
    "STALE": REQUEST_ESTIMATE_LAST_ACTIVE_STALE,
    "CURRENT": REQUEST_ESTIMATE_CURRENT,
    "UNAVAILABLE": REQUEST_ESTIMATE_UNAVAILABLE,
}

# Defensive alias lists; the C++ provisional contract names are first.
GAIN_ALIASES = {
    "kp_axis": ("kp_axis", "kp_per_axis", "per_axis_kp", "applied_kp_axis",
                "kp_applied_axis", "axis_kp"),
    "kd_axis": ("kd_axis", "kd_per_axis", "per_axis_kd", "applied_kd_axis",
                "kd_applied_axis", "axis_kd"),
    "kp_encoded": ("encoded_kp", "kp_encoded_axis", "encoded_kp_axis"),
    "kd_encoded": ("encoded_kd", "kd_encoded_axis", "encoded_kd_axis"),
}
GAIN_DYNAMIC_KEYS = {
    "kp": ("applied_kp", "kp_applied", "encoded_kp", "kp_encoded", "kp", "gain_kp"),
    "kd": ("applied_kd", "kd_applied", "encoded_kd", "kd_encoded", "kd", "gain_kd"),
}
_AXIS_GAIN_BOUNDS = {"kp": (0.0, POLICY_KP_MAX), "kd": (0.0, POLICY_KD_MAX)}

_FAULT_FIELD_ALIASES = {
    "error_deg": ("error_deg", "following_error_deg", "error_degrees"),
    "error_rad": ("error_rad", "following_error_rad"),
    "limit_deg": ("limit_deg", "following_error_limit_deg", "threshold_deg",
                  "limit_degrees"),
    "limit_rad": ("limit_rad", "following_error_limit_rad", "threshold_rad"),
    "target_deg": ("target_deg", "target_degrees"),
    "target_rad": ("target_rad",),
    "actual_deg": ("actual_deg", "feedback_deg", "measured_deg", "actual_degrees"),
    "actual_rad": ("actual_rad", "feedback_rad", "measured_rad"),
    "feedback_age_s": ("feedback_age_s", "age_s", "feedback_age"),
    "tau_ff_nm": ("encoded_tau_ff_nm", "tau_ff_nm", "tau_ff"),
    "request_estimate_nm": ("request_estimate_nm", "request_estimate"),
    "request_estimate_at": ("request_estimate_at", "request_estimate_updated_at"),
    "command_id": ("command_id",),
    "cycle_id": ("cycle_id", "cycle"),
    "boot_id": ("boot_id",),
    "detected_at": ("detected_at", "fault_at", "steady_s"),
    "last_complete_write_at": ("last_complete_write_at", "last_write_s",
                               "last_complete_write_s", "last_written_at"),
    "axis": ("axis",),
    "joint": ("joint", "axis_name", "joint_name"),
    "code": ("code", "kind", "fault_code"),
    "layer": ("layer",),
    "feedback_valid": ("feedback_valid",),
}


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


def _degrees(value, *, from_rad=False):
    number = _finite(value)
    if number is None:
        return None
    return math.degrees(number) if from_rad else number


def _pick(record, keys):
    if not isinstance(record, dict):
        return None
    for key in keys:
        if key in record and record[key] is not None:
            return record[key]
    return None


def _axis_index(value, joint_count):
    """1-based controlled-axis index or None; never guesses across numbering."""
    number = _finite(value)
    if number is None:
        return None
    integer = int(number)
    if integer != number:
        return None
    if 1 <= integer <= joint_count:
        return integer
    return None


def _axis_scalar(value, axis):
    """Index a scalar/list/dict per-axis field for one 1-based axis.

    Without a valid axis, a list/dict is NOT silently reduced to its last/first
    element; only a plain scalar can be shown as "not axis-specific".
    """
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        if axis is None:
            return None
        index = int(axis) - 1
        return value[index] if 0 <= index < len(value) else None
    if isinstance(value, dict):
        if axis is None:
            return None
        for key in (axis, "J%d" % int(axis), "slaihand_joint%d" % int(axis), str(axis)):
            if key in value:
                return value[key]
        return None
    return value


def _clamp_display(value):
    number = _finite(value)
    if number is None:
        return "—"
    return ("%.3f" % number).rstrip("0").rstrip(".")


def _clean_axis_label(axis, joint_names=None):
    if axis is None:
        return "全轴/非单轴"
    if joint_names and 0 <= axis - 1 < len(joint_names):
        return "J%d" % axis
    return "J%d" % axis


# --------------------------------------------------------------------- gains
def _array_source(identity, keys):
    """Return (values, field_name) for the first present list-like alias."""
    for key in keys:
        if key not in identity:
            continue
        value = identity.get(key)
        if value is None:
            return None, "%s=null" % key
        if isinstance(value, (list, tuple)):
            return list(value), "runtime_identity.%s" % key
        # A scalar under an array name is a contract violation, not a fallback.
        return _ScalarViolation(key, value), "runtime_identity.%s" % key
    return None, None


class _ScalarViolation:
    def __init__(self, key, value):
        self.key = key
        self.value = value


def _normalise_gain_channel(raw, axis, channel):
    """Validate one per-axis gain field; returns (value, reason)."""
    number = _finite(raw)
    if number is None:
        return None, "%s_AXIS_NONFINITE" % channel.upper()
    low, high = _AXIS_GAIN_BOUNDS[channel]
    if not (low < number <= high):
        return None, "%s_AXIS_OUT_OF_RANGE" % channel.upper()
    return number, None


def _dynamic_gain(axes, name, channel):
    values = axes.get(name) if isinstance(axes, dict) else None
    values = values if isinstance(values, dict) else {}
    for key in GAIN_DYNAMIC_KEYS[channel]:
        if key in values and values[key] is not None:
            return values[key], "dynamic_joint_state.%s" % key
    return None, None


def _ff_gain(ff_view, channel):
    if not isinstance(ff_view, dict):
        return None, None
    for key in ("%s_axis" % channel, "%s_per_axis" % channel):
        value = ff_view.get(key)
        if isinstance(value, (list, tuple)):
            return list(value), "gravity_ff_state.%s" % key
    return None, None


def parse_axis_gains(identity_payload=None, dynamic_axes=None, ff_view=None, *,
                     joint_count=5, joint_names=None):
    """Defensively parse per-axis applied/encoded Kp/Kd from driver evidence.

    Absent array + present legacy scalar -> ``SCALAR_ONLY``: the scalar is shown
    as the policy fallback but never claimed as the per-axis encoded value.
    Wrong length / non-finite / out-of-policy values are ``REJECTED`` for the
    affected channel; values are never clamped or padded.
    """
    try:
        joint_count = int(joint_count)
    except (TypeError, ValueError):
        joint_count = 5
    if joint_count < 1:
        joint_count = 1
    names = list(joint_names or ["slaihand_joint%d" % index
                                 for index in range(1, joint_count + 1)])
    identity = identity_payload if isinstance(identity_payload, dict) else {}
    axes = dynamic_axes if isinstance(dynamic_axes, dict) else {}

    scalar_kp = _finite(identity.get("kp"))
    scalar_kd = _finite(identity.get("kd"))
    kp_source_hint = _text(identity.get("kp_axis_source"))
    kd_source_hint = _text(identity.get("kd_axis_source"))

    kp_axis_values, kp_axis_source = _array_source(identity, GAIN_ALIASES["kp_axis"])
    kd_axis_values, kd_axis_source = _array_source(identity, GAIN_ALIASES["kd_axis"])
    kp_encoded_values, kp_encoded_source = _array_source(identity, GAIN_ALIASES["kp_encoded"])
    kd_encoded_values, kd_encoded_source = _array_source(identity, GAIN_ALIASES["kd_encoded"])

    if kp_axis_values is None:
        kp_axis_values, kp_axis_source = _ff_gain(ff_view, "kp")
    if kd_axis_values is None:
        kd_axis_values, kd_axis_source = _ff_gain(ff_view, "kd")

    def channel_state(values, source, channel, source_hint):
        """Return dict describing one per-axis array channel."""
        state = {"values": None, "source": source, "rejections": {},
                 "contract_error": None, "source_hint": source_hint}
        if values is None:
            return state
        if isinstance(values, _ScalarViolation):
            state["contract_error"] = "%s_AXIS_NOT_ARRAY" % channel.upper()
            return state
        if len(values) != joint_count:
            # Never pad or truncate a wrong-length gain vector.
            state["contract_error"] = "%s_AXIS_LENGTH: expected %d got %d" % (
                channel.upper(), joint_count, len(values))
            return state
        normalised = {}
        for axis in range(1, joint_count + 1):
            value, reason = _normalise_gain_channel(values[axis - 1], axis, channel)
            if reason:
                state["rejections"][axis] = reason
            normalised[axis] = value
        state["values"] = normalised
        return state

    kp_state = channel_state(kp_axis_values, kp_axis_source, "kp", kp_source_hint)
    kd_state = channel_state(kd_axis_values, kd_axis_source, "kd", kd_source_hint)
    kp_encoded_state = channel_state(kp_encoded_values, kp_encoded_source, "kp", None)
    kd_encoded_state = channel_state(kd_encoded_values, kd_encoded_source, "kd", None)

    sources = []
    rows = []
    for axis in range(1, joint_count + 1):
        name = names[axis - 1] if axis - 1 < len(names) else "slaihand_joint%d" % axis
        row = {"axis": axis, "name": name, "label": "J%d" % axis,
               "kp_applied": None, "kd_applied": None,
               "kp_encoded": None, "kd_encoded": None,
               "kp_configured": None, "kd_configured": None,
               "kp_source": None, "kd_source": None,
               "kp_encoded_source": None, "kd_encoded_source": None,
               "scalar_kp": scalar_kp, "scalar_kd": scalar_kd,
               "policy_cap": {"kp": POLICY_KP_MAX, "kd": POLICY_KD_MAX},
               "protocol_cap": {"kp": PROTOCOL_KP_MAX, "kd": PROTOCOL_KD_MAX},
               "status": "UNKNOWN", "reasons": []}

        def apply_channel(channel, state, dynamic_channel):
            value = None
            source = None
            reasons = []
            if state["contract_error"]:
                reasons.append(state["contract_error"])
            elif state["values"] is not None:
                if axis in state["rejections"]:
                    reasons.append(state["rejections"][axis])
                else:
                    value = state["values"][axis]
                    source = state["source"]
            else:
                dynamic_value, dynamic_source = _dynamic_gain(axes, name, dynamic_channel)
                if dynamic_value is not None:
                    normalised, reason = _normalise_gain_channel(dynamic_value, axis, channel)
                    if reason:
                        reasons.append(reason)
                    else:
                        value = normalised
                        source = dynamic_source
            return value, source, reasons

        kp_value, kp_value_source, kp_reasons = apply_channel("kp", kp_state, "kp")
        kd_value, kd_value_source, kd_reasons = apply_channel("kd", kd_state, "kd")
        row["kp_applied"] = kp_value
        row["kd_applied"] = kd_value
        row["kp_source"] = kp_value_source
        row["kd_source"] = kd_value_source
        if kp_encoded_state["values"] is not None and axis not in kp_encoded_state["rejections"]:
            row["kp_encoded"] = kp_encoded_state["values"][axis]
            row["kp_encoded_source"] = kp_encoded_state["source"]
        elif kp_encoded_state["contract_error"]:
            kp_reasons.append(kp_encoded_state["contract_error"])
        elif axis in kp_encoded_state["rejections"]:
            kp_reasons.append(kp_encoded_state["rejections"][axis])
        if kd_encoded_state["values"] is not None and axis not in kd_encoded_state["rejections"]:
            row["kd_encoded"] = kd_encoded_state["values"][axis]
            row["kd_encoded_source"] = kd_encoded_state["source"]
        elif kd_encoded_state["contract_error"]:
            kd_reasons.append(kd_encoded_state["contract_error"])
        elif axis in kd_encoded_state["rejections"]:
            kd_reasons.append(kd_encoded_state["rejections"][axis])

        # Legacy scalar is kept for compatibility, but only when no per-axis
        # array exists for that channel can it be shown as configuration.
        if kp_value is None:
            row["kp_configured"] = scalar_kp
        if kd_value is None:
            row["kd_configured"] = scalar_kd
        if kp_encoded_state["values"] is None and kp_value is not None:
            row["kp_encoded"] = None

        reasons = list(dict.fromkeys(kp_reasons + kd_reasons))
        if reasons:
            # An explicitly invalid/rejected contract value is never hidden by
            # a clamp or a scalar fallback.
            row["status"] = "REJECTED"
        elif kp_value is None and kd_value is None:
            if scalar_kp is not None or scalar_kd is not None:
                row["status"] = "SCALAR_ONLY"
                reasons = ["SCALAR_ONLY_APPLIED_UNPROVEN: 旧标量配置只证明策略值，"
                           "未证明逐轴实际编码 Kp/Kd"]
            else:
                row["status"] = "UNKNOWN"
                reasons = ["AXIS_GAIN_MISSING: 运行身份未提供逐轴或标量增益"]
        elif kp_value is None or kd_value is None:
            row["status"] = "PARTIAL"
            missing = []
            if kp_value is None:
                missing.append("Kp")
            if kd_value is None:
                missing.append("Kd")
            reasons = ["AXIS_GAIN_PARTIAL: %s 逐轴值缺失；不能补标量冒充逐轴实际"
                       % "、".join(missing)]
        else:
            row["status"] = "VERIFIED"
        row["reasons"] = reasons
        row["per_axis_verified"] = row["status"] == "VERIFIED"
        if row["status"] == "VERIFIED":
            hint = (kp_state["source_hint"] or "").lower()
            row["source_kind"] = "scalar_expanded" if hint == "scalar_legacy" else "per_axis"
        elif row["status"] == "SCALAR_ONLY":
            row["source_kind"] = "scalar_legacy"
        else:
            row["source_kind"] = None
        row["display"] = _gain_display(row)
        rows.append(row)
        for source in (kp_value_source, kd_value_source, row["kp_encoded_source"],
                       row["kd_encoded_source"]):
            if source and source not in sources:
                sources.append(source)

    all_verified = all(row["status"] == "VERIFIED" for row in rows)
    any_per_axis = any(row["status"] in ("VERIFIED", "PARTIAL", "REJECTED") for row in rows)
    reasons = []
    if not all_verified:
        if any(row["status"] == "SCALAR_ONLY" for row in rows):
            reasons.append("SCALAR_ONLY: 旧标量 Kp/Kd 仍兼容显示，但逐轴实际编码未证明")
        if any(row["status"] == "REJECTED" for row in rows):
            reasons.append("GAIN_CONTRACT_REJECTED: 逐轴增益数组长度/范围/数值不合格，"
                           "已拒绝该通道而不是截断或钳位")
        if any(row["status"] == "UNKNOWN" for row in rows):
            reasons.append("GAIN_UNKNOWN: 未收到逐轴增益")
    return {
        "schema": GAIN_VIEW_SCHEMA,
        "joint_count": joint_count,
        "axes": rows,
        "scalar_kp": scalar_kp,
        "scalar_kd": scalar_kd,
        "scalar_compatible": scalar_kp is not None and scalar_kd is not None,
        "all_axes_verified": all_verified,
        "per_axis_present": any_per_axis,
        "sources": sources,
        "reasons": reasons,
        "contract": {
            "canonical": ["kp_axis", "kd_axis", "kp_axis_source", "kd_axis_source",
                          "kp", "kd"],
            "aliases": {key: list(value) for key, value in GAIN_ALIASES.items()},
            "assumption": "kp_axis/kd_axis 为驱动接受并实际使用的逐轴有效值；"
                          "存在数组时不得用标量替代；编码量化值若提供则另列 encoded。",
        },
        "display": _gains_display(rows, all_verified),
    }


def _gain_display(row):
    encoded = ""
    if row["kp_encoded"] is not None or row["kd_encoded"] is not None:
        encoded = "（编码 Kp=%s Kd=%s）" % (_clamp_display(row["kp_encoded"]),
                                            _clamp_display(row["kd_encoded"]))
    if row["status"] == "VERIFIED":
        prefix = "逐轴实际（标量策略展开）" if row.get("source_kind") == "scalar_expanded" \
            else "逐轴实际"
        return "%s Kp=%s Kd=%s%s · %s" % (prefix, _clamp_display(row["kp_applied"]),
                                          _clamp_display(row["kd_applied"]), encoded,
                                          row["kp_source"] or row["kd_source"] or "—")
    if row["status"] == "SCALAR_ONLY":
        return "标量 Kp=%s Kd=%s（逐轴实际配置未证明%s）" % (
            _clamp_display(row["scalar_kp"]), _clamp_display(row["scalar_kd"]), encoded)
    if row["status"] == "PARTIAL":
        return "部分逐轴：Kp=%s Kd=%s%s（%s）" % (
            _clamp_display(row["kp_applied"]), _clamp_display(row["kd_applied"]),
            encoded, "；".join(row["reasons"]))
    return "UNKNOWN/拒绝：%s%s" % ("；".join(row["reasons"]) or "无数据", encoded)


def merge_encoded_gains(gains_view, first_fault_view):
    """Fill the encoded-gain columns from the frozen first-fault vectors.

    ``runtime_identity.kp_axis/kd_axis`` are the effective per-axis values; the
    actual *encoded* MIT packet fields are decoded into the first-fault snapshot's
    ``encoded_kp``/``encoded_kd`` vectors.  When the identity has no separate
    encoded field, show those vectors in the same table (source-labelled) rather
    than leaving the encoded columns empty.  Never replaces effective values.
    """
    if not isinstance(gains_view, dict) or not isinstance(first_fault_view, dict):
        return gains_view
    if not first_fault_view.get("available"):
        return gains_view
    vectors = first_fault_view.get("encoded_vectors") or {}
    kp_vector = vectors.get("kp")
    kd_vector = vectors.get("kd")
    joint_count = gains_view.get("joint_count")
    try:
        joint_count = int(joint_count)
    except (TypeError, ValueError):
        return gains_view
    if not isinstance(kp_vector, list) or len(kp_vector) != joint_count:
        return gains_view
    if not isinstance(kd_vector, list) or len(kd_vector) != joint_count:
        return gains_view
    filled = False
    for row in gains_view.get("axes") or []:
        axis = row.get("axis")
        if not isinstance(axis, int) or not 1 <= axis <= joint_count:
            continue
        kp_value = _finite(kp_vector[axis - 1])
        kd_value = _finite(kd_vector[axis - 1])
        if row.get("kp_encoded") is None and kp_value is not None:
            row["kp_encoded"] = kp_value
            row["kp_encoded_source"] = "first_fault.encoded_kp"
            filled = True
        if row.get("kd_encoded") is None and kd_value is not None:
            row["kd_encoded"] = kd_value
            row["kd_encoded_source"] = "first_fault.encoded_kd"
            filled = True
        row["display"] = _gain_display(row)
    if filled:
        gains_view["encoded_from_first_fault"] = True
        note = ("编码 Kp/Kd 取自驱动首因快照最后一次完整 MIT 写包（解码值）；"
                "运行身份 kp_axis/kd_axis 仍是逐轴实际配置值。")
        notes = list(gains_view.get("notes") or [])
        if note not in notes:
            notes.append(note)
        gains_view["notes"] = notes
    return gains_view


def _gains_display(rows, all_verified):
    if all_verified:
        if any(row.get("source_kind") == "scalar_expanded" for row in rows):
            return ("逐轴实际 Kp/Kd 已确认：标量策略展开到各轴（旧标量兼容）；"
                    "未使用各轴不同配置。编码值另列（来自首因快照或运行身份）。")
        return "逐轴实际 Kp/Kd 已由驱动运行身份字段确认"
    if any(row["status"] == "SCALAR_ONLY" for row in rows):
        return "仅有旧标量 Kp/Kd（兼容显示）；逐轴实际编码值未证明，不能当作逐轴调参依据"
    return "逐轴 Kp/Kd 未确认：等待驱动的 kp_axis/kd_axis 或标注 UNKNOWN"


# ---------------------------------------------------------------- first fault
def _unwrap_first_fault(payload):
    """Return (fault_object_or_None, wrapper_dict)."""
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except (TypeError, ValueError):
            return None, {}
    if not isinstance(payload, dict):
        return None, {}
    wrapper = payload
    for key in ("first_fault", "last_first_fault", "driver_first_fault"):
        if key in payload:
            return payload.get(key), wrapper
    if payload.get("schema") == FIRST_FAULT_SCHEMA:
        return payload, {}
    # A bare object with driver-fault fields is accepted defensively.
    if any(key in payload for key in ("error_deg", "error_rad", "following_error_deg")):
        return payload, {}
    return None, wrapper


def _fault_degrees(fault, direct_key, rad_key):
    value = _finite(_pick(fault, _FAULT_FIELD_ALIASES[direct_key]))
    if value is not None:
        return value, "deg"
    value = _finite(_pick(fault, _FAULT_FIELD_ALIASES[rad_key]))
    if value is not None:
        return math.degrees(value), "rad"
    return None, None


def _fault_raw_number(fault, key):
    return _pick(fault, _FAULT_FIELD_ALIASES[key])


def parse_first_fault(payload, *, joint_count=5, joint_names=None, received_at=None):
    """Parse the frozen driver first-cause snapshot; never promotes a JTC error."""
    try:
        joint_count = int(joint_count)
    except (TypeError, ValueError):
        joint_count = 5
    names = list(joint_names or [])
    fault, wrapper = _unwrap_first_fault(payload)
    base = {
        "schema": "slaihand_first_fault_view/1",
        "available": False,
        "frozen": None,
        "layer": DRIVER_LAYER,
        "code": None,
        "axis": None,
        "axis_label": None,
        "axis_scope": None,
        "joint": None,
        "error_deg": None, "limit_deg": None,
        "target_deg": None, "actual_deg": None, "target_actual_available": False,
        "feedback_age_s": None, "feedback_valid": None,
        "encoded_kp": None, "encoded_kd": None, "encoded_tau_ff_nm": None,
        "request_estimate": estimate_view(None),
        "command_id": None, "cycle_id": None, "boot_id": None,
        "detected_at": None, "last_complete_write_at": None,
        "last_write_age_s": None,
        "exceeded": None, "reasons": [], "display": None,
        "source": None,
    }
    if not isinstance(fault, dict):
        base["reasons"] = ["FIRST_FAULT_MISSING: 运行身份中没有驱动首因快照"]
        base["display"] = "驱动首因：未提供（未知，不能由后续 JTC 错误推定）"
        return base

    reasons = []
    schema = _text(fault.get("schema"))
    if schema is not None and schema != FIRST_FAULT_SCHEMA:
        base["reasons"] = ["FIRST_FAULT_SCHEMA_UNKNOWN: %s" % schema]
        base["display"] = "驱动首因：schema 不符（拒绝解析，不猜测）"
        return base

    layer = _text(fault.get("layer"))
    if layer is not None and layer.upper() != DRIVER_LAYER:
        base["reasons"] = ["FIRST_FAULT_NOT_DRIVER: layer=%s，JTC 层结果不能当驱动首因" % layer]
        base["display"] = "驱动首因：非驱动层（拒绝把 JTC 当首因）"
        return base

    code = _text(_pick(fault, _FAULT_FIELD_ALIASES["code"]))
    base["code"] = code.upper() if code else None
    base["source"] = ("runtime_identity.first_fault" if wrapper
                      else "standalone_first_fault")
    base["frozen"] = fault.get("frozen") is True

    # ---- axis attribution (1-based; C++ uses 0 for "not axis-specific").
    raw_axis = _pick(fault, _FAULT_FIELD_ALIASES["axis"])
    axis = _axis_index(raw_axis, joint_count)
    joint = _text(_pick(fault, _FAULT_FIELD_ALIASES["joint"]))
    if axis is None and joint:
        match = re.search(r"(\d+)$", joint)
        if match:
            axis = _axis_index(int(match.group(1)), joint_count)
    if axis is None:
        numeric_axis = _finite(raw_axis)
        if numeric_axis == 0:
            base["axis_scope"] = "not_axis_specific"
            base["axis_label"] = "全轴/非单轴"
            base["reasons"].append("FIRST_FAULT_AXIS_NOT_SPECIFIC: axis=0，不归到 J1")
        elif numeric_axis is not None:
            base["axis_scope"] = "invalid"
            base["reasons"].append("FIRST_FAULT_AXIS_INVALID: axis=%s" % _clamp_display(numeric_axis))
        else:
            base["axis_scope"] = "missing"
            base["reasons"].append("FIRST_FAULT_AXIS_MISSING")
    else:
        base["axis"] = axis
        base["axis_label"] = _clean_axis_label(axis, names)
        base["axis_scope"] = "axis"
    base["joint"] = joint or ("slaihand_joint%d" % axis if axis else None)

    # ---- error / limit / target / actual with explicit unit handling.
    error_deg, error_unit = _fault_degrees(fault, "error_deg", "error_rad")
    if error_deg is None:
        legacy = _finite(fault.get("error"))
        if legacy is not None:
            unit = _text(fault.get("error_unit")) or "deg"
            error_deg = math.degrees(legacy) if unit.lower().startswith("rad") else legacy
            error_unit = unit
    limit_deg, limit_unit = _fault_degrees(fault, "limit_deg", "limit_rad")
    limit_source = "first_fault" if limit_deg is not None else None
    if limit_deg is None:
        fallback = _finite(wrapper.get("following_error_deg")) if wrapper else None
        if fallback is not None:
            limit_deg = fallback
            limit_source = "runtime_identity.following_error_deg"
    target_deg, _ = _fault_degrees(fault, "target_deg", "target_rad")
    actual_deg, _ = _fault_degrees(fault, "actual_deg", "actual_rad")
    base.update({"error_deg": error_deg, "limit_deg": limit_deg,
                 "limit_source": limit_source, "error_unit": error_unit,
                 "limit_unit": limit_unit,
                 "target_deg": target_deg, "actual_deg": actual_deg,
                 "target_actual_available": target_deg is not None and actual_deg is not None})
    if error_deg is None:
        reasons.append("FIRST_FAULT_ERROR_MISSING")
    elif error_deg < 0:
        reasons.append("FIRST_FAULT_ERROR_INVALID")
    if limit_deg is None:
        reasons.append("FIRST_FAULT_LIMIT_MISSING: 快照/运行身份都未提供驱动门限")
    if error_deg is not None and limit_deg is not None:
        exceeded = error_deg >= limit_deg - 1e-12
        base["exceeded"] = exceeded
        if not exceeded:
            reasons.append("FIRST_FAULT_BELOW_LIMIT: 快照误差 %.4f° < 门限 %.4f°（保留原样，不推断）"
                           % (error_deg, limit_deg))

    base["feedback_age_s"] = _finite(_pick(fault, _FAULT_FIELD_ALIASES["feedback_age_s"]))
    valid = _pick(fault, _FAULT_FIELD_ALIASES["feedback_valid"])
    base["feedback_valid"] = None if valid is None else bool(valid)
    if base["feedback_age_s"] is None:
        reasons.append("FIRST_FAULT_FEEDBACK_AGE_MISSING")

    # ---- encoded per-axis values decoded from the last complete MIT batch.
    encoded_kp_raw = fault.get("encoded_kp", fault.get("kp"))
    encoded_kd_raw = fault.get("encoded_kd", fault.get("kd"))
    tau_ff_raw = _pick(fault, _FAULT_FIELD_ALIASES["tau_ff_nm"])
    base["encoded_kp"] = _finite(_axis_scalar(encoded_kp_raw, axis))
    base["encoded_kd"] = _finite(_axis_scalar(encoded_kd_raw, axis))
    base["encoded_tau_ff_nm"] = _finite(_axis_scalar(tau_ff_raw, axis))
    # Keep the full per-axis vectors too: a non-axis-specific fault (axis=0) or
    # a fault without an axis index must still be evidenced without guessing.
    vectors = {}
    for key, raw in (("kp", encoded_kp_raw), ("kd", encoded_kd_raw),
                     ("tau_ff_nm", tau_ff_raw)):
        if isinstance(raw, (list, tuple)):
            vectors[key] = [None if _finite(value) is None else round(_finite(value), 6)
                            for value in raw]
    base["encoded_vectors"] = vectors or None
    if base["encoded_kp"] is None or base["encoded_kd"] is None:
        reasons.append("FIRST_FAULT_GAIN_MISSING: 快照未含编码 Kp/Kd")
    if base["encoded_tau_ff_nm"] is None and "tau_ff_nm" not in vectors:
        reasons.append("FIRST_FAULT_TAU_FF_MISSING")

    base["command_id"] = _finite(_pick(fault, _FAULT_FIELD_ALIASES["command_id"]))
    if base["command_id"] is not None:
        base["command_id"] = int(base["command_id"])
    base["cycle_id"] = _finite(_pick(fault, _FAULT_FIELD_ALIASES["cycle_id"]))
    if base["cycle_id"] is not None:
        base["cycle_id"] = int(base["cycle_id"])
    base["boot_id"] = _text(_pick(fault, _FAULT_FIELD_ALIASES["boot_id"])) \
        or _text(wrapper.get("boot_id"))
    detected_at = _finite(_pick(fault, _FAULT_FIELD_ALIASES["detected_at"]))
    base["detected_at"] = detected_at if detected_at is not None and detected_at >= 0 \
        else None
    last_write = _finite(_pick(fault, _FAULT_FIELD_ALIASES["last_complete_write_at"]))
    # The C++ snapshot uses -1 as "no complete MIT write batch yet"; JSON still
    # renders that finite sentinel, so never treat it as a real timestamp.
    base["last_complete_write_at"] = last_write if last_write is not None and last_write >= 0 \
        else None
    if (base["detected_at"] is not None and base["last_complete_write_at"] is not None
            and base["detected_at"] >= base["last_complete_write_at"]):
        base["last_write_age_s"] = round(
            base["detected_at"] - base["last_complete_write_at"], 6)
    if base["last_complete_write_at"] is None:
        reasons.append("FIRST_FAULT_LAST_WRITE_MISSING")
    if base["boot_id"] is None:
        reasons.append("FIRST_FAULT_BOOT_MISSING")
    if base["command_id"] is None:
        reasons.append("FIRST_FAULT_COMMAND_MISSING")
    if base["cycle_id"] is None:
        reasons.append("FIRST_FAULT_CYCLE_MISSING")

    # ---- frozen request estimate: last-active/stale, not current output.
    state = _text(fault.get("request_estimate_state")) \
        or _text(wrapper.get("request_estimate_state"))
    raw_estimate = _axis_scalar(_pick(fault, _FAULT_FIELD_ALIASES["request_estimate_nm"]),
                                axis or 0)
    estimate_at = _finite(_pick(fault, _FAULT_FIELD_ALIASES["request_estimate_at"]))
    if estimate_at is not None and estimate_at < 0:
        estimate_at = None
    base["request_estimate"] = estimate_view(
        raw_estimate, state=state, source="first_fault.request_estimate_nm",
        cycle=base["cycle_id"], command_id=base["command_id"], at=estimate_at)
    base["request_estimate_state"] = base["request_estimate"]["state"]

    base["available"] = True
    base["reasons"] = list(dict.fromkeys(base["reasons"] + reasons))
    base["display"] = _first_fault_display(base)
    return base


def _first_fault_display(view):
    if not view.get("available"):
        return view.get("display") or "驱动首因：未知"
    parts = ["驱动首因（%s）" % (view.get("code") or "未知代码")]
    label = view.get("axis_label") or "轴未定位"
    if view.get("error_deg") is not None and view.get("limit_deg") is not None:
        parts.append("%s 误差 %.4f° / 驱动门限 %.2f°%s"
                     % (label, view["error_deg"], view["limit_deg"],
                        "（已超过）" if view.get("exceeded") else "（未超过，保留原样）"))
    elif view.get("error_deg") is not None:
        parts.append("%s 误差 %.4f°（门限未提供）" % (label, view["error_deg"]))
    if view.get("target_deg") is not None and view.get("actual_deg") is not None:
        parts.append("目标 %.4f° / 实际 %.4f°" % (view["target_deg"], view["actual_deg"]))
    if view.get("encoded_kp") is not None:
        parts.append("编码 Kp=%s Kd=%s" % (_clamp_display(view["encoded_kp"]),
                                          _clamp_display(view["encoded_kd"])))
    if view.get("cycle_id") is not None:
        parts.append("cycle=%s" % view["cycle_id"])
    if view.get("command_id") is not None:
        parts.append("command=%s" % view["command_id"])
    return "；".join(parts)


# ------------------------------------------------------- JTC / guard config
def parse_jtc_config(config_text, *, joint_names=None):
    """Parse per-joint JTC ``trajectory``/``goal`` tolerances from YAML text.

    Display-only: this reader never writes a controller parameter.
    """
    names = list(joint_names or [])
    joints = {}
    current = None
    order = []
    for raw in str(config_text or "").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        match = re.search(r"slaihand_joint(\d+)\s*:", line)
        if match:
            current = "slaihand_joint%s" % match.group(1)
            if current not in joints:
                joints[current] = {"joint": current, "trajectory_rad": None,
                                   "trajectory_deg": None, "goal_rad": None,
                                   "goal_deg": None}
                order.append(current)
            inline = re.search(r"\{(.*)\}", line)
            if inline:
                body = inline.group(1)
                for key, field in (("trajectory", "trajectory_rad"), ("goal", "goal_rad")):
                    value = re.search(r"%s\s*:\s*([0-9.eE+-]+)" % key, body)
                    if value:
                        rad = _finite(value.group(1))
                        if rad is not None:
                            joints[current][field] = rad
                            joints[current][field.replace("_rad", "_deg")] = math.degrees(rad)
            continue
        if current is None:
            continue
        for key, field in (("trajectory", "trajectory_rad"), ("goal", "goal_rad")):
            value = re.match(r"\s*%s\s*:\s*([0-9.eE+-]+)" % key, line)
            if value:
                rad = _finite(value.group(1))
                if rad is not None:
                    joints[current][field] = rad
                    joints[current][field.replace("_rad", "_deg")] = math.degrees(rad)
    rows = [row for row in (joints[name] for name in order)
            if row["trajectory_deg"] is not None or row["goal_deg"] is not None]
    trajectories = [row["trajectory_deg"] for row in rows
                    if row["trajectory_deg"] is not None]
    tolerance = None
    mixed = False
    if trajectories:
        tolerance = trajectories[0]
        mixed = any(abs(value - tolerance) > 1e-9 for value in trajectories[1:])
    goals = [row["goal_deg"] for row in rows if row["goal_deg"] is not None]
    goal_tolerance = goals[0] if goals else None
    return {
        "joints": {row["joint"]: row for row in rows},
        "tolerance_deg": None if tolerance is None else round(tolerance, 6),
        "goal_tolerance_deg": None if goal_tolerance is None else round(goal_tolerance, 6),
        "mixed": mixed,
        "joint_count": len(rows),
        "source": "config/joint_trajectory_controller.constraints",
    }


def parse_driver_guard_deg(policy_text):
    """Parse the driver FOLLOWING_ERROR guard from the policy YAML text."""
    match = re.search(r"^\s*following_error_deg\s*:\s*([0-9.eE+-]+)",
                      str(policy_text or ""), re.MULTILINE)
    if not match:
        return None
    return _finite(match.group(1))


# --------------------------------------------------------- request estimates
def estimate_view(value_nm, *, state=None, active=None, source=None, cycle=None,
                  command_id=None, at=None, note=None):
    """View a torque request estimate without ever calling it measured output.

    ``request_estimate_state`` from the driver wins; otherwise a frozen/inactive
    context marks the value ``LAST_ACTIVE_STALE``.  A request estimate is always
    ``is_current_output=False`` because it is computed from a command packet and
    the last feedback, not from a torque sensor.
    """
    state_text = _text(state)
    normalized = None
    if state_text:
        candidate = state_text.strip().upper().replace("-", "_")
        normalized = _REQUEST_ESTIMATE_STATE_ALIASES.get(candidate)
        if normalized is None and candidate in _REQUEST_ESTIMATE_STATES:
            normalized = candidate
    if normalized is None:
        if active is True:
            normalized = REQUEST_ESTIMATE_CURRENT
        elif _finite(value_nm) is not None:
            normalized = REQUEST_ESTIMATE_LAST_ACTIVE_STALE
        else:
            normalized = REQUEST_ESTIMATE_UNAVAILABLE
    labels = {
        REQUEST_ESTIMATE_CURRENT: "当前控制周期的请求估算（仍不是实测输出）",
        REQUEST_ESTIMATE_LAST_ACTIVE_STALE:
            "last-active/stale（最后一次完整控制周期的请求估算；停止/故障后不是当前输出）",
        REQUEST_ESTIMATE_UNAVAILABLE: "不可用",
    }
    return {
        "value_nm": _finite(value_nm),
        "state": normalized,
        "label": labels[normalized],
        "stale": normalized == REQUEST_ESTIMATE_LAST_ACTIVE_STALE,
        "is_current_output": False,
        "source": source,
        "cycle_id": cycle,
        "command_id": command_id,
        "at": at,
        "note": note or ("请求估算由命令包与上次反馈推算，可以是预筛查，"
                         "不是实际电流/扭矩或实测输出。"),
    }


# --------------------------------------------------------------- two layers
def later_events_from(operation=None, stop=None, *, current_jtc_error_deg=None,
                      jtc_tolerance_deg=None):
    """Collect post-first-cause events (JTC / operator stop) as *later* evidence."""
    events = []
    operation = operation if isinstance(operation, dict) else {}
    result = operation.get("result") if isinstance(operation.get("result"), dict) else {}
    if result:
        code = result.get("trajectory_error_code")
        message = _text(result.get("trajectory_error_string"))
        stop_error = _text(result.get("stop_error"))
        failed = ((isinstance(code, (int, float)) and int(code) != 0)
                  or message is not None or stop_error is not None)
        if failed:
            events.append({"layer": JTC_LAYER, "role": "后续（JTC 结果，非首因）",
                           "code": code, "message": message or stop_error,
                           "source": "operation.result.trajectory_error"})
    stop_reason = _text(operation.get("stop_reason"))
    if stop_reason:
        events.append({"layer": "OPERATOR/STOP", "role": "后续（操作员/全局停止，非首因）",
                       "code": stop_reason, "message": stop_reason,
                       "source": "operation.stop_reason"})
    stop = stop if isinstance(stop, dict) else {}
    if stop.get("state") in ("requested", "confirmed") and stop.get("reason"):
        events.append({"layer": "OPERATOR/STOP", "role": "后续（操作员/全局停止，非首因）",
                       "code": stop.get("reason"), "message": stop.get("reason"),
                       "source": "panel.stop"})
    if current_jtc_error_deg is not None:
        errors = [value for value in current_jtc_error_deg if _finite(value) is not None]
        if errors:
            worst = max(errors, key=abs)
            exceeded = (jtc_tolerance_deg is not None
                        and abs(worst) >= jtc_tolerance_deg - 1e-12)
            if exceeded:
                events.append({"layer": JTC_LAYER,
                               "role": "后续（JTC 当前误差，非首因）",
                               "code": "JTC_FOLLOWING_ERROR",
                               "message": "JTC 参考误差最大 %.4f°（JTC 门限 %.2f°）"
                                          % (worst, jtc_tolerance_deg),
                               "source": "controller_state.error"})
    for index, event in enumerate(events, 1):
        event["order"] = index
        event["is_first_cause"] = False
    return events


def two_layer_view(first_fault, *, jtc_config=None, driver_guard_deg=None,
                   operation=None, stop=None, current_jtc_error_deg=None,
                   joint_names=None):
    """Build the explicit driver-first-cause vs later-JTC two-layer view.

    The driver layer is always first when available.  Later OPERATOR_GLOBAL_STOP
    or JTC errors are appended as later events and never replace the first
    cause.  Without a driver snapshot the panel says the first cause is unknown
    instead of attributing the JTC error.
    """
    jtc_config = jtc_config if isinstance(jtc_config, dict) else {}
    jtc_tolerance = jtc_config.get("tolerance_deg")
    jtc_goal = jtc_config.get("goal_tolerance_deg")
    guard = driver_guard_deg
    first = first_fault if isinstance(first_fault, dict) else {}
    available = first.get("available") is True
    later = later_events_from(operation, stop,
                              current_jtc_error_deg=current_jtc_error_deg,
                              jtc_tolerance_deg=_finite(jtc_tolerance))
    current_errors = None
    if current_jtc_error_deg is not None:
        current_errors = [None if _finite(value) is None else round(_finite(value), 4)
                          for value in current_jtc_error_deg]

    limit = first.get("limit_deg") if available else None
    limit_source = first.get("limit_source") if available else None
    if limit is None and guard is not None:
        limit = guard
        limit_source = "policy.following_error_deg"
    mismatch = None
    mismatch_deg = None
    if _finite(limit) is not None and _finite(jtc_tolerance) is not None:
        mismatch = abs(_finite(limit) - _finite(jtc_tolerance)) > 1e-6
        mismatch_deg = round(_finite(jtc_tolerance) - _finite(limit), 6)

    layers = [{
        "order": 1,
        "layer": DRIVER_LAYER,
        "role": "首因（驱动层，优先保留）",
        "available": available,
        "code": first.get("code") if available else None,
        "axis": first.get("axis") if available else None,
        "axis_label": first.get("axis_label") if available else None,
        "error_deg": first.get("error_deg") if available else None,
        "limit_deg": limit,
        "limit_source": limit_source,
        "detected_at": first.get("detected_at") if available else None,
        "frozen": first.get("frozen") if available else None,
        "display": first.get("display") if available else
        "未收到驱动层首因快照（UNKNOWN）；后续 JTC 误差不能推定为驱动首因",
    }, {
        "order": 2,
        "layer": JTC_LAYER,
        "role": "后续（JTC 路径层，非首因）",
        "available": jtc_tolerance is not None,
        "tolerance_deg": jtc_tolerance,
        "goal_tolerance_deg": jtc_goal,
        "source": jtc_config.get("source"),
        "current_error_deg": current_errors,
        "display": "JTC 路径门限 %s°，目标门限 %s°%s"
                   % (_clamp_display(jtc_tolerance), _clamp_display(jtc_goal),
                      "（当前误差最大 %.4f°）" % max(
                          (abs(value) for value in current_errors
                           if value is not None), default=0.0)
                      if current_errors else ""),
    }]

    recommended = _finite(guard) if _finite(guard) is not None else (
        _finite(limit) if _finite(limit) is not None else None)
    matched_thresholds = mismatch is False
    recommendation = {
        "status": "MATCHED_NO_CHANGE_NEEDED" if matched_thresholds
        else "CANDIDATE_NOT_DEPLOYED",
        "apply_now": False,
        "current_jtc_tolerance_deg": jtc_tolerance,
        "recommended_jtc_tolerance_deg": None if matched_thresholds else recommended,
        "driver_guard_deg": _finite(guard),
        "mismatch": mismatch,
        "mismatch_deg": mismatch_deg,
        # The protected client sets goal_tolerance explicitly per goal; only the
        # YAML trajectory (path) tolerance is a config-only candidate.
        "client_goal_tolerance": {
            "normal_plan_deg": CLIENT_GOAL_TOLERANCE_DEG,
            "teaching_deg": CLIENT_TEACHING_GOAL_TOLERANCE_DEG,
            "calibration_deg": CLIENT_CALIBRATION_GOAL_TOLERANCE_DEG,
            "plain_delta_goal": "未显式设置，沿用 YAML goal 1°",
            "source": CLIENT_GOAL_TOLERANCE_SOURCE,
            "note": "goal 容差按模式由客户端显式覆盖，只改 YAML goal 不生效。",
        },
        "changes": (["JTC 路径门限 %s° 已与驱动门限 %s° 一致；无需恢复（四轴模式），"
                     "仍不改变负载偏差" % (_clamp_display(jtc_tolerance),
                                         _clamp_display(limit))]
                    if matched_thresholds else
                    ["config/canusb_five_axis_controllers.yaml: "
                     "constraints.slaihand_joint1..5.trajectory 从 %s° 恢复为 %s°（候选，未部署）"
                     % (_clamp_display(jtc_tolerance), _clamp_display(recommended)),
                     "客户端 goal_tolerance 另按模式覆盖：普通计划 %s°、示教/标定 %s°；"
                     "候选只针对 YAML 路径 trajectory，不能只改 YAML goal"
                     % (_clamp_display(CLIENT_GOAL_TOLERANCE_DEG),
                        _clamp_display(CLIENT_CALIBRATION_GOAL_TOLERANCE_DEG))]),
        "note": ("JTC 路径门限已与驱动门限一致：只说明保护层级不误导，"
                 "不构成跟踪性能或 1.5 倍裕量结论；本轮不改配置、不部署、不重启。"
                 if matched_thresholds else
                 "把 JTC 路径门限恢复为驱动 4° 只消除两层门限不一致的误导，"
                 "不能消除负载偏差/跟随误差本身；客户端 goal_tolerance 另按模式覆盖"
                 "（普通计划 0.5°、示教/标定 4°）。本轮不改配置、不部署、不重启。"),
    }
    if available:
        headline = "首因：驱动层 %s%s；后续 JTC/操作员停止不替代首因" % (
            first.get("code") or "故障",
            "（%s %.4f° ≥ 门限 %.2f°）" % (first.get("axis_label") or "轴未定位",
                                          first.get("error_deg"),
                                          first.get("limit_deg"))
            if first.get("error_deg") is not None and limit is not None else "")
    elif later:
        headline = ("驱动层首因快照缺失（UNKNOWN）；存在 %d 条后续事件，"
                    "但不能把 JTC/停止事件写成通电时首因" % len(later))
    else:
        headline = "暂无驱动首因快照（UNKNOWN）"
    return {
        "schema": "slaihand_five_axis_two_layer/1",
        "layers": layers,
        "first_cause": first if available else None,
        "first_cause_available": available,
        "first_cause_preserved": available,
        "later_events_present": bool(later),
        "first_cause_missing_reason": None if available else
        "FIRST_CAUSE_MISSING: 未收到驱动层 FOLLOWING_ERROR 快照；不把后续 JTC 错误当首因",
        "later_events": later,
        "driver_guard_deg": _finite(guard),
        "jtc_tolerance_deg": jtc_tolerance,
        "jtc_goal_tolerance_deg": jtc_goal,
        "jtc_current_error_deg": current_errors,
        "thresholds": {
            "driver_guard_deg": _finite(guard),
            "driver_first_fault_limit_deg": limit,
            "driver_limit_source": limit_source,
            "jtc_tolerance_deg": jtc_tolerance,
            "jtc_goal_tolerance_deg": jtc_goal,
            "matched": None if mismatch is None else not mismatch,
            "mismatch_deg": mismatch_deg,
        },
        "recommendation": recommendation,
        "headline": headline,
        "note": "两层诊断：驱动层 4° FOLLOWING_ERROR 是首因；JTC 10° 是后续路径容差。"
                "不要用失能后的大误差判定通电时跟踪性能。",
    }


# ---------------------------------------------------------------- summary
def build_summary(gains=None, two_layer=None, request_estimate=None):
    """Compact status summary for the overview chip/card."""
    two_layer = two_layer if isinstance(two_layer, dict) else {}
    gains = gains if isinstance(gains, dict) else {}
    thresholds = two_layer.get("thresholds") or {}
    estimate = request_estimate if isinstance(request_estimate, dict) else {}
    if two_layer.get("first_cause_available"):
        cause = two_layer.get("first_cause") or {}
        parts = ["驱动首因", cause.get("axis_label") or "轴未定位"]
        if cause.get("error_deg") is not None:
            parts.append("%.4f°" % cause["error_deg"])
        if cause.get("limit_deg") is not None:
            parts.append("≥ %.2f° 门限" % cause["limit_deg"])
        cause_text = " ".join(parts)
    else:
        cause_text = "驱动首因 UNKNOWN"
    return {
        "first_cause": two_layer.get("headline"),
        "first_cause_text": cause_text,
        "driver_limit_deg": thresholds.get("driver_guard_deg"),
        "driver_first_fault_limit_deg": thresholds.get("driver_first_fault_limit_deg"),
        "jtc_tolerance_deg": thresholds.get("jtc_tolerance_deg"),
        "thresholds_matched": thresholds.get("matched"),
        "gains_verified": gains.get("all_axes_verified") is True,
        "request_estimate_label": estimate.get("label"),
        "request_estimate_stale": estimate.get("stale"),
        "request_estimate_state": estimate.get("state"),
    }
