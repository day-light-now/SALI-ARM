"""Pure J5 motion calibration artifact builder/validator (admission, no ROS).

An artifact binds the *current* hardware files plus one real timeout-setup
evidence JSON into a canonical payload and a canonical payload SHA256.  It is an
admission record, not a measurement: every value is copied from the files it
names and no missing calibration value is invented.  There is no ROS import and
no hardware access here; the candidate launch validates the artifact and only
then passes ``j5_qualified=true`` to the driver.

Chain of evidence required before an artifact can be built:

1. ``canusb_five_axis_motion_policy.yaml``  position-only candidate policy
   (``gravity_ff_alpha == 0`` and ``gravity_float_enabled == false``);
2. ``load_3p5kg_provisional.yaml``          PROVISIONAL load declaration whose
   actual file SHA256 is bound as ``mass_profile_sha256`` (never ``no_end_load``);
3. ``hardware_mapping.yaml``                J5 sign/zero/limits/profile, and the
   J5 watchdog declaration must already be non-zero (it stays ``0`` until Codex
   executes the timeout setup tool against real hardware);
4. ``j5_timeout_setup_evidence.json``       a successful real write+readback or
   read-only ``verified_existing`` check with zero parameter writes
   (``timeout_after == readback == 2000``, all five axes disabled/state 0).

While item 3 is still ``0`` the builder refuses with ``TIMEOUT_SOURCE_ZERO``: no
artifact can exist before the real timeout execution is recorded.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

import load_declaration as _load_declaration
import control_panel_model as _panel_model

SCHEMA = "slaihand_j5_motion_calibration/1"
EVIDENCE_SCHEMA = "slaihand_j5_timeout_setup/1"
QUALIFICATION = "j5_motion"
JOINT = "slaihand_joint5"
J5_AXIS = 5
AXIS_COUNT = 5
EXPECTED_TIMEOUT = 2000
EXPECTED_MASTER_ID = 0
EXPECTED_CONTROL_MODE = 1
EXPECTED_WRITE_DATA_HEX = "05 00 55 09 d0 07 00 00"
DEFAULT_MAX_AGE_S = 1800.0          # 30 minutes, per Lead decision #4
DEFAULT_MAX_EVIDENCE_AGE_S = 1800.0
MAX_CLOCK_SKEW_S = 300.0
EVIDENCE_FEEDBACK_MAX_AGE_S = 0.25
POLICY_PROFILE = "candidate_five_axis_native_mit_j5_gated"

_HASH_CODE = {
    "mapping_sha256": "MAPPING_HASH_MISMATCH",
    "model_sha256": "MODEL_HASH_MISMATCH",
    "policy_sha256": "POLICY_HASH_MISMATCH",
    "load_declaration_sha256": "LOAD_HASH_MISMATCH",
}


class AdmissionError(ValueError):
    """An artifact was rejected; ``code`` is a stable machine-readable reason."""

    def __init__(self, code, message):
        super().__init__("%s: %s" % (code, message))
        self.code = code
        self.message = message


# --------------------------------------------------------------------- basics
def _require(condition, code, message):
    if not condition:
        raise AdmissionError(code, message)


def sha256_file(path):
    """SHA256 of the actual bytes on disk."""
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def payload_sha256(payload):
    return hashlib.sha256(canonical_json(payload).encode("utf-8")).hexdigest()


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _integer(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value) if math.isfinite(value) and value.is_integer() else None
    if isinstance(value, str):
        try:
            return int(value.strip())
        except ValueError:
            return None
    return None


def _text(value):
    if not isinstance(value, str):
        return None
    value = value.strip()
    return value or None


def _loose_number(value):
    """Number from a mapping scalar; the tiny mapping reader keeps list items as
    text, so ``limits_rad`` entries arrive as numeric strings."""
    number = _number(value)
    if number is not None:
        return number
    if isinstance(value, str):
        try:
            number = float(value.strip())
        except ValueError:
            return None
        return number if math.isfinite(number) else None
    return None


def _sha256_text(value):
    text = _text(value)
    if text is None or len(text) != 64:
        return None
    try:
        int(text, 16)
    except ValueError:
        return None
    return text.lower()


def _same_float(left, right, tolerance=1e-12):
    first, second = _number(left), _number(right)
    if first is None or second is None:
        return False
    return abs(first - second) <= tolerance * max(1.0, abs(first), abs(second))


def _is_false(value):
    return value is False


def _parse_wall_time(value):
    """Parse the evidence wall timestamp into epoch seconds; ``None`` invalid."""
    text = _text(value)
    if text is None:
        return None
    candidate = text[:-1] + "+00:00" if text.endswith("Z") else text
    try:
        parsed = datetime.fromisoformat(candidate)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.timestamp()


# ------------------------------------------------------------------- policies
def read_policy(path):
    """Load the candidate motion policy YAML and require position-only gravity."""
    path = Path(path)
    _require(path.is_file(), "POLICY_MISSING", "策略文件不存在：%s" % path)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise AdmissionError("POLICY_PARSE", "策略 YAML 解析失败：%s" % error)
    _require(isinstance(document, dict), "POLICY_SHAPE", "策略必须是映射对象")
    alpha = _number(document.get("gravity_ff_alpha"))
    _require(alpha is not None and alpha == 0.0, "POLICY_GRAVITY_ENABLED",
             "候选策略 gravity_ff_alpha 必须为 0；暂定负载不允许重力前馈")
    _require(_is_false(document.get("gravity_float_enabled")), "POLICY_FLOAT_ENABLED",
             "候选策略 gravity_float_enabled 必须为 false；暂定负载不允许 float")
    profile = _text(document.get("profile"))
    _require(profile == POLICY_PROFILE, "POLICY_PROFILE",
             "候选策略 profile 必须是 %s" % POLICY_PROFILE)
    return {"document": document, "profile": profile, "gravity_ff_alpha": 0.0,
            "gravity_float_enabled": False}


# -------------------------------------------------------------- mapping reader
def read_mapping_j5(mapping_path):
    """Read only the ``slaihand_joint5`` record from ``hardware_mapping.yaml``.

    Reuses the panel's canonical tiny reader (``control_panel_model``) so the
    launch path, the panel and this admission tool cannot drift apart.
    """
    path = Path(mapping_path)
    _require(path.is_file(), "MAPPING_MISSING", "映射文件不存在：%s" % path)
    record = _panel_model.parse_j5_mapping(path.read_text(encoding="utf-8"))
    _require(isinstance(record, dict) and record, "MAPPING_J5_MISSING",
             "hardware_mapping.yaml 缺少 slaihand_joint5 记录")

    can_id = _integer(record.get("can_id"))
    _require(can_id == J5_AXIS, "MAPPING_J5_CAN_ID",
             "J5 can_id 必须是 %d，实际 %r" % (J5_AXIS, record.get("can_id")))
    sign = _integer(record.get("hardware_sign"))
    _require(sign in (1, -1), "MAPPING_J5_SIGN",
             "J5 hardware_sign 未标定（必须为 +1/-1）")
    zero = _number(record.get("zero_offset_rad"))
    _require(zero is not None, "MAPPING_J5_ZERO", "J5 zero_offset_rad 未标定")
    raw_limits = record.get("limits_rad")
    _require(isinstance(raw_limits, (list, tuple)) and len(raw_limits) == 2,
             "MAPPING_J5_LIMITS", "J5 limits_rad 必须是两个有限端点")
    lower, upper = _loose_number(raw_limits[0]), _loose_number(raw_limits[1])
    _require(lower is not None and upper is not None and lower < upper,
             "MAPPING_J5_LIMITS", "J5 limits_rad 端点无效或未排序")
    control_mode = _integer(record.get("commissioning_control_mode"))
    _require(control_mode == EXPECTED_CONTROL_MODE, "MAPPING_J5_MODE",
             "J5 commissioning_control_mode 必须是 MIT(1)")
    profile = {
        "pmax_rad": _number(record.get("commissioning_mit_pmax_rad")),
        "vmax_rad_s": _number(record.get("commissioning_mit_vmax_rad_s")),
        "tmax_nm": _number(record.get("commissioning_mit_tmax_nm")),
    }
    _require(all(value is not None and value > 0.0 for value in profile.values()),
             "MAPPING_J5_PROFILE", "J5 PMAX/VMAX/TMAX 必须有限且为正")
    timeout = _integer(record.get("commissioning_timeout_count"))
    _require(timeout is not None, "MAPPING_J5_TIMEOUT",
             "J5 commissioning_timeout_count 缺失")
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "can_id": can_id,
        "hardware_sign": sign,
        "zero_offset_rad": zero,
        "limits_rad": [lower, upper],
        "commissioning_timeout_count": timeout,
        "control_mode": control_mode,
        "profile": profile,
        "motor_model": _text(record.get("motor_model")),
    }


# ------------------------------------------------------------- evidence reader
def _evidence_axes(document):
    axes = document.get("axes")
    if axes is None:
        return None
    _require(isinstance(axes, list) and len(axes) == AXIS_COUNT,
             "TIMEOUT_EVIDENCE_AXES", "timeout 证据 axes 必须是五轴数组")
    seen = {}
    for row in axes:
        _require(isinstance(row, dict), "TIMEOUT_EVIDENCE_AXES",
                 "timeout 证据 axes 行必须是对象")
        can_id = _integer(row.get("can_id"))
        label = _text(row.get("axis"))
        if can_id is None and label and label.upper().startswith("J"):
            can_id = _integer(label[1:])
        _require(can_id in range(1, AXIS_COUNT + 1), "TIMEOUT_EVIDENCE_AXES",
                 "timeout 证据 axes 行缺少 1..5 can_id")
        _require(can_id not in seen, "TIMEOUT_EVIDENCE_AXES",
                 "timeout 证据 axes 出现重复 can_id %d" % can_id)
        state = _integer(row.get("motor_state", row.get("state")))
        _require(state == 0, "TIMEOUT_EVIDENCE_MOTOR_STATE",
                 "timeout 证据 J%d motor_state 不是 0（需五轴全部 DISABLED）" % can_id)
        valid = row.get("valid")
        _require(valid is True, "TIMEOUT_EVIDENCE_AXIS_INVALID",
                 "timeout 证据 J%d feedback valid 不是 true" % can_id)
        age = _number(row.get("feedback_age"))
        _require(age is not None and 0.0 <= age <= EVIDENCE_FEEDBACK_MAX_AGE_S,
                 "TIMEOUT_EVIDENCE_AXIS_AGE",
                 "timeout 证据 J%d feedback_age 不在 0..%.2fs 内" % (can_id, EVIDENCE_FEEDBACK_MAX_AGE_S))
        seen[can_id] = state
    _require(set(seen) == set(range(1, AXIS_COUNT + 1)), "TIMEOUT_EVIDENCE_AXES",
             "timeout 证据 axes 必须覆盖 J1..J5")
    return seen


def read_timeout_evidence(path, *, now=None, max_age_s=None):
    """Read one real ``j5_timeout_setup_evidence.json`` and enforce the contract.

    Missing/zero timeout, unmatched readback, non-zero motor state, a non-2000
    write outcome, or a device mismatch is rejected; there is no placeholder
    evidence path in the tree and no synthetic acceptance here.
    """
    path = Path(path)
    _require(path.is_file(), "TIMEOUT_EVIDENCE_MISSING",
             "timeout 证据文件不存在：%s（需要真实 TIMEOUT 读回证据；不生成/不接受伪造证据）" % path)
    raw = path.read_bytes()
    try:
        document = json.loads(raw)
    except (ValueError, UnicodeDecodeError) as error:
        raise AdmissionError("TIMEOUT_EVIDENCE_PARSE", "timeout 证据 JSON 解析失败：%s" % error)
    _require(isinstance(document, dict), "TIMEOUT_EVIDENCE_SHAPE",
             "timeout 证据必须是 JSON 对象")

    _require(document.get("schema") == EVIDENCE_SCHEMA, "TIMEOUT_EVIDENCE_SCHEMA",
             "timeout 证据 schema 必须是 %s" % EVIDENCE_SCHEMA)
    _require(document.get("ok") is True, "TIMEOUT_EVIDENCE_NOT_OK",
             "timeout 证据 ok 不是 true")
    _require(_integer(document.get("exit_code", 0)) == 0, "TIMEOUT_EVIDENCE_EXIT_CODE",
             "timeout 证据 exit_code 不是 0")
    _require(document.get("error") is None, "TIMEOUT_EVIDENCE_ERROR",
             "timeout 证据仍带 error，不能作为成功证据")
    write_outcome = document.get("write_outcome")
    _require(write_outcome in ("complete", "verified_existing"),
             "TIMEOUT_EVIDENCE_WRITE_OUTCOME",
             "timeout 证据 write_outcome 必须是 complete 或 verified_existing")
    if write_outcome == "complete":
        _require(_integer(document.get("write_attempts")) == 1, "TIMEOUT_EVIDENCE_WRITE_ATTEMPTS",
                 "timeout 证据 write_attempts 必须是 1（不得重试写入）")
        _require(_integer(document.get("parameter_writes")) == 1,
                 "TIMEOUT_EVIDENCE_PARAMETER_WRITES",
                 "timeout 证据 parameter_writes 必须是 1（恰好一次成功参数写）")
        write_data = _text(document.get("write_data_hex"))
        _require(write_data is not None
                 and " ".join(write_data.replace(",", " ").split()).lower() == EXPECTED_WRITE_DATA_HEX,
                 "TIMEOUT_EVIDENCE_WRITE_FRAME",
                 "timeout 证据 write_data_hex 与预期 RID9/2000 帧不符")
        _require(_integer(document.get("timeout_before")) == 0, "TIMEOUT_BEFORE_NOT_ZERO",
                 "timeout 证据 timeout_before 必须是 0（证明写入前确实 TIMEOUT0）")
    else:
        # Readback-only verification: no parameter write was made. This keeps the
        # watchdog value truthful and avoids requiring a J5 power cycle.
        _require(_integer(document.get("write_attempts")) == 0,
                 "TIMEOUT_EVIDENCE_WRITE_ATTEMPTS",
                 "readback-only 证据 write_attempts 必须是 0")
        _require(_integer(document.get("parameter_writes")) == 0,
                 "TIMEOUT_EVIDENCE_PARAMETER_WRITES",
                 "readback-only 证据 parameter_writes 必须是 0")
        _require(_integer(document.get("timeout_before")) == EXPECTED_TIMEOUT,
                 "TIMEOUT_BEFORE_NOT_EXPECTED",
                 "readback-only 证据 timeout_before 必须已经是 %d" % EXPECTED_TIMEOUT)
    _require("parameter_writes" in document, "TIMEOUT_EVIDENCE_PARAMETER_WRITES",
             "timeout 证据缺少 parameter_writes 记录")
    timeout_after = _integer(document.get("timeout_after"))
    _require(timeout_after == EXPECTED_TIMEOUT, "TIMEOUT_AFTER_INVALID",
             "timeout 证据 timeout_after 必须是 %d（实际 %r）；TIMEOUT0 不合格"
             % (EXPECTED_TIMEOUT, document.get("timeout_after")))
    readback = document.get("readback")
    _require(isinstance(readback, dict), "TIMEOUT_EVIDENCE_READBACK",
             "timeout 证据缺少 readback 记录")
    _require(_integer(readback.get("rid")) == 9, "TIMEOUT_EVIDENCE_READBACK",
             "timeout 证据 readback.rid 必须是 9")
    _require(_integer(readback.get("expected")) == EXPECTED_TIMEOUT,
             "TIMEOUT_EVIDENCE_READBACK", "timeout 证据 readback.expected 必须是 %d" % EXPECTED_TIMEOUT)
    _require(_integer(readback.get("observed")) == EXPECTED_TIMEOUT,
             "TIMEOUT_EVIDENCE_READBACK", "timeout 证据 readback.observed 必须是 %d" % EXPECTED_TIMEOUT)
    _require(readback.get("matched") is True, "TIMEOUT_READBACK_UNMATCHED",
             "timeout 证据 readback.matched 不是 true")

    _require(document.get("all_motor_state_zero") is True, "TIMEOUT_MOTOR_STATE",
             "timeout 证据 all_motor_state_zero 不是 true；五轴必须全部 motor_state=0")
    if "motor_state_all_zero" in document:
        _require(document.get("motor_state_all_zero") is True, "TIMEOUT_MOTOR_STATE",
                 "timeout 证据 motor_state_all_zero 不是 true")
    _evidence_axes(document)

    for key, code in (("flash_written", "TIMEOUT_FLASH_WRITTEN"),
                      ("enabled", "TIMEOUT_EVIDENCE_ENABLED"),
                      ("mit_sent", "TIMEOUT_EVIDENCE_MIT"),
                      ("motor_zero_written", "TIMEOUT_EVIDENCE_MOTOR_ZERO")):
        _require(_is_false(document.get(key)), code,
                 "timeout 证据 %s 必须为 false（只写 TIMEOUT，不使能、不发 MIT、不写零）" % key)

    _require(_integer(document.get("j5_id")) == J5_AXIS, "TIMEOUT_EVIDENCE_J5_ID",
             "timeout 证据 j5_id 必须是 5")
    _require(_integer(document.get("master_id")) == EXPECTED_MASTER_ID,
             "TIMEOUT_EVIDENCE_MASTER_ID", "timeout 证据 master_id 必须是 0")
    _require(_integer(document.get("control_mode")) == EXPECTED_CONTROL_MODE,
             "TIMEOUT_EVIDENCE_MODE", "timeout 证据 control_mode 必须是 MIT(1)")

    profile = document.get("profile")
    _require(isinstance(profile, dict), "TIMEOUT_EVIDENCE_PROFILE",
             "timeout 证据缺少 J5 profile")
    evidence_profile = {
        "pmax_rad": _number(profile.get("pmax_rad")),
        "vmax_rad_s": _number(profile.get("vmax_rad_s")),
        "tmax_nm": _number(profile.get("tmax_nm")),
    }
    _require(all(value is not None and value > 0.0 for value in evidence_profile.values()),
             "TIMEOUT_EVIDENCE_PROFILE", "timeout 证据 J5 profile 必须有限且为正")

    device = _text(document.get("device"))
    port_argument = _text(document.get("port_argument"))
    _require(device is not None and device.startswith("/dev/"),
             "TIMEOUT_EVIDENCE_DEVICE", "timeout 证据 device 必须是 /dev/... 实际端口")
    if port_argument is not None:
        _require(port_argument.startswith("/dev/"), "TIMEOUT_EVIDENCE_DEVICE",
                 "timeout 证据 port_argument 必须是 /dev/...")
    if _text(document.get("baud")) is not None:
        _require(_integer(document.get("baud")) == 921600, "TIMEOUT_EVIDENCE_BAUD",
                 "timeout 证据 baud 必须是 921600")

    session_id = _text(document.get("session_id"))
    host_boot_id = _text(document.get("host_boot_id"))
    _require(session_id is not None and host_boot_id is not None,
             "TIMEOUT_EVIDENCE_BOOT", "timeout 证据 session_id/host_boot_id 缺失")
    observed_at = _parse_wall_time(document.get("observed_at"))
    _require(observed_at is not None, "TIMEOUT_EVIDENCE_TIME",
             "timeout 证据 observed_at 必须是 UTC 时间戳")
    reference = float(now if now is not None else time.time())
    if max_age_s is not None:
        age = reference - observed_at
        _require(age <= float(max_age_s), "TIMEOUT_EVIDENCE_STALE",
                 "timeout 证据已过期（%.0f 秒 > %.0f 秒）；请在有效期内重新执行/构建"
                 % (age, float(max_age_s)))
        _require(age >= -MAX_CLOCK_SKEW_S, "TIMEOUT_EVIDENCE_FROM_FUTURE",
                 "timeout 证据 observed_at 位于未来，时钟不可信")

    tool = document.get("tool")
    _require(isinstance(tool, dict), "TIMEOUT_EVIDENCE_TOOL", "timeout 证据缺少 tool 记录")
    _require(_text(tool.get("name")) is not None, "TIMEOUT_EVIDENCE_TOOL",
             "timeout 证据 tool.name 缺失")
    _require(_sha256_text(tool.get("sha256")) is not None, "TIMEOUT_EVIDENCE_TOOL_SHA",
             "timeout 证据 tool.sha256 必须是 64 位十六进制")
    _require(_text(document.get("audit_path")) is not None, "TIMEOUT_EVIDENCE_AUDIT_PATH",
             "timeout 证据缺少 audit_path")

    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "document": document,
        "session_id": session_id,
        "host_boot_id": host_boot_id,
        "device": device,
        "port_argument": port_argument,
        "observed_at": observed_at,
        "observed_at_text": _text(document.get("observed_at")),
        "timeout_before": _integer(document.get("timeout_before")),
        "write_outcome": write_outcome,
        "write_attempts": _integer(document.get("write_attempts")),
        "parameter_writes": _integer(document.get("parameter_writes")),
        "timeout_after": timeout_after,
        "readback_observed": _integer(readback.get("observed")),
        "j5_id": J5_AXIS,
        "master_id": EXPECTED_MASTER_ID,
        "control_mode": EXPECTED_CONTROL_MODE,
        "profile": evidence_profile,
    }


# ------------------------------------------------------------- artifact build
def build_artifact(*, mapping, model, policy, load_profile, timeout_evidence,
                   device=None, output=None, now=None,
                   max_evidence_age_s=DEFAULT_MAX_EVIDENCE_AGE_S):
    """Build one canonical J5 motion artifact from the current real files.

    Refuses while the mapping J5 watchdog declaration is still 0: the source
    TIMEOUT declaration only becomes non-zero after Codex executes the timeout
    tool against real hardware.  No synthetic/placeholder evidence is accepted.
    """
    mapping_path, model_path, policy_path = Path(mapping), Path(model), Path(policy)
    load_path, evidence_path = Path(load_profile), Path(timeout_evidence)
    for label, path in (("mapping", mapping_path), ("model", model_path),
                        ("policy", policy_path), ("load", load_path)):
        _require(path.is_file(), "ARTIFACT_SOURCE_MISSING",
                 "生成 artifact 所需的 %s 文件不存在：%s" % (label, path))

    j5 = read_mapping_j5(mapping_path)
    policy_data = read_policy(policy_path)
    declaration = _load_declaration.load_declaration(load_path)
    evidence = read_timeout_evidence(evidence_path, now=now,
                                     max_age_s=max_evidence_age_s)

    # Source TIMEOUT declaration: 0 until Codex executes the real write.  This is
    # the guard that makes a pre-execution artifact impossible.
    _require(j5["commissioning_timeout_count"] > 0, "TIMEOUT_SOURCE_ZERO",
             "hardware_mapping.yaml 的 J5 commissioning_timeout_count 仍为 0；"
             "Codex 执行真实 TIMEOUT 写入并回写 source 声明前拒绝生成 artifact")
    _require(j5["commissioning_timeout_count"] == evidence["timeout_after"],
             "TIMEOUT_SOURCE_MISMATCH",
             "mapping J5 TIMEOUT=%d 与 timeout 证据 readback=%d 不一致"
             % (j5["commissioning_timeout_count"], evidence["timeout_after"]))

    _require(evidence["j5_id"] == j5["can_id"], "EVIDENCE_J5_ID_MISMATCH",
             "timeout 证据 j5_id 与 mapping J5 can_id 不一致")
    _require(evidence["control_mode"] == j5["control_mode"], "EVIDENCE_MODE_MISMATCH",
             "timeout 证据 control_mode 与 mapping J5 commissioning_control_mode 不一致")
    for key in ("pmax_rad", "vmax_rad_s", "tmax_nm"):
        _require(_same_float(evidence["profile"][key], j5["profile"][key], 1e-9),
                 "EVIDENCE_PROFILE_MISMATCH",
                 "timeout 证据 J5 profile.%s 与 mapping 不一致" % key)
    if device is not None:
        expected = _text(device)
        _require(expected is not None
                 and expected in (evidence["device"], evidence["port_argument"]),
                 "EVIDENCE_DEVICE_MISMATCH",
                 "timeout 证据 device=%s 与指定设备 %s 不一致"
                 % (evidence["device"], device))

    generated_at = float(now if now is not None else time.time())
    payload = {
        "qualification": QUALIFICATION,
        "joint": JOINT,
        "generated_at": generated_at,
        "boot_id": evidence["host_boot_id"],
        "session_id": evidence["session_id"],
        "device": evidence["device"],
        "port_argument": evidence["port_argument"],
        "load_id": declaration["load_id"],
        "hashes": {
            "mapping_sha256": j5["sha256"],
            "model_sha256": sha256_file(model_path),
            "policy_sha256": sha256_file(policy_path),
            "load_declaration_sha256": declaration["sha256"],
            "timeout_evidence_sha256": evidence["sha256"],
        },
        "j5": {
            "can_id": j5["can_id"],
            "master_id": evidence["master_id"],
            "hardware_sign": j5["hardware_sign"],
            "zero_offset_rad": j5["zero_offset_rad"],
            "limits_rad": list(j5["limits_rad"]),
            "control_mode": j5["control_mode"],
            "profile": dict(j5["profile"]),
        },
        "timeout": {
            "timeout_before": evidence["timeout_before"],
            "timeout_after": evidence["timeout_after"],
            "readback": evidence["readback_observed"],
            "readback_matched": True,
            "all_motor_state_zero": True,
            "write_outcome": evidence["write_outcome"],
            "write_attempts": evidence["write_attempts"],
            "parameter_writes": evidence["parameter_writes"],
            "observed_at": evidence["observed_at_text"],
        },
        "load": {
            "schema": declaration["schema"],
            "mass_kg": declaration["mass_kg"],
            "mass_source": declaration["mass_source"],
            "geometry_status": declaration["geometry_status"],
            "com_status": declaration["com_status"],
            "mount_status": declaration["mount_status"],
            "gravity_status": declaration["gravity_status"],
            "formal_gravity_allowed": False,
            "gravity_fit_allowed": False,
            "gravity_ff_alpha": 0.0,
            "float_enabled": False,
        },
        "policy": {
            "profile": policy_data["profile"],
            "gravity_ff_alpha": policy_data["gravity_ff_alpha"],
            "gravity_float_enabled": policy_data["gravity_float_enabled"],
        },
    }
    artifact = {"schema": SCHEMA, "payload": payload,
                "sha256": payload_sha256(payload)}
    if output is not None:
        write_artifact(output, artifact)
    return artifact


def write_artifact(path, artifact):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(artifact, indent=2, ensure_ascii=False) + "\n"
    path.write_text(text, encoding="utf-8")
    return path


def load_artifact(path):
    """Read an artifact file and validate only its envelope shape."""
    path = Path(path)
    _require(path.is_file(), "ARTIFACT_MISSING", "J5 motion artifact 不存在：%s" % path)
    try:
        artifact = json.loads(path.read_text(encoding="utf-8"))
    except (ValueError, UnicodeDecodeError) as error:
        raise AdmissionError("ARTIFACT_PARSE", "J5 motion artifact JSON 解析失败：%s" % error)
    _require(isinstance(artifact, dict), "ARTIFACT_SHAPE", "artifact 必须是 JSON 对象")
    _require(artifact.get("schema") == SCHEMA, "ARTIFACT_SCHEMA",
             "artifact schema 必须是 %s" % SCHEMA)
    _require(isinstance(artifact.get("payload"), dict), "ARTIFACT_SHAPE",
             "artifact 缺少 payload 对象")
    _require(_sha256_text(artifact.get("sha256")) is not None, "ARTIFACT_SHA_FORMAT",
             "artifact sha256 必须是 canonical payload 的 64 位十六进制")
    return artifact


# ---------------------------------------------------------- artifact validate
def _check_hashes(payload, mapping, model, policy, load_profile, timeout_evidence):
    hashes = payload.get("hashes")
    _require(isinstance(hashes, dict), "ARTIFACT_HASHES", "artifact payload 缺少 hashes")
    paths = {
        "mapping_sha256": mapping,
        "model_sha256": model,
        "policy_sha256": policy,
        "load_declaration_sha256": load_profile,
    }
    expected = {}
    for key, path in paths.items():
        actual = sha256_file(path)
        expected[key] = actual
        recorded = hashes.get(key)
        _require(_sha256_text(recorded) == actual, _HASH_CODE[key],
                 "artifact %s 与当前文件实际 SHA256 不一致（文件已变更/未重建）" % key)
    evidence_sha = _sha256_text(hashes.get("timeout_evidence_sha256"))
    _require(evidence_sha is not None, "TIMEOUT_EVIDENCE_SHA",
             "artifact 缺少 timeout evidence sha256")
    if timeout_evidence is not None:
        evidence = read_timeout_evidence(timeout_evidence)
        _require(evidence["sha256"] == evidence_sha, "TIMEOUT_EVIDENCE_HASH_MISMATCH",
                 "artifact 记录的 timeout 证据 SHA256 与当前证据文件不一致")
        timeout = payload.get("timeout") if isinstance(payload.get("timeout"), dict) else {}
        _require(_integer(timeout.get("timeout_after")) == evidence["timeout_after"],
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 timeout_after 不一致")
        for key in ("timeout_before", "write_attempts", "parameter_writes"):
            _require(_integer(timeout.get(key)) == evidence[key],
                     "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 %s 不一致" % key)
        _require(timeout.get("write_outcome") == evidence["write_outcome"],
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 write_outcome 不一致")
        _require(timeout.get("observed_at") == evidence["observed_at_text"],
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 observed_at 不一致")
        _require(_text(payload.get("session_id")) == evidence["session_id"],
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 session_id 不一致")
        _require(_text(payload.get("boot_id")) == evidence["host_boot_id"],
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 host_boot_id 不一致")
        _require(_text(payload.get("device")) in (evidence["device"], evidence["port_argument"]),
                 "TIMEOUT_EVIDENCE_MISMATCH", "artifact 与当前证据 device 不一致")
    return hashes


def _check_load(payload, declaration):
    _require(payload.get("load_id") == declaration["load_id"], "LOAD_ID_MISMATCH",
             "artifact load_id 与当前负载声明不一致")
    load = payload.get("load")
    _require(isinstance(load, dict), "LOAD_SECTION", "artifact 缺少 load 段")
    _require(load.get("gravity_status") == _load_declaration.PROVISIONAL,
             "LOAD_NOT_PROVISIONAL", "artifact 负载不是 PROVISIONAL")
    _require(load.get("formal_gravity_allowed") is False, "LOAD_GRAVITY_ENABLED",
             "provisional 负载不得声明 formal_gravity_allowed=true")
    _require(load.get("gravity_fit_allowed") is False, "LOAD_GRAVITY_FIT_ENABLED",
             "provisional 负载不得允许重力拟合")
    _require(load.get("float_enabled") is False, "LOAD_FLOAT_ENABLED",
             "provisional 负载不得启用 float")
    _require(_number(load.get("gravity_ff_alpha")) == 0.0, "LOAD_FF_ALPHA_NONZERO",
             "provisional 负载 gravity_ff_alpha 必须为 0")
    _require(_same_float(load.get("mass_kg"), declaration["mass_kg"]), "LOAD_MASS_MISMATCH",
             "artifact 负载质量与当前负载声明不一致")
    for key in ("geometry_status", "com_status", "mount_status"):
        _require(load.get(key) == _load_declaration.UNKNOWN, "LOAD_NOT_UNKNOWN",
                 "provisional 负载 %s 必须保持 unknown" % key)


def _check_j5(payload, j5, timeout):
    section = payload.get("j5")
    _require(isinstance(section, dict), "J5_SECTION", "artifact 缺少 j5 段")
    _require(_integer(section.get("can_id")) == j5["can_id"], "J5_CAN_ID_MISMATCH",
             "artifact J5 can_id 与 mapping 不一致")
    _require(_integer(section.get("hardware_sign")) == j5["hardware_sign"],
             "J5_SIGN_MISMATCH", "artifact J5 hardware_sign 与 mapping 不一致")
    _require(_same_float(section.get("zero_offset_rad"), j5["zero_offset_rad"]),
             "J5_ZERO_MISMATCH", "artifact J5 zero_offset_rad 与 mapping 不一致")
    limits = section.get("limits_rad")
    _require(isinstance(limits, (list, tuple)) and len(limits) == 2,
             "J5_LIMITS_MISMATCH", "artifact J5 limits_rad 形状无效")
    _require(_same_float(limits[0], j5["limits_rad"][0])
             and _same_float(limits[1], j5["limits_rad"][1]),
             "J5_LIMITS_MISMATCH", "artifact J5 limits_rad 与 mapping 不一致")
    _require(_integer(section.get("control_mode")) == j5["control_mode"],
             "J5_MODE_MISMATCH", "artifact J5 control_mode 与 mapping 不一致")
    profile = section.get("profile")
    _require(isinstance(profile, dict), "J5_PROFILE_MISMATCH", "artifact J5 profile 缺失")
    for key in ("pmax_rad", "vmax_rad_s", "tmax_nm"):
        _require(_same_float(profile.get(key), j5["profile"][key], 1e-9),
                 "J5_PROFILE_MISMATCH", "artifact J5 profile.%s 与 mapping 不一致" % key)

    outcome = timeout.get("write_outcome")
    _require(outcome in ("complete", "verified_existing"),
             "TIMEOUT_EVIDENCE_WRITE_OUTCOME", "artifact timeout write_outcome 无效")
    before = 0 if outcome == "complete" else EXPECTED_TIMEOUT
    writes = 1 if outcome == "complete" else 0
    _require(_integer(timeout.get("timeout_before")) == before,
             "TIMEOUT_BEFORE_NOT_ZERO" if before == 0 else "TIMEOUT_BEFORE_NOT_EXPECTED",
             "artifact timeout_before 与 write_outcome 不一致")
    for key in ("write_attempts", "parameter_writes"):
        _require(_integer(timeout.get(key)) == writes,
                 "TIMEOUT_EVIDENCE_WRITE_ATTEMPTS" if key == "write_attempts"
                 else "TIMEOUT_EVIDENCE_PARAMETER_WRITES",
                 "artifact %s 与 write_outcome 不一致" % key)
    timeout_after = _integer(timeout.get("timeout_after"))
    _require(timeout_after is not None and timeout_after >= EXPECTED_TIMEOUT,
             "TIMEOUT_EVIDENCE_ZERO",
             "artifact timeout_after 必须 >= %d（TIMEOUT0/缺失不合格）" % EXPECTED_TIMEOUT)
    _require(_integer(timeout.get("readback")) == EXPECTED_TIMEOUT,
             "TIMEOUT_READBACK_INVALID", "artifact timeout readback 必须是 %d" % EXPECTED_TIMEOUT)
    _require(timeout.get("all_motor_state_zero") is True, "TIMEOUT_MOTOR_STATE",
             "artifact 未记录五轴全部 motor_state=0")
    j5_timeout = j5["commissioning_timeout_count"]
    _require(j5_timeout != 0, "TIMEOUT_SOURCE_ZERO",
             "hardware_mapping.yaml J5 TIMEOUT 仍为 0；source 声明未更新前拒绝准入")
    _require(j5_timeout == timeout_after, "TIMEOUT_SOURCE_MISMATCH",
             "artifact timeout_after 与 mapping J5 commissioning_timeout_count 不一致")


def validate_artifact(artifact, mapping, model, policy, load_profile, *,
                      expected_device=None, max_age_s=DEFAULT_MAX_AGE_S,
                      timeout_evidence=None, now=None):
    """Validate one artifact against the current real files.

    Recomputes the canonical payload SHA256 (an arbitrary 64-hex string cannot
    pass), verifies the schema, binds mapping/model/policy/load-declaration and
    timeout-evidence hashes, re-checks J5 sign/zero/limits/profile against the
    mapping, rejects TIMEOUT0/stale/device-mismatched artifacts, and refuses any
    attempt to turn the provisional load into formal gravity/feedforward.
    """
    if isinstance(artifact, dict):
        raw_artifact = artifact
        _require(raw_artifact.get("schema") == SCHEMA, "ARTIFACT_SCHEMA",
                 "artifact schema 必须是 %s" % SCHEMA)
        _require(isinstance(raw_artifact.get("payload"), dict), "ARTIFACT_SHAPE",
                 "artifact 缺少 payload 对象")
        _require(_sha256_text(raw_artifact.get("sha256")) is not None,
                 "ARTIFACT_SHA_FORMAT", "artifact sha256 格式无效")
    else:
        raw_artifact = load_artifact(artifact)

    payload = raw_artifact["payload"]
    recorded = _sha256_text(raw_artifact.get("sha256"))
    _require(recorded == payload_sha256(payload), "ARTIFACT_HASH_MISMATCH",
             "artifact canonical payload SHA256 不匹配（被篡改或任意 64hex；拒绝准入）")
    _require(payload.get("qualification") == QUALIFICATION, "ARTIFACT_QUALIFICATION",
             "artifact qualification 必须是 %s" % QUALIFICATION)
    _require(payload.get("joint") == JOINT, "ARTIFACT_JOINT",
             "artifact joint 必须是 %s" % JOINT)
    _require(_text(payload.get("boot_id")) is not None, "ARTIFACT_BOOT",
             "artifact boot_id 缺失")

    mapping_path, model_path, policy_path = Path(mapping), Path(model), Path(policy)
    load_path = Path(load_profile)
    for label, path in (("mapping", mapping_path), ("model", model_path),
                        ("policy", policy_path), ("load", load_path)):
        _require(path.is_file(), "ARTIFACT_SOURCE_MISSING",
                 "校验 artifact 所需的 %s 文件不存在：%s" % (label, path))

    _check_hashes(payload, mapping_path, model_path, policy_path, load_path, timeout_evidence)
    declaration = _load_declaration.load_declaration(load_path)
    _check_load(payload, declaration)

    policy_data = read_policy(policy_path)
    policy_section = payload.get("policy")
    _require(isinstance(policy_section, dict), "POLICY_SECTION", "artifact 缺少 policy 段")
    _require(_number(policy_section.get("gravity_ff_alpha")) == 0.0,
             "POLICY_GRAVITY_ENABLED", "artifact policy gravity_ff_alpha 必须为 0")
    _require(policy_section.get("gravity_float_enabled") is False,
             "POLICY_FLOAT_ENABLED", "artifact policy gravity_float_enabled 必须为 false")
    _require(policy_section.get("profile") == policy_data["profile"],
             "POLICY_PROFILE_MISMATCH", "artifact policy profile 与当前策略不一致")

    j5 = read_mapping_j5(mapping_path)
    timeout = payload.get("timeout")
    _require(isinstance(timeout, dict), "TIMEOUT_SECTION", "artifact 缺少 timeout 段")
    _check_j5(payload, j5, timeout)

    device = _text(payload.get("device"))
    _require(device is not None and device.startswith("/dev/"),
             "ARTIFACT_DEVICE", "artifact device 必须是 /dev/... 实际端口")
    port_argument = _text(payload.get("port_argument"))
    if expected_device is not None:
        expected = _text(expected_device)
        _require(expected is not None
                 and expected in (device, port_argument), "DEVICE_MISMATCH",
                 "artifact device=%s 与期望设备 %s 不一致" % (device, expected_device))

    generated_at = _number(payload.get("generated_at"))
    _require(generated_at is not None, "ARTIFACT_TIME", "artifact generated_at 无效")
    reference = float(now if now is not None else time.time())
    if max_age_s is not None:
        age = reference - generated_at
        _require(age <= float(max_age_s), "ARTIFACT_STALE",
                 "J5 motion artifact 已过期（%.0f 秒 > %.0f 秒）；需依据新鲜 timeout 证据重建"
                 % (age, float(max_age_s)))
        _require(age >= -MAX_CLOCK_SKEW_S, "ARTIFACT_FROM_FUTURE",
                 "artifact generated_at 位于未来，时钟不可信")
    return payload


# ------------------------------------------------------- candidate launch glue
def resolve_candidate_identity(*, mapping, model, policy, load_profile, port,
                               mass_profile_sha256="", test_only_mass_profile=False,
                               j5_artifact="", timeout_evidence="",
                               max_age_s=DEFAULT_MAX_AGE_S, now=None):
    """Pure admission decision used by the five-axis candidate launch.

    The load declaration is the authoritative position-only identity source:
    the launch passes its actual file SHA256 as ``mass_profile_sha256`` and its
    ``load_id``; the policy's ``load_id`` remains a fallback for the legacy
    launch only.  J5 sign/zero/limits always come from ``hardware_mapping.yaml``.
    A raw/arbitrary J5 hash is not accepted: only a validated artifact can turn
    ``j5_qualified`` on, and the hash passed is the artifact payload SHA256.
    """
    port = _text(port)
    _require(port is not None and port.startswith("/dev/"),
             "PORT_INVALID", "candidate launch 需要明确的 Linux/WSL CANUSB 端口（/dev/...）")

    declaration = _load_declaration.load_declaration(load_profile)
    policy_data = read_policy(policy)
    j5 = read_mapping_j5(mapping)

    actual_hash = declaration["sha256"]
    explicit = _text(mass_profile_sha256)
    test_only = False
    if explicit is not None and explicit != actual_hash:
        _require(test_only_mass_profile is True, "MASS_PROFILE_HASH_MISMATCH",
                 "显式 mass_profile_sha256 与负载声明文件实际 SHA256 不一致；"
                 "仅 test_only_mass_profile=true 的测试路径可绕过")
        _require(_sha256_text(explicit) is not None, "MASS_PROFILE_HASH_FORMAT",
                 "显式 mass_profile_sha256 必须是 64 位十六进制")
        effective_hash = explicit
        test_only = True
    else:
        effective_hash = actual_hash

    launch_arguments = {
        "load_id": declaration["load_id"],
        "mass_profile_sha256": effective_hash,
        # J5 finite identity is copied from the mapping file only.  The
        # candidate launch exposes these names solely to reject manual values.
        "j5_qualified": "false",
        "j5_calibration_sha256": "",
        "j5_hardware_sign": str(j5["hardware_sign"]),
        "j5_zero_offset_rad": repr(j5["zero_offset_rad"]),
        "j5_limit_lower": repr(j5["limits_rad"][0]),
        "j5_limit_upper": repr(j5["limits_rad"][1]),
    }
    admission = {
        "artifact": None,
        "j5_qualified": False,
        "artifact_sha256": None,
        "device": None,
        "boot_id": None,
        "reason": "J5_MOTION_UNQUALIFIED: no validated artifact supplied; feedback-only",
    }
    artifact = _text(j5_artifact)
    if artifact is not None:
        payload = validate_artifact(
            artifact, mapping, model, policy, load_profile,
            expected_device=port, max_age_s=max_age_s,
            timeout_evidence=(_text(timeout_evidence) or None), now=now)
        calibration_sha256 = payload_sha256(payload)
        launch_arguments["j5_qualified"] = "true"
        launch_arguments["j5_calibration_sha256"] = calibration_sha256
        admission = {
            "artifact": artifact,
            "j5_qualified": True,
            "artifact_sha256": calibration_sha256,
            "device": payload.get("device"),
            "boot_id": payload.get("boot_id"),
            "reason": "J5_QUALIFIED: validated %s artifact" % SCHEMA,
        }
    return {
        "kind": "slaihand_five_axis_candidate_admission/1",
        "load_id": declaration["load_id"],
        "mass_profile_sha256": effective_hash,
        "test_only_mass_profile": test_only,
        "load": {"mass_kg": declaration["mass_kg"],
                 "gravity_status": declaration["gravity_status"],
                 "formal_gravity_allowed": False,
                 "float_enabled": False},
        "policy": {"profile": policy_data["profile"],
                   "gravity_ff_alpha": 0.0,
                   "gravity_float_enabled": False},
        "j5": {"hardware_sign": j5["hardware_sign"],
               "zero_offset_rad": j5["zero_offset_rad"],
               "limits_rad": list(j5["limits_rad"])},
        "admission": admission,
        "launch_arguments": launch_arguments,
    }


# ------------------------------------------------------------------------ CLI
def _source_paths():
    here = Path(__file__).resolve()
    bringup = here.parent.parent
    if (bringup / "config" / "hardware_mapping.yaml").is_file():
        workspace = bringup.parent.parent
        return {
            "mapping": str(bringup / "config" / "hardware_mapping.yaml"),
            "policy": str(bringup / "config" / "canusb_five_axis_motion_policy.yaml"),
            "load": str(bringup / "config" / load_filename()),
            "model": str(workspace / "src" / "slaihand_description" / "urdf"
                         / "slaihand_five_axis.urdf"),
        }
    return {}


def load_filename():
    return "load_3p5kg_provisional.yaml"


def _add_source_arguments(parser, defaults):
    parser.add_argument("--mapping", default=defaults.get("mapping"))
    parser.add_argument("--model", default=defaults.get("model"))
    parser.add_argument("--policy", default=defaults.get("policy"))
    parser.add_argument("--load", default=defaults.get("load"))


def _require_cli_paths(args):
    for name, label in (("mapping", "mapping"), ("model", "model"),
                        ("policy", "policy"), ("load", "load")):
        if not getattr(args, name):
            raise AdmissionError("CLI_PATH_MISSING",
                                 "缺少 --%s（安装态运行需显式提供）" % label)


def main(argv=None):
    defaults = _source_paths()
    parser = argparse.ArgumentParser(
        description="Build/validate the J5 motion calibration admission artifact "
                    "(no ROS, no hardware, no motion).")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="build artifact from real files + timeout evidence")
    _add_source_arguments(build, defaults)
    build.add_argument("--evidence", required=True,
                       help="j5_timeout_setup_evidence.json produced by the real timeout tool")
    build.add_argument("--output", default=None, help="artifact output path")
    build.add_argument("--device", default=None, help="optional expected device path")
    build.add_argument("--max-evidence-age-s", type=float,
                       default=DEFAULT_MAX_EVIDENCE_AGE_S)

    check = sub.add_parser("validate", help="validate an artifact against current files")
    check.add_argument("artifact")
    _add_source_arguments(check, defaults)
    check.add_argument("--device", default=None, help="optional expected device path")
    check.add_argument("--evidence", default=None, help="optional timeout evidence file to re-bind")
    check.add_argument("--max-age-s", type=float, default=DEFAULT_MAX_AGE_S)

    args = parser.parse_args(argv)
    try:
        _require_cli_paths(args)
        if args.command == "build":
            artifact = build_artifact(
                mapping=args.mapping, model=args.model, policy=args.policy,
                load_profile=args.load, timeout_evidence=args.evidence,
                device=args.device, output=args.output,
                max_evidence_age_s=args.max_evidence_age_s)
            payload = artifact["payload"]
            print(json.dumps({"ok": True, "schema": SCHEMA,
                              "artifact": args.output,
                              "sha256": artifact["sha256"],
                              "qualification": payload["qualification"],
                              "load_id": payload["load_id"],
                              "generated_at": payload["generated_at"],
                              "device": payload["device"],
                              "boot_id": payload["boot_id"],
                              "j5": payload["j5"],
                              "timeout": payload["timeout"]},
                             ensure_ascii=False))
            return 0
        payload = validate_artifact(
            args.artifact, args.mapping, args.model, args.policy, args.load,
            expected_device=args.device, max_age_s=args.max_age_s,
            timeout_evidence=args.evidence)
        print(json.dumps({"ok": True, "schema": SCHEMA,
                          "artifact": args.artifact,
                          "sha256": payload_sha256(payload),
                          "qualification": payload["qualification"],
                          "load_id": payload["load_id"],
                          "device": payload["device"],
                          "boot_id": payload["boot_id"]},
                         ensure_ascii=False))
        return 0
    except (_load_declaration.LoadDeclarationError, AdmissionError) as error:
        print(json.dumps({"ok": False, "code": error.code,
                          "message": error.message}, ensure_ascii=False),
              file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
