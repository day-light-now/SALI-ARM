"""Offline calibration data model: schema, qualification gates and analysis.

C-phase rework rules (Codex 返工 2026-11-05):
- capture keeps raw records, but the pure-gravity fit only accepts statically
  qualified, enabled-hold samples taken in one sustained fresh window with no
  support/contact and a verified runtime configuration identity;
- feedback age is the total of driver age plus local receipt age, never just the
  cached message field;
- all samples must share one model/mapping/gravity/J3/load identity and match the
  analysis model; mixed versions are refused, negative/zero scales are refused;
- disabled feedback is geometry evidence only, never a static holding torque;
- the tool deliberately fits one global gravity scale; it does not claim that
  richer gravity parameters are generally unidentifiable.
"""
import json
import math
from pathlib import Path

SCHEMA_CAPTURE = "slaihand_calibration_capture/2"
SCHEMA_SAMPLE = "slaihand_calibration_sample/2"
SCHEMA_ANALYSIS = "slaihand_calibration_analysis/2"
SUPPORTS = ("free", "supported", "unknown")
CONTACTS = ("none", "table", "hand", "fixture", "cable")
MOTIONS = ("static", "up", "down")
MIN_SAMPLES = 8
MIN_POSES = 3
POSE_SPREAD_DEG = 5.0
GRAVITY_SIGNIFICANT_NM = 0.05
MAX_FEEDBACK_AGE_S = 0.06
FF_MAX_AGE_S = 1.5
WINDOW_MAX_GAP_S = 0.3
# Drive velocity feedback is a 12-bit field with no exact zero and real noise of
# +/-2-3 codes: measured 0.0305 rad/s (J1/J2) and 0.0330 rad/s (J3/J4) on a
# stationary enabled hold on 2026-11-05, while position drift stayed below
# 0.0004 rad over 4.8 s. The position-delta gate is the real stability check;
# this speed bound only has to sit above the protocol noise (0.03 was too tight
# and made every real static window fail).
STATIC_SPEED_RAD_S = 0.06
STATIC_POSITION_DELTA_RAD = 0.005
WINDOW_MIN_SAMPLES = 5
WINDOW_MIN_SPAN_S = 0.8
SIGN_AGREEMENT_MIN = 0.8
HOLDOUT_RMSE_MAX_NM = 0.2
HOLDOUT_RATIO_MAX = 3.0
STAGE_HOLD = 5
STAGE_TRACK = 6
HASH_KEYS = ("urdf_sha256", "mass_profile_sha256", "mapping_sha256", "policy_sha256")
IDENTITY_KEYS = HASH_KEYS + ("mass_profile_status",)


class CalibrationError(ValueError):
    pass


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _finite_list(values, length):
    return isinstance(values, list) and len(values) == length and all(_finite(v) for v in values)


def _int_list(values, length):
    return isinstance(values, list) and len(values) == length and \
        all(isinstance(v, int) and not isinstance(v, bool) for v in values)


def _identity(sample):
    model = sample.get("model") or {}
    identity = {key: model.get(key) for key in IDENTITY_KEYS}
    identity["gravity_base_m_s2"] = list(model.get("gravity_base_m_s2") or [])
    identity["j3_offset_rad"] = model.get("j3_offset_rad")
    identity["load_id"] = sample.get("load_id")
    return identity


def identity_matches(candidate, expected):
    if expected is None:
        return True
    for key, value in expected.items():
        if key in ("gravity_base_m_s2",):
            if list(candidate.get(key) or []) != list(value or []):
                return False
        elif key == "j3_offset_rad":
            if not (_finite(candidate.get(key)) and _finite(value)
                    and abs(candidate[key] - value) < 1e-9):
                return False
        elif candidate.get(key) != value:
            return False
    return True


def validate_sample(sample, require_zero_ff=True):
    """Schema validation. Returns (ok, reasons); keeps raw records write-once."""
    reasons = []
    if not isinstance(sample, dict):
        return False, ["记录不是对象"]
    if sample.get("schema") != SCHEMA_SAMPLE:
        reasons.append("schema 不匹配")
    if sample.get('role', 'training') not in ('training', 'holdout'):
        reasons.append('role 必须为 training 或 holdout')
    if not _finite_list(sample.get("positions_rad"), 4):
        reasons.append("缺少四轴位置")
    if not _finite_list(sample.get("velocities_rad_s"), 4):
        reasons.append("缺少四轴速度")
    if not _finite_list(sample.get("effort_estimate_nm"), 4):
        reasons.append("缺少四轴反馈力矩估计")
    if not _finite_list(sample.get("driver_temperature_c"), 4):
        reasons.append("缺少驱动温度")
    if not _finite_list(sample.get("motor_temperature_c"), 4):
        reasons.append("缺少电机温度")
    if not _finite_list(sample.get("feedback_age_s"), 4):
        reasons.append("缺少反馈时效")
    if not isinstance(sample.get("feedback_valid"), list) or \
            len(sample["feedback_valid"]) != 4 or not all(v is True for v in sample["feedback_valid"]):
        reasons.append("四轴反馈未全部有效")
    if not _finite(sample.get("capture_age_s")):
        reasons.append("缺少本地接收年龄 capture_age_s")
    elif not 0.0 <= sample["capture_age_s"] <= MAX_FEEDBACK_AGE_S:
        reasons.append("采样总年龄超出 %.2f s" % MAX_FEEDBACK_AGE_S)
    if not _finite(sample.get("local_receipt_age_s")):
        reasons.append("缺少本地经过时间 local_receipt_age_s")
    totals = sample.get("total_age_s")
    if not _finite_list(totals, 4):
        reasons.append("缺少每轴总年龄 total_age_s")
    elif any(age < 0 or age > MAX_FEEDBACK_AGE_S for age in totals):
        reasons.append("每轴总年龄（驱动年龄+本地经过时间）超出 %.2f s" % MAX_FEEDBACK_AGE_S)
    elif _finite(sample.get("capture_age_s")) and abs(sample["capture_age_s"] - max(totals)) > 1e-6:
        reasons.append("capture_age_s 与每轴总年龄最大值不一致")
    ages = sample.get("feedback_age_s")
    if _finite_list(ages, 4) and any(age < 0 or age > MAX_FEEDBACK_AGE_S for age in ages):
        reasons.append("驱动反馈时效超出 %.2f s" % MAX_FEEDBACK_AGE_S)
    if not isinstance(sample.get("stage"), int) or isinstance(sample.get("stage"), bool):
        reasons.append("缺少控制阶段 stage")
    if not _int_list(sample.get("motor_state"), 4):
        reasons.append("缺少四轴 motor_state")
    if sample.get("support") not in SUPPORTS:
        reasons.append("support 必须是 %s" % (SUPPORTS,))
    if sample.get("contact") not in CONTACTS:
        reasons.append("contact 必须是 %s" % (CONTACTS,))
    if sample.get("motion") not in MOTIONS:
        reasons.append("motion 必须是 %s" % (MOTIONS,))
    if not isinstance(sample.get("load_id"), str) or not sample["load_id"].strip():
        reasons.append("缺少负载标识 load_id")
    window = sample.get("static_window")
    if not isinstance(window, dict):
        reasons.append("缺少持续静止窗口 static_window")
    else:
        if not isinstance(window.get("samples"), int) or window.get("samples", 0) < WINDOW_MIN_SAMPLES:
            reasons.append("静止窗口样本数不足 %d" % WINDOW_MIN_SAMPLES)
        if not _finite(window.get("span_s")) or window.get("span_s", 0.0) < WINDOW_MIN_SPAN_S:
            reasons.append("静止窗口跨度不足 %.1f s" % WINDOW_MIN_SPAN_S)
        if not _finite(window.get("max_speed_rad_s")) or window["max_speed_rad_s"] > STATIC_SPEED_RAD_S:
            reasons.append("窗口内速度超过静态阈值")
        if not _finite(window.get("max_position_delta_rad")) or \
                window["max_position_delta_rad"] > STATIC_POSITION_DELTA_RAD:
            reasons.append("窗口内位置变化超过静态阈值")
        if not _finite(window.get("max_gap_s")) or window["max_gap_s"] > WINDOW_MAX_GAP_S:
            reasons.append("窗口内采样间隙超过 %.2f s" % WINDOW_MAX_GAP_S)
        if window.get("all_enabled") is not True:
            reasons.append("窗口并非逐帧全部使能")
        if window.get("fresh_all") is not True:
            reasons.append("窗口并非逐帧新鲜")
        if require_zero_ff and window.get("ff_all_zero") is not True:
            reasons.append("窗口内前馈并非逐帧为零")
        if window.get("ff_all_fresh") is not True:
            reasons.append("窗口内前馈状态并非逐帧新鲜")
        if not isinstance(window.get("command_ids"), list) or len(window["command_ids"]) != 1:
            reasons.append("窗口内命令 ID 发生变化")
        if window.get("valid") is not True:
            reasons.append("持续静止窗口未通过" )
        if window.get("stage") not in (STAGE_HOLD, STAGE_TRACK):
            reasons.append("窗口阶段不是使能保持")
        if not _int_list(window.get("motor_state"), 4) or any(v != 1 for v in window["motor_state"]):
            reasons.append("窗口内电机未全部处于使能状态")
    ff = sample.get("driver_ff")
    if not isinstance(ff, dict) or not _finite(ff.get("alpha")):
        reasons.append("缺少驱动前馈状态")
    model = sample.get("model") or {}
    for key in ("urdf_sha256", "mass_profile_sha256", "mapping_sha256", "policy_sha256"):
        if not isinstance(model.get(key), str) or len(model[key]) != 64:
            reasons.append("模型信息缺少 %s" % key)
    if model.get("mass_profile_status") not in ("ESTIMATE_ONLY_NOT_FOR_HARDWARE", "CALIBRATED"):
        reasons.append("质量档案状态缺失或未知")
    if not _finite(model.get("j3_offset_rad")):
        reasons.append("缺少 J3 偏置版本")
    if not _finite_list(model.get("gravity_base_m_s2"), 3):
        reasons.append("缺少重力向量版本")
    runtime = sample.get("runtime")
    if not isinstance(runtime, dict) or not isinstance(runtime.get("verified"), bool) \
            or not isinstance(runtime.get("source"), str):
        reasons.append("缺少运行中配置身份记录")
    elif runtime["verified"] is True:
        identity = runtime.get("identity")
        if not isinstance(identity, dict):
            reasons.append("运行时身份已验证但缺少身份数据")
        else:
            for model_key, identity_key in (("urdf_sha256", "model_sha256"),
                                            ("mass_profile_sha256", "mass_profile_sha256"),
                                            ("mapping_sha256", "mapping_sha256"),
                                            ("policy_sha256", "policy_sha256")):
                if identity.get(identity_key) != model.get(model_key):
                    reasons.append("运行时身份 %s 与样本模型不一致" % identity_key)
            if list(identity.get("gravity_base_m_s2") or []) != list(model.get("gravity_base_m_s2") or []):
                reasons.append("运行时重力向量与样本模型不一致")
            if not (_finite(identity.get("j3_offset_rad")) and _finite(model.get("j3_offset_rad"))
                    and abs(identity["j3_offset_rad"] - model["j3_offset_rad"]) < 1e-9):
                reasons.append("运行时 J3 偏置与样本模型不一致")
            if identity.get("load_id") != sample.get("load_id"):
                reasons.append("运行时负载标识与样本不一致")
    return not reasons, reasons


def exclusion_reason(sample):
    """Why a schema-valid sample is not eligible for the pure-gravity fit."""
    if sample.get("support") != "free":
        return "有支撑（%s），不能当作纯重力" % support_or_unknown(sample)
    if sample.get("contact") != "none":
        return "有外部接触（%s），不能当作纯重力" % sample.get("contact")
    if sample.get("motion") != "static":
        return "运动来向为 %s，不是静态" % sample.get("motion")
    stage = sample.get("stage")
    if stage not in (STAGE_HOLD, STAGE_TRACK):
        return "控制阶段 %s 不是使能保持（失能样本只能做几何检查）" % stage
    motor = sample.get("motor_state") or []
    if any(state != 1 for state in motor):
        return "四轴未全部使能（%s），不能取得静态保持力矩" % motor
    window = sample.get("static_window") or {}
    if window.get("stage") not in (STAGE_HOLD, STAGE_TRACK):
        return "采集窗口阶段不是使能保持"
    if window.get("all_enabled") is not True or \
            any(state != 1 for state in (window.get("motor_state") or [])):
        return "采集窗口内并非逐帧全部使能"
    if window.get("fresh_all") is not True:
        return "采集窗口内并非逐帧新鲜"
    if _finite(window.get("max_gap_s")) and window["max_gap_s"] > WINDOW_MAX_GAP_S:
        return "采集窗口内采样间隙 %.2f s 过大" % window["max_gap_s"]
    if window.get("max_speed_rad_s", 1.0) > STATIC_SPEED_RAD_S:
        return "窗口内实际速度 %.3f rad/s 超过静态阈值" % window["max_speed_rad_s"]
    if window.get("max_position_delta_rad", 1.0) > STATIC_POSITION_DELTA_RAD:
        return "窗口内位置变化 %.4f rad 超过静态阈值" % window["max_position_delta_rad"]
    ff = sample.get("driver_ff")
    if not isinstance(ff, dict):
        return "缺少驱动前馈状态，无法证明 alpha=0"
    if not _finite(ff.get("age_s")):
        return "驱动前馈状态缺少时效"
    if ff["age_s"] > FF_MAX_AGE_S:
        return "驱动前馈状态过期 %.2f s，无法证明当前 alpha=0" % ff["age_s"]
    if ff.get("alpha") not in (0, 0.0):
        return "驱动前馈 alpha=%s 非零，力矩含补偿量" % ff.get("alpha")
    if ff.get("stage") not in (STAGE_HOLD, STAGE_TRACK) or ff.get("stage") != stage:
        return "驱动前馈阶段 %s 与采样阶段 %s 不一致" % (ff.get("stage"), stage)
    if window.get("ff_all_zero") is not True:
        return "采集窗口内前馈并非逐帧为零"
    if window.get("ff_all_fresh") is not True:
        return "采集窗口内前馈状态并非逐帧新鲜"
    command_ids = window.get("command_ids") or []
    if len(command_ids) != 1:
        return "采集窗口内命令 ID 发生变化"
    if ff.get("command_id") not in command_ids:
        return "驱动前馈命令 ID 与采集窗口不一致"
    runtime = sample.get("runtime") or {}
    if runtime.get("verified") is not True:
        return "运行中配置身份未验证（%s），不能作为可信驱动标定" % runtime.get("source", "unknown")
    return None


def support_or_unknown(sample):
    return sample.get("support", "unknown")


def load_raw(path):
    path = Path(path)
    if not path.is_file():
        raise CalibrationError("原始数据不存在：%s" % path)
    header = None
    samples = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError as error:
            raise CalibrationError("第 %d 行不是 JSON：%s" % (number, error))
        if header is None:
            if payload.get("schema") != SCHEMA_CAPTURE:
                raise CalibrationError("采集文件头 schema 不匹配")
            header = payload
            continue
        if payload.get("schema") != SCHEMA_SAMPLE:
            raise CalibrationError("第 %d 行不是样本记录" % number)
        samples.append(payload)
    if header is None:
        raise CalibrationError("采集文件为空")
    return {"header": header, "samples": samples}


def _poses_are_diverse(samples):
    poses = [sample["positions_rad"] for sample in samples]
    distinct = []
    for pose in poses:
        if not any(max(abs(pose[i] - other[i]) for i in range(4)) < math.radians(1.0)
                   for other in distinct):
            distinct.append(pose)
    spread = 0.0
    for axis in range(4):
        values = [pose[axis] for pose in poses]
        spread = max(spread, math.degrees(max(values) - min(values)))
    return len(distinct), spread


def _residual_stats(rows, gravity_fn, guards):
    residuals = [[] for _ in range(4)]
    signs = [[] for _ in range(4)]
    significant = [0 for _ in range(4)]
    for sample in rows:
        gravity = [float(v) for v in gravity_fn(sample["positions_rad"])]
        for axis in range(4):
            residuals[axis].append(sample["effort_estimate_nm"][axis] - gravity[axis])
            if abs(gravity[axis]) >= GRAVITY_SIGNIFICANT_NM:
                significant[axis] += 1
                signs[axis].append(1 if sample["effort_estimate_nm"][axis] * gravity[axis] >= 0 else -1)
    per_axis = []
    for axis in range(4):
        values = residuals[axis]
        mean = sum(values) / len(values) if values else None
        variance = sum((v - mean) ** 2 for v in values) / len(values) if values else None
        same = sum(1 for s in signs[axis] if s > 0) / len(signs[axis]) if signs[axis] else None
        per_axis.append({"joint": axis + 1,
                         "residual_mean_nm": None if mean is None else round(mean, 4),
                         "residual_std_nm": None if variance is None else round(math.sqrt(variance), 4),
                         "model_significant_samples": significant[axis],
                         "measured_sign_agreement": None if same is None else round(same, 3),
                         "guard_nm": guards[axis]})
    return per_axis


def _fit_scale(rows, gravity_fn):
    numerator = 0.0
    denominator = 0.0
    for row in rows:
        gravity = [float(v) for v in gravity_fn(row["positions_rad"])]
        for axis in range(4):
            numerator += row["effort_estimate_nm"][axis] * gravity[axis]
            denominator += gravity[axis] ** 2
    return (numerator / denominator) if denominator > 1e-12 else None


def _rmse(rows, gravity_fn, scale):
    errors = []
    for row in rows:
        gravity = [float(v) for v in gravity_fn(row["positions_rad"])]
        for axis in range(4):
            errors.append(row["effort_estimate_nm"][axis] - scale * gravity[axis])
    return math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else None


def analyze(samples, gravity_fn, guards=None, expected_identity=None):
    """Qualification gates + diagnostics + conservative global-scale fit.

    `status` is PROVISIONAL_SCALE only when every gate passes; otherwise
    INSUFFICIENT with explicit reasons. Diagnostics are kept even when the data
    is refused, labelled diagnostic_only.
    """
    guards = list(guards or [8.0, 10.0, 2.0, 2.0])
    counts = {"total": len(samples), "invalid": 0, "excluded": 0, "usable": 0, "qualified": 0}
    invalid, excluded, usable, explicit_holdout = [], [], [], []
    for sample in samples:
        ok, reasons = validate_sample(sample)
        if not ok:
            counts["invalid"] += 1
            invalid.append({"reasons": reasons})
            continue
        reason = exclusion_reason(sample)
        if reason is not None:
            counts["excluded"] += 1
            excluded.append({"reason": reason, "stage": sample.get("stage"),
                             "support": sample.get("support"), "contact": sample.get("contact"),
                             "motion": sample.get("motion")})
            # Disabled samples may still be useful for geometry, never for torque.
            if sample.get("stage") not in (STAGE_HOLD, STAGE_TRACK):
                excluded[-1]["geometry_only"] = True
            continue
        if sample.get('role') == 'holdout':
            explicit_holdout.append(sample)
        else:
            usable.append(sample)
    counts["usable"] = len(usable)
    counts["qualified"] = len(usable)+len(explicit_holdout)
    counts['independent_holdout'] = len(explicit_holdout)
    report = {"schema": SCHEMA_ANALYSIS, "counts": counts, "status": "INSUFFICIENT",
              "reasons": [], "exclusions": excluded, "invalid": invalid,
              "identity": {}, "diagnostics": None, "fit": None, "holdout": None,
              "identifiability": None, "warnings": []}
    if counts["invalid"] or counts["excluded"]:
        report["warnings"].append("存在无效/被排除记录：invalid=%d excluded=%d"
                                  % (counts["invalid"], counts["excluded"]))
    # Version consistency: one identity for all valid samples, matching the model.
    identities = [_identity(sample) for sample in samples
                  if core_identity_complete(sample)]
    if identities:
        first = identities[0]
        if any(identity != first for identity in identities[1:]):
            report["reasons"].append("样本间模型/映射/重力/J3/负载版本不一致，拒绝混合拟合")
        elif expected_identity is not None and not identity_matches(first, expected_identity):
            report["reasons"].append("样本版本与当前分析模型不一致，拒绝拟合")
        report["identity"] = first
    else:
        report["reasons"].append("没有可用的模型/映射/重力/J3/负载版本信息")
    # Diagnostics over every schema-valid sample (clearly labelled).
    if samples:
        valid_rows = [sample for sample in samples if _identity(sample)]
        if valid_rows:
            try:
                diagnostics_scale = _fit_scale(valid_rows, gravity_fn)
                report["diagnostics"] = {
                    "diagnostic_only": True,
                    "scale": None if diagnostics_scale is None else round(diagnostics_scale, 5),
                    "residual_rmse_nm": None if diagnostics_scale is None else
                    round(_rmse(valid_rows, gravity_fn, diagnostics_scale), 4),
                    "per_axis": _residual_stats(valid_rows, gravity_fn, guards),
                    "note": "诊断值包含未通过资格门禁的样本，不作为标定结论",
                }
            except Exception as error:  # noqa: BLE001 - keep a report instead of crashing
                report["reasons"].append("模型计算失败：%s" % error)
    if not usable:
        report["reasons"].append("没有通过纯重力资格门禁的样本")
        return report
    poses, spread = _poses_are_diverse(usable)
    if counts["usable"] < MIN_SAMPLES:
        report["reasons"].append("合格样本不足：%d < %d" % (counts["usable"], MIN_SAMPLES))
    if poses < MIN_POSES or spread < POSE_SPREAD_DEG:
        report["reasons"].append("姿态多样性不足：%d 个不同姿态，最大关节跨度 %.1f° < %.1f°"
                                 % (poses, spread, POSE_SPREAD_DEG))
    try:
        scale = _fit_scale(usable, gravity_fn)
        rmse = None if scale is None else _rmse(usable, gravity_fn, scale)
    except Exception as error:  # noqa: BLE001
        report["reasons"].append("模型计算失败：%s" % error)
        return report
    fit_payload = None
    holdout_payload = None
    if scale is None:
        report["reasons"].append("重力模型输出接近零，无法拟合尺度")
    elif scale <= 0:
        report["reasons"].append("拟合尺度 %.3f 非正：符号/方向/模型异常或不可辨识，拒绝作为可用尺度" % scale)
    else:
        fit_payload = {"kind": "global_gravity_scale", "scale": round(scale, 5),
                       "residual_rmse_nm": round(rmse, 4),
                       "notes": "本工具只做全局尺度（范围选择），不推断单连杆质量或质心"}
        per_axis = _residual_stats(usable, gravity_fn, guards)
        report["diagnostics"] = report["diagnostics"] or {}
        report["diagnostics"].update({"diagnostic_only": False, "per_axis": per_axis})
        for axis, row in enumerate(per_axis):
            agreement = row["measured_sign_agreement"]
            if agreement is not None and agreement < SIGN_AGREEMENT_MIN:
                report["reasons"].append("J%d 力矩符号一致率 %.2f 过低，方向/映射存疑"
                                         % (axis + 1, agreement))
        if len(usable) >= 4:
            train, test = (usable, explicit_holdout) if explicit_holdout else (usable[0::2], usable[1::2])
            train_scale, test_scale = _fit_scale(train, gravity_fn), _fit_scale(test, gravity_fn)
            if train_scale is not None and test_scale is not None:
                test_rmse = _rmse(test, gravity_fn, train_scale)
                holdout_payload = {"train_samples": len(train), "test_samples": len(test),
                                   "split": 'explicit_independent' if explicit_holdout else 'alternating',
                                   "train_scale": round(train_scale, 5),
                                   "test_scale": round(test_scale, 5),
                                   "test_rmse_nm": round(test_rmse, 4),
                                   "scale_difference": round(test_scale - train_scale, 5)}
                if test_rmse > HOLDOUT_RMSE_MAX_NM or \
                        (rmse is not None and test_rmse > max(rmse, 1e-9) * HOLDOUT_RATIO_MAX):
                    report["reasons"].append("留出验证失败：测试 RMSE %.3f N·m 明显偏大" % test_rmse)
    report["identifiability"] = {
        "scope": "本工具主动只拟合一个全局重力尺度，不做单连杆质量/质心辨识",
        "not_a_theorem": "足够姿态激励下静态数据可辨识某些重力参数；本工具的限制是范围选择",
        "needs": "可比质量/质心数据或跨负载、更多姿态的独立测量才能进一步辨识",
    }
    if report["reasons"]:
        report["status"] = "INSUFFICIENT"
        if report['diagnostics'] is not None:
            report['diagnostics']['diagnostic_only'] = True
    else:
        report["status"] = "PROVISIONAL_SCALE"
        report["fit"] = fit_payload
        if holdout_payload is not None:
            report["holdout"] = holdout_payload
    for axis, row in enumerate((report["fit"] and report["diagnostics"] and
                                report["diagnostics"].get("per_axis")) or []):
        if row["residual_std_nm"] is not None and row["residual_std_nm"] > guards[axis] / 4.0:
            report["warnings"].append("J%d 残差标准差 %.2f N·m 偏大，模型或样本存在未解释偏差"
                                      % (axis + 1, row["residual_std_nm"]))
    return report


def core_identity_complete(sample):
    model = sample.get("model") or {}
    return all(isinstance(model.get(key), str) and len(model[key]) == 64 for key in HASH_KEYS) \
        and model.get("mass_profile_status") in ("ESTIMATE_ONLY_NOT_FOR_HARDWARE", "CALIBRATED") \
        and _finite_list(model.get("gravity_base_m_s2"), 3) \
        and _finite(model.get("j3_offset_rad")) \
        and isinstance(sample.get("load_id"), str) and bool(sample["load_id"].strip())


def preview_lines(report):
    lines = ["状态：%s" % report["status"],
             "样本：总 %d，无效 %d，排除 %d，合格 %d"
             % (report["counts"]["total"], report["counts"]["invalid"],
                report["counts"]["excluded"], report["counts"]["usable"])]
    for reason in report["reasons"]:
        lines.append("  不足：%s" % reason)
    if report["diagnostics"]:
        diagnostics = report["diagnostics"]
        lines.append("  诊断（%s）：尺度 %s，RMSE %s N·m"
                     % ("仅诊断" if diagnostics.get("diagnostic_only") else "合格样本",
                        diagnostics.get("scale"), diagnostics.get("residual_rmse_nm")))
        for row in diagnostics.get("per_axis") or []:
            lines.append("    J%d 残差均值 %s，标准差 %s，符号一致率 %s"
                         % (row["joint"], row["residual_mean_nm"], row["residual_std_nm"],
                            row["measured_sign_agreement"]))
    if report["fit"]:
        lines.append("  全局重力尺度：%s（残差 RMSE %s N·m）"
                     % (report["fit"]["scale"], report["fit"]["residual_rmse_nm"]))
    if report["holdout"]:
        lines.append("  留出：测试尺度 %s，训练尺度 %s，测试 RMSE %s N·m"
                     % (report["holdout"]["test_scale"], report["holdout"]["train_scale"],
                        report["holdout"]["test_rmse_nm"]))
    for warning in report["warnings"]:
        lines.append("  警告：%s" % warning)
    return lines
