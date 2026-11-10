"""Read-only calibration capture / import / preview for the four-axis arm.

C-phase rework rules:
- one fresh DynamicJointState snapshot supplies position/velocity/effort/temps;
  the total feedback age is driver age plus local receipt age and must stay
  within the policy limit; cached or repeated receipts are never counted as
  static evidence;
- a sample carries a sustained static window (count, span, speed/position
  spread) and the stage/motor_state that prove an enabled hold;
- raw files are created exclusively with unique names and finalized read-only;
  analysis reports are written to a different path;
- runtime configuration identity is verified against deployed resources;
  missing/stale/mismatched identities remain diagnostic only.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
import calibration_core as core  # noqa: E402

URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
PROFILE = ROOT / "src/slaihand_bringup/config/gravity_provisional.json"
MAPPING = ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_motion_policy.yaml"
if not URDF.is_file():
    from ament_index_python.packages import get_package_share_directory
    URDF = Path(get_package_share_directory('slaihand_description')) / 'urdf/slaihand_four_axis.urdf'
    _config = Path(get_package_share_directory('slaihand_bringup')) / 'config'
    PROFILE, MAPPING, POLICY = (_config / name for name in
        ('gravity_provisional.json', 'hardware_mapping.yaml', 'canusb_motion_policy.yaml'))
TOPICS = {"joint": "/slaihand/joint_states", "dynamic": "/slaihand/dynamic_joint_states",
          "controller": "/slaihand/slaihand_joint_trajectory_controller/controller_state",
          "ff": "/slaihand/gravity_ff_state", "identity": "/slaihand/runtime_identity"}
NAMES = ["slaihand_joint%d" % index for index in range(1, 5)]
INTERFACE_KEYS = ("position", "velocity", "effort", "feedback_age", "feedback_valid",
                  "driver_temperature", "motor_temperature", "motor_state", "control_state")
RUNNING_IDENTITY = {"verified": False, "source": "unknown",
                    "note": "驱动未提供可验证的运行时配置身份；本批仅作诊断，不作为可信驱动标定"}


def _sha256(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _policy_values():
    values = {}
    for raw in POLICY.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def provenance(load_id):
    policy = _policy_values()
    profile = json.loads(PROFILE.read_text(encoding="utf-8"))
    return {
        "urdf_sha256": _sha256(URDF),
        "mass_profile_sha256": _sha256(PROFILE),
        "mass_profile_status": (profile.get("metadata") or {}).get("status"),
        "mapping_sha256": _sha256(MAPPING),
        "policy_sha256": _sha256(POLICY),
        "gravity_base_m_s2": [float(policy.get("gravity_base_x", "nan")),
                              float(policy.get("gravity_base_y", "nan")),
                              float(policy.get("gravity_base_z", "nan"))],
        "j3_offset_rad": float(policy.get("gravity_j3_offset_rad", "nan")),
        "load_id": load_id,
        "units": {"position": "rad", "velocity": "rad/s", "torque": "N*m"},
    }


def _model_fn():
    import control_panel_gravity as gravity
    policy = _policy_values()
    base = tuple(float(policy[key]) for key in ("gravity_base_x", "gravity_base_y", "gravity_base_z"))
    offset = float(policy["gravity_j3_offset_rad"])
    model = gravity.FourAxisGravityModel(URDF, PROFILE, gravity_base=base,
                                         fixed_joint_offsets={"slaihand_joint3": offset} if offset else None)
    return lambda positions: model.compute(positions)["tau_model_nm"]


class Recorder:
    """Read-only snapshot node: one fresh DynamicJointState is the source."""

    def __init__(self, node=None):
        import rclpy
        from control_msgs.msg import JointTrajectoryControllerState
        from control_msgs.msg import DynamicJointState
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray, String

        self.rclpy = rclpy
        self._owns_node = node is None
        self.node = node if node is not None else rclpy.create_node("slaihand_calibration_capture")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        identity_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.joint = None
        self.dynamic = None
        self.dynamic_sequence = 0
        self.reference = None
        self.reference_at = None
        self.ff = None
        self.runtime_identity = None
        self.runtime_identity_at = None
        self.local_identity = None
        self.node.create_subscription(JointState, TOPICS["joint"], self._joint, qos)
        self.node.create_subscription(DynamicJointState, TOPICS["dynamic"], self._dynamic, qos)
        self.node.create_subscription(JointTrajectoryControllerState, TOPICS["controller"],
                                      self._controller, qos)
        self.node.create_subscription(Float64MultiArray, TOPICS["ff"], self._ff, qos)
        self.node.create_subscription(String, TOPICS["identity"], self._identity, identity_qos)

    def _joint(self, message):
        self.joint = {"at": time.monotonic(),
                      "positions": dict(zip(message.name, message.position))}

    def _dynamic(self, message):
        now = time.monotonic()
        axes = {}
        for name, values in zip(message.joint_names, message.interface_values):
            if name in NAMES:
                axes[name] = dict(zip(values.interface_names, values.values))
        self.dynamic = {"at": now, "axes": axes}
        self.dynamic_sequence += 1

    def _controller(self, message):
        self.reference = [float(value) for value in message.reference.positions]
        self.reference_at = time.monotonic()

    def _ff(self, message):
        values = [float(value) for value in message.data]
        if len(values) in (21, 22, 24):
            self.ff = {"at": time.monotonic(), "alpha": values[0], "ramp": values[1],
                       "active": values[2] >= 0.5, "stage": int(values[3]),
                       "command_id": int(values[4]), "applied": values[13:17]}

    def _identity(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict) or payload.get("schema") != "slaihand_runtime_identity/1":
            return
        self.runtime_identity = payload
        self.runtime_identity_at = time.monotonic()

    def _runtime_view(self):
        """Runtime identity as reported by the driver, verified against local files."""
        if self.runtime_identity is None or self.runtime_identity_at is None:
            return {"verified": False, "source": "driver_unavailable",
                    "reason": "未收到驱动运行身份消息"}
        age = time.monotonic() - self.runtime_identity_at
        if age > 3.0:
            return {"verified": False, "source": "driver_stale",
                    "reason": "驱动运行身份消息已 %.1f s 未刷新（活性未确认）" % age,
                    "identity": self.runtime_identity, "age_s": round(age, 3)}
        identity = self.runtime_identity
        local = self.local_identity or {}
        mismatches = []
        if identity.get('position_branch_ready') is not True:
            mismatches.append('position_branch_not_ready')
        for key in ("mapping_sha256", "model_sha256", "policy_sha256", "mass_profile_sha256"):
            if isinstance(local.get(key), str) and identity.get(key) != local.get(key):
                mismatches.append(key)
        if list(identity.get("gravity_base_m_s2") or []) != list(local.get("gravity_base_m_s2") or []):
            mismatches.append("gravity_base_m_s2")
        if not (core._finite(identity.get("j3_offset_rad")) and core._finite(local.get("j3_offset_rad"))
                and abs(identity["j3_offset_rad"] - local["j3_offset_rad"]) < 1e-9):
            mismatches.append("j3_offset_rad")
        if identity.get("load_id") != local.get("load_id"):
            mismatches.append("load_id")
        return {"verified": not mismatches, "source": "driver_runtime_identity",
                "age_s": round(age, 3), "identity": identity,
                "mismatches": mismatches,
                "reason": None if not mismatches else "运行时身份与本地文件不一致：%s"
                % ", ".join(mismatches)}

    def _build(self):
        now = time.monotonic()
        if not self.dynamic:
            return None
        local_age = now - self.dynamic["at"]
        if not math.isfinite(local_age) or local_age < 0:
            return None
        axes = self.dynamic["axes"]
        if set(axes) != set(NAMES):
            return None
        rows = {}
        for name in NAMES:
            interface = axes[name]
            if any(key not in interface for key in INTERFACE_KEYS):
                return None
            values = [float(interface[key]) for key in INTERFACE_KEYS if key != "feedback_valid"]
            if not all(math.isfinite(value) for value in values):
                return None
            if float(interface["feedback_valid"]) < 0.5:
                return None
            rows[name] = interface
        driver_ages = [float(rows[name]["feedback_age"]) for name in NAMES]
        if any(age < 0 or not math.isfinite(age) for age in driver_ages):
            return None
        # The transport delay is unknown: the total is a lower bound, and every
        # axis total is driver_age + local_receipt_age (never max).
        total_ages = [driver_age + local_age for driver_age in driver_ages]
        if any(not math.isfinite(age) or age > core.MAX_FEEDBACK_AGE_S for age in total_ages):
            return None
        stages = {int(round(float(rows[name]["control_state"]))) for name in NAMES}
        if len(stages) != 1:
            return None
        positions = [float(rows[name]["position"]) for name in NAMES]
        command_ids = {rows[name].get('command_id') for name in NAMES}
        joint_delta = None
        if self.joint is not None and now - self.joint["at"] <= core.MAX_FEEDBACK_AGE_S:
            joint_positions = [self.joint["positions"].get(name) for name in NAMES]
            if any(value is None for value in joint_positions):
                return None
            joint_delta = max(abs(float(positions[i]) - float(joint_positions[i])) for i in range(4))
            if joint_delta > 0.01:
                return None
        reference = None
        reference_age = None
        if self.reference is not None and self.reference_at is not None:
            reference_age = round(now - self.reference_at, 3)
            if reference_age <= 1.0:
                reference = [float(value) for value in self.reference]
        ff = None
        if self.ff is not None:
            ff = {key: value for key, value in self.ff.items() if key != "at"}
            ff["age_s"] = round(now - self.ff["at"], 4)
        return {"positions_rad": positions,
                "velocities_rad_s": [float(rows[name]["velocity"]) for name in NAMES],
                "effort_estimate_nm": [float(rows[name]["effort"]) for name in NAMES],
                "driver_temperature_c": [float(rows[name]["driver_temperature"]) for name in NAMES],
                "motor_temperature_c": [float(rows[name]["motor_temperature"]) for name in NAMES],
                "feedback_age_s": driver_ages,
                "local_receipt_age_s": round(local_age, 6),
                "total_age_s": [round(age, 6) for age in total_ages],
                "capture_age_s": round(max(total_ages), 6),
                "feedback_valid": [True] * 4,
                "motor_state": [int(round(float(rows[name]["motor_state"]))) for name in NAMES],
                "stage": stages.pop(),
                "dynamic_sequence": self.dynamic_sequence,
                "command_id": next(iter(command_ids)) if len(command_ids) == 1 else None,
                "joint_delta_rad": None if joint_delta is None else round(joint_delta, 5),
                "controller_reference_rad": reference,
                "controller_reference_age_s": reference_age,
                "driver_ff": ff,
                "runtime": self._runtime_view(),
                "_monotonic": now}

    def snapshot(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            sample = self._build()
            if sample is not None:
                return sample
        return None

    def close(self):
        if self._owns_node:
            self.node.destroy_node()


def _window_aggregate(snapshots, require_zero_ff=True):
    positions = [snapshot["positions_rad"] for snapshot in snapshots]
    mean_positions = [sum(row[axis] for row in positions) / len(positions) for axis in range(4)]
    mean_effort = [sum(snapshot["effort_estimate_nm"][axis] for snapshot in snapshots) / len(snapshots)
                   for axis in range(4)]
    mean_velocity = [sum(snapshot["velocities_rad_s"][axis] for snapshot in snapshots) / len(snapshots)
                     for axis in range(4)]
    max_speed = max(abs(snapshot["velocities_rad_s"][axis]) for snapshot in snapshots for axis in range(4))
    max_delta = max(abs(row[axis] - mean_positions[axis]) for row in positions for axis in range(4))
    span = snapshots[-1]["_monotonic"] - snapshots[0]["_monotonic"]
    gaps = [snapshots[index + 1]["_monotonic"] - snapshots[index]["_monotonic"]
            for index in range(len(snapshots) - 1)] or [0.0]
    stages = [snapshot["stage"] for snapshot in snapshots]
    stage = stages[0] if len(set(stages)) == 1 else None
    motor_min = [min(snapshot["motor_state"][axis] for snapshot in snapshots) for axis in range(4)]
    all_enabled = all(snapshot["stage"] in (core.STAGE_HOLD, core.STAGE_TRACK)
                      and all(state == 1 for state in snapshot["motor_state"]) for snapshot in snapshots)
    fresh_all = all(all(age <= core.MAX_FEEDBACK_AGE_S for age in snapshot["total_age_s"])
                    and snapshot["capture_age_s"] <= core.MAX_FEEDBACK_AGE_S for snapshot in snapshots)
    ff_list = [snapshot.get("driver_ff") for snapshot in snapshots]
    ff_all_zero = all(isinstance(ff, dict) and ff.get("alpha") in (0, 0.0) for ff in ff_list)
    ff_all_fresh = all(isinstance(ff, dict) and isinstance(ff.get("age_s"), (int, float))
                       and 0.0 <= ff["age_s"] <= core.FF_MAX_AGE_S for ff in ff_list)
    command_ids = sorted({ff.get("command_id") for ff in ff_list
                          if isinstance(ff, dict) and ff.get("command_id") is not None})
    valid = (span >= core.WINDOW_MIN_SPAN_S
             and len(snapshots) >= core.WINDOW_MIN_SAMPLES
             and max_speed <= core.STATIC_SPEED_RAD_S
             and max_delta <= core.STATIC_POSITION_DELTA_RAD
             and max(gaps) <= core.WINDOW_MAX_GAP_S
             and stage in (core.STAGE_HOLD, core.STAGE_TRACK)
             and all_enabled and fresh_all and ff_all_fresh and len(command_ids) == 1
             and (ff_all_zero if require_zero_ff else True))
    reasons = []
    if span < core.WINDOW_MIN_SPAN_S:
        reasons.append("窗口跨度不足")
    if max(gaps) > core.WINDOW_MAX_GAP_S:
        reasons.append("窗口内采样间隙过大")
    if not all_enabled:
        reasons.append("窗口内并非逐帧使能")
    if not fresh_all:
        reasons.append("窗口内并非逐帧新鲜")
    if require_zero_ff and not ff_all_zero:
        reasons.append("窗口内前馈并非逐帧为零")
    if not ff_all_fresh:
        reasons.append("窗口内前馈状态并非逐帧新鲜")
    if len(command_ids) != 1:
        reasons.append("窗口内命令 ID 变化")
    return {"samples": len(snapshots), "span_s": round(span, 4),
            "max_gap_s": round(max(gaps), 4),
            "max_speed_rad_s": round(max_speed, 5),
            "max_position_delta_rad": round(max_delta, 5),
            "stage": stage, "motor_state": motor_min, "all_enabled": all_enabled,
            "fresh_all": fresh_all, "ff_all_zero": ff_all_zero, "ff_all_fresh": ff_all_fresh,
            "command_ids": command_ids, "valid": valid, "reasons": reasons,
            "mean_positions_rad": mean_positions, "mean_velocity_rad_s": mean_velocity,
            "mean_effort_estimate_nm": mean_effort}


def collect_static(recorder, window_seconds, window_samples, trigger, timeout,
                   require_zero_ff=True):
    """Collect one sustained static window; returns an aggregate sample or None."""
    if trigger == "enter":
        try:
            input("把机械臂摆到目标姿态并保持静止，确认无接触/支撑后按 Enter 开始采集…")
        except EOFError:
            print("没有交互终端，拒绝采集（使用 --trigger auto 仅用于离线测试）", file=sys.stderr)
            return None
    deadline = time.monotonic() + timeout
    snapshots = []
    previous_sequence = None
    first_command = None
    while len(snapshots) < window_samples and time.monotonic() < deadline:
        snapshot = recorder.snapshot(timeout=1.0)
        if snapshot is None:
            time.sleep(0.02)
            continue
        if previous_sequence is not None and snapshot["dynamic_sequence"] == previous_sequence:
            time.sleep(0.02)  # cached receipt must not count twice
            continue
        previous_sequence = snapshot["dynamic_sequence"]
        command_id = (snapshot.get("driver_ff") or {}).get("command_id")
        if first_command is None:
            first_command = command_id
        elif command_id != first_command:
            # A new command/session must not be spliced into a stable window.
            snapshots = []
            first_command = command_id
            continue
        snapshots.append(snapshot)
        time.sleep(max(0.0, window_seconds / max(window_samples, 1) - 0.02))
    if len(snapshots) < core.WINDOW_MIN_SAMPLES:
        return None
    window = _window_aggregate(snapshots, require_zero_ff=require_zero_ff)
    if not window["valid"]:
        print("静止窗口未通过：%s" % "；".join(window["reasons"]), file=sys.stderr)
        return None
    last = snapshots[-1]
    aggregate = {key: value for key, value in last.items() if not key.startswith("_")}
    aggregate["positions_rad"] = window["mean_positions_rad"]
    aggregate["velocities_rad_s"] = window["mean_velocity_rad_s"]
    aggregate["effort_estimate_nm"] = window["mean_effort_estimate_nm"]
    aggregate["static_window"] = window
    return aggregate


def _open_raw(out_dir, tag):
    raw_dir = Path(out_dir) / "raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for attempt in range(1000):
        path = raw_dir / ("%s_%s_%09d.jsonl" % (stamp, tag, (time.time_ns() // 1000) % 1000000000))
        if attempt:
            path = raw_dir / ("%s_%s_%d.jsonl" % (stamp, tag, attempt))
        try:
            return path, path.open("x", encoding="utf-8")
        except FileExistsError:
            continue
    raise CalibrationError("无法创建唯一的原始数据文件")


def _local_identity(load_id):
    return {"mapping_sha256": _sha256(MAPPING), "model_sha256": _sha256(URDF),
            "policy_sha256": _sha256(POLICY), "mass_profile_sha256": _sha256(PROFILE),
            "gravity_base_m_s2": [float(_policy_values().get("gravity_base_x", "nan")),
                                  float(_policy_values().get("gravity_base_y", "nan")),
                                  float(_policy_values().get("gravity_base_z", "nan"))],
            "j3_offset_rad": float(_policy_values().get("gravity_j3_offset_rad", "nan")),
            "load_id": load_id}


def run_record(args):
    import rclpy
    rclpy.init(args=None)
    recorder = Recorder()
    recorder.local_identity = _local_identity(args.load_id)
    model = provenance(args.load_id)
    # Wait briefly for the driver runtime identity (latched, refreshed periodically).
    identity_deadline = time.monotonic() + args.identity_timeout
    identity = None
    while time.monotonic() < identity_deadline:
        rclpy.spin_once(recorder.node, timeout_sec=0.1)
        if recorder.runtime_identity is not None:
            identity = recorder._runtime_view()
            break
    if identity is None:
        identity = recorder._runtime_view()
    print("运行身份：verified=%s source=%s%s" % (
        identity.get("verified"), identity.get("source"),
        "" if identity.get("verified") else "（%s）" % identity.get("reason")))
    header = {"schema": core.SCHEMA_CAPTURE, "tool": "calibration_capture.py",
              "created": time.strftime("%Y-%m-%dT%H:%M:%S"), "recorded_at": time.time(),
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
              "model": model, "load_id": args.load_id, "runtime": identity,
              "window": {"seconds": args.window_seconds, "samples": args.window_samples,
                         "trigger": args.trigger, "speed_threshold_rad_s": core.STATIC_SPEED_RAD_S,
                         "position_threshold_rad": core.STATIC_POSITION_DELTA_RAD,
                         "ff_max_age_s": core.FF_MAX_AGE_S},
              "requested_count": args.count}
    stream = None
    path = None
    written = 0
    rejected = []
    try:
        path, stream = _open_raw(args.out, args.tag)
        stream.write(json.dumps(header, ensure_ascii=False) + "\n")
        stream.flush()
        while written < args.count:
            aggregate = collect_static(recorder, args.window_seconds, args.window_samples,
                                       args.trigger, args.timeout)
            if aggregate is None:
                rejected.append("静止窗口采集失败（样本不足/跨度不足/缓存重复）")
                break
            sample = {"schema": core.SCHEMA_SAMPLE, "recorded_at": time.time(),
                      "support": args.support, "contact": args.contact, "motion": args.motion,
                      "note": args.note, "operator": args.operator, "load_id": args.load_id,
                      "model": model, "runtime": RUNNING_IDENTITY, **aggregate}
            ok, reasons = core.validate_sample(sample)
            if not ok:
                rejected.append("；".join(reasons))
            else:
                stream.write(json.dumps(sample, ensure_ascii=False) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
                written += 1
                print("记录 %d/%d：%s" % (written, args.count,
                                         [round(math.degrees(v), 2) for v in aggregate["positions_rad"]]))
            if args.count > 1 and args.trigger == "enter":
                pass  # the next iteration asks for the next pose explicitly
        stream.close()
        stream = None
        os.chmod(path, 0o444)
    finally:
        if stream is not None:
            try:
                stream.close()
            except OSError:
                pass
            if path is not None and path.exists():
                os.chmod(path, 0o444)
        recorder.close()
        rclpy.shutdown()
    print(json.dumps({"written": written, "rejected": rejected,
                      "raw_file": str(path) if path else None,
                      "runtime_identity": RUNNING_IDENTITY["source"]}, ensure_ascii=False))
    if written == 0:
        print("没有写入任何样本", file=sys.stderr)
        return 2
    return 0


def _expected_identity(model):
    return {"urdf_sha256": model.get("urdf_sha256"),
            "mass_profile_sha256": model.get("mass_profile_sha256"),
            "mass_profile_status": model.get("mass_profile_status"),
            "mapping_sha256": model.get("mapping_sha256"),
            "policy_sha256": model.get("policy_sha256"),
            "gravity_base_m_s2": list(model.get("gravity_base_m_s2") or []),
            "j3_offset_rad": model.get("j3_offset_rad")}


def _build_report(path):
    payload = core.load_raw(path)
    header = payload["header"]
    model = _model_fn()
    expected = _expected_identity(header.get("model") or {})
    expected["load_id"] = header.get("load_id")
    report = core.analyze(payload["samples"], model, expected_identity=expected)
    report["capture"] = {"support": header.get("support"), "contact": header.get("contact"),
                         "motion": header.get("motion"), "note": header.get("note"),
                         "raw_file": str(path), "runtime": header.get("runtime"),
                         "load_id": header.get("load_id")}
    stored = header.get("model") or {}
    current = provenance(header.get("load_id") or "unknown")
    mismatches = [key for key in ("urdf_sha256", "mass_profile_sha256", "mapping_sha256", "policy_sha256")
                  if stored.get(key) and stored[key] != current.get(key)]
    if mismatches:
        report["warnings"].append("原始样本模型版本与当前文件不一致：%s" % ", ".join(mismatches))
        report["reasons"].append("模型档案版本不一致，需重新采集或还原对应版本")
        report["status"] = "INSUFFICIENT"
    if (header.get("runtime") or {}).get("verified") is not True:
        report["warnings"].append("运行时配置身份未验证/为 UNKNOWN：本批只作诊断，不能作为可信驱动标定")
    unverified = [sample for sample in payload["samples"]
                  if (sample.get("runtime") or {}).get("verified") is not True]
    if unverified:
        report["warnings"].append("有 %d 条样本运行时身份未验证，已被资格门禁排除" % len(unverified))
    return report


def run_import(path, as_json=False):
    payload = core.load_raw(path)
    header = payload["header"]
    counts = {"samples": len(payload["samples"]), "invalid": 0, "excluded": 0}
    reasons = []
    for sample in payload["samples"]:
        ok, invalid = core.validate_sample(sample)
        if not ok:
            counts["invalid"] += 1
            reasons.extend(invalid)
        elif core.exclusion_reason(sample):
            counts["excluded"] += 1
    result = {"header": {key: header.get(key) for key in
                         ("tool", "created", "support", "contact", "motion", "note",
                          "operator", "load_id", "runtime")},
              "counts": counts, "invalid_reasons": reasons[:20]}
    print(json.dumps(result, ensure_ascii=False, indent=2) if as_json else
          "样本 %d，无效 %d，排除 %d；运行时身份=%s"
          % (counts["samples"], counts["invalid"], counts["excluded"],
             (header.get("runtime") or {}).get("source")))
    return 0


def run_preview(path, as_json=False, save_report=None):
    report = _build_report(path)
    if save_report is not None:
        raw_resolved = Path(path).resolve()
        report_resolved = Path(save_report).resolve()
        if raw_resolved == report_resolved or \
                (Path(save_report).exists() and os.path.samefile(raw_resolved, report_resolved)):
            print("拒绝：报告路径与原始数据相同（或符号链接相同）", file=sys.stderr)
            return 2
        payload = {"schema": "slaihand_calibration_report/2",
                   "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "tool": "calibration_capture.py", "raw_file": str(path),
                   "raw_sha256": _sha256(Path(path)), "analysis": report}
        save_report = Path(save_report)
        save_report.parent.mkdir(parents=True, exist_ok=True)
        save_report.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("报告已写入：%s（与原始数据分开保存）" % save_report)
    if as_json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        for line in core.preview_lines(report):
            print(line)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--import", dest="import_path", type=Path)
    parser.add_argument("--preview", dest="preview_path", type=Path)
    parser.add_argument("--out", type=Path, default=Path.home() / ".ros" / "slaihand" / "calibration")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--window-seconds", type=float, default=2.0)
    parser.add_argument("--window-samples", type=int, default=10)
    parser.add_argument("--trigger", choices=("enter", "auto"), default="enter")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--identity-timeout", type=float, default=8.0)
    parser.add_argument("--support", choices=core.SUPPORTS, default="unknown")
    parser.add_argument("--contact", choices=core.CONTACTS, default="none")
    parser.add_argument("--motion", choices=core.MOTIONS, default="static")
    parser.add_argument("--load-id", default="no_end_load")
    parser.add_argument("--note", default="")
    parser.add_argument("--operator", default="")
    parser.add_argument("--tag", default="capture")
    parser.add_argument("--save-report", type=Path, help="预览时把分析报告写到独立文件")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    modes = [bool(args.record), args.import_path is not None, args.preview_path is not None]
    if sum(modes) != 1:
        parser.error("需要且仅需要 --record / --import / --preview 之一")
    if args.record:
        return run_record(args)
    if args.import_path is not None:
        return run_import(args.import_path, args.json)
    return run_preview(args.preview_path, args.json, args.save_report)


if __name__ == "__main__":
    sys.exit(main())
