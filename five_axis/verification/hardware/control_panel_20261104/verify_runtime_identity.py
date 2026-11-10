"""Verify the driver's read-only runtime identity topic (isolated domain 176)."""
import hashlib
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import verify_control_panel as e2e  # noqa: E402

ROOT = e2e.ROOT
URDF = ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
MAPPING = ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
POLICY = ROOT / "src/slaihand_bringup/config/canusb_motion_policy.yaml"
PROFILE = ROOT / "src/slaihand_bringup/config/gravity_provisional.json"


def sha256(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def policy_values():
    values = {}
    for raw in POLICY.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def main():
    rclpy.init(args=None)
    observer = e2e.Observer()
    node = rclpy.create_node("runtime_identity_check")
    messages = []

    def on_identity(message):
        messages.append((time.monotonic(), message.data))

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/slaihand/runtime_identity", on_identity, qos)
    session = None
    try:
        session = e2e.Session("identity",
                              Path("/tmp/panel-identity-%s" % time.strftime("%Y%m%d_%H%M%S")),
                              observer)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not messages:
            rclpy.spin_once(node, timeout_sec=0.1)
            observer.spin(0.01)
        e2e.require("收到驱动运行时身份（latched）", bool(messages), len(messages))
        payload = json.loads(messages[-1][1])
        e2e.require("身份 schema 正确", payload.get("schema") == "slaihand_runtime_identity/1",
                    payload.get("schema"))
        e2e.require("身份含启动标识", isinstance(payload.get("boot_id"), str) and payload["boot_id"],
                    payload.get("boot_id"))
        expected = {"mapping_sha256": sha256(MAPPING), "model_sha256": sha256(URDF),
                    "policy_sha256": sha256(POLICY), "mass_profile_sha256": sha256(PROFILE)}
        for key, value in expected.items():
            e2e.require("身份 %s 与加载文件一致" % key, payload.get(key) == value,
                        {"runtime": payload.get(key), "file": value})
        policy = policy_values()
        e2e.require("身份 load_id 来自 policy",
                    payload.get("load_id") == policy.get("load_id"), payload.get("load_id"))
        e2e.require("身份重力向量与 policy 一致",
                    [round(float(v), 9) for v in payload.get("gravity_base_m_s2") or []] ==
                    [round(float(policy[key]), 9) for key in
                     ("gravity_base_x", "gravity_base_y", "gravity_base_z")],
                    payload.get("gravity_base_m_s2"))
        e2e.require("身份 J3 偏置与 policy 一致",
                    abs(float(payload.get("j3_offset_rad")) -
                        float(policy["gravity_j3_offset_rad"])) < 1e-9,
                    payload.get("j3_offset_rad"))
        e2e.require("身份记录 kp/kd/alpha/ramp",
                    float(payload.get("kp")) == 160.0 and float(payload.get("kd")) == 5.0
                    and float(payload.get("gravity_ff_alpha")) == 0.0
                    and float(payload.get("gravity_ff_ramp_s")) == 1.0, payload)
        # Liveness: a latched message alone does not prove the publisher is alive;
        # the same session must refresh periodically.
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and len(messages) < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
            observer.spin(0.01)
        e2e.require("身份周期刷新（活性）", len(messages) >= 2, len(messages))
        refreshed = json.loads(messages[-1][1])
        e2e.require("刷新保持同一启动标识", refreshed.get("boot_id") == payload.get("boot_id"),
                    {"first": payload.get("boot_id"), "refreshed": refreshed.get("boot_id")})
        print("RESULT: PASS runtime_identity boot_id=%s" % payload.get("boot_id"))
        return 0
    except AssertionError as error:
        print("RESULT: FAIL %s" % error)
        return 1
    finally:
        node.destroy_node()
        if session is not None:
            try:
                session.close()
            except Exception as error:  # noqa: BLE001
                print("cleanup: %s" % error)
        try:
            e2e.wait_domain_clean(observer)
        except Exception:  # noqa: BLE001
            pass
        observer.close()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
