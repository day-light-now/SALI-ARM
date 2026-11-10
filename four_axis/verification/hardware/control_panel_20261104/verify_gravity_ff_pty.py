"""PTY verification of the driver gravity feedforward with alpha > 0.

Isolated domain 176, fresh PTY fake drive, real launch/serial path. The fake
drive decodes the official MIT bytes, so the applied tau is verified end to end.
"""
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import rclpy  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

import verify_control_panel as e2e  # noqa: E402
from control_panel_gravity import CoordinateMap, FourAxisGravityModel  # noqa: E402

sys.path.insert(0, str(e2e.ROOT / "src/slaihand_bringup/scripts"))
ALPHA = 0.55
URDF = e2e.ROOT / "src/slaihand_description/urdf/slaihand_four_axis.urdf"
PROFILE = e2e.ROOT / "src/slaihand_bringup/config/gravity_provisional.json"
MAPPING = e2e.ROOT / "src/slaihand_bringup/config/hardware_mapping.yaml"
GRAVITY = (0.0, 9.80665, 0.0)
J3_OFFSET = math.pi
TOLERANCE = (0.15, 0.15, 0.05, 0.05)


class FeedforwardTap:
    def __init__(self):
        self.node = rclpy.create_node("gravity_ff_tap")
        self.messages = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Float64MultiArray, "/slaihand/gravity_ff_state",
                                      self._on_message, qos)

    def _on_message(self, message):
        self.messages.append([float(value) for value in message.data])

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def close(self):
        self.node.destroy_node()


def main():
    rclpy.init(args=None)
    work = Path("/tmp/panel-ff-verify-%s" % time.strftime("%Y%m%d_%H%M%S"))
    observer = e2e.Observer()
    tap = FeedforwardTap()
    session = None
    failures = []
    try:
        session = e2e.Session("ff", work, observer,
                              stack_args=["gravity_ff_alpha:=%g" % ALPHA])
        panel, plant = session.panel, session.plant
        tap.spin(2.0)
        e2e.require("前馈状态话题已发布", bool(tap.messages), tap.messages[-1:])
        alpha_seen = tap.messages[-1][0]
        e2e.require("驱动报告 alpha=%.2f" % ALPHA, abs(alpha_seen - ALPHA) < 1e-9, alpha_seen)

        status = panel.status()
        start = [math.radians(value) for value in e2e.degrees_of(status)]
        document = e2e.plan("relative_start_deg", [([3, 2, -1, 2], 6, 1)])
        code, body = panel.preview(document)
        e2e.require("前馈场景预览通过", code == 200, body)
        ticket = body["ticket"]["id"]
        panel.heartbeat()
        code, body = panel.execute(ticket, document, e2e.confirmations())
        e2e.require("前馈场景执行受理", code == 202 and body["accepted"] and not body["completed"], body)

        samples = []
        operation = None
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            tap.spin(0.05)
            code, body = panel.call("GET", "/api/status")
            operation = (body or {}).get("operation") or {}
            sample = {"tau": list(getattr(plant, "last_tau", [0.0] * 4)),
                      "raw": list(plant.raw),
                      "stage": observer.stage(),
                      "alpha": tap.messages[-1][0] if tap.messages else None,
                      "active": (tap.messages[-1][2] >= 0.5) if tap.messages else None,
                      "driver_stage": int(tap.messages[-1][3]) if tap.messages else None,
                      "applied": list(tap.messages[-1][13:17]) if tap.messages else None,
                      "last_written": list(tap.messages[-1][17:21]) if tap.messages else None}
            samples.append(sample)
            observer.spin(0.01)
            if operation and not operation.get("running") and operation.get("phase") not in (
                    "starting", "running", "stopping"):
                break
        e2e.require("前馈场景动作结束", operation is not None and not operation.get("running"),
                    operation and operation.get("phase"))
        e2e.require("前馈场景客户端正常退出", (operation or {}).get("exit_code") == 0,
                    (operation or {}).get("exit_code"))
        e2e.require("PTY 无编码/路由错误", not plant.errors, plant.errors[:3])

        armed = [sample for sample in samples if sample["stage"] in (6, 7)]
        e2e.require("采样覆盖使能阶段", bool(armed), [sample["stage"] for sample in samples[:5]])
        model = FourAxisGravityModel(URDF, PROFILE, gravity_base=GRAVITY,
                                     fixed_joint_offsets={"slaihand_joint3": J3_OFFSET})
        mapping = CoordinateMap(MAPPING)
        peak = max(armed, key=lambda sample: sum(abs(value) for value in sample["tau"]))
        q_model = mapping.to_model_angle(peak["raw"])
        expected = [mapping.signs[index] * ALPHA * value
                    for index, value in enumerate(model.compute(q_model)["tau_model_nm"])]
        e2e.require("前馈帧确实非零", max(abs(value) for value in peak["tau"]) > 0.3, peak["tau"])
        worst = max(abs(got - want) for got, want in zip(peak["tau"], expected))
        e2e.require("解码前馈与模型一致（≤量化+跟踪）",
                    all(abs(got - want) <= tolerance
                        for got, want, tolerance in zip(peak["tau"], expected, TOLERANCE)),
                    {"decoded": peak["tau"], "expected": expected, "worst": worst})
        applied = peak["applied"]
        e2e.require("话题应用值与帧一致",
                    applied is not None and all(abs(got - want) <= 0.15
                                                for got, want in zip(applied, peak["tau"])),
                    {"applied": applied, "decoded": peak["tau"]})
        e2e.require("驱动报告 alpha 未漂移", peak["alpha"] is not None and abs(peak["alpha"] - ALPHA) < 1e-9,
                    peak["alpha"])
        e2e.require("使能阶段驱动报告前馈活动", any(sample["active"] is True for sample in armed),
                    [sample["active"] for sample in armed[:5]])
        # After the operation the driver must not keep reporting an active non-zero
        # feedforward: the current applied values go to zero and the history is separate.
        tap.spin(1.5)
        final = tap.messages[-1] if tap.messages else None
        e2e.require("停止后不再报告活动前馈",
                    final is not None and final[2] < 0.5
                    and all(abs(value) < 1e-12 for value in final[13:17]),
                    final and {"active": final[2], "applied": final[13:17]})
        print("RESULT: PASS alpha=%.2f peak=%s expected=%s" % (
            ALPHA, [round(value, 4) for value in peak["tau"]], [round(value, 4) for value in expected]))
        return 0
    except AssertionError as error:
        failures.append(str(error))
        print("RESULT: FAIL %s" % error)
        return 1
    finally:
        tap.close()
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
        print("sessions=%s" % e2e.SESSIONS)


if __name__ == "__main__":
    sys.exit(main())
