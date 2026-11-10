"""Record live five-axis controller state for signed static analysis.

Read-only ROS subscriber.  It does not create a controller, call arm/reset, or
publish commands.  Run only while the intended driver is already running.

Usage:
    python3 recorder.py --duration 5 --output /tmp/state.jsonl
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _vec(message):
    return list(message.positions)


class Recorder(Node):
    def __init__(self, output: Path):
        super().__init__("slaihand_signed_recorder")
        self.output = output
        self.start = time.monotonic()
        self.file = output.open("w", encoding="utf-8")
        self.count = {"controller": 0, "dynamic": 0}
        self.create_subscription(
            JointTrajectoryControllerState,
            "/slaihand/slaihand_joint_trajectory_controller/controller_state",
            self.on_controller, 10)
        # DynamicJointState is imported lazily; retain a clear failure if the
        # installed message type differs.
        try:
            from control_msgs.msg import DynamicJointState
        except ImportError as error:  # pragma: no cover - environment guard
            raise RuntimeError("control_msgs/DynamicJointState is required") from error
        self.create_subscription(
            DynamicJointState, "/slaihand/dynamic_joint_states", self.on_dynamic, 20)

    def _write(self, kind, payload):
        record = {"t": round(time.monotonic() - self.start, 6),
                  "kind": kind, "payload": payload}
        self.file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.file.flush()

    def on_controller(self, message):
        self.count["controller"] += 1
        self._write("controller_state", {
            "joint_names": list(message.joint_names),
            "reference_positions": list(message.reference.positions),
            "feedback_positions": list(message.feedback.positions),
            "error_positions": list(message.error.positions),
            "reference_velocities": list(message.reference.velocities),
            "feedback_velocities": list(message.feedback.velocities),
            "output": list(message.output.positions),
        })

    def on_dynamic(self, message):
        self.count["dynamic"] += 1
        values = {}
        for name, interfaces in zip(message.joint_names, message.interface_values):
            values[name] = dict(zip(interfaces.interface_names, interfaces.values))
        self._write("dynamic", {"joint_names": list(message.joint_names),
                                "interfaces": values})

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--output", default="/tmp/five_axis_state.jsonl")
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(Path(args.output))
    deadline = time.monotonic() + max(0.1, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        counts = dict(node.count)
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({"counts": counts, "output": args.output}, ensure_ascii=False))


if __name__ == "__main__":
    main()
