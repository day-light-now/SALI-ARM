#!/usr/bin/env python3
"""Four-axis offline check, including every axis separately and together."""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

NAMES = [f"slaihand_joint{i}" for i in range(1, 5)]


def wait_future(node, future, seconds=20):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise RuntimeError("Controller response timed out")
    return future.result()


def main():
    rclpy.init()
    node = Node("verify_slaihand_fake_control")
    state = {}

    def on_state(message):
        state.update(zip(message.name, message.position))

    subscription = node.create_subscription(JointState, "/joint_states", on_state, 10)
    client = ActionClient(node, FollowJointTrajectory,
                         "/slaihand_joint_trajectory_controller/follow_joint_trajectory")
    try:
        if not client.wait_for_server(timeout_sec=30):
            raise RuntimeError("Trajectory controller unavailable")
        targets = [[0.0] * 4]
        for i in range(4):
            single = [0.0] * 4
            single[i] = 0.08727
            targets.extend([single, [0.0] * 4])
        targets.extend([[0.17453, 0.08727, -0.08727, 0.08727], [0.0] * 4])
        for target in targets:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = NAMES
            point = JointTrajectoryPoint()
            point.positions = target
            point.time_from_start.sec = 3
            goal.trajectory.points = [point]
            handle = wait_future(node, client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError("Trajectory rejected")
            result = wait_future(node, handle.get_result_async())
            if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(result.result.error_string)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if all(name in state and abs(state[name] - angle) < 0.002
                       for name, angle in zip(NAMES, target)):
                    break
            else:
                raise RuntimeError(f"State mismatch: {state}")
            print(f"PASS target={target} measured={[state[name] for name in NAMES]}", flush=True)
        print("PASS: four axes tested individually, jointly, and returned to zero.", flush=True)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
