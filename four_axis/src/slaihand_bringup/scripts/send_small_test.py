#!/usr/bin/env python3
"""Send a four-joint offline trajectory."""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SmallTest(Node):
    def __init__(self):
        super().__init__("slaihand_small_test")
        self.client = ActionClient(
            self, FollowJointTrajectory,
            "/slaihand_joint_trajectory_controller/follow_joint_trajectory",
        )

    def run(self):
        if not self.client.wait_for_server(timeout_sec=15):
            raise RuntimeError("Open the SLAIHand ROS 2 simulation first.")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            "slaihand_joint1", "slaihand_joint2",
            "slaihand_joint3", "slaihand_joint4",
        ]
        point = JointTrajectoryPoint()
        # 10 degrees maximum; simulation-only baseline movement.
        point.positions = [0.17453, 0.08727, -0.08727, 0.08727]
        point.time_from_start = Duration(sec=3)
        goal.trajectory.points = [point]
        response = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, response, timeout_sec=10)
        if not response.done() or not response.result().accepted:
            raise RuntimeError("Trajectory was not accepted.")
        result = response.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=15)
        if not result.done() or result.result().result.error_code != 0:
            raise RuntimeError("Trajectory did not complete successfully.")
        self.get_logger().info("Four-axis OpenArm offline trajectory completed.")


def main():
    rclpy.init()
    node = SmallTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
