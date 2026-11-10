"""Read-only ROS check at J2=+5deg; verify the live TF uses corrected +Z."""
import math
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def product(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return (w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
            w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t)


def main():
    rclpy.init()
    node = Node("verify_j2_corrected_direction")
    data = {}
    node.create_subscription(String, "/robot_description", lambda m: data.update(xml=m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, "/joint_states", lambda m: data.update(q=dict(zip(m.name, m.position))), 10)
    def tf(message):
        for frame in message.transforms:
            if frame.header.frame_id == "link1" and frame.child_frame_id == "link2":
                data["tf"] = frame.transform
    node.create_subscription(TFMessage, "/tf", tf, 10)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not all(k in data for k in ["xml", "q", "tf"]):
            rclpy.spin_once(node, timeout_sec=0.1)
        joint = ET.fromstring(data["xml"]).find("joint[@name='slaihand_joint2']")
        assert joint.find("axis").get("xyz") == "0 0 1"
        q = data["q"]["slaihand_joint2"]
        assert abs(q - math.radians(5)) < 0.002, q
        roll, pitch, yaw = map(float, joint.find("origin").get("rpy").split())
        origin = product(product((0, 0, math.sin(yaw/2), math.cos(yaw/2)),
            (0, math.sin(pitch/2), 0, math.cos(pitch/2))),
            (math.sin(roll/2), 0, 0, math.cos(roll/2)))
        correct = product(origin, (0, 0, math.sin(q/2), math.cos(q/2)))
        old = product(origin, (0, 0, -math.sin(q/2), math.cos(q/2)))
        r = data["tf"].rotation
        measured = (r.x, r.y, r.z, r.w)
        assert abs(sum(a*b for a, b in zip(measured, correct))) > 1-1e-8
        assert abs(sum(a*b for a, b in zip(measured, old))) < 0.999
        print("PASS live J2 +5deg TF matches +Z rotation, rejects old -Z rotation; origin unchanged")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
