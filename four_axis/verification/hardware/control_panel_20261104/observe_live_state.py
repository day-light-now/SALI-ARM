"""Read ROS feedback only; no service, serial or motor commands."""
import json
import math
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.msg import DynamicJointState

rclpy.init()
node=rclpy.create_node('slaihand_readonly_recovery_observer')
rows=[]
def receive(msg):
    axes=[]
    for name, value in zip(msg.joint_names,msg.interface_values):
        data=dict(zip(value.interface_names,value.values))
        angle=data.get('position',float('nan'))
        axes.append(dict(joint=name,angle_deg=round(math.degrees(angle),3) if math.isfinite(angle) else None,
                         **{k:data.get(k) for k in ('feedback_valid','feedback_age','control_state','motor_state','control_mode','timeout_count')}))
    rows.append(axes)
node.create_subscription(DynamicJointState,'/slaihand/dynamic_joint_states',receive,
                         QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
print(json.dumps(dict(sample_count=len(rows),first=rows[0] if rows else None,last=rows[-1] if rows else None),allow_nan=True))
node.destroy_node()
rclpy.shutdown()
