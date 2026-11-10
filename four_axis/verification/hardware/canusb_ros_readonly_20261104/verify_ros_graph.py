"""Run the actual launch/controllers on a newly generated PTY, never COM/USB.

The child uses a separate ROS domain and process group, both local to this test.
"""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'verification/hardware'))
sys.path.insert(0, str(ROOT))
from four_axis_monitor import load_mapping, usb_packet
import rclpy
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState


def main():
    mapping = load_mapping()
    expected_deg = [15., 25., -10., 5.]
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    assert path.startswith('/dev/pts/')
    os.close(slave)
    stop = threading.Event()
    errors = []
    count = [0]

    def serve():
        buf = bytearray()
        while not stop.is_set():
            try:
                ready, _, _ = select.select([master], [], [], .01)
                if not ready:
                    continue
                buf.extend(os.read(master, 4096))
            except OSError:
                stop.wait(.002)
                continue
            while len(buf) >= 30:
                packet = bytes(buf[:30]); del buf[:30]
                axis = packet[21]
                rid = None if packet[23] == 0xCC else packet[24]
                try:
                    assert packet == usb_packet(axis, rid)
                except Exception:
                    errors.append('Non-readonly TX observed'); stop.set(); return
                count[0] += 1
                if rid is not None:
                    value = {7: 0, 8: axis, 9: 2000, 10: 1, 21: 12.5,
                             22: 25. if axis < 3 else 45., 23: 200. if axis < 3 else 54.}[rid]
                    payload = bytes([axis, 0, 0x33, rid]) + struct.pack('<I' if rid < 21 else '<f', value)
                else:
                    m = mapping[axis]
                    raw = m['zero_offset_rad'] + m['hardware_sign']*math.radians(expected_deg[axis-1])
                    code = round((raw+12.5)*65535/25.)
                    payload = bytes([axis, code >> 8, code & 255, 0x7F, 0xF7, 0xFF, 30+axis, 25+axis])
                os.write(master, b'\xAA\x11\x08'+b'\0'*4+payload+b'\x55')

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    # This subprocess never receives any physical port name.
    env = dict(os.environ, ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1')
    os.environ.update(ROS_DOMAIN_ID='173', ROS_LOCALHOST_ONLY='1')
    process = None; node = None; messages = {}; result = None
    with (HERE/'ros_graph.log').open('w', encoding='utf-8') as log:
        try:
            process = subprocess.Popen(['ros2', 'launch', 'slaihand_bringup',
                'slaihand_canusb_readonly.launch.py', 'port:='+path,
                'log_directory:='+str(HERE/'ros_fake_audit')], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('slai_canusb_fake_graph_verifier')
            node.create_subscription(JointState, '/slaihand/joint_states', lambda m: messages.update(joints=m), 10)
            node.create_subscription(DynamicJointState, '/slaihand/dynamic_joint_states', lambda m: messages.update(dynamic=m), 10)
            end = time.monotonic()+15.
            while time.monotonic() < end and process.poll() is None and not errors:
                rclpy.spin_once(node, timeout_sec=.05)
                if 'joints' not in messages or 'dynamic' not in messages:
                    continue
                q = dict(zip(messages['joints'].name, messages['joints'].position))
                diagnostic = {n: dict(zip(v.interface_names, v.values))
                              for n, v in zip(messages['dynamic'].joint_names, messages['dynamic'].interface_values)}
                if all(f'slaihand_joint{i}' in q and math.isfinite(q[f'slaihand_joint{i}'])
                       and abs(math.degrees(q[f'slaihand_joint{i}'])-expected_deg[i-1]) < .012
                       and diagnostic.get(f'slaihand_joint{i}', {}).get('feedback_valid') == 1.
                       for i in range(1, 5)):
                    result = dict(status='PASS', software_only=True, physical_port_opened=False,
                        topics=['/slaihand/joint_states', '/slaihand/dynamic_joint_states'],
                        axes=4, readonly_queries_seen=count[0], motion_or_parameter_tx=0)
                    break
            assert result is not None and not errors, 'ROS fake graph failed; inspect ros_graph.log: '+str(errors)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None and process.poll() is None:
                for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
                    if process.poll() is not None:
                        break
                    os.killpg(process.pid, sig)  # Only this test's new child process group.
                    try:
                        process.wait(timeout=timeout)
                    except subprocess.TimeoutExpired:
                        continue
            stop.set(); worker.join(timeout=2); os.close(master)
    (HERE/'ros_graph_result.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print('PASS: actual namespaced ROS launch -> CANUSB plugin -> generated PTY -> joint and diagnostic topics; no physical device.')


if __name__ == '__main__':
    main()
