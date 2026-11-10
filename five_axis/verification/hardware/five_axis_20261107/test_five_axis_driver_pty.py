"""Five-axis ROS driver on a generated PTY. No physical port, no motion command."""
import json
import math
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "verification/hardware"))
from four_axis_monitor import load_mapping  # noqa: E402


def usb_packet(axis, rid=None):
    frame = bytearray([0x55, 0xAA, 0x1E, 3, 1, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                       0xFF, 7, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    frame[21:29] = bytes([axis, 0, 0xCC if rid is None else 0x33,
                          0 if rid is None else rid, 0, 0, 0, 0])
    return bytes(frame)

AXES = (1, 2, 3, 4, 5)
PROFILES = {1: (12.5, 25.0, 200.0), 2: (12.5, 25.0, 200.0),
            3: (12.5, 45.0, 54.0), 4: (12.5, 45.0, 54.0),
            5: (12.5, 10.0, 28.0)}
INITIAL_DEG = {1: 15.0, 2: 25.0, 3: -10.0, 4: 5.0, 5: 0.0}


class Plant:
    def __init__(self):
        mapping = load_mapping()
        self.mapping = {i: mapping[i] for i in range(1, 5)}
        self.mapping[5] = {"hardware_sign": 1.0, "zero_offset_rad": 0.0}
        self.raw = {i: self.mapping[i]["zero_offset_rad"] +
                    self.mapping[i]["hardware_sign"] * math.radians(INITIAL_DEG[i])
                    for i in AXES}
        self.enabled = {i: False for i in AXES}
        self.fc = {i: 0 for i in AXES}
        self.fd = {i: 0 for i in AXES}
        self.mit = {i: 0 for i in AXES}
        self.reads = 0
        self.errors = []
        self.master, slave = pty.openpty()
        self.path = os.ttyname(slave)
        os.close(slave)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def reply(self, axis):
        code = round((self.raw[axis] + 12.5) * 65535.0 / 25.0)
        code = max(0, min(65535, code))
        state = 1 if self.enabled[axis] else 0
        return bytes([(state << 4) | axis, code >> 8, code & 255, 0x7F, 0xF7,
                      0xFF, 30, 25])

    def serve(self):
        buf = bytearray()
        while not self.stop.is_set():
            try:
                if not select.select([self.master], [], [], .001)[0]:
                    continue
                buf.extend(os.read(self.master, 4096))
            except OSError:
                self.stop.wait(.002)
                continue
            while len(buf) >= 30:
                packet = bytes(buf[:30])
                del buf[:30]
                cid = int.from_bytes(packet[13:17], "little")
                b = packet[21:29]
                try:
                    assert packet[:13] == usb_packet(1)[:13]
                    assert packet[17:21] == usb_packet(1)[17:21]
                    if cid == 0x7FF:
                        axis = b[0]
                        rid = None if b[2] == 0xCC else b[3]
                        assert axis in AXES, axis
                        assert packet == usb_packet(axis, rid), (axis, rid)
                        self.reads += 1
                        if rid is None:
                            payload = self.reply(axis)
                        else:
                            if rid in (7, 8, 9, 10):
                                value = 0 if rid == 7 else axis if rid == 8 else (
                                    0 if axis == 5 else 2000) if rid == 9 else 1
                                packed = struct.pack("<I", value)
                            else:
                                value = PROFILES[axis][{21: 0, 22: 1, 23: 2}[rid]]
                                packed = struct.pack("<f", value)
                            payload = bytes([axis, 0, 0x33, rid]) + packed
                    else:
                        axis = cid
                        assert axis in AXES, axis
                        if b[:7] == b"\xff" * 7:
                            assert b[7] in (0xFC, 0xFD)
                            self.enabled[axis] = b[7] == 0xFC
                            if self.enabled[axis]:
                                self.fc[axis] += 1
                            else:
                                self.fd[axis] += 1
                        else:
                            self.mit[axis] += 1
                        payload = self.reply(axis)
                    os.write(self.master, b"\xaa\x11\x08" + b"\0" * 4 + payload + b"\x55")
                except Exception as error:
                    self.errors.append(str(error))
                    self.stop.set()
                    return

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        try:
            os.close(self.master)
        except OSError:
            pass


def wait_for(node, predicate, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if predicate():
            return
    raise AssertionError("timeout: %s" % what)


def main():
    # Isolated from the field/domain0 and from old 175 test sessions, whose
    # transient-local robot_description publishers can otherwise feed a stale
    # URDF/audit path into a new controller_manager.
    os.environ.update(ROS_DOMAIN_ID="180", ROS_LOCALHOST_ONLY="1")
    plant = Plant()
    node = None
    process = None
    result = {}
    seen = {"dynamic": None, "identity": None, "ff": None}
    local_logs = Path("/tmp/slaihand-five-axis-fake")
    local_logs.mkdir(parents=True, exist_ok=True)
    try:
        with (HERE / "five_axis_driver.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "slaihand_bringup",
                 "slaihand_canusb_five_axis_motion.launch.py",
                 "port:=" + plant.path, "log_directory:=" + str(local_logs)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node("five_axis_fake_verifier")
            node.create_subscription(DynamicJointState, "/slaihand/dynamic_joint_states",
                                     lambda msg: seen.update(dynamic=msg), 10)
            node.create_subscription(String, "/slaihand/runtime_identity",
                                     lambda msg: seen.update(identity=msg.data), 10)
            node.create_subscription(Float64MultiArray, "/slaihand/gravity_ff_state",
                                     lambda msg: seen.update(ff=msg),
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

            def status():
                message = seen["dynamic"]
                return {} if message is None else {
                    name: dict(zip(values.interface_names, values.values))
                    for name, values in zip(message.joint_names, message.interface_values)}

            def all_field(key, value):
                rows = status()
                return len(rows) == 5 and all(
                    rows.get("slaihand_joint%d" % i, {}).get(key) == value
                    for i in AXES)

            def five_live():
                rows = status()
                return len(rows) == 5 and all(
                    rows.get("slaihand_joint%d" % i, {}).get("control_state") == 0
                    and rows.get("slaihand_joint%d" % i, {}).get("feedback_valid") == 1
                    and math.isfinite(rows.get("slaihand_joint%d" % i, {}).get("position"))
                    for i in AXES)
            wait_for(node, five_live, 10, "five-axis feedback")
            wait_for(node, lambda: plant.reads >= 50, 8, "register queries")
            rows = status()
            assert sorted(rows) == ["slaihand_joint%d" % i for i in AXES]
            assert all(rows["slaihand_joint%d" % i].get("timeout_count") == (0 if i == 5 else 2000)
                       for i in AXES)
            assert all(math.isfinite(rows["slaihand_joint%d" % i].get("position"))
                       for i in AXES)
            assert plant.mit == {i: 0 for i in AXES}
            assert plant.fc == {i: 0 for i in AXES}
            wait_for(node, lambda: seen["identity"] is not None, 5, "runtime identity")
            identity = json.loads(seen["identity"])
            assert identity["schema"] == "slaihand_runtime_identity/1"
            assert identity["active_axes"] == 5, identity
            assert identity["j5_motion_qualified"] is False, identity
            assert len(identity["mapping_sha256"]) == 64
            wait_for(node, lambda: seen["ff"] is not None, 5, "ff state")
            ff = seen["ff"]
            labels = [dim.label for dim in ff.layout.dim]
            assert labels[0] == "slaihand_gravity_ff_state/2" and ff.layout.dim[0].size == 5, labels
            assert labels[1] == "float" and ff.layout.dim[1].size == 0, labels
            # 5 header + 4*5 vectors + 15 hold-extension fields, no float block.
            assert len(ff.data) == 40, len(ff.data)
            arm = node.create_client(Trigger, "/slaihand/arm_supported_current_pose")
            assert arm.wait_for_service(timeout_sec=6)
            future = arm.call_async(Trigger.Request())
            wait_for(node, future.done, 5, "arm response")
            response = future.result()
            assert response is not None and response.success is False, response
            assert "calibration" in response.message.lower() or "J5" in response.message
            assert plant.mit == {i: 0 for i in AXES}
            assert plant.fc == {i: 0 for i in AXES}
            stop = node.create_client(Trigger, "/slaihand/stop_all")
            assert stop.wait_for_service(timeout_sec=6)
            future = stop.call_async(Trigger.Request())
            wait_for(node, future.done, 5, "stop response")
            assert future.result().success
            wait_for(node, lambda: all_field("control_state", 8) and
                     all_field("disabled_observed", 1), 5, "five-axis disabled")
            assert all(plant.fd[i] >= 1 for i in AXES), (plant.fc, plant.fd)
            assert not plant.errors, plant.errors
            result = {"status": "PASS", "physical_port_opened": False,
                      "single_owner": True, "axes": 5,
                      "j5_feedback": rows["slaihand_joint5"]["position"],
                      "j5_timeout_count": rows["slaihand_joint5"]["timeout_count"],
                      "j5_arm_refused": response.message,
                      "five_axis_stop_fd": plant.fd,
                      "fc_frames": plant.fc, "mit_frames": plant.mit,
                      "software_only": True}
            (HERE / "five_axis_driver_result.json").write_text(
                json.dumps(result, indent=2), encoding="utf-8")
            print(json.dumps(result, ensure_ascii=False))
    finally:
        if plant.errors:
            print("PLANT_ERRORS " + repr(plant.errors[:5]), flush=True)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
        plant.close()


if __name__ == "__main__":
    main()
