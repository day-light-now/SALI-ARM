"""Five-axis protected client -> standard JTC positive path on a generated PTY.

No physical port, no real calibration.  The fake drive exposes a *simulated*
fully-qualified J5 so the client/JTC five-axis path can be exercised end to end.
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
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(ROOT / "verification/hardware"))
from four_axis_monitor import load_mapping  # noqa: E402

NAMES = ["slaihand_joint%d" % i for i in range(1, 6)]
INITIAL_DEG = [15.0, 25.0, -10.0, 5.0, 0.0]
J5_LIMITS_DEG = (-30.0, 30.0)
PROFILES = {1: (12.5, 25.0, 200.0), 2: (12.5, 25.0, 200.0),
            3: (12.5, 45.0, 54.0), 4: (12.5, 45.0, 54.0),
            5: (12.5, 10.0, 28.0)}


def usb_packet(axis, rid=None):
    frame = bytearray([0x55, 0xAA, 0x1E, 3, 1, 0, 0, 0, 0x0A, 0, 0, 0, 0,
                       0xFF, 7, 0, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    frame[21:29] = bytes([axis, 0, 0xCC if rid is None else 0x33,
                          0 if rid is None else rid, 0, 0, 0, 0])
    return bytes(frame)


class Plant:
    def __init__(self):
        mapping = load_mapping()
        self.mapping = {i: mapping[i] for i in range(1, 5)}
        self.mapping[5] = {"hardware_sign": 1.0, "zero_offset_rad": 0.0}
        self.raw = {i: self.mapping[i]["zero_offset_rad"]
                    + self.mapping[i]["hardware_sign"] * math.radians(INITIAL_DEG[i - 1])
                    for i in range(1, 6)}
        self.enabled = {i: False for i in range(1, 6)}
        self.fc = {i: 0 for i in range(1, 6)}
        self.fd = {i: 0 for i in range(1, 6)}
        self.mit = {i: 0 for i in range(1, 6)}
        self.velocity = {i: 0.0 for i in range(1, 6)}
        self.first_targets = {}
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
        vmax = PROFILES[axis][1]
        velocity = self.velocity[axis] if self.enabled[axis] else 0.0
        encoded_v = round((velocity + vmax) * 4095.0 / (2.0 * vmax))
        encoded_v = max(0, min(4095, encoded_v))
        return bytes([(state << 4) | axis, code >> 8, code & 255, encoded_v >> 4,
                      ((encoded_v & 15) << 4) | 7, 0xFF, 30, 25])

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
                        assert axis in range(1, 6), axis
                        assert packet == usb_packet(axis, rid), (axis, rid)
                        self.reads += 1
                        if rid is None:
                            payload = self.reply(axis)
                        else:
                            if rid in (7, 8, 9, 10):
                                value = 0 if rid == 7 else axis if rid == 8 else 2000 if rid == 9 else 1
                                packed = struct.pack("<I", value)
                            else:
                                value = PROFILES[axis][{21: 0, 22: 1, 23: 2}[rid]]
                                packed = struct.pack("<f", value)
                            payload = bytes([axis, 0, 0x33, rid]) + packed
                    else:
                        axis = cid
                        assert axis in range(1, 6), axis
                        if b[:7] == b"\xff" * 7:
                            assert b[7] in (0xFC, 0xFD)
                            self.enabled[axis] = b[7] == 0xFC
                            if self.enabled[axis]:
                                self.fc[axis] += 1
                            else:
                                self.fd[axis] += 1
                                self.velocity[axis] = 0.0
                        else:
                            self.mit[axis] += 1
                            pmax, vmax, tmax = PROFILES[axis]
                            q = int.from_bytes(b[:2], "big") * 2.0 * pmax / 65535.0 - pmax
                            kp = (((b[3] & 15) << 8) | b[4]) * 500.0 / 4095.0
                            kd = ((b[5] << 4) | (b[6] >> 4)) * 5.0 / 4095.0
                            dq = ((b[2] << 4) | (b[3] >> 4)) * 2.0 * vmax / 4095.0 - vmax
                            tau = (((b[6] & 15) << 8) | b[7]) * 2.0 * tmax / 4095.0 - tmax
                            assert kp <= 160.0001 and kd <= 5.0001 and abs(dq) <= .05, (kp, kd, dq)
                            self.first_targets.setdefault(
                                axis, self.mapping[axis]["hardware_sign"]
                                * (q - self.mapping[axis]["zero_offset_rad"]))
                            if self.enabled[axis] and kp > 0:
                                step = 0.5 * 0.01  # 0.5 rad/s max follow rate
                                change = max(-step, min(step, q - self.raw[axis]))
                                self.velocity[axis] = change / 0.01
                                self.raw[axis] += change
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
        import rclpy
        rclpy.spin_once(node, timeout_sec=.01)
        if predicate():
            return
    raise AssertionError("timeout: " + what)


def main():
    os.environ.update(ROS_DOMAIN_ID="181", ROS_LOCALHOST_ONLY="1")
    import rclpy
    from control_msgs.msg import DynamicJointState
    from std_msgs.msg import String

    plant = Plant()
    node = None
    launch = None
    work = Path("/tmp/five-axis-jtc-%d" % os.getpid())
    work.mkdir(parents=True, exist_ok=True)
    try:
        driver_log = (work / "driver.log").open("w", encoding="utf-8")
        launch = subprocess.Popen(
            ["ros2", "launch", "slaihand_bringup",
             "slaihand_canusb_five_axis_motion.launch.py",
             "port:=" + plant.path, "log_directory:=" + str(work),
             "j5_qualified:=true",
             "j5_calibration_sha256:=" + "a" * 64,
             "j5_limit_lower:=%.17g" % math.radians(J5_LIMITS_DEG[0]),
             "j5_limit_upper:=%.17g" % math.radians(J5_LIMITS_DEG[1]),
             "mass_profile_sha256:=" + "b" * 64],
            stdout=driver_log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node("five_axis_client_jtc_verifier")
        seen = {"dynamic": None, "identity": None}

        def dynamic(msg):
            seen["dynamic"] = msg

        def identity(msg):
            seen["identity"] = msg.data

        node.create_subscription(DynamicJointState, "/slaihand/dynamic_joint_states", dynamic, 10)
        node.create_subscription(String, "/slaihand/runtime_identity", identity, 10)

        def status():
            msg = seen["dynamic"]
            return {} if msg is None else {
                name: dict(zip(values.interface_names, values.values))
                for name, values in zip(msg.joint_names, msg.interface_values)}

        def fields(key, value):
            rows = status()
            return len(rows) == 5 and all(rows.get(n, {}).get(key) == value for n in NAMES)

        wait_for(node, lambda: len(status()) == 5 and fields("control_state", 0)
                 and fields("feedback_valid", 1), 15, "five-axis feedback")
        wait_for(node, lambda: seen["identity"] is not None, 5, "identity")
        ident = json.loads(seen["identity"])
        assert ident.get("active_axes") == 5 and ident.get("j5_motion_qualified") is True, ident
        assert plant.fc == {i: 0 for i in range(1, 6)} and plant.mit == {i: 0 for i in range(1, 6)}

        from ament_index_python.packages import get_package_prefix
        client_script = (Path(get_package_prefix("slaihand_bringup")) / "lib"
                         / "slaihand_bringup" / "commission_supported_trial.py")
        from std_srvs.srv import Trigger
        reset = node.create_client(Trigger, "/slaihand/reset_disabled")

        def reset_driver():
            if fields("control_state", 0):
                return
            assert reset.wait_for_service(timeout_sec=5), "reset service missing"
            future = reset.call_async(Trigger.Request())
            wait_for(node, future.done, 5, "reset response")
            wait_for(node, lambda: fields("control_state", 0), 5, "reset DISARMED")

        def run_client(extra, tag, timeout=45):
            output = work / tag
            output.mkdir()
            log = (work / ("client_%s.log" % tag)).open("w", encoding="utf-8")
            proc = subprocess.Popen(
                [sys.executable, str(client_script),
                 "--execute", "--ack-supported", "--ack-clearance", "--ack-native-output",
                 "--joint-count", "5", "--j5-limits-deg",
                 str(J5_LIMITS_DEG[0]), str(J5_LIMITS_DEG[1]),
                 "--output-directory", str(output)] + extra,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                wait_for(node, lambda: proc.poll() is not None, timeout, tag + " client")
            finally:
                if proc.poll() is None:
                    proc.terminate()
                    proc.wait(timeout=5)
            results = list(output.glob("*.json"))
            assert proc.returncode == 0, (tag, proc.returncode, results)
            assert len(results) == 1, (tag, results)
            return json.loads(results[0].read_text(encoding="utf-8"))

        # 1) Five-axis hold with no plan: real client -> driver -> stop.
        hold = run_client(["--hold-seconds", "1"], "hold")
        assert hold.get("joint_count") == 5, hold
        assert hold.get("arm_requested") is True and hold.get("phase") != "pre_arm", hold
        assert hold.get("stop_observations_confirmed") is True, hold
        assert hold.get("reason") == "finite_trial_complete", hold
        assert plant.fc[5] >= 1 and plant.fd[5] >= 1, (plant.fc, plant.fd)
        reset_driver()

        plan = {"schema": "slaihand_joint_plan/2", "coordinates": "absolute_deg",
                "waypoints": [{"degrees": [INITIAL_DEG[i] + (1.0 if i < 4 else 2.0)
                                            for i in range(5)],
                               "move_s": 4.0, "dwell_s": 1.0}]}
        plan_path = work / "plan.json"
        plan_path.write_text(json.dumps(plan), encoding="utf-8")
        executed = run_client(["--plan", str(plan_path), "--hold-seconds", "1"], "execute")
        assert executed.get("joint_count") == 5, executed
        assert executed.get("trajectory_started") is True, executed
        assert executed.get("trajectory_requested") is True, executed
        assert executed.get("stop_observations_confirmed") is True, executed
        assert executed.get("reason") == "finite_trial_complete", executed
        final = executed.get("final_delta_from_start_deg") or []
        assert len(final) == 5 and final[4] > .5, final
        reset_driver()

        # 2) Five-axis teaching playback: versioned /2 schema, identity-bound.
        import drag_teaching as teach
        sys.path.insert(0, str(HERE))
        import test_five_axis_client_teaching as fixture
        teach.configure_axes(5)
        identity = json.loads(seen["identity"])
        capture = fixture.make_capture(axes=5)
        capture["source"]["identity"] = dict(identity)
        capture["source"]["identity_sha256"] = teach.identity_fingerprint(identity)
        capture["source"]["boot_id"] = identity["boot_id"]
        capture["source"]["limits_deg"] = [[-80, 200], [-10, 190], [-90, 90],
                                           [-50, 90], list(J5_LIMITS_DEG)]
        capture["content_sha256"] = teach.content_sha256(capture)
        capture = teach.validate_capture(capture)
        teaching = fixture.make_teaching(axes=5)
        teaching.update({"source": capture["source"], "quality": capture["quality"],
                         "samples": capture["samples"], "complete": True, "reasons": [],
                         "content_sha256": capture["content_sha256"]})
        teaching = teach.validate_teaching(teaching)
        bounds = [tuple(map(math.radians, row)) for row in
                  [[-80, 200], [-10, 190], [-90, 90], [-50, 90], list(J5_LIMITS_DEG)]]
        start = [math.radians(value) for value in INITIAL_DEG]
        compiled = teach.compile_playback(teaching, start, bounds, 0.5, identity=identity)
        playback_path = work / "playback.json"
        playback_path.write_text(json.dumps(compiled["frozen"]), encoding="utf-8")
        playback = run_client(["--teaching-plan", str(playback_path), "--hold-seconds", "1"],
                              "teaching")
        assert playback.get("joint_count") == 5, playback
        assert playback.get("mode") == "drag_teaching", playback
        assert playback.get("trajectory_requested") is True, playback
        assert playback.get("stop_observations_confirmed") is True, playback
        assert playback.get("reason") == "finite_trial_complete", playback
        assert not plant.errors, plant.errors
        passed = {"status": "PASS", "physical_port_opened": False, "software_only": True,
                  "joint_count": 5, "client_axes": 5, "j5_qualified_simulation": True,
                  "j5_hold": hold["reason"], "j5_execute": executed["reason"],
                  "j5_teaching_schema": teach.SCHEMA_PLAYBACK,
                  "j5_teaching_playback": playback["reason"],
                  "j5_fc": plant.fc[5], "j5_fd": plant.fd[5],
                  "j5_mit_frames": plant.mit[5],
                  "final_delta_deg": final,
                  "teaching_tracking": playback.get("playback_tracking")}
        (HERE / "five_axis_client_jtc_result.json").write_text(
            json.dumps(passed, indent=2), encoding="utf-8")
        print(json.dumps(passed, ensure_ascii=False))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=3)
        plant.close()


if __name__ == "__main__":
    main()
