"""Live five-axis browser fixture: real C++ driver over fake PTY + real panel."""
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from test_five_axis_driver_pty import Plant  # noqa: E402

URL = HERE / "five_axis_live_url.txt"


def main():
    os.environ.update(ROS_DOMAIN_ID="175", ROS_LOCALHOST_ONLY="1")
    plant = Plant()
    work = Path(tempfile.mkdtemp(prefix="five-axis-live-"))
    driver = None
    panel = None
    try:
        with (work / "driver.log").open("w", encoding="utf-8") as log:
            driver = subprocess.Popen(
                ["ros2", "launch", "slaihand_bringup",
                 "slaihand_canusb_five_axis_motion.launch.py",
                 "port:=" + plant.path, "log_directory:=" + str(work)],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        # Wait for the driver feedback topic before starting the panel.
        import rclpy
        from control_msgs.msg import DynamicJointState
        rclpy.init()
        node = rclpy.create_node("five_axis_live_wait")
        seen = {"count": 0}
        node.create_subscription(DynamicJointState, "/slaihand/dynamic_joint_states",
                                 lambda msg: seen.update(count=len(msg.joint_names)), 10)
        end = time.monotonic() + 15
        while time.monotonic() < end and seen["count"] != 5:
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()
        if seen["count"] != 5:
            raise RuntimeError("five-axis driver did not publish 5 axes")

        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = "175"
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["PYTHONPATH"] = str(ROOT / "src/slaihand_bringup/scripts") + ":" + env.get("PYTHONPATH", "")
        with (work / "panel.log").open("w", encoding="utf-8") as log:
            panel = subprocess.Popen(
                [sys.executable, str(ROOT / "src/slaihand_bringup/scripts/control_panel.py"),
                 "--port", "18898", "--joint-count", "5",
                 "--archive-dir", str(work / "archive")],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        end = time.monotonic() + 25
        url = None
        while time.monotonic() < end:
            text = (work / "panel.log").read_text(encoding="utf-8", errors="replace")
            match = re.search(r"http://127\.0\.0\.1:\d+/\?token=[A-Za-z0-9_\-]+", text)
            if match:
                url = match.group(0).strip()
                break
            if panel.poll() is not None:
                raise RuntimeError("panel exited: " + text[-1500:])
            time.sleep(.2)
        if not url:
            raise RuntimeError("no panel URL: " + (work / "panel.log").read_text(errors="replace")[-1500:])
        URL.write_text(url + "\n")
        print("READY", url, flush=True)
        while True:
            time.sleep(1)
    finally:
        for process in (panel, driver):
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
        plant.close()


if __name__ == "__main__":
    main()
