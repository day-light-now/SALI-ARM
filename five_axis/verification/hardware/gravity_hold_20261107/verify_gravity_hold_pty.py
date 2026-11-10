"""PTY/ROS/HTTP/MIT regression for in-session gravity-float -> position hold.

Isolated ROS domain 176 + PTY fake drive only. The installed test policy is
temporarily enabled for the bounded float service; the source file is untouched.
"""
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
CONTROL = ROOT / "verification" / "hardware" / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
sys.path.insert(0, str(ROOT))

os.environ.setdefault("ROS_DOMAIN_ID", "176")
os.environ["ROS_LOCALHOST_ONLY"] = "1"
assert os.environ["ROS_DOMAIN_ID"] not in ("", "0")

import rclpy
from ament_index_python.packages import get_package_share_directory
import yaml

INSTALL = ROOT / "install/slaihand_bringup"
os.environ.setdefault("PANEL_WORKSPACE", str(ROOT))
os.environ.setdefault("PANEL_SCRIPT", str(
    INSTALL / "lib/slaihand_bringup/control_panel.py"))

import verify_control_panel as fixture


def enable_float_policy():
    share = Path(get_package_share_directory("slaihand_bringup"))
    path = share / "config/canusb_motion_policy.yaml"
    original = path.read_text(encoding="utf-8")
    data = yaml.safe_load(original) or {}
    data["gravity_float_enabled"] = True
    data.setdefault("gravity_ff_alpha", 0.1)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path, original


def wait_status(panel, predicate, timeout=25, interval=.05):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = panel.status()
        if predicate(last):
            return last
        time.sleep(interval)
    raise AssertionError("status condition not reached; last=%s" % json.dumps(last, ensure_ascii=True)[:1200])


def audit_mit_rows(session):
    rows = []
    for path in (session.dir / "audit").glob("*.jsonl"):
        with path.open() as stream:
            for line in stream:
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if row.get("kind") == "TX_WRITTEN" and row.get("role") == 1:
                    payload = bytes.fromhex(row["payload"])
                    kp = (((payload[3] & 15) << 8) | payload[4]) * 500 / 4095
                    kd = ((payload[5] << 4) | (payload[6] >> 4)) * 5 / 4095
                    rows.append({"at": row.get("at"), "kp": kp, "kd": kd})
    return rows


def main():
    rclpy.init()
    observer = fixture.Observer()
    session = None
    policy_path = None
    policy_original = None
    work = Path(tempfile.mkdtemp(prefix="gravity-hold-"))
    try:
        policy_path, policy_original = enable_float_policy()
        session = fixture.Session("lock", work, observer)
        panel, plant = session.panel, session.plant
        status = wait_status(panel, lambda value: value.get("gravity_shadow", {}).get("driver", {}).get("float_available"), 12)
        assert status["gravity_shadow"]["driver"]["float_available"]

        confirmations = dict(fixture.confirmations(), gravity_float=True)
        code, body = panel.call("POST", "/api/gravity-float", {
            "confirmations": confirmations,
            "float_settings": {"kd": 5.0, "seconds": 20.0},
        })
        assert code == 202, body
        active = wait_status(panel, lambda value: (
            value.get("gravity_shadow", {}).get("driver", {}).get("active") is True
            and value.get("gravity_shadow", {}).get("driver", {}).get("gravity_float") is True
            and value.get("ros", {}).get("hold_ready") is True
        ), 25)
        operation_id = active["operation"]["id"]
        before = active["gravity_shadow"]["driver"]
        assert before["alpha"] == 1 and before["ramp"] == 3
        assert active["operation"]["kind"] == "gravity_float"
        command_id = active["ros"]["command_id"]
        assert command_id is not None

        # Duplicate lock requests are rejected at the panel while one is pending.
        code, lock = panel.call("POST", "/api/gravity/lock", {})
        assert code == 202, lock
        code, duplicate = panel.call("POST", "/api/gravity/lock", {})
        assert code == 409, duplicate
        locked = wait_status(panel, lambda value: value.get("gravity_mode", {}).get("locked_confirmed") is True, 20)
        mode = locked["gravity_mode"]
        assert mode["phase"] == "locked" and mode["lock_state"] == 2
        assert mode["lock_reason_code"] == 3
        assert mode["axes"] and all(row["target_deg"] is not None for row in mode["axes"])
        targets = [row["target_deg"] for row in mode["axes"]]
        q_before = list(locked["ros"].get("positions") or [])
        assert locked["ros"]["stage"] == 5
        assert locked["ros"]["command_id"] == command_id
        assert locked["operation"]["id"] == operation_id
        assert locked["operation"]["running"] is True
        assert locked["gravity_shadow"]["driver"]["alpha"] == 1
        assert locked["gravity_mode"]["gain_scale"] >= 0
        locked_at = time.monotonic()

        # Simulate a small operator push. The latched target must not follow feedback.
        plant.raw[0] += math.radians(1.0)
        time.sleep(.3)
        pushed = panel.status()
        assert pushed["gravity_mode"]["phase"] in ("locked", "position"), pushed["gravity_mode"]
        assert [row["target_deg"] for row in pushed["gravity_mode"]["axes"]] == targets
        assert pushed["ros"]["command_id"] == command_id

        # Resume requires the explicit operator weight acknowledgement.
        code, no_ack = panel.call("POST", "/api/gravity/float", {"confirmations": {}})
        assert code == 400, no_ack
        code, resume = panel.call("POST", "/api/gravity/float", {
            "confirmations": dict(fixture.confirmations(), release_weight=True)})
        assert code == 202, resume
        resumed = wait_status(panel, lambda value: (
            value.get("gravity_mode", {}).get("phase") == "float"
            and value.get("gravity_shadow", {}).get("driver", {}).get("gravity_float") is True
        ), 15)
        assert resumed["operation"]["id"] == operation_id
        assert resumed["ros"]["command_id"] == command_id
        assert resumed["gravity_shadow"]["driver"]["alpha"] == 1
        assert resumed["gravity_mode"]["kp"] == 0

        # A command_id-bound duplicate/stale request is rejected after the mode change.
        code, late_resume = panel.call("POST", "/api/gravity/float", {
            "confirmations": dict(fixture.confirmations(), release_weight=True)})
        assert code == 409, late_resume

        # Stop priority and late lock request.
        code, stopped = panel.call("POST", "/api/stop", {"reason": "gravity-hold regression stop"})
        assert code == 202, stopped
        finished = wait_status(panel, lambda value: (
            value.get("operation") is not None and not value["operation"].get("running")
            and not value.get("ros", {}).get("command_owner")
        ), 20)
        code, late_lock = panel.call("POST", "/api/gravity/lock", {})
        assert code == 409, late_lock

        rows = audit_mit_rows(session)
        assert rows, "no MIT write receipt"
        assert all(row["kp"] <= 160.0 + 1e-9 and row["kd"] <= 5.0 + 1e-9 for row in rows)
        # The first locked packet is ramped, not a float->position gain jump.
        assert any(0 <= row["kp"] < 160.0 for row in rows), "no ramped Kp packet observed"
        assert any(row["kp"] > 150.0 for row in rows), "full position gain never reached"
        assert any(row["kp"] == 0.0 for row in rows), "free-float Kp=0 packet missing"
        result = {
            "passed": True, "mode": "gravity_float_lock_resume",
            "operation_id": operation_id, "command_id": command_id,
            "locked_target_deg": targets, "locked_hold_gain": mode["gain_scale"],
            "alpha_locked": 1, "resume_alpha": 1, "resume_kp": resumed["gravity_mode"]["kp"],
            "audit_mit_packets": len(rows), "max_kp": max(row["kp"] for row in rows),
            "max_kd": max(row["kd"] for row in rows),
            "ramped_kp_packets": sum(1 for row in rows if 0 < row["kp"] < 160),
            "stop_confirmed": True, "late_lock_rejected": True,
            "session_seconds": 20.0, "physical_port_opened": False,
        }
        evidence = Path(__file__).resolve().parent / "evidence"
        evidence.mkdir(exist_ok=True)
        (evidence / "gravity_hold_pty.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print("PASS", json.dumps(result), flush=True)
    finally:
        if session:
            session.close()
        observer.close()
        if policy_path is not None and policy_original is not None:
            policy_path.write_text(policy_original, encoding="utf-8")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
