"""HTTP-level J5 read-only/manual calibration through the real panel handler.

Uses the panel's production HTTP/backend path with an isolated PTY fake J5.
No ROS driver, no real serial, no arm/motion command.
"""
import http.client
import json
import os
import pty
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(HERE))
from test_j5_readonly import FakeJ5, wait_ready  # noqa: E402
from j5_controller import J5Controller  # noqa: E402
from control_panel import PanelBackend  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402


class FakeNode:
    def create_subscription(self, *args, **kwargs):
        return None

    def create_client(self, *args, **kwargs):
        return None


class Client:
    def __init__(self, port, token):
        self.port, self.token = port, token

    def call(self, method, path, body=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        payload = json.dumps(body).encode() if body is not None else None
        headers = {"Host": "127.0.0.1:%d" % self.port, "X-Panel-Token": self.token,
                   "Origin": "http://127.0.0.1:%d" % self.port,
                   "Content-Type": "application/json"}
        if payload is not None:
            headers["Content-Length"] = str(len(payload))
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            return response.status, json.loads(response.read().decode() or "{}")
        finally:
            connection.close()


def main():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fake = FakeJ5(master)
    directory = Path(tempfile.mkdtemp(prefix="j5-panel-"))
    record = directory / "j5_calibration.json"
    controller = J5Controller(path, record, directory / "readonly.jsonl")
    backend = PanelBackend(FakeNode(), directory, 60, 30, "j5-panel-token", j5=controller)
    server = PanelServer(backend, "j5-panel-token", backend.static_dir, 0)
    server.start()
    client = Client(server.port, "j5-panel-token")
    evidence = {"passed": False, "physical_port_opened": False, "pty": path}
    try:
        deadline = time.monotonic() + 8
        status = {}
        while time.monotonic() < deadline:
            backend.heartbeat()
            code, body = client.call("GET", "/api/j5/status")
            assert code == 200, body
            status = body["j5"]
            if status.get("fresh"):
                break
            time.sleep(.05)
        assert status.get("fresh"), status
        assert status["profile"]["can_id"] == 5
        assert status["hardware_sign"] is None and status["zero_offset_rad"] is None
        assert status["limits"] is None

        def post(path, body):
            backend.heartbeat()
            code, result = client.call("POST", path, body)
            assert code == 202, (code, result)
            return result

        post("/api/j5/calibration/zero",
             {"confirmations": {"confirm_disabled_support": True}})
        fake.position = __import__("math").radians(10)
        time.sleep(.3)
        backend.heartbeat()
        code, zero_status = client.call("GET", "/api/j5/status")
        assert code == 200
        post("/api/j5/calibration/direction",
             {"confirmations": {"confirm_manual_rotation": True}})
        fake.position = __import__("math").radians(-35)
        time.sleep(.3)
        post("/api/j5/calibration/limit",
             {"which": "min", "confirmations": {"confirm_cable_limit": True}})
        fake.position = __import__("math").radians(45)
        time.sleep(.3)
        post("/api/j5/calibration/limit",
             {"which": "max", "confirmations": {"confirm_cable_limit": True}})
        saved = post("/api/j5/calibration/save",
                     {"confirmations": {"confirm_save": True,
                                        "notes": "panel PTY manual calibration"}})
        assert saved["saved"]["saved"] is True
        payload = json.loads(record.read_text(encoding="utf-8"))
        assert payload["validated"] is True and payload["sign"] == 1
        audit = json.loads(record.with_suffix(record.suffix + ".audit.json").read_text(
            encoding="utf-8"))
        assert [event["step"] for event in audit[:5]] == [
            "zero", "direction", "limit_min", "limit_max", "save"]
        # Re-zero must clear direction and both limits in the UI/backend status;
        # the old saved record cannot remain presented as the current state.
        before = client.call("GET", "/api/j5/status")[1]["j5"]["calibration"]
        assert before["saved"] is True
        post("/api/j5/calibration/zero",
             {"confirmations": {"confirm_disabled_support": True}})
        after = client.call("GET", "/api/j5/status")[1]["j5"]["calibration"]
        assert after["zero"] is True
        assert after["direction"] is None and after["limits"] is None
        assert after["saved"] is False
        evidence["rezero_cleared_direction_and_limits"] = True
        evidence.update({"passed": True, "record": payload,
                         "audit_steps": [event["step"] for event in audit],
                         "http": "GET /api/j5/status; POST zero/direction/limit/save",
                         "commands_sent": "read requests only"})
        (HERE / "evidence" / "j5_panel_http.json").write_text(
            json.dumps(evidence, indent=2), encoding="utf-8")
        print("PASS", json.dumps(evidence, ensure_ascii=False))
    finally:
        server.stop()
        controller.close()
        backend.shutdown()
        fake.stop()
        try:
            os.close(master)
            os.close(slave)
        except OSError:
            pass


if __name__ == "__main__":
    main()
