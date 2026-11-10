"""Panel HTTP surface of a J5 transport fault and explicit read-only reconnect."""
import json
import math
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
from test_j5_readonly import FakeJ5  # noqa: E402
from test_j5_panel import Client, FakeNode  # noqa: E402
from test_j5_transport import FaultFactory  # noqa: E402
from j5_controller import J5Controller  # noqa: E402
from control_panel import PanelBackend  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402


def wait_for(client, backend, predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        backend.heartbeat()
        code, body = client.call("GET", "/api/j5/status")
        assert code == 200, body
        j5 = body["j5"]
        if predicate(j5):
            return j5
        time.sleep(.04)
    raise AssertionError("J5 panel fault status timeout: %r" % j5)


def main():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    fake = FakeJ5(master)
    factory = FaultFactory()
    directory = Path(tempfile.mkdtemp(prefix="j5-panel-fault-"))
    controller = J5Controller(path, directory / "cal.json", directory / "readonly.jsonl",
                              serial_factory=factory, frame_writer=factory.writer)
    backend = PanelBackend(FakeNode(), directory, 60, 30, "j5-fault-token", j5=controller)
    server = PanelServer(backend, "j5-fault-token", backend.static_dir, 0)
    server.start()
    client = Client(server.port, "j5-fault-token")
    evidence = {"passed": False, "physical_port_opened": False}
    try:
        online = wait_for(client, backend,
                          lambda j5: j5.get("connected") and j5.get("fresh"))
        backend.heartbeat()
        code, _ = client.call("POST", "/api/j5/calibration/zero",
                              {"confirmations": {"confirm_disabled_support": True}})
        assert code == 202, code
        serial = factory.instances[-1]
        serial.mode = "normal"
        factory.writer.mode = "eagain"
        factory.writer.fault_after = 2
        factory.writer.writes = 0
        faulted = wait_for(client, backend, lambda j5: j5.get("error") is not None)
        assert faulted["connected"] is False and faulted["fresh"] is False
        assert faulted["feedback"] is None and faulted["age_s"] is None
        assert faulted["transport"]["state"] == "fault"
        assert faulted["reconnect_available"] is True
        assert faulted["calibration"]["zero"] is False
        assert faulted["calibration"]["invalidated_reason"]
        evidence["fault"] = {"error": faulted["error"],
                             "connected": faulted["connected"],
                             "fresh": faulted["fresh"],
                             "feedback_is_none": faulted["feedback"] is None,
                             "calibration_zero": faulted["calibration"]["zero"]}
        factory.mode = "normal"
        factory.writer.mode = "normal"
        # A failed reopen must be a real HTTP failure, not an ok=true wrapper.
        factory.fail_next = True
        backend.heartbeat()
        code, failed = client.call("POST", "/api/j5/reconnect", {})
        assert code == 409, (code, failed)
        assert failed.get("ok") is not True, failed
        evidence["failed_reconnect_http"] = {"status": code, "ok": failed.get("ok"),
                                             "error": failed.get("message") or failed.get("error")}
        backend.heartbeat()
        code, reconnect = client.call("POST", "/api/j5/reconnect", {})
        assert code == 202, (code, reconnect)
        assert reconnect["reconnected"] is True, reconnect
        online_again = wait_for(client, backend,
                                lambda j5: j5.get("connected") and j5.get("fresh"))
        assert online_again["calibration"]["zero"] is False
        assert online_again["error"] is None
        evidence.update({"passed": True, "reconnected": True,
                         "rezero_required": True,
                         "http": ["GET /api/j5/status", "POST /api/j5/calibration/zero",
                                  "POST /api/j5/reconnect"]})
        (HERE / "evidence" / "j5_panel_transport_fault.json").write_text(
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
