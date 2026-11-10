"""Standalone J5 transport-fault browser fixture (in-process panel, PTY + fake serial).

No ROS driver, no real serial. Control HTTP 18892 only toggles the same
FaultSerial behaviour used by test_j5_transport.
"""
import json
import os
import pty
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "src/slaihand_bringup/scripts"))
sys.path.insert(0, str(HERE))
from test_j5_readonly import FakeJ5  # noqa: E402
from test_j5_panel import FakeNode  # noqa: E402
from test_j5_transport import FaultFactory  # noqa: E402
from j5_controller import J5Controller  # noqa: E402
from control_panel import PanelBackend  # noqa: E402
from control_panel_http import PanelServer  # noqa: E402

WORK = Path(tempfile.mkdtemp(prefix="j5-fault-browser-"))
URL = HERE / "j5_fault_url.txt"
TOKEN = "<redacted-panel-token>"
controller = None
factory = None


class Control(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def do_POST(self):
        global controller, factory
        if self.path == "/fail-next":
            factory.fail_next = True
            body = {"ok": True, "fail_next": True}
            self.send_response(200)
        elif self.path == "/fault":
            factory.writer.mode = "eagain"
            factory.writer.fault_after = 2
            factory.writer.writes = 0
            body = {"ok": True, "mode": "eagain"}
            self.send_response(200)
        elif self.path == "/normal":
            factory.writer.mode = "normal"
            body = {"ok": True, "mode": "normal"}
            self.send_response(200)
        else:
            self.send_response(404)
            self.end_headers()
            return
        data = json.dumps(body).encode()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    global controller, factory
    master, slave = pty.openpty()
    fake = FakeJ5(master)
    factory = FaultFactory()
    controller = J5Controller(os.ttyname(slave), WORK / "cal.json",
                              WORK / "readonly.jsonl", serial_factory=factory,
                              frame_writer=factory.writer)
    controller._serial_factory_instances = factory.instances
    backend = PanelBackend(FakeNode(), WORK, 60, 30, TOKEN, j5=controller)
    server = PanelServer(backend, TOKEN, backend.static_dir, 18891)
    server.start()
    control = ThreadingHTTPServer(("127.0.0.1", 18892), Control)
    threading.Thread(target=control.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:18891/?token=%s" % TOKEN
    URL.write_text(url + "\n")
    (HERE / "evidence").mkdir(exist_ok=True)
    print("READY", url, flush=True)
    try:
        while True:
            time.sleep(1)
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
