"""Standalone isolated fixture: fake J5 PTY + real panel HTTP server on 18889.

No ROS driver, no real serial. A text file changes the fake J5 raw position so
the Windows browser test can walk the manual calibration sequence.
"""
import json
import math
import os
import pty
import re
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CONTROL = ROOT / "verification/hardware/control_panel_20261104"
WORK = Path(tempfile.mkdtemp(prefix="j5-browser-"))
POSITION = WORK / "position_deg"
URL = WORK / "url.txt"
STATE = WORK / "fixture.json"
PORT = 18889


def pack_register(rid, value):
    import struct
    data = bytes([5, 0, 0x33, rid]) + (struct.pack("<f", value)
                                      if rid in (21, 22, 23) else struct.pack("<I", int(value)))
    return frame(0, data)


def frame(can_id, data):
    return (bytes([0xAA, 0x11, 0x08]) + int(can_id).to_bytes(4, "little") +
            bytes(data) + b"\x55")


def feedback(position):
    import struct
    pmax, vmax, tmax = 12.5, 25.0, 5.0
    p = int(round((position + pmax) * 65535 / (2 * pmax)))
    v = 2048
    t = 2048
    return frame(0x10, bytes([5, p >> 8, p & 255, v >> 4,
                             ((v & 15) << 4) | (t >> 8), t & 255, 30, 28]))


def fake_loop(fd):
    while True:
        try:
            chunk = os.read(fd, 128)
        except OSError:
            return
        if not chunk:
            return
        degrees = 0.0
        try:
            degrees = float(POSITION.read_text())
        except Exception:
            pass
        while len(chunk) >= 30:
            packet, chunk = chunk[:30], chunk[30:]
            request = packet[21:29]
            if request[0] != 5:
                continue
            if request[2] == 0x33:
                values = {7: 0x10, 8: 5, 9: 2000, 10: 1, 21: 12.5, 22: 25.0, 23: 5.0}
                if request[3] in values:
                    os.write(fd, pack_register(request[3], values[request[3]]))
            elif request[2] == 0xCC:
                os.write(fd, feedback(math.radians(degrees)))


class PositionHandler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass

    def do_POST(self):
        if self.path != "/position":
            self.send_response(404)
            self.end_headers()
            return
        size = int(self.headers.get("Content-Length") or 0)
        try:
            value = float(json.loads(self.rfile.read(size) or b"{}").get("degrees"))
            if not math.isfinite(value) or abs(value) > 180:
                raise ValueError("degrees outside test range")
            POSITION.write_text("%.9f\n" % value)
            body = json.dumps({"ok": True, "degrees": value}).encode()
            self.send_response(200)
        except Exception as error:
            body = json.dumps({"ok": False, "error": str(error)}).encode()
            self.send_response(400)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    POSITION.write_text("0\n")
    control = ThreadingHTTPServer(("127.0.0.1", 18890), PositionHandler)
    threading.Thread(target=control.serve_forever, daemon=True).start()
    master, slave = pty.openpty()
    pty_path = os.ttyname(slave)
    threading.Thread(target=fake_loop, args=(master,), daemon=True).start()
    record = WORK / "j5_calibration.json"
    panel_log = (WORK / "panel.log").open("w")
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT / "src/slaihand_bringup/scripts") + ":" + env.get("PYTHONPATH", "")
    panel = subprocess.Popen(
        [sys.executable, str(ROOT / "src/slaihand_bringup/scripts/control_panel.py"),
         "--port", str(PORT), "--archive-dir", str(WORK / "archive"),
         "--j5-port", pty_path, "--j5-record", str(record)],
        env=env, stdout=panel_log, stderr=subprocess.STDOUT)
    STATE.write_text(json.dumps({"panel": panel.pid, "master": master, "slave": slave,
                                 "pty": pty_path, "position": str(POSITION)}))
    url = None
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        text = (WORK / "panel.log").read_text(errors="replace")
        for line in text.splitlines():
            match = re.search(r"http://127\.0\.0\.1:\d+/\?token=[A-Za-z0-9_\-]+", line)
            if match:
                url = match.group(0)
                break
        if url:
            break
        if panel.poll() is not None:
            raise SystemExit("panel died:\n" + text[-2000:])
        time.sleep(.2)
    if not url:
        raise SystemExit("no panel URL:\n" + (WORK / "panel.log").read_text(errors="replace")[-2000:])
    URL.write_text(url + "\n")
    (HERE / "j5_browser_url.txt").write_text(url + "\n")
    print("READY", url, flush=True)
    try:
        panel.wait()
    finally:
        try:
            panel.terminate()
            panel.wait(timeout=5)
        except Exception:
            pass


if __name__ == "__main__":
    main()
