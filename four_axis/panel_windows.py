"""Windows loopback frontend, forwarding over WSL stdin rather than network."""
import json
import os
import socket
import subprocess
import threading
import time
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class ExclusiveServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main():
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if not 0 <= domain <= 232:
        raise ValueError("Invalid ROS_DOMAIN_ID")
    command = ("source /opt/ros/jazzy/setup.bash; "
               "source /home/xuyu/slaihand_ros2_ws/install/setup.bash; "
               f"export ROS_DOMAIN_ID={domain}; "
               "exec ros2 run slaihand_bringup panel_bridge.py")
    bridge = None
    lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def forward(self):
            try:
                if self.command == "GET" and self.path == "/health":
                    reply = {"status": 200, "mime": "application/json", "body": json.dumps({"app": "slaihand-offline-panel"})}
                    self.send_response(200)
                    self.send_header("Content-Type", reply["mime"])
                    self.end_headers()
                    self.wfile.write(reply["body"].encode())
                    return
                if self.path not in ["/", "/status", "/command"]:
                    raise ValueError("Unsupported route")
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 <= size <= 2048:
                    raise ValueError("Invalid request size")
                if self.command == "POST" and self.headers.get("Content-Type") != "application/json":
                    raise ValueError("JSON required")
                item = {"method": self.command, "path": self.path,
                        "token": self.headers.get("X-Panel-Token", ""),
                        "body": self.rfile.read(size).decode("utf-8") if self.command == "POST" else None}
                with lock:
                    if bridge.poll() is not None:
                        raise RuntimeError("WSL bridge stopped")
                    bridge.stdin.write(json.dumps(item) + "\n")
                    bridge.stdin.flush()
                    reply = json.loads(bridge.stdout.readline())
            except Exception as error:
                reply = {"status": 503, "mime": "application/json", "body": json.dumps({"message": str(error)})}
            payload = reply["body"].encode("utf-8")
            self.send_response(reply["status"])
            self.send_header("Content-Type", reply["mime"] or "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        do_GET = forward
        do_POST = forward

    try:
        server = ExclusiveServer(("127.0.0.1", 8766), Handler)
    except OSError:
        try:
            opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open("http://127.0.0.1:8766/health", timeout=2) as response:
                if json.load(response).get("app") != "slaihand-offline-panel":
                    raise RuntimeError("Port belongs to another program")
            print("Panel is already running. Open http://127.0.0.1:8766 ; do not launch twice.", flush=True)
            webbrowser.open("http://127.0.0.1:8766/")
        except Exception:
            print("Port 8766 is occupied by another or an older program. Close the old panel window and retry.", flush=True)
        return
    # Reserve the Windows port before creating any WSL process.
    try:
        bridge = subprocess.Popen(["wsl.exe", "-d", "Ubuntu-24.04", "-u", "xuyu", "--", "bash", "-lc", command],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, encoding="utf-8")
    except Exception:
        server.server_close()
        raise
    print("Windows panel: http://127.0.0.1:8766", flush=True)
    def open_when_ready():
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                with opener.open("http://127.0.0.1:8766/", timeout=5) as response:
                    if response.status == 200:
                        webbrowser.open("http://127.0.0.1:8766/")
                        return
            except Exception:
                pass
            time.sleep(0.5)
        print("Automatic opening timed out. Open http://127.0.0.1:8766/ manually.", flush=True)

    threading.Thread(target=open_when_ready, daemon=True).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.stdin.close()
        try:
            bridge.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bridge.terminate()


if __name__ == "__main__":
    main()
