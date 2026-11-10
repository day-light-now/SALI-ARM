#!/usr/bin/env python3
"""Stdio-only bridge: keeps the ROS web backend private inside WSL."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import urllib.error
import urllib.request


def main():
    backend = subprocess.Popen([sys.executable, str(Path(__file__).with_name("offline_panel.py"))],
                               stdout=subprocess.PIPE, stderr=sys.stderr, text=True,
                               env={**os.environ, "SLAIHAND_PANEL_PORT": "0"})
    try:
        startup = backend.stdout.readline().strip()
        prefix = "Offline panel: http://127.0.0.1:"
        if not startup.startswith(prefix) or not startup[len(prefix):].isdigit():
            raise RuntimeError("Offline backend failed to start")
        base = startup.removeprefix("Offline panel: ")
        print(startup, file=sys.stderr, flush=True)
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for line in sys.stdin:
            try:
                item = json.loads(line)
                path = item["path"]
                if path not in ["/", "/status", "/command"]:
                    raise ValueError("Unsupported route")
                body = item.get("body")
                request = urllib.request.Request(base + path,
                    data=body.encode() if body is not None else None,
                    headers={"X-Panel-Token": item.get("token", ""), "Content-Type": "application/json"},
                    method=item["method"])
                try:
                    response = opener.open(request, timeout=4)
                except urllib.error.HTTPError as error:
                    response = error
                with response:
                    reply = {"status": response.code, "mime": response.headers.get("Content-Type"),
                             "body": response.read().decode("utf-8")}
            except Exception as error:
                reply = {"status": 503, "mime": "application/json", "body": json.dumps({"message": str(error)})}
            print(json.dumps(reply), flush=True)
    finally:
        if backend.poll() is None:
            backend.send_signal(signal.SIGINT)
        try:
            backend.wait(timeout=5)
        except subprocess.TimeoutExpired:
            backend.terminate()
            backend.wait(timeout=5)


if __name__ == "__main__":
    main()
