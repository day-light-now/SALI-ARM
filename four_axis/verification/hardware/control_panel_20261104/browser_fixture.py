"""Keep a PTY fake stack + panel alive for the Windows browser check.

Isolated domain 176 only; the panel listens on 127.0.0.1 and is reached from the
Windows browser through WSL localhost forwarding.

Ownership: this process runs as its own session/PGID (started via setsid) and
records its PID/PGID plus child PIDs in /tmp/panel-browser-fixture/fixture.json.
Children stay in that process group, so stop_browser_fixture.sh can verify and
stop exactly this instance - never a pattern-matched or another panel.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

os.environ.update(ROS_DOMAIN_ID="176", ROS_LOCALHOST_ONLY="1")
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "verification" / "hardware" / "canusb_motion_20261104"))
sys.path.insert(0, str(ROOT / "verification" / "hardware"))
sys.path.insert(0, str(ROOT))
from verify_ros_motion import Plant  # noqa: E402

PANEL = Path(os.environ.get('PANEL_SCRIPT',"/home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py"))
PORT = 18888
WORK = Path("/tmp/panel-browser-fixture")
STATE = WORK / "fixture.json"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def write_state(**fields):
    payload = {"pid": os.getpid(), "pgid": os.getpgid(0), "started_at": time.time(),
               "domain": os.environ.get("ROS_DOMAIN_ID"), "script": str(Path(__file__).resolve())}
    payload.update(fields)
    STATE.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")


def main():
    if STATE.exists():
        raise SystemExit("fixture state file already exists; run stop_browser_fixture.sh first: %s" % STATE)
    shutil.rmtree(WORK, ignore_errors=True)
    WORK.mkdir(parents=True)
    write_state(stage="starting")
    plant = Plant()
    stack_log = (WORK / "stack.log").open("w", encoding="utf-8")
    stack = subprocess.Popen(
        ["ros2", "launch", "slaihand_bringup", "slaihand_canusb_motion.launch.py",
         "port:=" + plant.path, "log_directory:=" + str(WORK / "audit")],
        stdout=stack_log, stderr=subprocess.STDOUT)
    write_state(stage="stack", pty=plant.path, stack_pid=stack.pid)
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        text = (WORK / "stack.log").read_text(encoding="utf-8", errors="replace")
        if "Successful 'activate' of hardware" in text:
            break
        if stack.poll() is not None:
            raise SystemExit("fake stack died:\n" + text[-2000:])
        time.sleep(0.2)
    else:
        raise SystemExit("fake stack not ready:\n"
                         + (WORK / "stack.log").read_text(errors="replace")[-2000:])
    time.sleep(1.0)
    panel_log = (WORK / "panel.log").open("w", encoding="utf-8")
    panel = subprocess.Popen(
        [sys.executable, str(PANEL), "--port", str(PORT), "--archive-dir", str(WORK / "archive")],
        stdin=subprocess.DEVNULL, stdout=panel_log, stderr=subprocess.STDOUT)
    write_state(stage="panel", pty=plant.path, stack_pid=stack.pid, panel_pid=panel.pid)
    url = None
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        text = (WORK / "panel.log").read_text(encoding="utf-8", errors="replace")
        for line in text.splitlines():
            if "http://127.0.0.1:" in line and "token=" in line:
                url = line.split("：", 1)[-1].strip()
                break
        if url:
            break
        time.sleep(0.2)
    if not url:
        raise SystemExit("panel did not print its URL:\n" + (WORK / "panel.log").read_text(errors="replace"))
    token = url.split("token=", 1)[1]
    status_url = "http://127.0.0.1:%d/api/status" % PORT
    while time.monotonic() < deadline:
        try:
            request = urllib.request.Request(status_url, headers={"X-Panel-Token": token})
            with OPENER.open(request, timeout=3) as response:
                payload = json.load(response)
            if payload["ros"]["connected"]:
                break
        except Exception:
            pass
        time.sleep(0.3)
    (WORK / "url.txt").write_text(url + "\n", encoding="utf-8")
    (HERE / "browser_url.txt").write_text(url + "\n", encoding="utf-8")
    write_state(stage="ready", pty=plant.path, stack_pid=stack.pid, panel_pid=panel.pid, url=url)
    print("READY " + url, flush=True)

    def dump_counts():
        try:
            (WORK / "plant.json").write_text(json.dumps(
                {"fc": plant.fc, "fd": plant.fd, "mit": plant.mit, "errors": plant.errors,
                 "pty": plant.path}, ensure_ascii=False), encoding="utf-8")
        except Exception:
            pass

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    while running:
        dump_counts()
        time.sleep(0.5)
    dump_counts()
    panel.send_signal(signal.SIGTERM)
    try:
        panel.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(panel.pid), signal.SIGKILL)
    if stack.poll() is None:
        stack.send_signal(signal.SIGINT)
        try:
            stack.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(stack.pid), signal.SIGKILL)
    plant.close()
    try:
        STATE.unlink()
    except FileNotFoundError:
        pass
    print("FIXTURE_DONE", flush=True)


if __name__ == "__main__":
    main()
