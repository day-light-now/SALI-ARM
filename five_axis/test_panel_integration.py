"""Run only against an isolated four-axis fake controller and local panel."""
import json
import re
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8766"
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
page = opener.open(BASE, timeout=10).read().decode()
token = re.search(r"const token='([a-f0-9]+)'", page).group(1)


def status():
    return json.load(opener.open(BASE + "/status", timeout=10))


def command(name, degrees=None, use_token=True):
    request = urllib.request.Request(BASE + "/command",
        data=json.dumps({"command": name, "degrees": degrees}).encode(),
        headers={"Content-Type": "application/json", "X-Panel-Token": token if use_token else ""})
    try:
        response = opener.open(request, timeout=10)
    except urllib.error.HTTPError as error:
        response = error
    with response:
        return response.code, json.load(response)


def wait_idle():
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        current = status()
        if not current["busy"]:
            return current
        time.sleep(0.2)
    raise AssertionError("Trajectory did not finish")


assert status()["ready"]
assert command("move", [0, 0, 0, 60])[0] == 409
assert command("zero", use_token=False)[0] == 403
assert command("move", [10, 5, -5, 5])[0] == 200
assert command("zero")[0] == 409  # Busy gate: do not silently overwrite a goal.
s = wait_idle()
assert all(abs(a-b) < 0.12 for a, b in zip(s["positions"], [10, 5, -5, 5])), s
assert "code=0" in s["message"], s
assert command("demo")[0] == 200
time.sleep(0.4)
assert command("cancel")[0] == 200
wait_idle()
assert command("zero")[0] == 200
s = wait_idle()
assert all(abs(x) < 0.12 for x in s["positions"]), s
print("PASS: Windows/WSL page, readiness, limit rejection, token guard, busy guard, motion, cancellation, zero feedback")
