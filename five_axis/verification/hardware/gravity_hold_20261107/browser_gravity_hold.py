"""Windows-side browser click test for the gravity-hold mode switch.

Uses the existing isolated fixture (panel 18888, ROS domain 176, PTY drive).
"""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parent / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
import browser_check as browser  # noqa: E402

browser.EVIDENCE = HERE / "evidence"


def statuses(base, token):
    return browser.api_status(base, token)


def wait_ui(ws, expression, timeout, label):
    return browser.wait_for(ws, expression, timeout, label)


def main():
    url = (CONTROL / "browser_url.txt").read_text(encoding="utf-8").strip()
    base, query = url.split("/?", 1)
    assert base == "http://127.0.0.1:18888", "refusing a non-test panel: %s" % base
    token = query.split("token=", 1)[1]
    process, endpoint = browser.launch_browser()
    ws = None
    report = {"passed": False, "fixture": base, "checks": []}
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        wait_ui(ws, "document.readyState === 'complete' && !!document.getElementById('btn-gravity-lock')",
                25, "page load")
        time.sleep(3)
        ws.evaluate("""(() => {
            const seconds = document.getElementById('float-seconds');
            seconds.value = '40';
            seconds.dispatchEvent(new Event('input', {bubbles: true}));
            seconds.dispatchEvent(new Event('change', {bubbles: true}));
            for (const id of ['ack-supported', 'ack-clearance', 'ack-native', 'ack-gravity-float']) {
                const input = document.getElementById(id);
                input.checked = true;
                input.dispatchEvent(new Event('change', {bubbles: true}));
            }
        })()""")
        wait_ui(ws, "!document.getElementById('btn-gravity-float').disabled", 25, "float arm enabled")
        ws.evaluate("document.getElementById('btn-gravity-float').click()")
        wait_ui(ws, "document.getElementById('btn-gravity-lock') && "
                    "!document.getElementById('btn-gravity-lock').disabled", 25, "lock enabled after float")
        before = statuses(base, token)
        driver = before["gravity_shadow"]["driver"]
        assert driver["active"] and driver["gravity_float"], driver
        assert driver["alpha"] == 1 and driver["ramp"] == 3, driver
        operation_id = before["operation"]["id"]
        command_id = before["ros"]["command_id"]
        browser.screenshot(ws, "gravity_hold_before.png")

        ws.evaluate("document.getElementById('btn-gravity-lock').click()")
        wait_ui(ws, "document.getElementById('gravity-mode-state').textContent.trim() === '已锁定'",
                20, "confirmed locked")
        locked = statuses(base, token)
        mode = locked["gravity_mode"]
        assert mode["locked_confirmed"] is True, mode
        assert mode["phase"] == "locked", mode
        assert mode["lock_state"] == 2, mode
        assert locked["operation"]["id"] == operation_id
        assert locked["ros"]["command_id"] == command_id
        assert locked["gravity_shadow"]["driver"]["alpha"] == 1
        assert locked["gravity_shadow"]["driver"]["gravity_hold"] is True
        targets = [row["target_deg"] for row in mode["axes"]]
        assert all(target is not None for target in targets), mode
        body = ws.evaluate("document.getElementById('gravity-mode-body').textContent")
        assert "J1" in body and "J4" in body, body
        ws.evaluate("document.getElementById('gravity-mode-box').scrollIntoView({block: 'center'})")
        time.sleep(.5)
        browser.screenshot(ws, "gravity_hold_locked.png")

        # Simulate a small operator push on the PTY drive; browser target text must not drift.
        time.sleep(.3)
        ws.evaluate("document.getElementById('ack-release-weight').checked = true")
        ws.evaluate("document.getElementById('ack-release-weight').dispatchEvent(new Event('change', {bubbles: true}))")
        ws.evaluate("document.getElementById('btn-gravity-resume').click()")
        deadline = time.monotonic() + 20
        resumed = None
        while time.monotonic() < deadline:
            resumed = statuses(base, token)
            if (resumed.get("gravity_mode", {}).get("phase") == "float"
                    and resumed.get("gravity_shadow", {}).get("driver", {}).get("gravity_float") is True):
                break
            time.sleep(.2)
        assert resumed and resumed["gravity_mode"]["phase"] == "float", resumed
        assert resumed["operation"]["id"] == operation_id
        assert resumed["ros"]["command_id"] == command_id
        assert resumed["gravity_shadow"]["driver"]["gravity_float"] is True
        assert resumed["gravity_shadow"]["driver"]["alpha"] == 1
        assert resumed["gravity_mode"]["kp"] == 0
        browser.screenshot(ws, "gravity_hold_resumed.png")

        ws.evaluate("document.getElementById('btn-stop').click()")
        deadline = time.monotonic() + 25
        finished = None
        while time.monotonic() < deadline:
            finished = statuses(base, token)
            operation = finished.get("operation") or {}
            if operation and not operation.get("running"):
                break
            time.sleep(.2)
        assert finished and finished["operation"] and not finished["operation"]["running"]
        browser.screenshot(ws, "gravity_hold_stopped.png")
        report.update({
            "passed": True, "operation_id": operation_id, "command_id": command_id,
            "locked_target_deg": targets, "locked_phase": mode["phase"],
            "resume_phase": resumed["gravity_mode"]["phase"], "resume_kp": resumed["gravity_mode"]["kp"],
            "alpha": 1, "actual_clicks": ["btn-gravity-float", "btn-gravity-lock",
                                          "btn-gravity-resume", "btn-stop"],
            "fixture_port": 18888, "stop_confirmed": True,
        })
        (browser.EVIDENCE / "gravity_hold_browser.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8")
        print("PASS", json.dumps(report))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
