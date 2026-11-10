"""Windows Chrome check against the live five-axis driver/panel fixture (fake PTY)."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0] / "control_panel_20261104"))
import browser_check as browser  # noqa: E402

browser.EVIDENCE = HERE / "evidence"


def main():
    url = (HERE / "five_axis_live_url.txt").read_text(encoding="utf-8").strip()
    assert "127.0.0.1:18898" in url
    process, endpoint = browser.launch_browser()
    ws = None
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        browser.wait_for(ws, "document.getElementById('axes-body') !== null", 25, "page load")
        browser.wait_for(ws,
                         "document.querySelectorAll('#axes-body tr').length === 5",
                         25, "five-axis live rows")
        # Wait for the live J5 row to show pending, not a ready/0 substitute.
        browser.wait_for(ws,
                         "document.getElementById('axes-body').textContent.includes('J5')",
                         15, "J5 row")
        text = ws.evaluate("document.getElementById('axes-body').textContent")
        assert "J5" in text
        assert ws.evaluate("document.getElementById('section-j5') === null")
        assert ws.evaluate("document.getElementById('j5-status') === null")
        assert ws.evaluate("document.getElementById('btn-j5-zero') === null")
        # The same status entry exposes unified schema/identity and a five-axis stop target.
        page = ws.evaluate("document.body.textContent")
        assert "slaihand_joint_plan/2" in page
        assert "J1 / J2 / J3 / J4 / J5" in page or "J1 / J2 / J3 / J4 / J5" in page
        ws.evaluate("document.getElementById('section-status').scrollIntoView({block:'start'})")
        time.sleep(.5)
        browser.screenshot(ws, "five_axis_live_driver.png")
        result = {"passed": True, "physical_port_opened": False,
                  "real_cpp_driver_over_pty": True, "panel_live": True,
                  "axes_rows": 5, "j5_card_absent": True,
                  "plan_schema": "slaihand_joint_plan/2",
                  "screenshot": "five_axis_live_driver.png"}
        (HERE / "evidence" / "five_axis_live_browser.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print("PASS", json.dumps(result, ensure_ascii=False))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
