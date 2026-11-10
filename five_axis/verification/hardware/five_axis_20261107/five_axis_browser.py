"""Windows Chrome DOM check for the single five-axis table/entry (fake backend)."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parents[0] / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
import browser_check as browser  # noqa: E402

browser.EVIDENCE = HERE / "evidence"


def main():
    url = (HERE / "five_axis_browser_url.txt").read_text(encoding="utf-8").strip()
    process, endpoint = browser.launch_browser()
    ws = None
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        browser.wait_for(ws, "document.getElementById('axes-body') !== null", 20, "page load")
        browser.wait_for(ws,
                         "document.querySelectorAll('#axes-body tr').length === 5",
                         20, "five-axis table")
        browser.wait_for(ws,
                         "document.querySelectorAll('#waypoints-body tr').length >= 1",
                         10, "five-axis plan editor")
        body = ws.evaluate("document.body ? document.body.textContent : document.documentElement.textContent")
        assert "J1–J5" in body or "J1-J5" in body, body[:500]
        assert "J5" in body
        assert ws.evaluate("document.getElementById('section-j5') === null")
        assert ws.evaluate("document.getElementById('j5-status') === null")
        assert ws.evaluate("document.getElementById('btn-j5-zero') === null")
        assert ws.evaluate("document.querySelectorAll('#waypoints-body tr:first-child input').length >= 5")
        assert ws.evaluate("document.querySelectorAll('.tab').length === 5")
        assert ws.evaluate("document.querySelectorAll('#axis-summary .axis-card').length === 5")
        assert ws.evaluate("document.querySelector('main').dataset.tab === 'overview'")
        assert ws.evaluate("document.getElementById('chip-latency').textContent.includes('面板：')")
        ws.evaluate("document.querySelector('.tab[data-tab=\\\"gravity\\\"]').click()")
        browser.wait_for(ws, "document.querySelector('main').dataset.tab === 'gravity'", 5, "gravity tab")
        ws.evaluate("document.querySelector('.tab[data-tab=\\\"calibration\\\"]').click()")
        browser.wait_for(ws, "document.querySelector('main').dataset.tab === 'calibration'", 5,
                         "calibration tab")
        assert ws.evaluate("document.getElementById('section-axis-calibration') !== null")
        assert ws.evaluate("document.querySelectorAll('#axis-cal-joint option').length === 5")
        for button in ("btn-axis-cal-start", "btn-axis-cal-zero", "btn-axis-cal-positive",
                       "btn-axis-cal-lower", "btn-axis-cal-upper", "btn-axis-cal-abort"):
            assert ws.evaluate("document.getElementById('%s') !== null" % button), button
        assert ws.evaluate("document.body.textContent.includes('不会写 hardware_mapping.yaml')")
        browser.screenshot(ws, "five_axis_calibration_workflow.png")
        ws.evaluate("document.querySelector('.tab[data-tab=\\\"overview\\\"]').click()")
        ws.evaluate("document.getElementById('section-status').scrollIntoView({block:'start'})")
        time.sleep(.4)
        browser.screenshot(ws, "five_axis_single_table.png")
        result = {"passed": True, "physical_port_opened": False,
                  "axes_rows": 5, "single_table": True, "j5_card_absent": True,
                  "plan_inputs": 5, "screenshot": "five_axis_single_table.png"}
        (HERE / "evidence" / "five_axis_browser.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print("PASS", json.dumps(result, ensure_ascii=False))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
