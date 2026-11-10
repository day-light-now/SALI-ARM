"""Windows browser regression for the J5 read-only/manual-calibration card."""
import json
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONTROL = HERE.parent / "control_panel_20261104"
sys.path.insert(0, str(CONTROL))
import browser_check as browser  # noqa: E402

browser.EVIDENCE = HERE / "evidence"


def set_position(degrees):
    data = json.dumps({"degrees": degrees}).encode()
    request = urllib.request.Request("http://127.0.0.1:18890/position", data=data,
                                     headers={"Content-Type": "application/json"})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(request, timeout=5) as response:
        assert response.status == 200


def main():
    url = (HERE / "j5_browser_url.txt").read_text(encoding="utf-8").strip()
    base, query = url.split("/?", 1)
    assert base == "http://127.0.0.1:18889", "refusing non-test panel"
    process, endpoint = browser.launch_browser()
    ws = None
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        browser.wait_for(ws, "document.getElementById('j5-body') !== null", 25, "page load")
        browser.wait_for(ws, "document.getElementById('j5-status').textContent.includes('只读反馈新鲜')",
                         20, "J5 fresh")
        raw = ws.evaluate("document.getElementById('j5-body').textContent")
        assert "原始编码器" in raw and "DM-J4340-2EC" in raw, raw
        assert "否（阶段一只读）" in ws.evaluate("document.getElementById('j5-flags').textContent")

        def check_ack(cid):
            ws.evaluate("document.getElementById('%s').checked = true" % cid)
            ws.evaluate("document.getElementById('%s').dispatchEvent(new Event('change', {bubbles: true}))" % cid)

        def click(button):
            assert ws.evaluate("!document.getElementById('%s').disabled" % button), button
            ws.evaluate("document.getElementById('%s').click()" % button)

        check_ack("ack-j5-disabled")
        browser.wait_for(ws, "!document.getElementById('btn-j5-zero').disabled", 5, "zero enabled")
        click("btn-j5-zero")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('零位已采')",
                         10, "zero captured")

        set_position(10.0)
        time.sleep(1.2)
        check_ack("ack-j5-rotated")
        browser.wait_for(ws, "!document.getElementById('btn-j5-direction').disabled", 5, "direction enabled")
        click("btn-j5-direction")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('+1')",
                         10, "direction confirmed")

        check_ack("ack-j5-cable")
        browser.wait_for(ws, "!document.getElementById('btn-j5-min').disabled", 5, "limits enabled")
        set_position(-35.0)
        time.sleep(.8)
        click("btn-j5-min")
        set_position(45.0)
        time.sleep(.8)
        click("btn-j5-max")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('已记录')",
                         10, "limits recorded")
        check_ack("ack-j5-save")
        browser.wait_for(ws, "!document.getElementById('btn-j5-save').disabled", 5, "save enabled")
        click("btn-j5-save")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('j5_calibration.json')",
                         10, "record saved")
        ws.evaluate("document.getElementById('section-j5').scrollIntoView({block: 'center'})")
        time.sleep(.5)
        browser.screenshot(ws, "j5_calibration_card.png")
        result = {"passed": True, "url": base + "/?token=<redacted>",
                  "raw_shown": True, "zero": "captured", "direction": "+1",
                  "limits": "recorded", "saved": True,
                  "physical_port_opened": False,
                  "clicked": ["btn-j5-zero", "btn-j5-direction", "btn-j5-min",
                              "btn-j5-max", "btn-j5-save"]}
        (HERE / "evidence" / "j5_browser.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print("PASS", json.dumps(result, ensure_ascii=False))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
