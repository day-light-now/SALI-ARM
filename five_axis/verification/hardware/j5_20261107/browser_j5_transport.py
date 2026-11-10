"""Windows browser check: transport fault shows disconnected/no stale angle and reconnect clears calibration."""
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


def post(path):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request("http://127.0.0.1:18892" + path, data=b"{}",
                                     headers={"Content-Type": "application/json"})
    with opener.open(request, timeout=5) as response:
        assert response.status == 200


def main():
    url = (HERE / "j5_fault_url.txt").read_text(encoding="utf-8").strip()
    base = url.split("/?", 1)[0]
    assert base == "http://127.0.0.1:18891", "refusing non-test panel"
    process, endpoint = browser.launch_browser()
    ws = None
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        browser.wait_for(ws, "document.getElementById('j5-body') !== null", 25, "page load")
        browser.wait_for(ws, "document.getElementById('j5-status').textContent.includes('只读反馈新鲜')",
                         20, "J5 online")
        ws.evaluate("document.getElementById('ack-j5-disabled').checked = true")
        ws.evaluate("document.getElementById('ack-j5-disabled').dispatchEvent(new Event('change', {bubbles: true}))")
        browser.wait_for(ws, "!document.getElementById('btn-j5-zero').disabled", 5, "zero enabled")
        ws.evaluate("document.getElementById('btn-j5-zero').click()")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('零位已采')",
                         10, "zero captured")

        post("/fault")
        browser.wait_for(ws, "document.getElementById('j5-status').textContent.includes('连接故障')",
                         10, "fault shown")
        browser.wait_for(ws, "!document.getElementById('btn-j5-reconnect').disabled", 10,
                         "reconnect enabled")
        body = ws.evaluate("document.getElementById('j5-body').textContent")
        assert "连接状态 / 会话" in body, body
        assert "原始编码器—" in body or "原始编码器\n—" in body.replace("\n", ""), body
        assert "零位已采" not in body, body
        ws.evaluate("window.scrollTo(0, document.getElementById('section-j5').offsetTop - 70)")
        time.sleep(.4)
        browser.screenshot(ws, "j5_transport_fault.png")

        # A failed reopen must surface as an error, not "已重连".
        post("/fail-next")
        ws.evaluate("document.getElementById('btn-j5-reconnect').click()")
        browser.wait_for(ws, "document.getElementById('message').textContent.includes('J5 操作被拒绝')",
                         10, "failed reconnect message")
        message = ws.evaluate("document.getElementById('message').textContent")
        assert "已重连" not in message, message
        assert "拒绝" in message or "失败" in message or "未恢复" in message, message
        assert ws.evaluate("document.getElementById('j5-status').textContent.includes('连接故障')")
        post("/normal")
        # Second attempt succeeds; calibration must still require re-zero.
        ws.evaluate("document.getElementById('btn-j5-reconnect').click()")
        browser.wait_for(ws, "document.getElementById('j5-status').textContent.includes('只读反馈新鲜')",
                         15, "reconnected")
        browser.wait_for(ws, "document.getElementById('j5-body').textContent.includes('零位待定')",
                         10, "calibration invalidated after reconnect")
        ws.evaluate("document.getElementById('section-j5').scrollIntoView({block: 'center'})")
        time.sleep(.4)
        browser.screenshot(ws, "j5_transport_reconnected.png")
        result = {"passed": True, "physical_port_opened": False,
                  "fault_shown": True, "stale_angle_hidden": True,
                  "reconnect_button": True,
                  "failed_reconnect_shown_error": True,
                  "reconnected": True, "rezero_required": True,
                  "screenshots": ["j5_transport_fault.png", "j5_transport_reconnected.png"]}
        (HERE / "evidence" / "j5_browser_transport_fault.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8")
        print("PASS", json.dumps(result, ensure_ascii=False))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
