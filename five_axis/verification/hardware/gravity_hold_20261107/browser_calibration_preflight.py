"""Browser regression: old calibration group must fail before preview/start.

The page must show the alpha/identity/model refusal with observed values instead
of silently launching a five-second pre-arm wait. Isolated fixture only.
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


def main():
    url = (CONTROL / "browser_url.txt").read_text(encoding="utf-8").strip()
    base, query = url.split("/?", 1)
    assert base == "http://127.0.0.1:18888", "refusing a non-test panel"
    token = query.split("token=", 1)[1]
    process, endpoint = browser.launch_browser()
    ws = None
    try:
        ws = browser.WebSocket(endpoint)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Page.navigate", {"url": url})
        browser.wait_for(ws, "document.getElementById('btn-calibration-group') !== null",
                         25, "page load")
        time.sleep(2)
        ws.evaluate("document.getElementById('calibration-group').value = 'S01'")
        ws.evaluate("document.getElementById('btn-calibration-group').click()")
        browser.wait_for(ws, "document.getElementById('execution-mode').value === 'calibration'",
                         8, "S01 group loaded")
        ws.evaluate("document.getElementById('btn-preview').click()")
        browser.wait_for(ws, "document.getElementById('message').textContent.includes('CALIBRATION_PREFLIGHT')",
                         8, "calibration preflight message")
        text = ws.evaluate("document.getElementById('message').textContent")
        assert "alpha" in text.lower() and "0.1" in text, text
        assert "MODEL_UNQUALIFIED" in text, text
        browser.screenshot(ws, "calibration_preflight_error.png")
        evidence = {"passed": True, "message": text,
                    "group": "S01", "expected": ["ALPHA_NONZERO", "MODEL_UNQUALIFIED"],
                    "fixture_port": 18888, "physical_port_opened": False}
        (browser.EVIDENCE / "calibration_preflight_browser.json").write_text(
            json.dumps(evidence, indent=2, ensure_ascii=False), encoding="utf-8")
        print("PASS", json.dumps(evidence, ensure_ascii=False))
    finally:
        if ws is not None:
            ws.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == "__main__":
    main()
