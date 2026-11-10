"""Windows 浏览器验证：当前位置静态采样卡片（真实 PanelBackend/HTTP + PTY 假总线）。

运行：python browser_supported_capture.py [页面URL或browser_url.txt]
只做预览/保持/撤手确认/采样结果展示；真正动作在 WSL PTY E2E 中另行验证。
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "control_panel_20261104"))
import browser_check as bc  # noqa: E402

EVIDENCE = HERE / "evidence"
bc.EVIDENCE = EVIDENCE
RESULT = {"checks": [], "failures": []}


def check(name, condition, detail=None):
    entry = {"name": name, "passed": bool(condition)}
    if detail is not None:
        entry["detail"] = detail
    RESULT["checks"].append(entry)
    print(("PASS  " if condition else "FAIL  ") + name
          + ("" if detail is None else "  " + json.dumps(detail, ensure_ascii=False, default=str)[:500]),
          flush=True)
    if not condition:
        RESULT["failures"].append(name)
    return bool(condition)


OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def post_api(base, token, path, payload):
    request = urllib.request.Request(
        base + path, data=json.dumps(payload).encode("utf-8"),
        headers={"X-Panel-Token": token, "Content-Type": "application/json",
                 "Origin": base}, method="POST")
    try:
        with OPENER.open(request, timeout=10) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8") or "{}")


def wait_status(base, token, predicate, timeout=20):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = bc.api_status(base, token)
        if predicate(last):
            return last
        time.sleep(0.2)
    raise AssertionError("status condition timed out: %r"
                         % ((last or {}).get("supported_capture")))


def main():
    if len(sys.argv) > 1:
        url = sys.argv[1]
    else:
        url = (HERE.parent / "control_panel_20261104" / "browser_url.txt").read_text(
            encoding="utf-8").strip()
    base, query = url.split("/?", 1)
    token = query.split("token=", 1)[1]
    EVIDENCE.mkdir(exist_ok=True)
    process, ws_url = bc.launch_browser()
    ws = None
    try:
        ws = bc.WebSocket(ws_url)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Log.enable")
        ws.command("Emulation.setDeviceMetricsOverride",
                   {"width": 1470, "height": 1250, "deviceScaleFactor": 1, "mobile": False})
        navigation = ws.command("Page.navigate", {"url": url})
        if navigation.get("errorText"):
            raise SystemExit("navigation failed: %s" % navigation["errorText"])
        bc.wait_for(ws, "document.readyState === 'complete' && !!document.getElementById('section-capture')",
                    30, "page load")
        check("静态采样卡片存在", ws.evaluate("!!document.getElementById('section-capture')"))
        check("预览/开始/撤手确认入口齐全",
              all(ws.evaluate("!!document.getElementById('%s')" % element) for element in
                  ("btn-capture-preview", "btn-capture-start", "btn-capture-ack",
                   "ack-capture-start", "ack-operator-disengaged")))
        check("大角度补采组已加入选择器",
              ["L01", "L02", "L03", "L04", "V02"] ==
              ws.evaluate("""Array.from(document.getElementById('calibration-group').options)
                  .map(o => o.value).filter(v => ['L01','L02','L03','L04','V02'].includes(v))"""))

        status = bc.api_status(base, token)
        if status["ros"]["stage"] != 0:
            post_api(base, token, "/api/reset", {})
        status = wait_status(base, token, lambda body: body["ros"]["stage"] == 0
                             and body["ros"]["connected"]
                             and ((body.get("supported_capture") or {})
                                  .get("baseline", {}).get("alpha") is not None))
        check("面板初始未使能且基线可知",
              status["supported_capture"]["baseline"]["alpha"] is not None, status["ros"]["stage_label"])
        check("开始按钮初始禁用",
              ws.evaluate("document.getElementById('btn-capture-start').disabled") is True)

        ws.evaluate("document.getElementById('btn-capture-preview').click()")
        be = wait_status(base, token, lambda body: (body.get("preview") or {}).get("mode") == "supported_capture")
        check("服务器生成静态采样预览票根",
              be["preview"]["mode"] == "supported_capture"
              and be["preview"]["frozen"]["baseline"]["satisfied"] is True,
              be["preview"]["frozen"]["start_deg"])
        shown = bc.wait_for(ws, "!document.getElementById('capture-preview').classList.contains('hidden')",
                            10, "preview panel")
        check("页面显示预览与身份/α 基线", bool(shown)
              and "α=0" in ws.evaluate("document.getElementById('capture-preview-meta').textContent")
              or "0.000" in ws.evaluate("document.getElementById('capture-preview-meta').textContent"))
        check("预览值在页面显示起点角度",
              "14.99" in ws.evaluate("document.getElementById('capture-preview-meta').textContent"))

        ws.evaluate("""(() => { for (const id of ['ack-supported', 'ack-clearance', 'ack-native',
            'ack-capture-start']) { const box = document.getElementById(id); box.checked = true;
            box.dispatchEvent(new Event('change', {bubbles: true})); } return true; })()""")
        bc.wait_for(ws, "document.getElementById('btn-capture-start').disabled === false",
                    10, "capture start enabled")
        check("四项确认后开始按钮可用",
              ws.evaluate("document.getElementById('btn-capture-start').disabled") is False)
        ws.evaluate("document.getElementById('btn-capture-start').click()")
        status = wait_status(base, token, lambda body: ((body.get("operation") or {}).get("kind")
                             == "supported_capture") or (((body.get("supported_capture") or {})
                             .get("current") or {}).get("id") is not None), 15)
        check("页面启动位置保持操作",
              ((status.get("operation") or {}).get("kind") == "supported_capture")
              or (((status.get("supported_capture") or {}).get("current") or {}).get("id") is not None))
        status = wait_status(base, token, lambda body: ((body.get("supported_capture") or {})
                                                        .get("current") or {}).get("ready") is True, 25)
        current = status["supported_capture"]["current"]
        check("页面显示保持已就绪", current["ready"] is True,
              {"command_id": current.get("command_id"), "boot": str(current.get("boot_id"))[:16]})
        bc.wait_for(ws, "document.getElementById('capture-reason').textContent.includes('撤手')",
                    10, "ready instructions")
        text = ws.evaluate("document.getElementById('capture-reason').textContent")
        check("就绪提示包含撤手说明", "撤手" in text, text[:120])
        check("撤手按钮在勾选前禁用",
              ws.evaluate("document.getElementById('btn-capture-ack').disabled") is True)
        ws.evaluate("""(() => { const box = document.getElementById('ack-operator-disengaged');
            box.checked = true; box.dispatchEvent(new Event('change', {bubbles: true}));
            return true; })()""")
        bc.wait_for(ws, "document.getElementById('btn-capture-ack').disabled === false",
                    10, "ack enabled")
        check("勾选撤手确认后按钮可用",
              ws.evaluate("document.getElementById('btn-capture-ack').disabled") is False)
        ws.evaluate("document.getElementById('btn-capture-ack').click()")
        bc.wait_for(ws, """(() => { const text = document.getElementById('capture-result')
            .textContent || ''; return text.includes('合格') && text.includes('拟合资格'); })()""",
                    30, "capture result")
        result_text = ws.evaluate("document.getElementById('capture-result').textContent")
        check("页面显示采样合格与拟合资格", "合格" in result_text
              and "可进入静态拟合" in result_text, result_text[:200])
        check("页面显示 α=0 基线与残差",
              "α" in result_text and "只读残差" in result_text, result_text[:220])
        check("页面显示原始文件与停止确认",
              "raw" in result_text and "已确认新失能" in result_text)
        status = wait_status(base, token, lambda body: (body.get("operation") or {})
                             and not body["operation"].get("running"), 20)
        check("采样后四轴已失能", status["ros"]["stage"] == 8, status["ros"]["stage_label"])
        check("操作结果不再运行", not (status.get("operation") or {}).get("running"))
        bc.screenshot(ws, "capture_card.png")
        RESULT["ok"] = not RESULT["failures"]
        return 0 if RESULT["ok"] else 1
    finally:
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass
        try:
            process.terminate()
        except Exception:
            pass
        EVIDENCE.mkdir(exist_ok=True)
        (EVIDENCE / "browser_supported_capture_result.json").write_text(
            json.dumps(RESULT, ensure_ascii=False, indent=2), encoding="utf-8")
        print("RESULT: %s (%d/%d checks)" % ("PASS" if not RESULT["failures"] else "FAIL",
                                             sum(1 for item in RESULT["checks"] if item["passed"]),
                                             len(RESULT["checks"])), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
