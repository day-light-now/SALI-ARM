"""Windows 侧浏览器验证：拖动示教记录状态、保存、载入列表与回放预览。

对真实 PanelBackend/HTTP 页面操作（页面由 WSL 的独立假总线 fixture 提供），
只做记录/保存/载入/预览；不执行回放（回放在 PTY E2E 中验证）。
运行：python browser_drag_teaching.py [页面URL或browser_url.txt]
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

OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
EVIDENCE = HERE / "evidence"
bc.EVIDENCE = EVIDENCE  # screenshots/JSON land in the drag-teaching evidence dir
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


def api(base, token, path):
    request = urllib.request.Request(base + path, headers={"X-Panel-Token": token})
    with OPENER.open(request, timeout=10) as response:
        return json.load(response)


def wait_status(base, token, predicate, timeout=20):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = api(base, token, "/api/status")
        if predicate(last):
            return last
        time.sleep(0.25)
    raise AssertionError("status condition timed out: %r" % (last and last.get("teaching")))


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
                   {"width": 1470, "height": 1100, "deviceScaleFactor": 1, "mobile": False})
        navigation = ws.command("Page.navigate", {"url": url})
        if navigation.get("errorText"):
            raise SystemExit("navigation failed: %s" % navigation["errorText"])
        bc.wait_for(ws, "document.readyState === 'complete' && !!document.getElementById('section-teaching')",
                    30, "page load")
        check("拖动示教卡片存在", ws.evaluate("!!document.getElementById('section-teaching')"))
        check("开始/停止/保存/载入/预览入口齐全",
              all(ws.evaluate("!!document.getElementById('%s')" % element) for element in
                  ("btn-teaching-start", "btn-teaching-stop", "btn-teaching-save",
                   "btn-teaching-load", "btn-teaching-preview", "btn-teaching-execute")))
        check("未勾选拖动示教确认时开始按钮禁用",
              ws.evaluate("document.getElementById('btn-teaching-start').disabled") is True)

        ws.evaluate("""(() => { for (const id of ['ack-supported', 'ack-clearance', 'ack-native',
            'ack-drag-teaching']) { const box = document.getElementById(id); box.checked = true;
            box.dispatchEvent(new Event('change', {bubbles: true})); } return true; })()""")
        time.sleep(0.8)
        check("四项确认后开始按钮可用",
              ws.evaluate("document.getElementById('btn-teaching-start').disabled") is False)

        controls = ws.evaluate("""(() => {
          const kd = document.getElementById('float-kd');
          const seconds = document.getElementById('float-seconds');
          kd.value = '2'; kd.dispatchEvent(new Event('input', {bubbles: true}));
          seconds.value = '20'; seconds.dispatchEvent(new Event('input', {bubbles: true}));
          return {type: kd.type, label: document.getElementById('float-kd-value').textContent};
        })()""")
        check("Kd 滑条可操作并显示数值", controls == {"type": "range", "label": "2.0"}, controls)
        time.sleep(1)
        check("轮询保留操作员的 Kd 和时间输入", ws.evaluate(
            "[document.getElementById('float-kd').value, document.getElementById('float-seconds').value]")
            == ["2", "20"])
        ws.evaluate("document.getElementById('btn-teaching-start').click()")
        phase = bc.wait_for(ws, """(() => { const text = document.getElementById('teaching-flags').textContent;
            return (text.includes('渐入中') || text.includes('有效记录中')) ? text.slice(0, 120) : ''; })()""",
                            18, "teaching phase")
        check("页面显示渐入/有效记录阶段", bool(phase), phase)
        bc.wait_for(ws, "document.getElementById('teaching-flags').textContent.includes('有效记录中')",
                    18, "valid recording phase")
        check("页面进入有效记录阶段", True,
              ws.evaluate("document.getElementById('teaching-flags').textContent.slice(0, 160)"))
        live = api(base, token, "/api/status")
        check("网页设置经驱动回执确认", live["teaching"]["driver"].get("float_kd") == 2.0
              and live["teaching"]["driver"].get("float_seconds") == 20.0)
        check("活动期间参数输入锁定", ws.evaluate(
            "document.getElementById('float-kd').disabled && document.getElementById('float-seconds').disabled"))
        time.sleep(1.3)
        ws.evaluate("document.getElementById('btn-teaching-stop').click()")
        status = wait_status(base, token, lambda body: (
            body.get("operation") and not body["operation"].get("running")
            and ((body.get("teaching") or {}).get("last") or {}).get("recording_id")), timeout=25)
        last = status["teaching"]["last"]
        check("页面停止记录后记录完整落盘", last["complete"] is True, last.get("reasons")[:3])
        bc.wait_for(ws, "document.getElementById('teaching-flags').textContent.includes('最近记录帧数/时长')",
                    12, "recent record rendered")
        check("页面显示最近记录帧数/时长",
              "最近记录帧数/时长" in ws.evaluate("document.getElementById('teaching-flags').textContent"),
              ws.evaluate("document.getElementById('teaching-flags').textContent.slice(0, 220)"))
        check("记录后进入已失能", status["ros"]["stage"] == 8, status["ros"]["stage"])

        ws.evaluate("""(() => { const input = document.getElementById('teaching-name');
            input.value = '浏览器拖动示教';
            input.dispatchEvent(new Event('input', {bubbles: true})); return true; })()""")
        bc.wait_for(ws, "document.getElementById('btn-teaching-save').disabled === false",
                    12, "save button enabled")
        check("保存按钮在记录完整后可用",
              ws.evaluate("document.getElementById('btn-teaching-save').disabled") is False)
        ws.evaluate("document.getElementById('btn-teaching-save').click()")
        bc.wait_for(ws, """Array.from(document.querySelectorAll('#teaching-select option'))
            .some((option) => option.textContent.includes('浏览器拖动示教'))""", 20, "saved teaching")
        check("保存后出现在拖动示教列表", True)
        bc.wait_for(ws, """Array.from(document.querySelectorAll('#saved-select option'))
            .some((option) => option.textContent.includes('[示教]'))""", 20, "existing load list")
        check("保存后出现在现有载入列表", True,
              ws.evaluate("document.getElementById('saved-select').textContent.slice(0, 160)"))

        ws.evaluate("document.getElementById('btn-reset').click()")
        wait_status(base, token, lambda body: body["ros"]["stage"] == 0
                    and body["ros"]["stage_fresh"] is True, timeout=25)
        check("复位后回到未使能", True)

        ws.evaluate("""(() => { const select = document.getElementById('teaching-select');
            const option = Array.from(select.options).find((item) => item.textContent.includes('浏览器拖动示教'));
            select.value = option.value;
            select.dispatchEvent(new Event('change', {bubbles: true})); return select.value; })()""")
        bc.wait_for(ws, "document.getElementById('btn-teaching-load').disabled === false", 10,
                    "teaching selected")
        check("载入按钮可用", True)
        bc.wait_for(ws, "document.getElementById('btn-teaching-preview').disabled === false", 15,
                    "preview enabled")
        ws.evaluate("document.getElementById('btn-teaching-preview').click()")
        bc.wait_for(ws, "!document.getElementById('teaching-preview').classList.contains('hidden')",
                    20, "teaching preview")
        meta = ws.evaluate("document.getElementById('teaching-preview-meta').textContent")
        check("预览显示接近段/重定时总时长/峰值速度",
              all(keyword in meta for keyword in ("接近段", "重定时总时长", "峰值回放速度")), meta[:300])
        status = api(base, token, "/api/status")
        check("浏览器预览未使能", status["ros"]["stage"] == 0
              and not ((status.get("operation") or {}).get("running")), status["ros"]["stage"])
        screenshot = bc.screenshot(ws, "drag_teaching_browser.png")
        print("SCREENSHOT", screenshot)
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
    RESULT["ok"] = not RESULT["failures"]
    return 0 if RESULT["ok"] else 1


if __name__ == "__main__":
    try:
        exit_code = main()
    except Exception as error:  # noqa: BLE001
        RESULT["error"] = "%s: %s" % (type(error).__name__, error)
        RESULT["ok"] = False
        print("BROWSER_CHECK_ERROR", RESULT["error"])
        exit_code = 1
    EVIDENCE.mkdir(exist_ok=True)
    (EVIDENCE / "drag_teaching_browser.json").write_text(
        json.dumps(RESULT, ensure_ascii=False, indent=2), encoding="utf-8")
    print("RESULT: %s (%d/%d checks)" % ("PASS" if not RESULT["failures"] else "FAIL",
                                          sum(1 for item in RESULT["checks"] if item["passed"]),
                                          len(RESULT["checks"])))
    raise SystemExit(exit_code)
