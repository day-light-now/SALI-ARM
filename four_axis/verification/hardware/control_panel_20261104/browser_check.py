"""Windows-side browser check for the four-axis panel candidate page.

Opens the real page served from WSL in headless Chrome, drives it over the
DevTools protocol (CDP), verifies readability/state/preview/stop visibility and
that polling never overwrites operator input, then saves screenshots.
"""
import base64
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
EVIDENCE = HERE / "evidence"
CHROME_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
RESULT = {"checks": [], "failures": []}


def check(name, condition, detail=None):
    entry = {"name": name, "passed": bool(condition)}
    if detail is not None:
        entry["detail"] = detail
    RESULT["checks"].append(entry)
    print(("PASS  " if condition else "FAIL  ") + name
          + ("" if detail is None else "  " + json.dumps(detail, ensure_ascii=False)[:400]), flush=True)
    if not condition:
        RESULT["failures"].append(name)
    return bool(condition)


class WebSocket:
    def __init__(self, url, timeout=25):
        assert url.startswith("ws://"), url
        authority, path = url[5:].split("/", 1)
        host, port = authority.rsplit(":", 1)
        self.sock = socket.create_connection((host, int(port)), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        handshake = ("GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                     "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (path, authority, key))
        self.sock.sendall(handshake.encode())
        self.buffer = b""
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("devtools handshake closed")
            head += chunk
        status_line, rest = head.split(b"\r\n\r\n", 1)
        if b" 101 " not in status_line.split(b"\r\n")[0]:
            raise RuntimeError("devtools handshake rejected: %r" % status_line[:120])
        self.buffer = rest
        self.next_id = 0
        self.events = []

    def _read(self, count):
        while len(self.buffer) < count:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("devtools socket closed")
            self.buffer += chunk
        data, self.buffer = self.buffer[:count], self.buffer[count:]
        return data

    def _frame(self):
        first, second = self._read(2)
        opcode = first & 0x0F
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._read(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._read(8))[0]
        return opcode, (self._read(length) if length else b"")

    def _send(self, payload, opcode=0x1):
        mask = os.urandom(4)
        header = bytes([0x80 | opcode])
        size = len(payload)
        if size < 126:
            header += bytes([0x80 | size])
        elif size < 65536:
            header += bytes([0x80 | 126]) + struct.pack("!H", size)
        else:
            header += bytes([0x80 | 127]) + struct.pack("!Q", size)
        self.sock.sendall(header + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def command(self, method, params=None):
        self.next_id += 1
        ident = self.next_id
        self._send(json.dumps({"id": ident, "method": method, "params": params or {}}).encode())
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            opcode, payload = self._frame()
            if opcode == 0x8:
                raise RuntimeError("devtools closed the session")
            if opcode == 0x9:
                self._send(payload, opcode=0xA)
                continue
            if opcode not in (0x0, 0x1):
                continue
            message = json.loads(payload.decode("utf-8"))
            if message.get("id") == ident:
                if "error" in message:
                    raise RuntimeError("CDP %s failed: %s" % (method, json.dumps(message["error"])[:300]))
                return message.get("result", {})
            if "method" in message:
                self.events.append(message)
        raise RuntimeError("CDP timeout for %s" % method)

    def evaluate(self, expression):
        result = self.command("Runtime.evaluate",
                              {"expression": expression, "returnByValue": True, "awaitPromise": True})
        if "exceptionDetails" in result:
            raise RuntimeError("page exception: %s" % json.dumps(result["exceptionDetails"])[:400])
        return result.get("result", {}).get("value")

    def close(self):
        try:
            self._send(b"", opcode=0x8)
        except Exception:
            pass
        self.sock.close()


def find_chrome():
    for path in CHROME_CANDIDATES:
        if Path(path).is_file():
            return path
    raise SystemExit("no Chrome/Edge found")


def free_port(start=9222):
    for port in range(start, start + 20):
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port
    raise SystemExit("no free devtools port")


def launch_browser():
    port = free_port()
    profile = tempfile.mkdtemp(prefix="panel-chrome-")
    process = subprocess.Popen(
        [find_chrome(), "--headless=new", "--disable-gpu", "--hide-scrollbars", "--no-first-run",
         "--no-default-browser-check", "--disable-background-networking", "--disable-extensions",
         "--no-proxy-server", "--proxy-bypass-list=*",
         "--remote-debugging-address=127.0.0.1", "--remote-debugging-port=%d" % port,
         "--user-data-dir=" + profile, "--window-size=1440,900", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + 25
    target = None
    while time.monotonic() < deadline:
        try:
            with OPENER.open("http://127.0.0.1:%d/json/list" % port, timeout=2) as response:
                targets = json.load(response)
            page = [item for item in targets if item.get("type") == "page"]
            if page:
                target = page[0]["webSocketDebuggerUrl"]
                break
        except Exception:
            pass
        time.sleep(0.3)
    if target is None:
        process.kill()
        raise SystemExit("Chrome devtools endpoint did not appear")
    return process, target


def wait_for(ws, expression, timeout, label):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = ws.evaluate(expression)
            if last:
                return last
        except RuntimeError as error:
            last = str(error)
        time.sleep(0.25)
    try:
        location = ws.evaluate("location.href")
        title = ws.evaluate("document.title")
        text = ws.evaluate("document.body ? document.body.textContent.slice(0, 200) : null")
    except Exception as error:
        location, title, text = str(error), None, None
    raise AssertionError("%s timed out (last=%r href=%r title=%r body=%r)"
                         % (label, last, location, title, text))


def screenshot(ws, path):
    data = ws.command("Page.captureScreenshot", {"format": "png"})["data"]
    EVIDENCE.mkdir(exist_ok=True)
    target = EVIDENCE / path
    target.write_bytes(base64.b64decode(data))
    return str(target)


def api_status(base, token):
    request = urllib.request.Request(base + "/api/status", headers={"X-Panel-Token": token})
    with OPENER.open(request, timeout=8) as response:
        return json.load(response)


def wait_operation(base, token, timeout=45):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = api_status(base, token)
        operation = last.get("operation") or {}
        if operation and not operation.get("running") and operation.get("phase") not in (
                "starting", "running", "stopping"):
            return last
        time.sleep(0.5)
    raise AssertionError("operation did not finish: %r" % (last and last.get("operation")))


def main():
    url = (HERE / "browser_url.txt").read_text(encoding="utf-8").strip()
    base, query = url.split("/?", 1)
    token = query.split("token=", 1)[1]
    RESULT["url"] = base + "/?token=<redacted>"
    process, ws_url = launch_browser()
    ws = None
    try:
        ws = WebSocket(ws_url)
        ws.command("Page.enable")
        ws.command("Runtime.enable")
        ws.command("Log.enable")
        ws.command("Emulation.setDeviceMetricsOverride",
                   {"width": 1440, "height": 900, "deviceScaleFactor": 1, "mobile": False})
        navigation = ws.command("Page.navigate", {"url": url})
        if navigation.get("errorText"):
            raise SystemExit("navigation failed: %s" % navigation["errorText"])
        wait_for(ws, "document.readyState === 'complete' && !!document.getElementById('chip-ros')",
                 25, "page load")
        check("页面标题正确", ws.evaluate("document.title") == "四轴 ROS 统一控制台",
              ws.evaluate("document.title"))
        wait_for(ws, "document.getElementById('chip-ros').textContent.indexOf('在线') >= 0", 20,
                 "ros feedback chip")
        check("状态显示真实反馈在线", True,
              ws.evaluate("document.getElementById('chip-ros').textContent"))
        check("控制阶段显示未使能",
              "未使能" in (ws.evaluate("document.getElementById('chip-stage').textContent") or ""),
              ws.evaluate("document.getElementById('chip-stage').textContent"))
        chip_lease = wait_for(
            ws, "document.getElementById('chip-lease').textContent.indexOf('正常') >= 0"
                " ? document.getElementById('chip-lease').textContent : ''", 6, "lease chip")
        check("心跳租约显示正常", "正常" in chip_lease, chip_lease)

        axes = ws.evaluate("""(() => Array.from(document.querySelectorAll('#axes-body tr')).map(
            row => Array.from(row.children).map(cell => cell.textContent)))()""")
        check("四轴行数正确", len(axes) == 4, len(axes))
        status = api_status(base, token)
        expected = ["%.3f" % axis["actual_deg"] for axis in status["axes"]]
        check("界面实际角度与服务器一致", [row[1] for row in axes] == expected,
              {"ui": [row[1] for row in axes], "api": expected})
        check("无目标时目标列显示占位", all(row[2] == "—" for row in axes), [row[2] for row in axes])
        speed_ui = ws.evaluate("document.getElementById('speed-limit').textContent")
        speed_api = str(status["server"]["reference_speed_rad_s"])
        check("页面峰值速度上限与规划器一致", speed_ui == speed_api,
              {"ui": speed_ui, "api": speed_api})
        shadow_rows = ws.evaluate("document.querySelectorAll('#gravity-body tr').length")
        shadow_flags = ws.evaluate("document.getElementById('gravity-flags').textContent")
        shadow_reason = ws.evaluate("document.getElementById('gravity-reason').textContent")
        check("重力 shadow 显示四轴估算行", shadow_rows == 4, shadow_rows)
        check("重力 shadow 标明未前馈", "前馈已应用否" in shadow_flags.replace(" ", "").replace("\n", ""),
              shadow_flags[:200])
        check("重力 shadow 说明未写入命令", "未写入命令" in shadow_reason, shadow_reason[:200])
        check("重力卡片显示驱动侧前馈状态", "驱动前馈α" in shadow_flags.replace(" ", ""),
              shadow_flags[:240])

        stop = ws.evaluate("""(() => { const b = document.getElementById('btn-stop');
            const r = b.getBoundingClientRect(); const s = getComputedStyle(b);
            return {w: r.width, h: r.height, left: r.left, top: r.top, display: s.display,
                    visibility: s.visibility, disabled: b.disabled}; })()""")
        check("停止入口可见且可用", stop["w"] > 60 and stop["h"] > 30 and stop["display"] != "none"
              and stop["visibility"] != "hidden" and not stop["disabled"], stop)

        # Operator input must not be overwritten by polling.
        ws.evaluate("""(() => { const input = document.querySelectorAll('#waypoints-body input')[0];
            input.value = '1.000';
            input.dispatchEvent(new Event('input', {bubbles: true}));
            return input.value; })()""")
        time.sleep(3.0)
        kept = ws.evaluate("document.querySelectorAll('#waypoints-body input')[0].value")
        check("轮询不覆盖操作员输入", kept == "1.000", kept)
        check("编辑后提示需要重新预览",
              "已修改" in (ws.evaluate("document.getElementById('editor-hint').textContent") or ""))

        ws.evaluate("document.getElementById('btn-preview').click()")
        wait_for(ws, "!document.getElementById('preview-panel').classList.contains('hidden')", 15,
                 "preview panel")
        rows = ws.evaluate("""(() => Array.from(document.querySelectorAll('#preview-body tr')).map(
            row => Array.from(row.children).map(cell => cell.textContent)))()""")
        check("预览显示服务器返回的绝对目标", len(rows) == 1 and len(rows[0]) == 8, rows)
        check("预览目标为软件绝对角度", rows[0][4].startswith(("15.", "16.")), rows[0][4])
        meta = ws.evaluate("document.getElementById('preview-meta').textContent")
        check("预览显示票根与预算", "票根" in meta and "时间预算" in meta, meta[:200])
        after = api_status(base, token)
        check("浏览器预览未使能", after["ros"]["stage"] == 0 and after["ros"]["command_id"] in (0, None)
              and after["operation"] is None, {"stage": after["ros"]["stage"],
                                               "command_id": after["ros"]["command_id"]})
        check("执行按钮在确认前不可用",
              ws.evaluate("document.getElementById('btn-execute').disabled") is True)

        # -- explicit confirmations, then execute through the real UI --------
        ws.evaluate("""(() => { for (const id of ['ack-supported', 'ack-clearance', 'ack-native']) {
            const box = document.getElementById(id); box.checked = true;
            box.dispatchEvent(new Event('change', {bubbles: true})); } return true; })()""")
        check("确认勾选后执行按钮可用",
              ws.evaluate("document.getElementById('btn-execute').disabled") is False)
        ws.evaluate("document.getElementById('btn-execute').click()")
        status = wait_operation(base, token, timeout=45)
        operation = status.get("operation") or {}
        project = operation.get("result") or {}
        check("浏览器执行完成且退出码 0", operation.get("exit_code") == 0,
              {"exit": operation.get("exit_code"), "phase": operation.get("phase")})
        check("浏览器执行逐点精度通过", project.get("accuracy_passed") is True, project.get("waypoint_results"))
        check("浏览器执行确认新失能", project.get("stop_observations_confirmed") is True)
        time.sleep(1.2)
        summary = ws.evaluate("document.getElementById('operation-summary').textContent") or ""
        check("操作结果阶段显示为完成", "完成" in summary, summary[:160])
        accuracy = ws.evaluate("""(() => Array.from(document.querySelectorAll('#accuracy-body tr')).map(
            row => Array.from(row.children).map(cell => cell.textContent)))()""")
        check("逐点判据表显示通过", len(accuracy) == 1 and accuracy[0][-1] == "通过", accuracy)
        stop_text = ws.evaluate("document.getElementById('operation-stop').textContent") or ""
        check("停止确认在结果中显示", "已确认" in stop_text, stop_text[:160])
        check("客户端原始输出可见",
              len((ws.evaluate("document.getElementById('operation-stdout').textContent") or "")) > 20)
        wait_for(ws, "document.getElementById('chip-stage').textContent.indexOf('已失能') >= 0", 10,
                 "disabled stage chip")

        # -- UI stop gate: stop locks the page until manual reset + new DISARMED
        ws.evaluate("document.getElementById('btn-stop').click()")
        wait_for(ws, "document.getElementById('banner').textContent.indexOf('停止已受理') >= 0", 10,
                 "stop lock banner")
        check("停止后横幅提示门禁锁定", True,
              ws.evaluate("document.getElementById('banner').textContent")[:160])
        check("停止后预览按钮禁用", ws.evaluate("document.getElementById('btn-preview').disabled") is True)
        stage_text = wait_for(
            ws,
            "(() => { const t = document.getElementById('chip-stage').textContent;"
            " return (t.indexOf('已失能') >= 0 || t.indexOf('UNKNOWN') >= 0) ? t : ''; })()",
            10, "stop stage")
        if "已失能" in stage_text:
            wait_for(ws, "document.getElementById('btn-reset').disabled === false", 10, "reset enabled")
            ws.evaluate("document.getElementById('btn-reset').click()")
            wait_for(ws, "document.getElementById('chip-stage').textContent.indexOf('未使能') >= 0", 12,
                     "unlocked stage")
            wait_for(ws, "document.getElementById('banner').classList.contains('hidden')", 10,
                     "stop banner cleared")
            check("复位后门禁解锁并恢复预览",
                  ws.evaluate("document.getElementById('btn-preview').disabled") is False)
        else:
            banner = ws.evaluate("document.getElementById('banner').textContent") or ""
            check("UNKNOWN 时横幅如实提示且复位禁用",
                  "UNKNOWN" in banner and ws.evaluate("document.getElementById('btn-reset').disabled") is True,
                  banner[:160])

        events = ws.evaluate("document.querySelectorAll('#event-list li').length")
        event_text = ws.evaluate("document.getElementById('event-list').textContent")
        check("面板事件日志可读", isinstance(events, int) and events >= 2
              and ("面板启动" in (event_text or "") or "预览" in (event_text or "")),
              {"count": events, "text": (event_text or "")[:160]})
        check("ROS/服务明细可读",
              "stop_all" in (ws.evaluate("document.getElementById('ros-detail').textContent") or ""))

        RESULT["desktop"] = screenshot(ws, "panel_desktop.png")
        size = ws.command("Page.getLayoutMetrics")["cssContentSize"]
        full = ws.command("Page.captureScreenshot",
                          {"format": "png", "captureBeyondViewport": True,
                           "clip": {"x": 0, "y": 0, "width": size["width"],
                                    "height": min(size["height"], 4000), "scale": 1}})["data"]
        (EVIDENCE / "panel_full.png").write_bytes(base64.b64decode(full))
        RESULT["full"] = str(EVIDENCE / "panel_full.png")
        ws.command("Emulation.setDeviceMetricsOverride",
                   {"width": 420, "height": 900, "deviceScaleFactor": 1, "mobile": False})
        time.sleep(0.8)
        stop_narrow = ws.evaluate("""(() => { const b = document.getElementById('btn-stop');
            const r = b.getBoundingClientRect(); return {w: r.width, h: r.height, top: r.top,
            right: r.right, viewport: window.innerWidth}; })()""")
        check("窄屏停止入口仍可见", stop_narrow["w"] > 60 and stop_narrow["h"] > 30
              and stop_narrow["right"] <= stop_narrow["viewport"] + 1, stop_narrow)
        overflow = ws.evaluate("document.documentElement.scrollWidth")
        check("窄屏无整页横向溢出", overflow <= 421, {"scrollWidth": overflow})
        check("窄屏提示表格可滑动",
              ws.evaluate("getComputedStyle(document.querySelector('.scroll-hint')).display") != "none")
        RESULT["narrow"] = screenshot(ws, "panel_narrow.png")
        ws.command("Emulation.clearDeviceMetricsOverride")

        exceptions = [event for event in ws.events
                      if event.get("method") == "Runtime.exceptionThrown"]
        errors = [event for event in ws.events
                  if event.get("method") == "Log.entryAdded"
                  and event.get("params", {}).get("entry", {}).get("level") == "error"]
        check("页面无未捕获异常", not exceptions, exceptions[:2])
        check("页面无脚本错误日志", not errors, errors[:2])
        RESULT["status"] = "PASS" if not RESULT["failures"] else "FAIL"
        return RESULT
    finally:
        if ws is not None:
            ws.close()
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
        EVIDENCE.mkdir(exist_ok=True)
        (EVIDENCE / "browser_result.json").write_text(
            json.dumps(RESULT, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    outcome = main()
    print(json.dumps({"status": outcome.get("status"), "checks": len(outcome["checks"]),
                      "failures": outcome["failures"]}, ensure_ascii=False))
    raise SystemExit(0 if outcome.get("status") == "PASS" else 1)
