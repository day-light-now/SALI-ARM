"""Local-only HTTP surface for the unified five-axis control panel.

The handler is production code: tests drive it through a real socket.  It holds
no ROS objects; a narrow backend object is injected so the same callbacks can be
exercised against the isolated ROS stack or a local fake.

There is intentionally no separate J5 read-only endpoint: J5 is one row in
``/api/status`` and one coordinate in the single plan/execute entry.  A second
serial owner is never created by this surface.
"""
from __future__ import annotations

import json
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from control_panel_model import MAX_PLAN_BYTES, PanelError

STATIC_ROUTES = {
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/panel.css": ("panel.css", "text/css; charset=utf-8"),
    "/panel.js": ("panel.js", "text/javascript; charset=utf-8"),
}
CSP = ("default-src 'none'; script-src 'self'; style-src 'self'; connect-src 'self'; "
       "img-src 'self'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'")


def make_handler(backend, token, static_dir):
    static_dir = Path(static_dir)

    class Handler(BaseHTTPRequestHandler):
        server_version = "SLAIControlPanel/1.0"
        sys_version = ""
        protocol_version = "HTTP/1.1"

        # -- plumbing -----------------------------------------------------
        def log_message(self, *_args):
            pass

        @property
        def port(self):
            return self.server.server_address[1]

        def _allowed_hosts(self):
            return {"127.0.0.1:%d" % self.port, "localhost:%d" % self.port}

        def _host_ok(self):
            return self.headers.get("Host", "") in self._allowed_hosts()

        def _token_header_ok(self):
            supplied = self.headers.get("X-Panel-Token", "")
            return bool(supplied) and secrets.compare_digest(supplied, token)

        def _query_token_ok(self, parsed):
            supplied = (parse_qs(parsed.query).get("token") or [""])[0]
            return bool(supplied) and secrets.compare_digest(supplied, token)

        def _origin_ok(self):
            allowed = {"http://127.0.0.1:%d" % self.port, "http://localhost:%d" % self.port}
            return self.headers.get("Origin", "") in allowed

        def _origin_ok_if_present(self):
            origin = self.headers.get("Origin")
            if origin is None:
                return True
            return origin in {"http://127.0.0.1:%d" % self.port, "http://localhost:%d" % self.port}

        def _fetch_site_ok(self):
            site = self.headers.get("Sec-Fetch-Site")
            return site is None or site in ("same-origin", "none")

        def _send(self, status, body, mime="application/json; charset=utf-8", extra=None):
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("X-Frame-Options", "DENY")
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.send_header("Connection", "close")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)
            self.close_connection = True

        def _json(self, status, payload, extra=None):
            body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
            self._send(status, body, extra=extra)

        def _error(self, status, code, message, extra=None):
            payload = {"ok": False, "accepted": False, "completed": False,
                       "code": code, "message": message}
            payload.update(extra or {})
            self._json(status, payload)

        # -- request reading ---------------------------------------------
        def _read_json(self):
            if self.headers.get("Transfer-Encoding"):
                raise PanelError(400, "BODY", "不接受分块传输")
            raw = self.headers.get("Content-Length")
            if raw is None:
                raise PanelError(411, "BODY", "需要 Content-Length")
            try:
                size = int(raw)
            except ValueError:
                raise PanelError(400, "BODY", "Content-Length 无效")
            if size < 0 or size > MAX_PLAN_BYTES:
                raise PanelError(413, "BODY", "请求体超过 %d 字节限制" % MAX_PLAN_BYTES)
            self.connection.settimeout(5.0)
            data = self.rfile.read(size) if size else b""
            if len(data) != size:
                raise PanelError(400, "BODY", "请求体不完整")
            if not data:
                return {}
            try:
                value = json.loads(data.decode("utf-8"))
            except Exception:
                raise PanelError(400, "BODY", "请求体必须是 UTF-8 JSON")
            if not isinstance(value, dict):
                raise PanelError(400, "BODY", "请求体必须是 JSON 对象")
            return value

        # -- GET ----------------------------------------------------------
        def do_GET(self):
            try:
                parsed = urlparse(self.path)
                path = parsed.path
                if not self._host_ok():
                    return self._error(403, "HOST", "拒绝的 Host 头")
                if path == "/":
                    if not self._query_token_ok(parsed):
                        return self._error(403, "TOKEN", "页面 token 无效，请使用启动时打印的完整地址")
                    return self._send_page()
                if path in STATIC_ROUTES:
                    return self._send_static(path)
                if path == "/favicon.ico":
                    return self._send(204, b"")
                if path.startswith("/api/"):
                    if not self._token_header_ok():
                        return self._error(403, "TOKEN", "缺少或错误的会话 token")
                    if not self._origin_ok_if_present() or not self._fetch_site_ok():
                        return self._error(403, "ORIGIN", "跨源请求被拒绝")
                    return self._api_get(path)
                return self._error(404, "NOT_FOUND", "未知路径")
            except PanelError as error:
                return self._json(error.status, error.payload())
            except Exception as error:  # pragma: no cover - defensive
                backend.record_event("GET 处理异常：%s" % error, "error")
                return self._error(500, "INTERNAL", "面板内部错误")

        def _send_page(self):
            template = (static_dir / "index.html").read_text(encoding="utf-8")
            body = template.replace("__PANEL_TOKEN__", token).encode("utf-8")
            self._send(200, body, mime="text/html; charset=utf-8",
                       extra={"Content-Security-Policy": CSP})

        def _send_static(self, path):
            name, mime = STATIC_ROUTES[path]
            body = (static_dir / name).read_bytes()
            self._send(200, body, mime=mime)

        def _api_get(self, path):
            if path == '/api/calibration/groups':
                from calibration_session import (GROUPS, BIDIRECTIONAL_GROUPS, group_document,
                                                 qualified_group_names)
                groups = {name: group_document(name) for name in
                          [*BIDIRECTIONAL_GROUPS, 'G01-G06', *GROUPS,
                           *qualified_group_names()]}
                # Wide five-axis plans are position-only candidates.  They are
                # exposed in the same picker but the UI must load them as
                # ordinary trajectory plans, never as formal gravity groups.
                from five_axis_trajectory_groups import groups as five_axis_groups
                groups.update(five_axis_groups())
                return self._json(200, {'ok': True, 'groups': groups})
            if path == '/api/calibration/axis':
                return self._json(200, backend.axis_calibration_status())
            if path == '/api/connection':
                return self._json(200, backend.connection())
            if path == "/api/status":
                return self._json(200, backend.status())
            if path == "/api/diagnostics":
                report = getattr(backend, "diagnostics_report", None)
                if not callable(report):
                    return self._error(404, "NOT_FOUND", "该后端不提供诊断接口")
                return self._json(200, report())
            if path == "/api/margin":
                report = getattr(backend, "margin_report", None)
                if not callable(report):
                    return self._error(404, "NOT_FOUND", "该后端不提供裕量接口")
                return self._json(200, report())
            if path == "/api/plans":
                return self._json(200, {"ok": True, "plans": backend.list_plans()})
            if path == "/api/teachings":
                return self._json(200, backend.list_teachings())
            if path.startswith("/api/teachings/"):
                teaching_id = path[len("/api/teachings/"):]
                return self._json(200, backend.load_teaching(teaching_id))
            if path.startswith("/api/plans/"):
                plan_id = path[len("/api/plans/"):]
                return self._json(200, backend.load_plan(plan_id))
            return self._error(404, "NOT_FOUND", "未知接口")

        def _discard_body(self):
            """Drain a rejected request body so clients see the JSON rejection."""
            try:
                size = int(self.headers.get("Content-Length") or "0")
            except ValueError:
                return
            if 0 < size <= MAX_PLAN_BYTES:
                self.connection.settimeout(2.0)
                try:
                    self.rfile.read(size)
                except Exception:
                    pass

        # -- POST ---------------------------------------------------------
        def do_POST(self):
            try:
                parsed = urlparse(self.path)
                path = parsed.path
                if not self._host_ok():
                    self._discard_body()
                    return self._error(403, "HOST", "拒绝的 Host 头")
                if not self._token_header_ok():
                    self._discard_body()
                    return self._error(403, "TOKEN", "缺少或错误的会话 token")
                if not self._origin_ok():
                    self._discard_body()
                    return self._error(403, "ORIGIN", "跨源或缺失 Origin 的写操作被拒绝")
                if not self._fetch_site_ok():
                    self._discard_body()
                    return self._error(403, "ORIGIN", "跨源请求被拒绝")
                if not path.startswith("/api/"):
                    return self._error(404, "NOT_FOUND", "未知路径")
                body = self._read_json()
                return self._api_post(path, body)
            except PanelError as error:
                return self._json(error.status, error.payload())
            except Exception as error:  # pragma: no cover - defensive
                backend.record_event("POST 处理异常：%s" % error, "error")
                return self._error(500, "INTERNAL", "面板内部错误")

        def _api_post(self, path, body):
            if path == '/api/connection/reconnect':
                return self._json(202, backend.reconnect(body.get('ack_power_cycle') is True))
            if path == "/api/calibration/axis/start":
                return self._json(202, backend.start_axis_calibration(
                    body.get("joint"), body.get("confirmations")))
            if path == "/api/calibration/axis/capture":
                session_id = body.get("session_id")
                phase = body.get("phase")
                if not isinstance(session_id, str) or not isinstance(phase, str):
                    raise PanelError(400, "REQUEST", "轴标定采样需要 session_id 与 phase")
                return self._json(202, backend.capture_axis_calibration(
                    session_id, phase, body.get("confirmations")))
            if path == "/api/calibration/axis/abort":
                return self._json(202, backend.abort_axis_calibration(body.get("reason")))
            if path == "/api/heartbeat":
                return self._json(200, backend.heartbeat())
            if path == "/api/preview":
                if "document" not in body:
                    raise PanelError(400, "PLAN_INVALID", "缺少 document")
                group = body.get("calibration_group")
                if 'execution_mode' in body and group is not None:
                    return self._json(200, backend.preview(body["document"], body['execution_mode'],
                                                          calibration_group=group))
                if 'execution_mode' in body:
                    return self._json(200, backend.preview(body["document"],body['execution_mode']))
                return self._json(200, backend.preview(body["document"]))
            if path == "/api/execute":
                ticket = body.get("ticket")
                if not isinstance(ticket, str):
                    raise PanelError(400, "REQUEST", "execute 需要 ticket")
                if not isinstance(body.get("document"), dict):
                    raise PanelError(400, "REQUEST", "execute 需要当前计划 document")
                group = body.get("calibration_group")
                if 'execution_mode' in body and group is not None:
                    return self._json(202, backend.execute(ticket, body["document"],
                                                          body.get("confirmations"),
                                                          body['execution_mode'],
                                                          calibration_group=group))
                if 'execution_mode' in body:
                    return self._json(202, backend.execute(ticket, body["document"],
                                                          body.get("confirmations"),
                                                          body['execution_mode']))
                return self._json(202, backend.execute(ticket, body["document"], body.get("confirmations")))
            if path == "/api/hold":
                return self._json(202, backend.hold(body.get("confirmations")))
            if path == '/api/gravity-float':
                return self._json(202, backend.hold(body.get('confirmations'), gravity_float=True,
                                                  float_settings=body.get('float_settings')))
            if path == '/api/gravity/lock':
                return self._json(202, backend.lock_current_pose())
            if path == '/api/gravity/float':
                return self._json(202, backend.resume_gravity_float(body.get('confirmations')))
            if path == "/api/stop":
                return self._json(202, backend.stop(body.get("reason")))
            if path == "/api/reset":
                return self._json(202, backend.reset())
            if path == "/api/lease/abandon":
                return self._json(200, backend.abandon_lease())
            if path == "/api/capture/preview":
                return self._json(200, backend.preview_supported_capture())
            if path == "/api/capture/start":
                ticket = body.get("ticket")
                if not isinstance(ticket, str):
                    raise PanelError(400, "REQUEST", "启动静态采样需要 ticket")
                return self._json(202, backend.start_supported_capture(
                    ticket, body.get("confirmations")))
            if path == "/api/capture/ack":
                operation_id = body.get("operation_id")
                if not isinstance(operation_id, str):
                    raise PanelError(400, "REQUEST", "撤手确认需要 operation_id")
                return self._json(202, backend.ack_supported_capture(
                    operation_id, body.get("confirmations")))
            if path == "/api/plans/save":
                if "document" not in body:
                    raise PanelError(400, "PLAN_INVALID", "缺少 document")
                return self._json(201, backend.save_plan(body.get("name"), body["document"]))
            if path == "/api/teaching/record/start":
                return self._json(202, backend.start_teaching_record(body.get("confirmations"),
                                                                   body.get("float_settings")))
            if path == "/api/teaching/record/stop":
                return self._json(202, backend.stop_teaching_record())
            if path == "/api/teaching/save":
                recording_id = body.get("recording_id")
                if not isinstance(recording_id, str):
                    raise PanelError(400, "REQUEST", "保存拖动示教需要 recording_id")
                return self._json(201, backend.save_teaching(recording_id, body.get("name")))
            if path == "/api/teaching/preview":
                teaching_id = body.get("teaching_id")
                if not isinstance(teaching_id, str):
                    raise PanelError(400, "REQUEST", "预览拖动示教需要 teaching_id")
                return self._json(200, backend.preview_teaching(teaching_id,
                                                               body.get("speed_factor")))
            if path == "/api/teaching/execute":
                ticket = body.get("ticket")
                if not isinstance(ticket, str):
                    raise PanelError(400, "REQUEST", "回放拖动示教需要 ticket")
                return self._json(202, backend.execute_teaching(ticket,
                                                               body.get("confirmations")))
            return self._error(404, "NOT_FOUND", "未知接口")

    return Handler


class PanelServer:
    """ThreadingHTTPServer wrapper with an explicit shutdown path."""

    def __init__(self, backend, token, static_dir, port=0):
        self._handler = make_handler(backend, token, static_dir)
        self._server = ThreadingHTTPServer(("127.0.0.1", int(port)), self._handler)
        self._server.daemon_threads = True
        self._thread = None

    @property
    def port(self):
        return self._server.server_address[1]

    def start(self):
        self._thread = threading.Thread(target=self._server.serve_forever, kwargs={"poll_interval": 0.2},
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=3)
