"""End-to-end verification of the installed four-axis control panel.

Runs the real panel HTTP server against real ROS/JTC on an isolated domain (176)
and newly allocated PTY fake drives.  Never opens a physical port and never
touches ROS domain 0.

Each scenario gets its own fake stack, PTY and panel, so a genuine UNKNOWN (or
any other failure) is reported honestly and cannot mask the remaining required
scenarios.  Process cleanup only ever targets the Popen objects this script
created.
"""
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import deque
from pathlib import Path

DOMAIN = "176"
os.environ.update(ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY="1")
assert os.environ["ROS_DOMAIN_ID"] not in ("", "0"), "refusing ROS domain 0"

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CANUSB = ROOT / "verification" / "hardware" / "canusb_motion_20261104"
sys.path.insert(0, str(CANUSB))
sys.path.insert(0, str(ROOT / "verification" / "hardware"))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "src" / "slaihand_bringup" / "scripts"))

from verify_ros_motion import Plant  # noqa: E402
from joint_waypoint_plan import compile_plan  # noqa: E402

import rclpy  # noqa: E402
from control_msgs.msg import DynamicJointState  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

PANEL_SCRIPT = Path(os.environ.get(
    "PANEL_SCRIPT",
    "/home/xuyu/slaihand_control_panel_ws_20261104/install/slaihand_bringup/lib/slaihand_bringup/control_panel.py"))
WORKSPACE = Path(os.environ.get("PANEL_WORKSPACE", "/home/xuyu/slaihand_control_panel_ws_20261104"))
NAMES = ["slaihand_joint%d" % i for i in range(1, 5)]
BOUNDS_DEG = [[-80.0, 200.0], [-10.0, 190.0], [-90.0, 90.0], [-50.0, 90.0]]
BOUNDS = [tuple(math.radians(v) for v in pair) for pair in BOUNDS_DEG]
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))

CHECKS = []
FAILURES = []
SESSIONS = []


def check(name, condition, detail=None):
    entry = {"name": name, "passed": bool(condition)}
    if detail is not None:
        entry["detail"] = detail
    CHECKS.append(entry)
    print(("PASS  " if condition else "FAIL  ") + name
          + ("" if detail is None else "  " + json.dumps(detail, ensure_ascii=False)[:600]), flush=True)
    if not condition:
        FAILURES.append(name)
    return bool(condition)


def require(name, condition, detail=None):
    if not check(name, condition, detail):
        raise AssertionError(name)


def decode(raw):
    text = raw.decode("utf-8", "replace")
    if not text:
        return {}
    try:
        return json.loads(text)
    except ValueError:
        return text


def write_text_retry(path, text, attempts=8):
    """/mnt/d is occasionally locked by Windows for a moment; retry briefly."""
    last = None
    for _ in range(attempts):
        try:
            Path(path).write_text(text, encoding="utf-8")
            return True
        except OSError as error:
            last = error
            time.sleep(0.5)
    print("WARNING: could not write %s: %s" % (path, last), flush=True)
    fallback = Path("/tmp") / Path(path).name
    fallback.write_text(text, encoding="utf-8")
    print("WARNING: wrote fallback %s" % fallback, flush=True)
    return False


def pid_alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def pgid_alive(pgid):
    if not pgid:
        return False
    try:
        os.killpg(int(pgid), 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


SESSION_MARKER = "SLAIHAND_PANEL_TEST_SESSION"


def marked_processes(marker=None):
    """PIDs whose environment carries our test marker (this harness only)."""
    found = []
    needle = (SESSION_MARKER + "=").encode()
    exact = None if marker is None else (SESSION_MARKER + "=" + marker).encode()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            environ = (entry / "environ").read_bytes().split(b"\0")
        except OSError:
            continue
        for item in environ:
            if exact is not None and item == exact:
                found.append(int(entry.name))
                break
            if exact is None and item.startswith(needle):
                found.append(int(entry.name))
                break
    return found


def reap_marked(marker=None, timeout=6.0):
    """TERM then KILL only processes carrying our marker; returns remaining PIDs."""
    for pid in marked_processes(marker):
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = marked_processes(marker)
        if not remaining:
            return []
        time.sleep(0.1)
    for pid in marked_processes(marker):
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        remaining = marked_processes(marker)
        if not remaining:
            return []
        time.sleep(0.1)
    return marked_processes(marker)


def domain_description_publishers(observer):
    try:
        return observer.node.get_publishers_info_by_topic("/slaihand/robot_description")
    except Exception:
        return []


def wait_domain_clean(observer, timeout=12.0):
    """A stale transient-local description would make a new driver open a dead PTY."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not domain_description_publishers(observer):
            return True
        reap_marked(None)
        time.sleep(0.2)
    return False


class Observer:
    def __init__(self):
        self.node = rclpy.create_node("panel_verification_observer")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.dynamic = None
        self.joints = None
        self.node.create_subscription(DynamicJointState, "/slaihand/dynamic_joint_states", self._dynamic, qos)
        self.node.create_subscription(JointState, "/slaihand/joint_states", self._joints, qos)

    def _dynamic(self, msg):
        self.dynamic = {name: dict(zip(values.interface_names, values.values))
                        for name, values in zip(msg.joint_names, msg.interface_values)}

    def _joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def reset(self):
        self.dynamic = None
        self.joints = None

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate(self):
                return True
        return False

    def axis(self, index, key):
        if not self.dynamic:
            return None
        return self.dynamic.get(NAMES[index], {}).get(key)

    def stage(self):
        if not self.dynamic:
            return None
        values = [self.axis(i, "control_state") for i in range(4)]
        if any(value is None for value in values):
            return None
        return int(values[0]) if len(set(int(value) for value in values)) == 1 else -1

    def disabled(self):
        return all(self.axis(i, "disabled_observed") == 1 for i in range(4))

    def close(self):
        self.node.destroy_node()


class Panel:
    URL = re.compile(r"http://127\.0\.0\.1:(\d+)/\?token=([A-Za-z0-9_\-]+)")

    def __init__(self, archive, ttl=5, port=0, domain=DOMAIN, heartbeat_timeout=3, marker=None):
        self.archive = Path(archive)
        self.archive.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(domain)
        if marker:
            env[SESSION_MARKER] = marker
        self.proc = subprocess.Popen(
            [sys.executable, str(PANEL_SCRIPT), "--port", str(port), "--archive-dir", str(self.archive),
             "--preview-ttl-seconds", str(ttl), "--heartbeat-timeout-seconds", str(heartbeat_timeout)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            encoding="utf-8", errors="replace", start_new_session=True, env=env)
        self.lines = deque(maxlen=600)
        self.port = None
        self.token = None
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            for line in list(self.lines):
                match = self.URL.search(line)
                if match:
                    self.port = int(match.group(1))
                    self.token = match.group(2)
                    break
            if self.port:
                break
            if self.proc.poll() is not None:
                break
            time.sleep(0.05)
        if not self.port:
            raise AssertionError("panel did not start:\n" + "\n".join(self.lines))

    def _read(self):
        try:
            for line in self.proc.stdout:
                self.lines.append(line.rstrip("\n"))
        except Exception:
            pass

    def _close_pipe(self):
        """Close stdout explicitly; a later GC-time close could hit a reused fd."""
        try:
            self._reader.join(timeout=2)
        except Exception:
            pass
        try:
            if self.proc.stdout is not None:
                self.proc.stdout.close()
        except Exception:
            pass

    @property
    def base(self):
        return "http://127.0.0.1:%d" % self.port

    @property
    def origin(self):
        return self.base

    def call(self, method, path, body=None, token=True, origin=True, headers=None, timeout=15):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        request = urllib.request.Request(self.base + path, data=data, method=method)
        if token:
            request.add_header("X-Panel-Token", self.token)
        if origin:
            request.add_header("Origin", self.origin)
        if data is not None:
            request.add_header("Content-Type", "application/json")
        for key, value in (headers or {}).items():
            request.add_header(key, value)
        try:
            with OPENER.open(request, timeout=timeout) as response:
                return response.status, decode(response.read())
        except urllib.error.HTTPError as error:
            return error.code, decode(error.read())

    def status(self):
        code, body = self.call("GET", "/api/status")
        if code != 200:
            raise AssertionError("status failed: %s %s" % (code, body))
        return body

    def wait_status(self, predicate, timeout=15):
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            code, body = self.call("GET", "/api/status")
            last = body
            if code == 200 and predicate(body):
                return body
            time.sleep(0.2)
        raise AssertionError("panel status never matched: %s" % json.dumps(last, ensure_ascii=False)[:500])

    def heartbeat(self):
        return self.call("POST", "/api/heartbeat", {})

    def preview(self, document, **kwargs):
        return self.call("POST", "/api/preview", {"document": document}, **kwargs)

    def execute(self, ticket, document, confirmations, **kwargs):
        return self.call("POST", "/api/execute",
                         {"ticket": ticket, "document": document, "confirmations": confirmations}, **kwargs)

    def stop(self, reason="操作员全轴停止"):
        return self.call("POST", "/api/stop", {"reason": reason})

    def reset(self):
        return self.call("POST", "/api/reset", {})

    def wait_operation(self, observe, timeout=40):
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            code, body = self.call("GET", "/api/status")
            last = body
            operation = body.get("operation") or {}
            observe.spin(0.1)
            if operation and not operation.get("running") and operation.get("phase") not in (
                    "starting", "running", "stopping"):
                return operation
        raise AssertionError("operation did not finish in %ss; last phase=%s"
                             % (timeout, (last or {}).get("operation", {}).get("phase")))

    def terminate(self, timeout=15):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            code = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
            code = self.proc.wait(timeout=5)
        self._close_pipe()
        return code

    def kill(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGKILL)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        self._close_pipe()


class Heartbeat(threading.Thread):
    def __init__(self, panel, interval=1.0):
        super().__init__(daemon=True)
        self.panel = panel
        self.interval = interval
        self.stopped = threading.Event()

    def run(self):
        while not self.stopped.is_set():
            try:
                self.panel.heartbeat()
            except Exception:
                pass
            self.stopped.wait(self.interval)

    def stop(self):
        self.stopped.set()


class Session:
    """A fresh PTY + fake stack + panel; fully torn down before the next one."""

    def __init__(self, name, work_root, observer, ttl=5, domain=DOMAIN, stack_args=None):
        self.name = name
        self.observer = observer
        self.dir = Path(work_root) / name
        self.dir.mkdir(parents=True, exist_ok=True)
        self.plant = None
        self.stack = None
        self.stack_log = None
        self.stack_log_path = None
        self.stack_pgid = None
        self.panel = None
        self.heartbeat = None
        self._closed = False
        try:
            self._start(ttl, domain, list(stack_args or []))
        except BaseException:
            try:
                self.close()
            except Exception:
                pass
            raise

    def _start(self, ttl, domain, stack_args):
        observer = self.observer
        if not wait_domain_clean(observer):
            raise AssertionError("stale /slaihand/robot_description publisher still on domain %s "
                                 "(would configure a new driver with a dead PTY)" % DOMAIN)
        observer.reset()
        self.plant = Plant()
        if not self.plant.path.startswith("/dev/pts/"):
            raise AssertionError("fake drive is not a fresh PTY: %s" % self.plant.path)
        if not os.path.exists(self.plant.path):
            raise AssertionError("fake PTY vanished before launch: %s" % self.plant.path)
        self.stack_log_path = self.dir / "stack.log"
        self.stack_log = self.stack_log_path.open("w", encoding="utf-8")
        self.marker = uuid.uuid4().hex
        env = dict(os.environ)
        env[SESSION_MARKER] = self.marker
        self.stack = subprocess.Popen(
            ["ros2", "launch", "slaihand_bringup", "slaihand_canusb_motion.launch.py",
             "port:=" + self.plant.path, "log_directory:=" + str(self.dir / "audit")] + stack_args,
            stdout=self.stack_log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        self.stack_pgid = os.getpgid(self.stack.pid)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            text = self.stack_log_path.read_text(encoding="utf-8", errors="replace")
            if "Successful 'activate' of hardware" in text:
                break
            if self.stack.poll() is not None:
                raise AssertionError("fake stack died (pty=%s):\n%s" % (self.plant.path, text[-1500:]))
            if not os.path.exists(self.plant.path):
                raise AssertionError("fake PTY %s disappeared during startup" % self.plant.path)
            time.sleep(0.2)
        else:
            raise AssertionError("fake stack not ready (pty=%s):\n%s"
                                 % (self.plant.path,
                                    self.stack_log_path.read_text(errors="replace")[-1500:]))
        if not observer.wait(lambda item: item.stage() == 0
                             and all(item.axis(i, "feedback_valid") == 1 for i in range(4)), 20):
            raise AssertionError("stack feedback not ready (stage=%s)" % observer.stage())
        self.panel = Panel(self.dir / "archive", ttl=ttl, domain=domain, marker=self.marker)
        self.panel.wait_status(lambda body: body["ros"]["connected"] is True, timeout=20)
        self.heartbeat = Heartbeat(self.panel)
        self.heartbeat.start()
        SESSIONS.append({"name": self.name, "pty": self.plant.path, "dir": str(self.dir),
                         "marker": self.marker})

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.heartbeat is not None:
            self.heartbeat.stop()
        if self.panel is not None:
            try:
                self.panel.terminate(timeout=15)
            except Exception:
                self.panel.kill()
        if self.stack is not None:
            if self.stack.poll() is None:
                self.stack.send_signal(signal.SIGINT)
                try:
                    self.stack.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    pass
            if self.stack_pgid:
                # ros2 launch children can get their own process group and keep a
                # stale transient-local robot_description alive.  Kill the saved
                # group and, above all, every process carrying this session marker
                # (never a command-line pattern).
                try:
                    os.killpg(self.stack_pgid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError, OSError):
                    pass
                remaining = reap_marked(getattr(self, "marker", None))
                if remaining:
                    raise AssertionError("session marker processes survived: %s" % remaining)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and pgid_alive(self.stack_pgid):
                    time.sleep(0.1)
            try:
                self.stack.wait(timeout=2)
            except Exception:
                pass
        if self.stack_log is not None:
            try:
                self.stack_log.close()
            except Exception:
                pass
        if self.plant is not None:
            self.plant.close()
        time.sleep(0.3)  # let devpts entries and fds settle before the next session


def scenario(name, work, observer, function, **kwargs):
    session = None
    last_error = None
    for attempt in (1, 2):
        try:
            session = Session(name, work, observer, **kwargs)
            break
        except Exception as error:  # noqa: BLE001 - fixture startup is environmental
            last_error = error
            if attempt == 1:
                check("场景 %s 首次夹具启动失败，重建后重试" % name, True,
                      {"attempt": attempt, "error": str(error)[:500]})
                time.sleep(1.5)
    if session is None:
        check("场景 %s 启动失败：%s" % (name, last_error), False)
        return
    try:
        function(session)
    except AssertionError as error:
        check("场景 %s 中断：%s" % (name, error), False)
    except Exception as error:  # noqa: BLE001 - a broken scenario must not stop the rest
        check("场景 %s 异常：%s" % (name, error), False)
    finally:
        try:
            session.close()
        except Exception as error:  # noqa: BLE001
            check("场景 %s 清理异常：%s" % (name, error), False)


def confirmations(**overrides):
    result = {"supported": True, "clearance_and_local_power_cut": True,
              "native_output_not_hardware_clamp": True}
    result.update(overrides)
    return result


def degrees_of(status):
    values = [axis.get("actual_deg") for axis in status["axes"]]
    if any(value is None for value in values):
        raise AssertionError("missing feedback in status")
    return values


def plan(mode, waypoints):
    return {"coordinates": mode,
            "waypoints": [{"degrees": list(degrees), "move_s": move, "dwell_s": dwell}
                          for degrees, move, dwell in waypoints]}


def reset_when_disabled(panel, observer, attempts=8):
    """Retry only while the driver is visibly disabled; returns (accepted, body)."""
    last = None
    for _ in range(attempts):
        code, body = panel.reset()
        last = body
        if code == 202:
            return True, body
        if body.get("code") == "NOT_DISABLED" and observer.stage() != 8:
            break
        time.sleep(0.3)
    return False, last


def scenario_core(session):
    panel, observer, plant = session.panel, session.observer, session.plant
    status = panel.status()
    require("面板状态接入真实 ROS", status["ros"]["connected"] is True)
    require("控制阶段为未使能（0）", status["ros"]["stage"] == 0, status["ros"]["stage"])
    require("四轴状态齐全", len(status["axes"]) == 4)
    require("URDF 完整限位", status["ros"]["limits_deg"] == BOUNDS_DEG, status["ros"]["limits_deg"])
    require("实际角度来自反馈", max(abs(a - b) for a, b in zip(degrees_of(status), plant.initial)) < 1.0,
            degrees_of(status))
    require("没有目标时不伪造目标", all(axis["target_deg"] is None for axis in status["axes"]))
    require("受保护客户端已安装", str(status["ros"]["client_script"]).endswith("commission_supported_trial.py")
            and bool(status["ros"]["client_sha256"]))
    require("停止门禁初始未锁定", status["stop_gate"]["locked"] is False, status["stop_gate"])

    # -- transport / auth against the real server ----------------------
    code, _ = panel.call("GET", "/", token=False)
    require("页面无 token 被拒绝", code == 403, code)
    code, _ = panel.call("GET", "/api/status", token=False)
    require("API 无 token 被拒绝", code == 403, code)
    code, _ = panel.call("POST", "/api/stop", {}, origin=False)
    require("POST 缺 Origin 被拒绝", code == 403, code)
    code, _ = panel.call("POST", "/api/stop", {}, headers={"Origin": "http://evil.example"})
    require("POST 跨源被拒绝", code == 403, code)
    code, page = panel.call("GET", "/?token=" + panel.token, token=False)
    require("页面可打开且含停止入口", code == 200 and isinstance(page, str)
            and "全轴停止" in page and panel.token in page, code)
    require("打开页面不使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0,
            {"fc": plant.fc, "mit": plant.mit})

    # -- preview / import are data only --------------------------------
    start = [math.radians(v) for v in degrees_of(status)]
    single = plan("relative_start_deg", [([2, 1, -1, 2], 4, 1)])
    code, body = panel.preview(single)
    require("单点预览通过", code == 200 and body["accepted"] and body["completed"], body.get("message"))
    expected = [round(math.degrees(start[i] + math.radians(v)), 3) for i, v in enumerate([2, 1, -1, 2])]
    require("单点冻结为绝对角度", body["preview"]["waypoints"][0]["target_deg"] == expected,
            {"got": body["preview"]["waypoints"][0]["target_deg"], "expected": expected})
    require("预览不使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)

    motion_delta = [10, 5, -5, 10] if os.environ.get('SLAIHAND_TEST_SPEED01') == '1' else [2, 1, -1, 2]
    if os.environ.get('SLAIHAND_TEST_SPEED05') == '1':
        motion_delta = [10, 5, -5, 10]
    leg_s = .8 if os.environ.get('SLAIHAND_TEST_SPEED05') == '1' else 4
    multi = plan("relative_start_deg", [(motion_delta, leg_s, 1), ([0, 0, 0, 0], leg_s, 1)])
    code, body = panel.preview(multi)
    require("多点预览通过", code == 200)
    points, windows = compile_plan(multi, start, BOUNDS)
    require("预览与生产规划器结果一致",
            all(abs(float(a) - float(b)) < 1e-6 for row, window in
                zip(body["preview"]["waypoints"], windows)
                for a, b in zip(row["target_deg"], [round(math.degrees(q), 3) for q in window["target_rad"]])),
            body["preview"]["waypoints"])
    require("多点预览仍不使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)

    import_document = plan("absolute_deg", [([round(math.degrees(v), 3) for v in start], 2, 1)])
    code, body = panel.call("POST", "/api/plans/save", {"name": "e2e", "document": import_document})
    require("计划存档保存", code == 201 and body["plan"]["id"], body)
    plan_id = body["plan"]["id"]
    code, body = panel.call("GET", "/api/plans")
    require("存档列表含新计划", any(item["id"] == plan_id for item in body["plans"]))
    code, body = panel.call("GET", "/api/plans/" + plan_id)
    require("存档载入为数据", code == 200 and body["document"]["coordinates"] == "absolute_deg")
    code, _ = panel.preview(body["document"])
    require("载入后重放预览通过", code == 200)
    require("存档/载入不使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)

    bad_cases = [
        ("超出 URDF 限位", plan("relative_start_deg", [([201, 0, 0, 0], 4, 1)]), "路点超出完整URDF限位"),
        ("峰值超速", plan("relative_start_deg", [([10, 0, 0, 0], .2, 1)]), "峰值速度超过1rad/s"),
        ("13 个路点", plan("relative_start_deg", [([0, 0, 0, 0], 4, 1)] * 13), "需要1～12个路点"),
        ("停留过短", plan("relative_start_deg", [([0, 0, 0, 0], 4, 0)]), "dwell_s至少0.5秒"),
        ("未知键", {"coordinates": "relative_start_deg",
                    "waypoints": [{"degrees": [0, 0, 0, 0], "move_s": 4, "dwell_s": 1, "run": "x"}]},
         "必须且仅包含"),
        ("含糊坐标模式", plan("relative", [([0, 0, 0, 0], 4, 1)]), "coordinates 必须是"),
    ]
    for name, document_in, message in bad_cases:
        code, body = panel.preview(document_in)
        require("拒绝：" + name, code == 400 and body.get("code") == "PLAN_INVALID"
                and message in (body.get("message") or ""), body)
    require("被拒预览不使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)

    # -- ticket / authorisation ----------------------------------------
    code, body = panel.preview(single)
    require("票根场景预览", code == 200 and body.get("ticket"), body)
    ticket = body["ticket"]["id"]
    code, body = panel.execute(ticket, multi, {})
    require("无确认执行被拒绝", code == 403 and body.get("code") == "CONFIRMATION_REQUIRED", body)
    code, body = panel.execute(ticket, multi, {"supported": True})
    require("确认不全被拒绝", code == 403 and body.get("code") == "CONFIRMATION_REQUIRED", body)
    code, body = panel.execute(ticket, multi, confirmations())
    require("错误文档被执行拒绝", code == 409 and body.get("code") == "PLAN_CHANGED", body)
    require("被拒的执行没有使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)
    code, body = panel.execute("0" * 32, multi, confirmations())
    require("未知票根被拒绝", code == 409 and body.get("code") == "TICKET_UNKNOWN", body)
    code, body = panel.call("POST", "/api/hold", {"confirmations": {}})
    require("无确认保持被拒绝", code == 403, body)
    require("被拒绝的请求没有使能", plant.fc == [0, 0, 0, 0] and plant.mit == 0, plant.fc)

    code, body = panel.preview(multi)
    expired_ticket = body["ticket"]["id"]
    time.sleep(6.0)
    code, body = panel.execute(expired_ticket, multi, confirmations())
    require("过期票根被拒绝", code == 409 and body.get("code") == "TICKET_EXPIRED", body)
    require("过期票根不使能", plant.fc == [0, 0, 0, 0], plant.fc)

    # -- real execution -------------------------------------------------
    code, body = panel.preview(multi)
    require("执行前预览", code == 200)
    ticket = body["ticket"]["id"]
    panel.heartbeat()
    code, body = panel.execute(ticket, multi, confirmations())
    require("执行受理返回 202/accepted", code == 202 and body["accepted"] and not body["completed"], body)
    code, body = panel.execute(ticket, multi, confirmations())
    require("并发执行被拒绝", code == 409 and body.get("code") == "BUSY", body)
    operation = panel.wait_operation(observer, timeout=40)
    project = operation.get("result") or {}
    require("执行客户端正常结束", operation["exit_code"] == 0, operation.get("stdout_tail"))
    require("逐点精度通过", project.get("accuracy_passed") is True, project.get("waypoint_results"))
    require("逐点观测成立", project.get("waypoints_observed") is True)
    require("动作成功标记", project.get("action_passed") is True and project.get("trajectory_error_code") == 0)
    require("新失能确认", project.get("stop_observations_confirmed") is True)
    require("两点判据齐全", len(project.get("waypoint_results") or []) == 2)
    require("J3 独立负向行程被观测", project["observed_delta_range_deg"][2][0] < -0.8,
            project.get("observed_delta_range_deg"))
    if os.environ.get('SLAIHAND_TEST_SPEED01') == '1' or os.environ.get('SLAIHAND_TEST_SPEED05') == '1':
        require("新速度档真实JTC完整行程",project['observed_delta_range_deg'][0][1]>9.5
                and project['observed_delta_range_deg'][2][0]<-4.5,project['observed_delta_range_deg'])
    require("仅本操作使能一次", plant.fc == [1, 1, 1, 1] and plant.mit > 0, plant.fc)
    require("四轴均收到新失能帧", all(value >= 1 for value in plant.fd), plant.fd)
    last_tau = getattr(plant, "last_tau", [0.0] * 4)
    require("α=0 时前馈帧仅为协议量化半步",
            all(abs(value) <= limit for value, limit in zip(last_tau, [0.05, 0.05, 0.014, 0.014])),
            last_tau)
    require("PTY 无编码/路由错误", not plant.errors, plant.errors[:3])
    status = panel.status()
    require("执行后面板看到已失能", status["ros"]["stage"] == 8 and status["ros"]["disabled_observed"] is True,
            status["ros"]["stage"])
    require("失能后驱动不再报告活动前馈",
            status["gravity_shadow"]["driver"].get("active") is False,
            status["gravity_shadow"]["driver"])
    code, body = panel.execute(ticket, multi, confirmations())
    require("票根不可重用", code == 409 and body.get("code") in ("TICKET_USED", "NOT_DISARMED"), body)
    code, body = panel.preview(multi)
    require("已失能时拒绝新预览", code == 409 and body.get("code") == "NOT_DISARMED", body)

    require("失能后可请求软件复位", reset_when_disabled(panel, observer)[0])
    require("复位后回到未使能", observer.wait(lambda obs: obs.stage() == 0, 8), observer.stage())
    require("复位不使能", plant.fc == [1, 1, 1, 1], plant.fc)
    code, body = panel.reset()
    require("未使能时复位被拒绝", code == 409 and body.get("code") == "NOT_DISABLED", body)
    require("普通复位不解锁（本无停止）", panel.status()["stop_gate"]["locked"] is False)

    # -- finite hold ----------------------------------------------------
    before = list(plant.fc)
    code, body = panel.call("POST", "/api/hold", {"confirmations": confirmations()})
    require("有限保持受理", code == 202 and body["accepted"] and not body["completed"], body)
    operation = panel.wait_operation(observer, timeout=20)
    project = operation.get("result") or {}
    require("保持模式与结果", project.get("mode") == "current_pose_hold" and operation["exit_code"] == 0, project)
    require("保持也确认新失能", project.get("stop_observations_confirmed") is True)
    require("保持不冒充精度通过", project.get("accuracy_passed") is None and project.get("overall_ok") is True)
    require("保持只使能一次", plant.fc == [value + 1 for value in before], plant.fc)
    require("保持后已失能", observer.wait(lambda obs: obs.stage() == 8, 4), observer.stage())
    require("保持后复位", reset_when_disabled(panel, observer)[0])
    require("保持后回到未使能", observer.wait(lambda obs: obs.stage() == 0, 8), observer.stage())

    # -- JTC following error is preserved -------------------------------
    failure_plan = plan("relative_start_deg", [(motion_delta, 4, 1), ([0, 0, 0, 0], 4, 1)])
    code, body = panel.preview(failure_plan)
    ticket = body["ticket"]["id"]
    panel.heartbeat()
    code, body = panel.execute(ticket, failure_plan, confirmations())
    require("偏差场景执行受理", code == 202, body)
    time.sleep(5.0)
    plant.tracking_bias = math.radians(0.6)
    operation = panel.wait_operation(observer, timeout=30)
    project = operation.get("result") or {}
    require("跟随偏差导致动作失败", operation["exit_code"] != 0 and operation["phase"] == "failed",
            {"exit": operation["exit_code"], "phase": operation["phase"]})
    require("保留原始错误码 -5", project.get("trajectory_error_code") == -5,
            project.get("trajectory_error_code"))
    require("保留原始错误消息", bool(project.get("trajectory_error_string")),
            project.get("trajectory_error_string"))
    require("逐点未通过不显示成功", project.get("accuracy_passed") is False
            and project.get("waypoints_observed") is False and project.get("overall_ok") is False, project)
    require("失败也确认新失能", project.get("stop_observations_confirmed") is True)
    check_state = panel.status()["operation"]["result"]
    require("GUI/HTTP 状态同样保留失败", check_state.get("trajectory_error_code") == -5
            and check_state.get("accuracy_passed") is False)
    plant.tracking_bias = 0.0
    plant.applied_bias = [0.0] * 4
    require("失败后已失能", observer.wait(lambda obs: obs.stage() == 8, 5), observer.stage())
    require("失败后复位", reset_when_disabled(panel, observer)[0])
    require("失败后回到未使能", observer.wait(lambda obs: obs.stage() == 0, 8), observer.stage())


def scenario_stop(session):
    panel, observer, plant = session.panel, session.observer, session.plant
    long_plan = plan("relative_start_deg", [([3, 2, -2, 3], 8, 1)])
    code, body = panel.preview(long_plan)
    require("停止场景预览", code == 200, body)
    ticket = body["ticket"]["id"]
    panel.heartbeat()
    code, body = panel.execute(ticket, long_plan, confirmations())
    require("停止场景执行受理", code == 202, body)
    time.sleep(2.5)
    fd_before = list(plant.fd)
    fc_at_stop = list(plant.fc)
    code, body = panel.stop("端到端测试停止")
    require("停止请求立即受理", code == 202 and body["accepted"] and not body["completed"]
            and body["phase"] == "stop_requested", body)
    require("停止受理即锁定门禁", body["stop_gate"]["locked"] is True, body["stop_gate"])
    # Codex P1 counterexample over real HTTP: the cached DISARMED snapshot must
    # not allow a new preview/hold/execute immediately after stop acceptance.
    code, body = panel.execute(ticket, long_plan, confirmations())
    require("停止后立即执行被门禁拒绝", code == 409 and body.get("code") == "STOP_LOCKED", body)
    code, body = panel.call("POST", "/api/hold", {"confirmations": confirmations()})
    require("停止后立即保持被门禁拒绝", code == 409 and body.get("code") == "STOP_LOCKED", body)
    code, body = panel.preview(long_plan)
    require("停止后立即预览被门禁拒绝", code == 409 and body.get("code") == "STOP_LOCKED", body)

    operation = panel.wait_operation(observer, timeout=15)
    require("停止后客户端终止", operation["phase"] == "stopped" and bool(operation.get("stop_reason")),
            {"phase": operation["phase"], "stop_reason": operation.get("stop_reason")})
    require("停止后收到新失能帧", all(a > b for a, b in zip(plant.fd, fd_before)), plant.fd)
    time.sleep(1.5)
    require("停止后没有迟到使能", plant.fc == fc_at_stop, {"before": fc_at_stop, "after": plant.fc})
    status = panel.wait_status(lambda body: body["stop"]["state"] in ("confirmed", "unknown"), timeout=6)
    state = status["stop"]["state"]
    check("停止状态为已确认或明确未知（不冒充）", state in ("confirmed", "unknown"), status["stop"])
    require("停止记录保留操作员原因", status["stop"]["reason"] == "端到端测试停止", status["stop"])
    require("停止后门禁保持锁定", status["stop_gate"]["locked"] is True, status["stop_gate"])

    if state == "confirmed" and observer.stage() == 8:
        require("停止后手动复位受理", reset_when_disabled(panel, observer)[0])
        # The unlock must follow a fresh, consistent DISARMED observation rather
        # than the reset reply alone; the deterministic proof is in
        # test_control_panel_callbacks.py.
        status = panel.wait_status(lambda body: body["stop_gate"]["locked"] is False, timeout=8)
        require("复位并观测新 DISARMED 后解锁", status["stop_gate"]["locked"] is False
                and status["stop_gate"]["unlocked_at"] is not None, status["stop_gate"])
        require("解锁后回到未使能", observer.wait(lambda obs: obs.stage() == 0, 8), observer.stage())
        code, body = panel.preview(long_plan)
        require("解锁后可重新预览", code == 200, body)
    else:
        check("停止进入 UNKNOWN：保持锁定，不冒充已确认", observer.stage() == 9, {"stage": observer.stage()})
        code, body = panel.reset()
        require("UNKNOWN 时复位被拒绝", code == 409 and body.get("code") == "NOT_DISABLED", body)
        code, body = panel.preview(long_plan)
        require("UNKNOWN 时预览仍锁定", code == 409 and body.get("code") == "STOP_LOCKED", body)
        require("UNKNOWN 状态如实上报", panel.status()["stop"]["state"] == "unknown",
                panel.status()["stop"])


def scenario_lease(session):
    panel, observer, plant = session.panel, session.observer, session.plant
    long_plan = plan("relative_start_deg", [([3, 2, -2, 3], 8, 1)])
    code, body = panel.preview(long_plan)
    require("心跳场景预览", code == 200, body)
    ticket = body["ticket"]["id"]
    panel.heartbeat()
    session.heartbeat.stop()
    code, body = panel.execute(ticket, long_plan, confirmations())
    require("心跳丢失场景执行受理", code == 202, body)
    operation = panel.wait_operation(observer, timeout=15)
    require("心跳丢失触发有界停止", operation.get("lease_expired") is True, operation.get("lease_expired"))
    require("心跳丢失场景客户端终止", operation["phase"] == "stopped", operation["phase"])
    require("心跳丢失后确认或明确未知",
            observer.wait(lambda obs: obs.disabled() or obs.stage() == 9, 6),
            {"disabled": observer.disabled(), "stage": observer.stage()})
    status = panel.wait_status(lambda body: body["stop"]["state"] in ("confirmed", "unknown"), timeout=6)
    require("心跳停止同样锁定门禁", status["stop_gate"]["locked"] is True, status["stop_gate"])


def scenario_exit(session):
    panel, observer, plant = session.panel, session.observer, session.plant
    long_plan = plan("relative_start_deg", [([3, 2, -2, 3], 8, 1)])
    code, body = panel.preview(long_plan)
    require("退出场景预览", code == 200, body)
    panel.heartbeat()
    code, body = panel.execute(body["ticket"]["id"], long_plan, confirmations())
    require("退出场景执行受理", code == 202, body)
    time.sleep(2.0)
    operation = panel.status()["operation"] or {}
    child_pid = operation.get("pid")
    require("退出场景记录子进程 PID", isinstance(child_pid, int) and child_pid > 0, operation.get("pid"))
    started = time.monotonic()
    code = panel.terminate(timeout=15)
    require("面板正常退出返回 0", code == 0, {"exit": code, "seconds": time.monotonic() - started})
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and pid_alive(child_pid):
        time.sleep(0.1)
    require("面板退出后在途客户端已终止", not pid_alive(child_pid), child_pid)
    require("面板退出请求停止并确认", observer.wait(lambda obs: obs.disabled() or obs.stage() == 9, 6),
            {"disabled": observer.disabled(), "stage": observer.stage()})
    session_file = sorted(panel.archive.glob("session_*.json"))
    require("面板退出写出会话记录", bool(session_file), [str(path) for path in session_file])


def scenario_stale(session):
    panel, observer, plant = session.panel, session.observer, session.plant
    os.killpg(session.stack.pid, signal.SIGSTOP)
    try:
        time.sleep(1.2)
        code, body = panel.call("GET", "/api/status")
        require("冻结驱动后反馈陈旧", body["ros"]["connected"] is False
                and body["ros"]["dynamic_age_s"] > 0.5, body["ros"]["dynamic_age_s"])
        require("陈旧反馈不显示 0", all(axis["actual_deg"] is None for axis in body["axes"]))
        code, body = panel.preview(plan("relative_start_deg", [([0, 0, 0, 0], 2, 1)]))
        require("陈旧反馈拒绝预览", code == 409 and body.get("code") == "FEEDBACK_STALE", body)
    finally:
        os.killpg(session.stack.pid, signal.SIGCONT)
    time.sleep(1.0)


def scenario_empty(work):
    panel = Panel(work / "archive-empty", ttl=5, domain="177")
    try:
        body = panel.wait_status(lambda item: item["ros"]["connected"] is False, timeout=8)
        require("无驱动域：状态失联", body["ros"]["stage"] is None and body["ros"]["connected"] is False)
        require("无驱动域：不用 0 填充",
                all(axis["actual_deg"] is None and axis["target_deg"] is None and axis["velocity_deg_s"] is None
                    for axis in body["axes"]), body["axes"][0])
        code, body = panel.preview(plan("relative_start_deg", [([0, 0, 0, 0], 2, 1)]))
        require("无驱动域：预览被拒绝", code == 409 and body.get("code") == "FEEDBACK_STALE", body)
        code, body = panel.execute("0" * 32, plan("relative_start_deg", [([0, 0, 0, 0], 2, 1)]),
                                   confirmations())
        require("无驱动域：执行被拒绝", code == 409 and body.get("code") == "FEEDBACK_STALE", body)
        require("无驱动域退出正常", panel.terminate(timeout=10) == 0)
    finally:
        panel.kill()


def main():
    assert PANEL_SCRIPT.is_file(), PANEL_SCRIPT
    assert WORKSPACE.is_dir(), WORKSPACE
    work = Path("/tmp") / ("panel-verify-" + time.strftime("%Y%m%d_%H%M%S"))
    work.mkdir(parents=True, exist_ok=True)
    result = {"software_only": True, "physical_port_opened": False, "ros_domain_id": DOMAIN,
              "panel_script": str(PANEL_SCRIPT), "workspace": str(WORKSPACE),
              "started_at": time.time(), "work": str(work), "sessions": SESSIONS}
    observer = None
    try:
        rclpy.init()
        observer = Observer()
        scenario("core", work, observer, scenario_core)
        scenario("stop", work, observer, scenario_stop)
        scenario("lease", work, observer, scenario_lease)
        scenario("exit", work, observer, scenario_exit)
        scenario("stale", work, observer, scenario_stale)
        scenario_empty(work)
    finally:
        if observer is not None:
            observer.close()
        if rclpy.ok():
            rclpy.shutdown()
        result["checks"] = CHECKS
        result["failures"] = FAILURES
        result["status"] = "PASS" if CHECKS and not FAILURES else "FAIL"
        result["finished_at"] = time.time()
        write_text_retry(HERE / "panel_result.json",
                         json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        try:
            evidence = HERE / "evidence"
            evidence.mkdir(exist_ok=True)
            for path in sorted(work.glob("*/archive/session_*.json")):
                target = evidence / ("session_%s_%s" % (path.parent.parent.name, path.name))
                shutil.copy2(path, target)
            operations = sorted(work.glob("core/archive/operations/*"))
            if operations:
                target = evidence / "operation"
                target.mkdir(exist_ok=True)
                for name in ("plan.json", "source.json", "panel.json", "ticket.json"):
                    source = operations[-1] / name
                    if source.is_file():
                        shutil.copy2(source, target / name)
                for path in sorted((operations[-1] / "client").glob("*.json")):
                    shutil.copy2(path, target / "client_result.json")
            stack_log = work / "stop" / "stack.log"
            if stack_log.is_file():
                shutil.copy2(stack_log, HERE / "panel_ros.log")
        except OSError as error:
            print("WARNING: evidence copy failed: %s" % error, flush=True)


if __name__ == "__main__":
    main()
    outcome = json.loads((HERE / "panel_result.json").read_text(encoding="utf-8"))
    print(json.dumps({"status": outcome.get("status"), "checks": len(outcome.get("checks", [])),
                      "failures": outcome.get("failures"), "sessions": outcome.get("sessions")},
                     ensure_ascii=False))
    raise SystemExit(0 if outcome.get("status") == "PASS" else 1)
