#!/usr/bin/env python3
"""Local-only four-axis panel. Requires the explicit OpenArm fake transport."""
import concurrent.futures
import json
import math
import os
import queue
import secrets
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

NAMES = [f"slaihand_joint{i}" for i in range(1, 5)]


def read_limits(xml):
    root = ET.fromstring(xml)
    systems = root.findall("ros2_control")
    if len(systems) != 1:
        raise ValueError("Expected one offline hardware system")
    hardware = systems[0].find("hardware")
    if hardware is None or hardware.findtext("plugin") != "slaihand_hardware/OpenArmJointFake":
        raise ValueError("Only the SLAI OpenArm offline plugin is allowed")
    if hardware.findtext("param[@name='transport']") != "fake":
        raise ValueError("Real transports are blocked")
    if [j.get("name") for j in systems[0].findall("joint")] != NAMES:
        raise ValueError("Expected the four-axis controller configuration")
    limits = []
    for name in NAMES:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get("type") != "revolute":
            raise ValueError("Expected four bounded revolute joints")
        limit = joint.find("limit")
        lo, hi = float(limit.get("lower")), float(limit.get("upper"))
        if not all(map(math.isfinite, [lo, hi])) or not lo <= 0 <= hi or lo >= hi:
            raise ValueError("Invalid joint limits")
        limits.append((lo, hi))
    return limits


def plan(kind, degrees, positions, limits):
    if len(positions) != 4 or not all(map(math.isfinite, positions)):
        raise ValueError("No valid four-axis feedback")
    if kind == "move":
        if not isinstance(degrees, list) or len(degrees) != 4:
            raise ValueError("Enter four angles in degrees")
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in degrees):
            raise ValueError("Angles must be numbers")
        targets = [[math.radians(x) for x in degrees]]
    elif kind == "zero":
        targets = [[0.0] * 4]
    elif kind == "demo":
        targets = [[0.0] * 4]
        for i in range(4):
            q = [0.0] * 4
            q[i] = math.radians(5)
            targets.extend([q, [0.0] * 4])
        targets.extend([[math.radians(x) for x in [10, 5, -5, 5]], [0.0] * 4])
    else:
        raise ValueError("Unknown command")
    result, elapsed, previous = [], 0.0, positions
    for target in targets:
        if any(not math.isfinite(q) or not lo <= q <= hi for q, (lo, hi) in zip(target, limits)):
            raise ValueError("Target exceeds the URDF joint limits")
        # Linear interpolation at <= 0.2 rad/s; longer than the fake hardware limit.
        elapsed += max(1.5, max(abs(a-b) for a, b in zip(target, previous)) / 0.2)
        result.append((target, elapsed))
        previous = target
    return result


PAGE = """<!doctype html><meta charset="utf-8"><title>SLAI 四轴离线控制</title>
<style>body{font:18px system-ui;max-width:800px;margin:40px auto;background:#f3f5f7}
section{background:white;padding:24px;border-radius:12px}label{display:block;margin:14px 0}
input{width:110px;font-size:18px}button{padding:12px;margin:6px;font-size:16px}
pre{white-space:pre-wrap} .note{color:#a33}</style>
<section><h1>SLAI 四轴离线控制</h1><p class="note">仅仿真 · 不发送实机 CAN · 角度单位：度</p>
<p>修改目标后点击“执行目标”，不会因拖动或输入立即运动。</p><div id="axes"></div>
<button class="motion" onclick="send('move')">执行目标</button>
<button class="motion" onclick="send('zero')">回仿真零位</button>
<button class="motion" onclick="send('demo')">逐轴小幅演示</button>
<button id="stop" onclick="send('cancel')">取消仿真轨迹</button>
<p>“取消”不是硬件急停。零位是模型关节零位，不是实机编码器零点。</p><pre id="status">等待反馈…</pre><p id="message"></p></section>
<script>
const token='TOKEN';let initialized=false;
for(let i=0;i<4;i++) axes.innerHTML+=`<label>J${i+1} 目标 <input id="q${i}" type="number" step="0.5" value="0"> ° <span id="s${i}"></span></label>`;
async function send(command){try{let r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':token},body:JSON.stringify({command,degrees:[0,1,2,3].map(i=>Number(document.getElementById('q'+i).value))})});let v=await r.json();if(r.ok&&command==='zero'){[0,1,2,3].forEach(i=>document.getElementById('q'+i).value='0');}document.getElementById('message').textContent=v.message;}catch(e){document.getElementById('message').textContent=String(e);}}
async function update(){try{let r=await fetch('/status');let s=await r.json();
document.getElementById('status').textContent=s.message;
document.querySelectorAll('.motion').forEach(b=>b.disabled=!s.ready||s.busy);
document.getElementById('stop').disabled=!s.busy;
if(s.limits){s.limits.forEach((p,i)=>{let x=document.getElementById('q'+i);x.min=p[0];x.max=p[1];document.getElementById('s'+i).textContent=`限位 ${p[0].toFixed(1)} ~ ${p[1].toFixed(1)}°；反馈 ${(s.positions?.[i]??0).toFixed(2)}°`;});initialized=true;}
}catch(e){document.getElementById('status').textContent='面板服务断开';document.querySelectorAll('button').forEach(b=>b.disabled=true);}}
update();setInterval(update,400);
</script>"""


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint

    rclpy.init()
    node = Node("slaihand_offline_panel")
    client = ActionClient(node, FollowJointTrajectory,
                         "/slaihand_joint_trajectory_controller/follow_joint_trajectory")
    requests = queue.Queue(maxsize=8)
    shared = {"ready": False, "busy": False, "message": "等待四轴离线模型及反馈"}
    lock = threading.Lock()
    token = secrets.token_hex(24)
    limits, positions, stamp = None, None, 0.0
    busy, handle, cancel_requested = False, None, False
    message = "等待四轴离线模型及反馈"

    def description(msg):
        nonlocal limits, message
        try:
            limits = read_limits(msg.data)
        except Exception as e:
            limits = None
            message = f"禁止控制：{e}"

    def feedback(msg):
        nonlocal positions, stamp
        values = dict(zip(msg.name, msg.position))
        if all(name in values and math.isfinite(values[name]) for name in NAMES):
            positions = [values[name] for name in NAMES]
            stamp = time.monotonic()

    def completed(future):
        nonlocal busy, handle, message
        try:
            result = future.result()
            message = f"轨迹结束：status={result.status}, code={result.result.error_code} {result.result.error_string}"
        except Exception as e:
            message = f"轨迹结果异常：{e}"
        busy, handle = False, None

    def accepted(future):
        nonlocal busy, handle, message
        try:
            handle = future.result()
            if not handle.accepted:
                busy, handle, message = False, None, "轨迹被控制器拒绝"
                return
            handle.get_result_async().add_done_callback(completed)
            if cancel_requested:
                handle.cancel_goal_async()
        except Exception as e:
            busy, handle, message = False, None, f"发送异常：{e}"

    node.create_subscription(String, "/robot_description", description,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, "/joint_states", feedback, 10)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, body, mime="application/json"):
            payload = body.encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", mime + "; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == "/":
                self.reply(200, PAGE.replace("TOKEN", token), "text/html")
            elif self.path == "/status":
                with lock:
                    body = json.dumps(shared)
                self.reply(200, body)
            else:
                self.reply(404, '{}')

        def do_POST(self):
            future = None
            if self.path != "/command" or self.headers.get("X-Panel-Token") != token:
                self.reply(403, json.dumps({"message": "拒绝请求"}))
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2048:
                    raise ValueError("Invalid request size")
                request = json.loads(self.rfile.read(size))
                if not isinstance(request, dict):
                    raise ValueError("Invalid request")
                future = concurrent.futures.Future()
                requests.put_nowait((request, future))
                reply = future.result(timeout=3)
                self.reply(200 if reply[0] else 409, json.dumps({"message": reply[1]}))
            except Exception as e:
                if future is not None:
                    future.cancel()
                self.reply(400, json.dumps({"message": str(e)}))

    server = ThreadingHTTPServer(("127.0.0.1", int(os.environ.get("SLAIHAND_PANEL_PORT", "8765"))), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"Offline panel: http://127.0.0.1:{server.server_address[1]}", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            ready = limits is not None and positions is not None and time.monotonic() - stamp < 1.0 and client.server_is_ready()
            if busy and not ready and not cancel_requested:
                cancel_requested = True
                if handle is not None:
                    handle.cancel_goal_async()
                message = "反馈/离线描述失效，已请求取消仿真轨迹"
            try:
                request, future = requests.get_nowait()
            except queue.Empty:
                request = None
            if request is not None:
                if future.cancelled():
                    continue
                try:
                    command = request.get("command")
                    if command == "cancel":
                        cancel_requested = True
                        if handle is not None:
                            handle.cancel_goal_async()
                        message = "已请求取消仿真轨迹（不是硬件急停）"
                    else:
                        if not ready or busy:
                            raise ValueError("未就绪或轨迹运行中，请等待")
                        points = plan(command, request.get("degrees"), positions, limits)
                        goal = FollowJointTrajectory.Goal()
                        goal.trajectory.joint_names = NAMES
                        for q, seconds in points:
                            point = JointTrajectoryPoint()
                            point.positions = q
                            nanoseconds = int(seconds * 1e9)
                            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 10**9)
                            goal.trajectory.points.append(point)
                        sent = client.send_goal_async(goal)
                        busy, cancel_requested = True, False
                        sent.add_done_callback(accepted)
                        message = f"运行 {command}，预计 {points[-1][1]:.1f} 秒"
                    future.set_result((True, message))
                except Exception as e:
                    future.set_result((False, str(e)))
            with lock:
                shared.update(ready=ready, busy=busy, hardware_plugin="slaihand_hardware/OpenArmJointFake" if limits is not None else None,
                              transport="fake" if limits is not None else None, joints=NAMES,
                              message=message if ready or busy else "未就绪：等待离线描述、控制器和新鲜反馈。" + message,
                              positions=[math.degrees(x) for x in positions] if positions else None,
                              limits=[[math.degrees(x) for x in pair] for pair in limits] if limits else None)
    except KeyboardInterrupt:
        pass
    finally:
        if handle is not None:
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=0.2)
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
