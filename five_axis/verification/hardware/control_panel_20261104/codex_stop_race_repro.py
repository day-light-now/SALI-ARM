"""Production HTTP/backend repro; no ROS spin, services or serial, Popen mocked."""
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch
import urllib.request

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/slaihand_bringup/scripts'))
from control_panel import PanelBackend
from control_panel_http import PanelServer

class Node:
    def create_subscription(self,*args):pass
    def create_client(self,*args):return None

with tempfile.TemporaryDirectory() as directory:
    backend=PanelBackend(Node(),directory,60,3,'review-token')
    axes={f'slaihand_joint{i}':dict(control_state=0,feedback_valid=1,position=0.,velocity=.1,effort=.2,
              driver_temperature=30,motor_temperature=25,motor_state=0,disabled_observed=0,
              command_owner=0,audit_ok=1) for i in range(1,5)}
    backend._store('dynamic',dict(at=time.monotonic(),axes=axes))
    backend._store('joints',dict(at=time.monotonic(),positions=[0.]*4))
    server=PanelServer(backend,'review-token',backend.static_dir,0);server.start()
    url=f'http://127.0.0.1:{server.port}'
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def post(path,data):
        req=urllib.request.Request(url+path,data=json.dumps(data).encode(),headers={
             'Origin':url,'X-Panel-Token':'review-token','Content-Type':'application/json'})
        try:
            with opener.open(req) as reply:return reply.status,json.load(reply)
        except urllib.error.HTTPError as error:return error.code,json.load(error)
    try:
        # Stop accepted while the next ROS callback hasn't arrived yet.
        stopped=post('/api/stop',{})
        proc=SimpleNamespace(pid=999999999,stdout=iter([]))
        with patch('control_panel.subprocess.Popen',return_value=proc) as spawn:
            held=post('/api/hold',dict(confirmations=dict(supported=True,
                clearance_and_local_power_cut=True,native_output_not_hardware_clamp=True)))
        finding=dict(stop_status=stopped[0],hold_status=held[0],spawn_count=spawn.call_count,
                     ros_callbacks_after_stop=0,hardware_actions_performed=False)
        print(json.dumps(finding,indent=2))
        Path(__file__).with_name('codex_stop_race_result.json').write_text(json.dumps(finding,indent=2))
        assert held[0]==409 and spawn.call_count==0,'Stop accepted, but cached DISARMED allowed a new client spawn'
    finally:
        server.stop()
