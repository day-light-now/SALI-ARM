"""Gravity estimate with explicitly gated four-axis fake simulation linkage. No real motors."""
from pathlib import Path
import json
import socket
import secrets
import urllib.error
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
import numpy as np
from gravity_model import GravityModel
import gravity_sim_bridge as simulation

ROOT=Path(__file__).parent
DATA=json.loads((ROOT/'gravity_preparation/provisional_gravity.json').read_text(encoding='utf-8'))
URDF=ROOT.parent/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
MODEL=GravityModel(URDF,DATA['links'],[0,0,0])
TOKEN=secrets.token_hex(24)
GRAVITY={'-x':[-9.80665,0,0],'+x':[9.80665,0,0],'-y':[0,-9.80665,0],'+y':[0,9.80665,0]}
LIMITS=[[float(j.find('limit').get(k))*180/np.pi for k in ['lower','upper']] for j in MODEL.joints]
PAGE='''<!doctype html><meta charset="utf-8"><title>SLAI 重力力矩估算</title>
<style>body{font:18px system-ui;max-width:900px;margin:35px auto;background:#eef2f6}section{padding:24px;background:white;border-radius:14px}input[type=range]{width:65%}label{display:block;margin:24px 0}strong{color:#a43}table{width:100%;border-collapse:collapse}td,th{padding:12px;text-align:left;border-bottom:1px solid #ddd}small{color:#555}button{font-size:17px;padding:10px}</style>
<section><h1>SLAI 四轴重力力矩估算</h1><strong>暂定参数 · 可联动离线仿真 · 不控制实机</strong>
<p>拖动角度查看保持该姿态所需的力矩。电机总质量来自手册，重心、质量分拆和部分结构件密度仍为估算。</p>
<p>肩部安装：J1 横放。根据底座坐标图，+Y 朝下，重力默认沿 +Y。估算力矩不会发送给驱动器。</p>
<label>重力方向 <select id="gravity"><option value="">待确认（请选择朝下轴）</option><option value="-x">底座 −X 朝下</option><option value="+x">底座 +X 朝下</option><option value="-y">底座 −Y 朝下</option><option value="+y" selected>底座 +Y 朝下（按截图）</option></select></label><div id="axes"></div>
<label>全部质量倍率 <input id="scale" type="range" min="0.7" max="1.3" step="0.01" value="1"><span id="factor">1.00</span></label>
<small>倍率仅展示参数敏感性，不代表误差范围或置信区间。</small>
<p><button id="zero">目标清零（不运动）</button><button id="execute">执行到仿真</button><button id="cancel">取消仿真轨迹</button></p><p id="sim">正在检查仿真连接…</p>
<table><thead><tr><th>关节</th><th>目标角度（°）</th><th>目标姿态力矩（N·m）</th></tr></thead><tbody id="result"></tbody></table>
<h3>仿真反馈姿态的估算力矩</h3><pre id="feedback">等待四轴 fake 反馈</pre>
<p id="msg"></p><details><summary>估算假设</summary><ul><li>电机 CAD 各部分按官方整机总重同比缩放；重心和转子/定子分拆未核实。</li><li>连杆归属沿用 CAD；未设材料的结构件沿用默认密度，ABS 按实心计算。</li><li>仅四轴；包含 4340，不包含额外负载与线缆。重力方向需用户确认。</li><li>联动通过原四轴 fake 轨迹控制器，滑条本身不会触发运动；估算力矩不会发送给驱动器。</li></ul></details></section>
<script>
const limits=LIMITS;
const token='TOKEN';let moving=false;
axes.innerHTML=limits.map((l,i)=>`<label>J${i+1} <input id="q${i}" type="range" min="${l[0]}" max="${l[1]}" step="any" value="0"><span id="angle${i}">0.0°</span></label>`).join('');
let revision=0;
async function update(){const r=++revision;const degrees=limits.map((_,i)=>Number(document.getElementById('q'+i).value));const scale=Number(document.getElementById('scale').value);degrees.forEach((v,i)=>document.getElementById('angle'+i).textContent=v.toFixed(1)+'°');factor.textContent=scale.toFixed(2);
try{const response=await fetch('/calculate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({degrees,scale,gravity:gravity.value})});const data=await response.json();if(r!==revision)return;if(!response.ok)throw Error(data.error);
result.innerHTML=data.torque.map((t,i)=>`<tr><td>J${i+1}</td><td>${degrees[i].toFixed(1)}</td><td>${t.toFixed(3)}</td></tr>`).join('');msg.textContent='目标姿态估算；不等于驱动器实际输出力矩。';}catch(e){result.innerHTML='';msg.textContent=String(e);}}
async function poll(){try{const s=await (await fetch('/sim-status')).json();if(s.error)throw Error(s.error);moving=s.busy;execute.disabled=moving;cancel.disabled=!moving;sim.textContent=moving?'仿真轨迹执行中':'四轴 fake 已连接；RViz 显示该反馈姿态';
const response=await fetch('/calculate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({degrees:s.positions,scale:Number(scale.value),gravity:gravity.value})});const t=await response.json();feedback.textContent=s.positions.map((q,i)=>`J${i+1}: ${q.toFixed(2)}°  保持力矩 ${t.torque?t.torque[i].toFixed(3)+' N·m':'待确认重力方向'}`).join(String.fromCharCode(10));}catch(e){execute.disabled=true;cancel.disabled=true;sim.textContent=String(e);feedback.textContent='无有效新鲜反馈';}}
async function send(command){execute.disabled=true;try{const r=await fetch('/sim-command',{method:'POST',headers:{'Content-Type':'application/json','X-Gravity-Token':token},body:JSON.stringify({command,degrees:limits.map((_,i)=>Number(document.getElementById('q'+i).value))})});const s=await r.json();msg.textContent=s.error||s.message;}catch(e){msg.textContent=String(e);}poll();}
execute.onclick=()=>send('move');cancel.onclick=()=>send('cancel');
document.querySelectorAll('input').forEach(x=>x.addEventListener('input',update));gravity.onchange=()=>{update();poll();};zero.onclick=()=>{limits.forEach((_,i)=>document.getElementById('q'+i).value=0);update();};update();poll();setInterval(poll,700);
</script>'''.replace('LIMITS',json.dumps(LIMITS)).replace('TOKEN',TOKEN)

def calculate(payload):
    degrees=payload['degrees']; scale=payload.get('scale',1.)
    if not isinstance(degrees,list) or len(degrees)!=4 or any(type(x) not in [int,float] for x in degrees):
        raise ValueError('Four numeric angles required')
    if type(scale) not in [int,float] or not np.isfinite(scale) or not .7<=scale<=1.3:
        raise ValueError('Mass sensitivity factor must be 0.7 to 1.3')
    direction=payload.get('gravity',DATA.get('gravity_direction'))
    if direction not in GRAVITY:
        raise ValueError('请先确认并选择底座坐标中朝下的方向；旧 −Z 假设已取消')
    model=GravityModel(URDF,DATA['links'],GRAVITY[direction])
    torque,energy=model.evaluate(np.deg2rad(degrees))
    return {'status':DATA['status'],'torque':(torque*scale).tolist(),'energy_J':energy*scale}

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*_): pass
    def reply(self,status,body,mime='application/json'):
        raw=body.encode('utf-8'); self.send_response(status)
        self.send_header('Content-Type',mime+'; charset=utf-8'); self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(raw))); self.end_headers(); self.wfile.write(raw)
    def do_GET(self):
        if self.path=='/': self.reply(200,PAGE,'text/html')
        elif self.path=='/health': self.reply(200,json.dumps({'app':'slaihand-gravity-estimate','motor_output':False}))
        elif self.path=='/sim-status':
            try:self.reply(200,json.dumps(simulation.status()))
            except (OSError,ValueError) as exc:self.reply(503,json.dumps({'error':str(exc)}))
        else:self.reply(404,'{}')
    def do_POST(self):
        try:
            if self.path not in ['/calculate','/sim-command']: self.reply(404,'{}'); return
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=2048: raise ValueError('Invalid payload size')
            if self.headers.get('Content-Type')!='application/json': raise ValueError('JSON required')
            payload=json.loads(self.rfile.read(size))
            if self.path=='/sim-command':
                if self.headers.get('X-Gravity-Token')!=TOKEN:
                    self.reply(403,json.dumps({'error':'安全令牌无效，请刷新页面'})); return
                kind=payload.get('command')
                if kind=='move':
                    # Validate finite values and limits independently of gravity selection.
                    degrees=payload.get('degrees')
                    if not isinstance(degrees,list) or len(degrees)!=4 or any(type(x) not in [int,float] for x in degrees):
                        raise ValueError('Four numeric angles required')
                    MODEL.evaluate(np.deg2rad(degrees))
                self.reply(200,json.dumps(simulation.command(kind,payload.get('degrees'))))
            else:self.reply(200,json.dumps(calculate(payload)))
        except (OSError,ValueError,KeyError,TypeError) as exc:self.reply(400,json.dumps({'error':str(exc)}))

class Server(ThreadingHTTPServer):
    allow_reuse_address=False
    def server_bind(self):
        if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
        super().server_bind()

if __name__=='__main__':
    print('Calculation only: http://127.0.0.1:8767/',flush=True)
    with Server(('127.0.0.1',8767),Handler) as server:server.serve_forever()
