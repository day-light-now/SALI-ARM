"""Restricted client for the existing loopback, verified four-axis fake panel."""
import json
import re
import urllib.request

BASE='http://127.0.0.1:8766'
NAMES=[f'slaihand_joint{i}' for i in range(1,5)]
OPENER=urllib.request.build_opener(urllib.request.ProxyHandler({}))

def get(path):
    with OPENER.open(BASE+path,timeout=3) as response:
        return response.read().decode('utf-8')

def validate_status(status):
    if status.get('hardware_plugin')!='slaihand_hardware/OpenArmJointFake' or status.get('transport')!='fake' or status.get('joints')!=NAMES:
        raise ValueError('拒绝联动：尚未确认四轴 fake 仿真，请重启原离线控制面板')
    if not status.get('ready'):
        raise ValueError('仿真未就绪或反馈过期')
    return status

def status():
    if json.loads(get('/health')).get('app')!='slaihand-offline-panel':
        raise ValueError('端口 8766 不是 SLAI 离线面板')
    return validate_status(json.loads(get('/status')))

def command(kind,degrees):
    current=status()
    if kind not in ['move','cancel']:
        raise ValueError('Unsupported simulation command')
    if kind=='move' and current.get('busy'):
        raise ValueError('仿真正在执行轨迹，请等待或取消')
    match=re.search(r"const token='([a-f0-9]+)'",get('/'))
    if not match: raise ValueError('离线面板安全令牌缺失')
    request=urllib.request.Request(BASE+'/command',data=json.dumps({'command':kind,'degrees':degrees}).encode(),
        headers={'Content-Type':'application/json','X-Panel-Token':match.group(1)})
    with OPENER.open(request,timeout=3) as response:
        return json.load(response)
