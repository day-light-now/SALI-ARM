"""Exercise automatic calibration controls in a headless browser on PTY only."""
import json,time
from pathlib import Path
import browser_check as b

def main():
 deadline=time.monotonic()+30
 while not (b.HERE/'browser_url.txt').exists() and time.monotonic()<deadline:time.sleep(.2)
 url=(b.HERE/'browser_url.txt').read_text(encoding='utf-8').strip()
 base,query=url.split('/?',1);token=query.split('token=',1)[1]
 assert base.endswith(':18888'),'refusing a real control panel'
 proc,endpoint=b.launch_browser();ws=b.WebSocket(endpoint)
 checks=[]
 def check(name,value):
  assert value,name
  checks.append(name);print('PASS',name,flush=True)
 try:
  ws.command('Page.enable');ws.command('Runtime.enable')
  ws.command('Page.navigate',{'url':url})
  b.wait_for(ws,"document.getElementById('chip-ros')?.textContent.includes('在线')",20,'feedback')
  ws.evaluate("document.getElementById('btn-calibration-group').click()")
  b.wait_for(ws,"document.querySelectorAll('#waypoints-body tr').length===12",8,'batch loaded')
  check('载入自动12姿态且不运动',b.api_status(base,token)['ros']['stage']==0)
  check('自动选择重力采样模式',ws.evaluate("document.getElementById('execution-mode').value")== 'calibration')
  ws.evaluate("document.getElementById('btn-preview').click()")
  b.wait_for(ws,"document.querySelectorAll('#preview-body tr').length===12",8,'preview')
  check('预览冻结12点',True)
  check('预览显示162秒计划', '162' in ws.evaluate("document.getElementById('preview-meta').textContent"))
  check('预览保持未使能',b.api_status(base,token)['ros']['stage']==0)
  ws.evaluate("document.getElementById('calibration-group').value='G01';document.getElementById('btn-calibration-group').click()")
  b.wait_for(ws,"document.querySelectorAll('#waypoints-body tr').length===2",8,'group loaded')
  ws.evaluate("document.getElementById('btn-preview').click()")
  b.wait_for(ws,"document.querySelectorAll('#preview-body tr').length===2",8,'group preview')
  ws.evaluate("for(const id of ['ack-supported','ack-clearance','ack-native']){const e=document.getElementById(id);e.checked=true;e.dispatchEvent(new Event('change',{bubbles:true}));}")
  check('未确认自由承重时不可执行',ws.evaluate("document.getElementById('btn-execute').disabled") is True)
  ws.evaluate("const e=document.getElementById('ack-free-static');e.checked=true;e.dispatchEvent(new Event('change',{bubbles:true}));")
  b.wait_for(ws,"!document.getElementById('btn-execute').disabled",5,'confirmed')
  ws.evaluate("document.getElementById('btn-execute').click()")
  status=b.wait_operation(base,token,45)
  c=status['operation']['result']['calibration']
  check('页面执行自动采到2/2',c['campaign_complete'] and c['qualified_points']==2)
  check('采样后新鲜失能确认',status['operation']['result']['stop_observations_confirmed'])
  b.wait_for(ws,"document.getElementById('calibration-results').textContent.includes('2/2')",5,'capture result')
  check('逐点合格及原始文件显示', '原始记录' in ws.evaluate("document.getElementById('calibration-results').textContent"))
  b.screenshot(ws,'panel_auto_calibration.png')
  ws.evaluate("document.getElementById('calibration-results').scrollIntoView({block:'center'})")
  b.screenshot(ws,'panel_auto_calibration_results.png')
  ws.command('Emulation.setDeviceMetricsOverride',{'width':420,'height':900,'deviceScaleFactor':1,'mobile':False})
  check('窄屏无整页溢出',ws.evaluate('document.documentElement.scrollWidth<=window.innerWidth') is True)
  b.screenshot(ws,'panel_auto_calibration_narrow.png')
  (b.EVIDENCE/'browser_calibration_result.json').write_text(json.dumps({'passed':True,'checks':checks},ensure_ascii=False,indent=2),encoding='utf-8')
 finally:
  ws.close();proc.terminate()
if __name__=='__main__':main()
