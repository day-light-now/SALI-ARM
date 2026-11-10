"""Exercise the new selector and production page on a PTY-only test fixture."""
from pathlib import Path
import json
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'control_panel_20261104'))
import browser_check as browser
url=(browser.HERE/'browser_url.txt').read_text().strip()
base,token=url.split('/?token=')
process,address=browser.launch_browser()
ws=None
try:
    ws=browser.WebSocket(address)
    ws.command('Page.enable');ws.command('Runtime.enable')
    ws.command('Page.navigate',dict(url=url))
    browser.wait_for(ws,"document.getElementById('execution-mode') && document.querySelectorAll('#waypoints-body tr').length===1",12,'page ready')
    ws.evaluate("document.getElementById('mode-relative').click(); document.getElementById('btn-add').click()")
    ws.evaluate("""(() => {
      const rows=document.querySelectorAll('#waypoints-body tr');
      [[10,5,0,0,2,1],[0,0,0,0,2,1]].forEach((values,i)=>{
        rows[i].querySelectorAll('input').forEach((input,j)=>{input.value=values[j];input.dispatchEvent(new Event('input',{bubbles:true}));});
      });
      const mode=document.getElementById('execution-mode');mode.value='point_to_point';mode.dispatchEvent(new Event('change',{bubbles:true}));
      document.getElementById('btn-preview').click();
    })()""")
    browser.wait_for(ws,"document.getElementById('message').textContent.includes('校验通过')",8,'preview')
    state=browser.api_status(base,token)
    assert state['preview']['execution_mode']=='point_to_point',state['preview']
    ws.evaluate("['ack-supported','ack-clearance','ack-native'].forEach(id=>document.getElementById(id).click())")
    browser.wait_for(ws,"!document.getElementById('btn-execute').disabled",5,'execute enabled')
    ws.evaluate("document.getElementById('btn-execute').click()")
    final=browser.wait_operation(base,token,25)
    result=final['operation']['result']
    assert result['mode']=='point_to_point' and result['overall_ok'],result
    assert all(row['sampled_span_s']>=.5 and row['passed'] for row in result['waypoint_results'])
    browser.wait_for(ws,"document.body.textContent.includes('finite_trial_complete')",5,'result visible')
    browser.screenshot(ws,'point_to_point_desktop.png')
    Path(__file__).with_name('browser_ptp_result.json').write_text(json.dumps(dict(status='PASS',software_only=True,
        physical_port_opened=False,result=result),indent=2))
    print('BROWSER_PTP_PASS')
finally:
    if ws:ws.close()
    process.terminate();process.wait(timeout=8)
