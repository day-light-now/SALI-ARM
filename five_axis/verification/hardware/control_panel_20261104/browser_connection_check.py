"""Render/check connection controls. Never requests an acknowledged restart."""
import re
import subprocess
import time
import browser_check as browser

log=subprocess.run(['wsl','-d','Ubuntu-24.04','--','cat','/tmp/slaihand-ptp-panel.log'],
                   capture_output=True,text=True,encoding='utf-8',errors='replace',check=True).stdout
url=re.search(r'http://127\.0\.0\.1:42663/\?token=\S+',log).group()
base,token=url.split('/?token=')
process,address=browser.launch_browser();ws=None
try:
    ws=browser.WebSocket(address)
    ws.command('Page.enable');ws.command('Runtime.enable')
    ws.command('Page.navigate',dict(url=url))
    browser.wait_for(ws,"!!(document.getElementById('btn-reconnect') && document.getElementById('btn-connection-check'))",10,'connection controls')
    ws.evaluate("document.getElementById('btn-connection-check').click()")
    browser.wait_for(ws,"document.getElementById('message').textContent.includes('连接检查：')",8,'connection check response')
    state=browser.api_status(base,token)
    # Only exercise the denial path when state is unknown/disconnected. With
    # known-disabled feedback an unchecked reconnect would be a real restart.
    if state['ros']['stage'] in (None,9) or not state['ros']['feedback_valid_all']:
        ws.evaluate("document.getElementById('ack-power-cycle').checked=false; document.getElementById('btn-reconnect').click()")
        browser.wait_for(ws,"document.getElementById('message').textContent.includes('POWER_REVIEW')",8,'explicit recovery confirmation')
    else:print('Skipped reconnect click: live known-disabled driver')
    browser.screenshot(ws,'panel_connection_controls.png')
    print('PASS connection controls rendered; check responds; no acknowledged restart or motor action')
finally:
    if ws:ws.close()
    process.terminate();process.wait(timeout=8)
