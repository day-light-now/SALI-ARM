"""Check the demo's saved-plan loader, corrected J4 limits and speed display."""
import json
import urllib.request
import browser_check as browser


def main():
    url = (browser.HERE / 'browser_url.txt').read_text().strip()
    base, query = url.split('/?', 1)
    assert base == 'http://127.0.0.1:18888'
    token = query.split('token=', 1)[1]
    root = browser.HERE.parents[2]
    document = json.loads((root / 'src/slaihand_bringup/config/leader_demo_d01.json').read_text())
    request = urllib.request.Request(base + '/api/plans/save',
        data=json.dumps({'name': 'D01 四轴联动巡游展示（12点·90秒）', 'document': document}).encode(),
        headers={'X-Panel-Token': token, 'Origin': base, 'Content-Type': 'application/json'})
    with browser.OPENER.open(request, timeout=5) as response:
        identifier = json.load(response)['plan']['id']
    process, endpoint = browser.launch_browser()
    websocket = None
    try:
        websocket = browser.WebSocket(endpoint)
        websocket.command('Page.enable')
        websocket.command('Runtime.enable')
        websocket.command('Page.navigate', {'url': url})
        browser.wait_for(websocket,
            "document.getElementById('saved-select') && "
            "document.getElementById('saved-select').textContent.includes('D01')", 25, 'saved demo')
        assert websocket.evaluate("document.getElementById('speed-limit').textContent") == '1'
        status = browser.api_status(base, token)
        assert status['ros']['limits_deg'][3] == [-50, 90]
        websocket.evaluate("document.getElementById('saved-select').value=" + json.dumps(identifier))
        websocket.evaluate("document.getElementById('btn-load').click()")
        browser.wait_for(websocket, "document.querySelectorAll('#waypoints-body tr').length === 12", 8, '12 demo rows')
        assert websocket.evaluate("document.getElementById('execution-mode').value") == 'trajectory'
        websocket.evaluate("document.getElementById('btn-preview').click()")
        browser.wait_for(websocket, "document.querySelectorAll('#preview-body tr').length >= 12", 8, 'demo preview')
        assert browser.api_status(base, token)['ros']['command_id'] == 0
        browser.screenshot(websocket, 'speed1_j4_demo_browser.png')
        proof = {'passed': True, 'loaded_waypoints': 12, 'duration_s': 90,
                 'reference_speed_rad_s': 1, 'j4_limits_deg': [-50, 90],
                 'preview_passed': True, 'motion_requested': False}
        (browser.EVIDENCE / 'speed1_j4_demo_browser.json').write_text(json.dumps(proof, indent=2))
        print('PASS', json.dumps(proof))
    finally:
        if websocket:
            websocket.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == '__main__':
    main()
