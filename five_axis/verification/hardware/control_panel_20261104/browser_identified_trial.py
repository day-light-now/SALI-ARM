"""Exercise the identified-profile finite hold in the isolated browser fixture."""
import json
import argparse
import time

import browser_check as browser


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gravity-float', action='store_true')
    arguments = parser.parse_args()
    url = (browser.HERE / 'browser_url.txt').read_text().strip()
    base, query = url.split('/?', 1)
    assert base == 'http://127.0.0.1:18888', 'refusing a physical panel'
    token = query.split('token=', 1)[1]
    process, endpoint = browser.launch_browser()
    websocket = None
    try:
        websocket = browser.WebSocket(endpoint)
        websocket.command('Page.enable')
        websocket.command('Runtime.enable')
        websocket.command('Page.navigate', {'url': url})
        browser.wait_for(websocket,
            "document.getElementById('gravity-flags') && "
            "document.getElementById('gravity-flags').textContent.includes('实测重力基参数')",
            25, 'identified model card')
        flags = websocket.evaluate("document.getElementById('gravity-flags').textContent")
        assert '未辨识' in flags and '0.10 / 2.00 s' in flags, flags
        status = browser.api_status(base, token)
        assert status['gravity_shadow']['driver']['alpha'] == 0.1
        assert status['ros']['stage'] == 0
        websocket.evaluate("""(() => {
            for (const id of ['ack-supported', 'ack-clearance', 'ack-native']) {
                const input = document.getElementById(id);
                input.checked = true;
                input.dispatchEvent(new Event('change', {bubbles: true}));
            }
        })()""")
        button = 'btn-gravity-float' if arguments.gravity_float else 'btn-hold'
        if arguments.gravity_float:
            assert websocket.evaluate("document.getElementById('btn-gravity-float').disabled")
            websocket.evaluate("""(() => {
                const input = document.getElementById('ack-gravity-float');
                input.checked = true;
                input.dispatchEvent(new Event('change', {bubbles: true}));
            })()""")
        browser.wait_for(websocket, f"!document.getElementById('{button}').disabled",
                         8, 'finite hold ready')
        websocket.evaluate(f"document.getElementById('{button}').click()")
        active_seen = False
        deadline = time.monotonic() + 25
        operation = None
        while time.monotonic() < deadline:
            status = browser.api_status(base, token)
            operation = status.get('operation') or {}
            driver = status.get('gravity_shadow', {}).get('driver', {})
            active_seen |= bool(driver.get('active'))
            if arguments.gravity_float and driver.get('active'):
                assert driver['alpha'] == 1 and driver['gravity_float']
                assert status['gravity_shadow']['metadata']['kp'] == 0
            if operation and not operation.get('running'):
                break
            time.sleep(0.1)
        assert active_seen, 'no active feedforward observation'
        assert operation and operation['exit_code'] == 0, operation
        assert operation['result']['stop_observations_confirmed']
        browser.wait_for(websocket,
            "document.getElementById('gravity-reason').textContent.includes('驱动前馈未活动')",
            5, 'inactive after stop')
        driver = browser.api_status(base, token)['gravity_shadow']['driver']
        assert not driver['active'] and driver['applied'] == [0.0] * 4, driver
        stem = 'gravity_float_browser' if arguments.gravity_float else 'identified_trial_browser'
        browser.screenshot(websocket, stem + '.png')
        report = {'passed': True, 'fixture_port': 18888, 'alpha': 0.1,
                  'ramp_s': 2.0, 'ui_hold_executed': True,
                  'active_observed': active_seen, 'stop_confirmed': True}
        report['mode'] = 'gravity_float' if arguments.gravity_float else 'current_pose_hold'
        report['alpha'] = 1 if arguments.gravity_float else 0.1
        report['ramp_s'] = 3 if arguments.gravity_float else 2
        (browser.EVIDENCE / (stem + '.json')).write_text(
            json.dumps(report, indent=2))
        print('PASS identified-profile browser hold', json.dumps(report))
    finally:
        if websocket:
            websocket.sock.close()
        process.terminate()
        process.wait(timeout=10)


if __name__ == '__main__':
    main()
