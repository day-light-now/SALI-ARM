"""Desktop entry: reuse existing estimate service or start one calculation-only child."""
from pathlib import Path
import json
import subprocess
import sys
import time
import urllib.request
import webbrowser

URL='http://127.0.0.1:8767/'
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

def ready():
    try:
        with opener.open(URL+'health',timeout=1) as response:
            info=json.load(response)
            return info.get('app')=='slaihand-gravity-estimate' and info.get('motor_output') is False
    except (OSError,ValueError): return False

def main():
    if ready():
        webbrowser.open(URL)
        print('Estimate page already running; reused existing service.',flush=True)
        return 0
    child=subprocess.Popen([sys.executable,'-u',str(Path(__file__).with_name('gravity_demo.py'))])
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            if child.poll() is not None:
                print('Could not start the estimate service. Port 8767 may be occupied.',flush=True)
                return 1
            if ready():
                webbrowser.open(URL)
                print('Calculation only. Keep this window open; Ctrl+C stops this service.',flush=True)
                return child.wait()
            time.sleep(.2)
        print('Startup timed out.',flush=True)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=5)

if __name__=='__main__':sys.exit(main())
