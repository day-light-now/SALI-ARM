"""Observe an existing startup stall; no ptrace, serial open, USB or ROS writes."""
import argparse
import json
from pathlib import Path
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('pid',type=int)
p.add_argument('audit',type=Path)
a=p.parse_args()
threads=[]
for path in (Path('/proc')/str(a.pid)/'task').glob('*'):
    try:
        state=(path/'stat').read_text().rsplit(')',1)[1].split()[0]
        wait=(path/'wchan').read_text().strip()
        if state=='D' or 'usb' in wait:threads.append(dict(tid=int(path.name),state=state,wait=wait))
    except (OSError,ValueError):
        continue
events=[]
with a.audit.open('rb') as f:
    raw=f.read(65536)
for line in raw.splitlines():
    try:events.append(json.loads(line))
    except json.JSONDecodeError:pass
stall=any(t['state']=='D' and 'usb' in t['wait'] for t in threads) and not any(e.get('kind')!='SESSION' for e in events)
result=dict(pid=a.pid,audit=str(a.audit),audit_bytes=a.audit.stat().st_size,usb_wait_threads=threads,
            status='STARTUP_USB_KERNEL_STALL' if stall else 'STALL_NOT_CURRENTLY_OBSERVED',
            hardware_actions_performed=False,syscall_stack_readable=False)
print(json.dumps(result,indent=2))
out=Path(__file__).resolve().parent/('startup_probe_'+str(time.time_ns())+'.json')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
raise SystemExit(2 if stall else 0)
