"""Run bounded offline checks and bind six-page captures to exact inputs."""
import argparse
import json
from pathlib import Path
import os
import shutil
import subprocess
import sys
import time
from .core import ROOT
from .provenance import fingerprint, now, sha, write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'tmp/console_release/2026-10-31')
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    before=fingerprint()
    write_json(output/'capture_inputs_before.json',dict(at=now(),fingerprint=before))
    env=dict(os.environ,PYTHONIOENCODING='utf-8')
    results=[]
    commands=[('console_tests',['-m','unittest','slai_console.test_console','slai_console.test_commissioning','slai_console.test_timing','-v']),
              ('protocol_tests',['-m','unittest','discover','-s','verification/hardware','-p','test_*.py','-v']),
              ('screenshots',['-m','slai_console.capture_offline'])]
    for name,arguments in commands:
        start=time.monotonic()
        with (output/(name+'.log')).open('w',encoding='utf-8') as log:
            result=subprocess.run([sys.executable,*arguments],cwd=ROOT,env=env,
                                  stdout=log,stderr=subprocess.STDOUT,timeout=180)
        results.append(dict(name=name,command=[sys.executable,*arguments],exit_code=result.returncode,
                            elapsed_s=time.monotonic()-start,log=name+'.log'))
        print(name+': '+str(result.returncode),flush=True)
        write_json(output/'verification.json',dict(at=now(),offline=True,checks=results))
        if result.returncode:
            raise RuntimeError(name+' failed; see saved log')
    after=fingerprint()
    if before!=after:
        raise RuntimeError('Runtime inputs changed during validation; do not label these captures current')
    destination=output/'验证截图'
    destination.mkdir(exist_ok=True)
    screenshots={}
    for i in range(1,7):
        source=ROOT/'tmp/desktop_console/screenshots'/f'{i:02d}.png'
        if not source.is_file() or source.stat().st_size==0:
            raise RuntimeError('Screenshot missing: '+str(source))
        shutil.copy2(source,destination/source.name)
        screenshots[source.name]=sha(source)
    write_json(output/'capture_provenance.json',dict(at=now(),runtime_fingerprint=after,
        inputs_unchanged=True,real_serial_forbidden=True,capture_command=commands[-1][1],screenshots=screenshots,
        limitations='Offline UI captures only; historical DeepSeek reports review the 2026-10-30 revision, not these later maintenance changes.'))


if __name__=='__main__':
    main()
