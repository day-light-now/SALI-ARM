"""Host scheduling probe only. No Qt, serial, robot model or hardware."""
import argparse
import json
from pathlib import Path
import time
from .clock import now, sleep, describe


def probe(seconds=3.):
    if not .1<=seconds<=20:
        raise ValueError('Probe duration must be 0.1–20 seconds')
    began=now()
    deadline=began
    samples=[]
    while now()-began<seconds:
        stamp=now()
        if stamp<deadline:
            sleep(min(.002,deadline-stamp))
            continue
        samples.append(stamp)
        deadline+=.01
        if deadline<now():
            deadline=now()+.01
    gaps=sorted((b-a)*1000 for a,b in zip(samples,samples[1:]))
    return dict(kind='host_probe_not_shadow_or_bus_acceptance',actuation_tx=0,query_tx=0,
        clock=describe(),legacy_clock=vars(time.get_clock_info('monotonic')),
        elapsed_s=now()-began,cycles=len(samples),effective_hz=(len(samples)-1)/(samples[-1]-samples[0]),
        period_ms=dict(min=min(gaps),max=max(gaps),p95=gaps[min(len(gaps)-1,int(len(gaps)*.95))]))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    data=probe()
    text=json.dumps(data,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text,encoding='utf-8')
    print(text)


if __name__=='__main__':
    main()
