"""100 Hz target host scheduler, existing observations only; no serial access."""
import copy
import json
from pathlib import Path
import threading
import time
import uuid
from PySide6.QtCore import QThread, Signal
from .core import ROOT
from .commissioning import ShadowEngine, capture
from .clock import now as clock_now, sleep as clock_sleep, describe


class ShadowWorker(QThread):
    report=Signal(object)

    def __init__(self,initial,mapping,sample,duration=20.,directory=None,sample_provider=None):
        super().__init__()
        if not 0<duration<=60:
            raise ValueError('Shadow record duration must be <=60 seconds')
        self.initial=initial
        self.mapping=copy.deepcopy(mapping)
        self.sample=copy.deepcopy(sample)
        self.duration=duration
        self.sample_provider=sample_provider
        self.lock=threading.Lock()
        folder=Path(directory or ROOT/'tmp/commissioning_shadow')
        folder.mkdir(parents=True,exist_ok=True)
        self.path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jsonl')

    def submit(self,sample):
        with self.lock:
            self.sample=copy.deepcopy(sample)

    def run(self):
        engine=ShadowEngine(self.initial)
        began=clock_now()
        deadline=began
        published=0.
        try:
            with self.path.open('x',encoding='utf-8') as log:
                def write(row):
                    log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
                write(dict(kind='snapshot',snapshot=self.initial.serializable(),host_target_hz=100,
                           duration_limit_s=self.duration,physical_prediction=False,clock=describe()))
                while not self.isInterruptionRequested():
                    now=clock_now()
                    if now-began>=self.duration:
                        break
                    if now<deadline:
                        clock_sleep(min(.002,deadline-now))
                        continue
                    if self.sample_provider is not None:
                        sample=self.sample_provider()
                    else:
                        with self.lock:
                            sample=copy.deepcopy(self.sample)
                    try:
                        observation=capture(sample,self.mapping,now)
                        row=engine.step(observation,now)
                    except Exception as exc:
                        engine.supervisor.reason='INVALID_OR_EXPIRED_OBSERVATION: '+str(exc)
                        row=dict(kind='shadow_cycle',global_latch=engine.supervisor.reason,actuation_tx=0,frames=[])
                    write(row)
                    if now-published>=.5 or engine.supervisor.reason:
                        log.flush()
                        self.report.emit(dict(**engine.summary(),path=str(self.path)))
                        published=now
                    if engine.supervisor.reason:
                        break
                    # Skip missed deadlines; never emit catch-up bursts.
                    deadline+=.010
                    if deadline<clock_now():
                        deadline=clock_now()+.010
                summary=dict(**engine.summary(),path=str(self.path),finished=True)
                write(dict(kind='summary',**summary))
                self.report.emit(summary)
        except Exception as exc:
            self.report.emit(dict(finished=True,error=str(exc),actuation_tx=0,path=str(self.path)))
