"""Two deliberately different adapters: simulated motion, physical reads only."""
import copy
import sys
import threading
import time
import uuid
from collections import deque
import numpy as np
from PySide6.QtCore import QThread, Signal
from .core import ROOT, ACTIVE, minjerk
from .clock import now as clock_now, sleep as clock_sleep, CLOCK_NAME

sys.path.insert(0, str(ROOT/'verification/hardware'))
from four_axis_monitor import Reader, load_mapping, STATES
sys.path.insert(0, str(ROOT/'verification/impedance_demo'))
from model import Simulation, mujoco


class RenderWorker(QThread):
    """Own the GL context on one thread; keep only the latest frame request."""
    frame = Signal(object, float)
    failed = Signal(str)

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.pending = None

    def submit(self, qpos, preview, camera):
        request = (qpos.copy(), dict(preview), tuple(camera.lookat),
                   float(camera.distance), float(camera.azimuth), float(camera.elevation))
        with self.lock:
            self.pending = request
        self.wake.set()

    def run(self):
        renderer = None
        try:
            renderer = mujoco.Renderer(self.model, height=360, width=640)
            data, target = mujoco.MjData(self.model), mujoco.MjData(self.model)
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(cam)
            while not self.isInterruptionRequested():
                self.wake.wait(.1)
                self.wake.clear()
                with self.lock:
                    request, self.pending = self.pending, None
                if request is None:
                    continue
                begin = clock_now()
                qpos, preview, lookat, distance, azimuth, elevation = request
                data.qpos[:] = qpos
                mujoco.mj_forward(self.model, data)
                cam.lookat[:], cam.distance = lookat, distance
                cam.azimuth, cam.elevation = azimuth, elevation
                renderer.update_scene(data, camera=cam)
                if preview:
                    target.qpos[:] = qpos
                    for i, value in preview.items():
                        target.qpos[i-1] = np.deg2rad(value)
                    mujoco.mj_forward(self.model, target)
                    scene = renderer.scene
                    start = scene.ngeom
                    mujoco.mjv_addGeoms(self.model,target,mujoco.MjvOption(),mujoco.MjvPerturb(),
                                       mujoco.mjtCatBit.mjCAT_DYNAMIC,scene)
                    for i in range(start,scene.ngeom):
                        scene.geoms[i].rgba[:] = [.30,.61,1.,.24]
                self.frame.emit(renderer.render().copy(),(clock_now()-begin)*1000)
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            if renderer is not None:
                renderer.close()


class ReadOnlyWorker(QThread):
    sample = Signal(object)
    status = Signal(str)

    def __init__(self, port, mapping, interval=.05):
        super().__init__()
        self.port, self.mapping = port, copy.deepcopy(mapping)
        self.session_id=uuid.uuid4().hex
        self.rate_lock=threading.Lock()
        self.sample_lock=threading.Lock()
        self.latest_sample=dict(rows={},at=0,session_id=self.session_id)
        self.set_interval(interval)

    def peek(self):
        """Immutable-copy observations only; never touches serial or its schedule."""
        with self.sample_lock:
            return copy.deepcopy(self.latest_sample)

    def set_interval(self,interval):
        if interval not in (.01,.05):
            raise ValueError('Only explicit 10ms/50ms read-only schedules allowed')
        with self.rate_lock:
            self.interval=interval

    def run(self):
        reader = None
        try:
            reader = Reader(self.port, self.mapping)
            self.status.emit('只读连接已建立；正在核对参数')
            next_publish=0.
            while not self.isInterruptionRequested():
                now = clock_now()
                previous_tx=reader.tx
                reader.tick(now)
                with self.rate_lock:
                    interval=self.interval
                if reader.tx!=previous_tx:
                    reader.next_send=now+interval
                s = reader.state
                rows = {}
                for i in ACTIVE:
                    if s.fresh(i, now):
                        rows[i] = dict(s.feedback[i], regs=dict(s.regs[i]), age=now-s.at[i],received_at=s.at[i])
                sample=dict(rows=rows, tx=reader.tx, rx=reader.rx,
                                      notice=s.notice, at=now,session_id=self.session_id,
                                      readonly_log=str(reader.path),source='live_usb2can_readonly',
                                      clock=CLOCK_NAME,query_interval_s=interval)
                with self.sample_lock:
                    self.latest_sample=sample
                if now>=next_publish:
                    self.sample.emit(sample)
                    next_publish=now+.02
                clock_sleep(.001)
        except Exception as exc:
            self.status.emit('连接结束：'+str(exc))
        finally:
            if reader is not None:
                reader.close()  # zero TX on disconnect, including no disable
            sample=dict(rows={}, tx=0, rx=0, notice='已断开', at=0,session_id=self.session_id)
            with self.sample_lock:
                self.latest_sample=sample
            self.sample.emit(sample)


class SimAdapter:
    def __init__(self):
        self.sim = Simulation()
        self.cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.cam)
        self.cam.lookat[:] = [.0, .08, -.08]
        self.cam.distance = 1.0
        self.cam.azimuth = 135
        self.cam.elevation = -18
        self.queue = deque()
        self.segment = None
        self.enabled = True
        self.paused = False
        self.tick_count = 0

    @property
    def busy(self):
        return self.segment is not None or bool(self.queue)

    def motion(self, targets, duration):
        if self.busy:
            raise ValueError('仿真正在执行')
        target = np.rad2deg(self.sim.data.qpos.copy())
        for i, v in targets.items():
            if i not in ACTIVE:
                raise ValueError('J5 不可控制')
            target[i-1] = v
        self.trajectory([target], duration)

    def trajectory(self, points, duration):
        from .core import validate_waypoints
        if self.busy:
            raise ValueError('仿真正在执行，不能覆盖已有轨迹')
        points = validate_waypoints(points, duration)
        self.queue = deque((np.deg2rad(p), duration) for p in points)
        self.segment = None
        self.sim.set_mode('spring')

    def stop(self):
        self.queue.clear()
        self.segment = None
        self.sim.capture()
        self.sim.set_mode('spring')

    def step(self, count=20):
        if self.paused or not self.enabled:
            return
        for _ in range(count):
            if self.segment is None and self.queue:
                target, duration = self.queue.popleft()
                self.segment = (self.sim.data.time, self.sim.target.copy(), target, duration)
            if self.segment:
                begin, start, end, duration = self.segment
                phase = (self.sim.data.time-begin)/duration
                self.sim.target[:] = start + (end-start)*minjerk(phase)
                if phase >= 1:
                    self.sim.target[:] = end
                    self.segment = None
            self.sim.step()
        self.tick_count += count

    def close(self):
        pass  # Rendering lifetime is owned exclusively by RenderWorker.
