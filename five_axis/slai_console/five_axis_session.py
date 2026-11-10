"""Five-axis exclusive-owner session over the read-only Reader5.

Builds AxisProfile values only from fresh device registers plus the canonical
J1-J4 mapping and the manual J5 calibration record. J5 is never added from a
guess. The session is manual-100Hz; the caller owns scheduling.
"""
import hashlib
import time
import uuid

from .clock import now as clock_now
from .core import ROOT
from .j5_calibration import DEFAULT_PATH as J5_PATH
from .five_axis_config import build_envelopes
from .five_axis_mit import (AXES5, AxisObservation, AxisProfile, FiveAxisController,
                            FiveAxisEndpoint, FiveSnapshot)
from .five_axis_monitor import load_mapping5, usb_packet


def build_profiles(reader, mapping, active_axes):
    profiles = {}
    for i in active_axes:
        if not reader.state.ready(i) or i not in reader.state.feedback:
            raise ValueError(f'J{i}: configuration or feedback incomplete/conflicted')
        reg = reader.state.regs[i]
        m = mapping[i]
        profiles[i] = AxisProfile(i, reg[10], reg[7], reg[9], reg[21], reg[22], reg[23],
                                  int(m['hardware_sign']), m['zero_offset_rad'])
        if profiles[i].mode != 1:
            raise ValueError(f'J{i}: CTRL_MODE is {reg[10]}, not 1')
    return profiles


def build_five_session(reader, settings, acceptance, audit, clock=clock_now,
                       calibration=None, recovery_axes=(), max_runtime=60.):
    settings.check()
    envelopes, active_axes = build_envelopes(settings, calibration)
    mapping = reader.state.mapping
    if mapping is None:
        raise ValueError('reader has no five-axis mapping')
    if 5 in active_axes:
        if calibration is None or not calibration.is_usable():
            raise ValueError('J5 cannot be active without a usable manual calibration')
        j5 = mapping.get(5, {})
        if not j5.get('calibrated'):
            raise ValueError('reader mapping did not load the J5 calibration')
        if (j5['hardware_sign'] != calibration.sign
                or abs(j5['zero_offset_rad']-calibration.zero_offset_rad) > 1e-12):
            raise ValueError('J5 calibration and reader mapping disagree; reconnect')
    for i in active_axes:
        f = reader.state.feedback.get(i)
        if f is None:
            raise ValueError(f'J{i}: no feedback')
        q = mapping[i]['hardware_sign']*(f['position_rad']-mapping[i]['zero_offset_rad'])
        if i not in recovery_axes:
            envelopes[i].check(q, i)
    profiles = build_profiles(reader, mapping, active_axes)
    acceptance.validate_scope(active_axes, 1.)
    return FiveAxisSession(reader, profiles, envelopes, acceptance, audit,
                           clock=clock, active_axes=active_axes,
                           max_runtime=max_runtime, ramp=1.,
                           recovery_axes=tuple(recovery_axes))


def fresh_active_feedback(reader, active_axes, clock=clock_now,
                          sleep=time.sleep, timeout=2., max_age=.03):
    """Burst-read every active axis immediately before takeover.

    The read-only UI polls one axis per 50ms, so J1 can be ~250ms old when the
    user presses the control button. A five-axis controller requires <=60ms.
    This preflight sends only read-only 0x7CC CC requests and returns only when
    every active axis has a feedback age <= max_age.
    """
    active = tuple(active_axes)
    if not active:
        raise ValueError('no active axes for preflight')
    end = clock()+timeout
    while clock() < end:
        for i in active:
            packet = usb_packet(i)
            if reader.serial.write(packet) != len(packet):
                raise IOError('fresh preflight partial write')
            reader.tx += 1
            reader.event(direction='tx', can_id=0x7FF,
                         payload=packet[21:29].hex(' '))
        sleep(.01)
        chunk = reader.serial.read(reader.serial.in_waiting)
        for cid, data in reader.stream.feed(chunk):
            reader.rx += 1
            reader.event(direction='rx', can_id=cid, payload=data.hex(' '))
            reader.state.feed(cid, data, clock())
        now = clock()
        if all(i in reader.state.feedback
               and i in reader.state.at
               and 0 <= now-reader.state.at[i] <= max_age
               for i in active):
            return True
    raise RuntimeError('fresh feedback preflight timeout; no control frame sent')


class FiveAxisSession:
    def __init__(self, reader, profiles, envelopes, acceptance, audit, clock=clock_now,
                 active_axes=AXES5, **controller_options):
        self.reader, self.clock, self.audit = reader, clock, audit
        self.session_id = uuid.uuid4().hex
        self.endpoint = FiveAxisEndpoint(reader.serial, audit, clock)
        self.controller = FiveAxisController(self.endpoint, profiles, envelopes,
                                             acceptance, audit,
                                             active_axes=active_axes,
                                             **controller_options)
        self.closed = False
        self.started = False
        self.last_cycle = None

    def _drain(self):
        r = self.reader
        fresh = load_mapping5(getattr(r, 'cal_path', J5_PATH))
        if r.state.mapping != fresh:
            raise ValueError('Reader still has an old five-axis mapping; new connection required')
        for cid, data in r.stream.feed(r.serial.read(r.serial.in_waiting)):
            r.rx += 1
            r.event(direction='rx', can_id=cid, payload=data.hex(' '))
            r.state.feed(cid, data, self.clock())

    def _snapshot(self):
        r = self.reader
        rows = []
        for i in self.controller.active_axes:
            if not r.state.ready(i) or i not in r.state.feedback:
                raise ValueError(f'J{i}: configuration or feedback incomplete/conflicted')
            reg = r.state.regs[i]
            f = r.state.feedback[i]
            m = r.state.mapping[i]
            p = AxisProfile(i, reg[10], reg[7], reg[9], reg[21], reg[22], reg[23],
                            int(m['hardware_sign']), m['zero_offset_rad'])
            rows.append(AxisObservation(p, f['position_rad'], f['velocity_rad_s'],
                                        f['torque_feedback_nm'], f['state_code'],
                                        r.state.at[i], f['driver_temp_c'],
                                        f['motor_temp_c'], f['payload']))
        digest = hashlib.sha256(
            (ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml').read_bytes()).hexdigest()
        return FiveSnapshot(self.session_id, self.clock(), tuple(rows), digest,
                            str(r.path), 'perf_counter')

    def start(self):
        if self.closed or self.started:
            raise ValueError('Session is closed or already started')
        self._drain()
        snapshot = self._snapshot()
        self.controller.start(snapshot, self.clock())
        self.started = True
        self.last_cycle = self.clock()

    def cycle(self):
        if self.closed or not self.started:
            raise ValueError('Five-axis session not active')
        c, r = self.controller, self.reader
        if c.state in ('DISABLED', 'UNKNOWN'):
            return c.state
        now = self.clock()
        if self.last_cycle is not None and 0 <= now-self.last_cycle < c.period:
            return c.state
        self.last_cycle = now
        try:
            self._drain()
            snapshot = self._snapshot()
            c.tick(snapshot, self.clock())
            query_axes = (c.active_axes if c.state == 'STOPPING'
                          else tuple(i for i in c.active_axes if i not in c.enabled))
            for i in query_axes:
                packet = usb_packet(i)
                if r.serial.write(packet) != len(packet):
                    raise IOError('Read-only verification query partial write')
                r.tx += 1
                r.event(direction='tx', can_id=0x7FF, payload=packet[21:29].hex(' '))
        except Exception as exc:
            if c.state == 'STOPPING':
                c.state = 'UNKNOWN'
            else:
                c.stop(str(exc), self.clock())
        return c.state

    def stop(self):
        if self.started:
            self.controller.stop('operator_stop', self.clock())

    def close(self):
        if not self.closed:
            self.stop()
            if self.controller.state not in ('IDLE', 'DISABLED'):
                self.controller.state = 'UNKNOWN'
            self.closed = True
            self.endpoint.closed = True
            self.reader.close()
