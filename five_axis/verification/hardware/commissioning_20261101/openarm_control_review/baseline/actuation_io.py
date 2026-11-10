"""Explicit exclusive-owner USB2CAN session for the control backend.

No executable entry point, auto-open, background thread or UI button. A caller
must hand over a fully populated Reader that is no longer used by its read-only
worker. No other code may read/write that connection during this session.
"""
import hashlib
import sys
import uuid

from .core import ROOT, ACTIVE
from .commissioning import Profile, Observation, Snapshot
from .actuation import FourAxisController, USB2CANEndpoint
from .clock import now as clock_now

sys.path.insert(0,str(ROOT/'verification/hardware'))
from four_axis_monitor import usb_packet, load_mapping


class ControlSession:
    def __init__(self, reader, envelopes, acceptance, audit, clock=clock_now,**controller_options):
        self.reader,self.clock,self.audit = reader,clock,audit
        self.session_id = uuid.uuid4().hex
        self.endpoint = USB2CANEndpoint(reader.serial,audit,clock)
        self.controller = FourAxisController(self.endpoint,envelopes,acceptance,audit,**controller_options)
        self.closed = False
        self.started = False
        self.last_cycle = None

    def _drain(self):
        r = self.reader
        if r.state.mapping != load_mapping():
            raise ValueError('Reader still has an old software mapping; new connection required')
        for cid,data in r.stream.feed(r.serial.read(r.serial.in_waiting)):
            r.rx += 1
            r.event(direction='rx',can_id=cid,payload=data.hex(' '))
            r.state.feed(cid,data,self.clock())

    def _snapshot(self):
        r = self.reader
        rows = []
        for i in ACTIVE:
            if not r.state.ready(i) or i not in r.state.feedback:
                raise ValueError(f'J{i}: configuration or feedback incomplete/conflicted')
            reg = r.state.regs[i]
            f = r.state.feedback[i]
            m = r.state.mapping[i]
            p = Profile(i,reg[10],reg[7],reg[9],reg[21],reg[22],reg[23],
                        int(m['hardware_sign']),m['zero_offset_rad'])
            rows.append(Observation(p,f['position_rad'],f['velocity_rad_s'],f['state_code'],
                                    r.state.at[i],f['driver_temp_c'],f['motor_temp_c'],f['payload']))
        digest = hashlib.sha256((ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml').read_bytes()).hexdigest()
        return Snapshot(self.session_id,self.clock(),tuple(rows),digest,str(r.path),'perf_counter')

    def start(self):
        if self.closed or self.started:
            raise ValueError('Session is closed or already started')
        self._drain()
        snapshot = self._snapshot()
        self.controller.start(snapshot,self.clock())
        self.started = True
        self.last_cycle = self.clock()

    def cycle(self):
        """Caller schedules at 10ms; never shares the Reader.tick scheduler.

        Unenabled axes need CC reads because no MIT frames are sent to them.
        Enabled axes return feedback from MIT control. During stop, query all
        four so actual disabled observations can be distinguished from FD writes.
        """
        if self.closed or not self.started:
            raise ValueError('Control session not active')
        c,r = self.controller,self.reader
        if c.state in ('DISABLED','UNKNOWN'):
            return c.state
        now = self.clock()
        if self.last_cycle is not None and 0 <= now-self.last_cycle < c.period:
            return c.state
        self.last_cycle = now
        try:
            self._drain()
            snapshot = self._snapshot()
            c.tick(snapshot,self.clock())
            query_axes = ACTIVE if c.state=='STOPPING' else tuple(i for i in ACTIVE if i not in c.enabled)
            for i in query_axes:
                packet = usb_packet(i)
                if r.serial.write(packet)!=len(packet):
                    raise IOError('Read-only verification query partial write')
                r.tx += 1
                r.event(direction='tx',can_id=0x7FF,payload=packet[21:29].hex(' '))
        except Exception as exc:
            if c.state == 'STOPPING':
                c.state = 'UNKNOWN'
            else:
                c.stop(str(exc),self.clock())
        return c.state

    def stop(self):
        if self.started:
            self.controller.stop('operator_stop',self.clock())

    def close(self):
        """Try FD before closing, but never claim this confirms physical stop."""
        if not self.closed:
            self.stop()
            if self.controller.state not in ('IDLE','DISABLED'):
                self.controller.state = 'UNKNOWN'
            self.closed = True
            self.endpoint.closed = True
            self.reader.close()
