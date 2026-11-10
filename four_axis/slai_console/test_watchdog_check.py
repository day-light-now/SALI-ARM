"""Synthetic watchdog experiment. No physical serial port is opened."""
import math
import io
import json
import struct
import threading
import unittest
from .watchdog_check import J4WatchdogCheck, make_event_writer
from .test_actuation_io import Bus
from four_axis_monitor import load_mapping
from .test_reader_fixture import make_reader


class Clock:
    def __init__(self):self.t=10.
    def now(self):return self.t
    def sleep(self,delta):self.t+=delta


class WatchdogBus(Bus):
    def __init__(self,mapping,clock,trip=True):
        super().__init__(mapping);self.clock=clock;self.last_control=None
        self.history=[];self.trip=trip

    def write(self,frame):
        cid=int.from_bytes(frame[13:17],'little');payload=frame[21:29]
        self.history.append((self.clock.now(),cid,payload))
        if self.trip and self.states[4]==1 and self.last_control is not None and self.clock.now()-self.last_control>.1:
            self.states[4]=13
        if cid==4 and payload[:7]!=b'\xff'*7:self.last_control=self.clock.now()
        if cid==0x7FF and payload[2]==0x33:
            self.frames.append(frame);i=payload[0];rid=payload[3]
            value={7:0,8:i,9:2000,10:1,21:12.5,22:45.,23:54.}[rid]
            response=bytes([i,0,0x33,rid])+struct.pack('<I' if rid<21 else '<f',value)
            self.buf.extend(bytes([0xAA,0x11,8,0,0,0,0])+response+b'\x55')
            return len(frame)
        return super().write(frame)


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.mapping=load_mapping()
        self.bus=WatchdogBus(self.mapping,self.clock)
        self.reader=make_reader('FAKE_ONLY',self.mapping,serial_factory=lambda *a,**k:self.bus)
        self.events=[];self.stop=threading.Event()
        self.check=J4WatchdogCheck(self.reader,lambda event,**data:self.events.append((event,data)),self.stop,
                                   self.clock.now,self.clock.sleep)

    def tearDown(self):self.reader.close()

    def test_full_watchdog_sequence_with_real_usb_parser(self):
        result=self.check.run()
        self.assertTrue(result['watchdog_d_observed'],result)
        self.assertFalse(result['physical_torque_off_verified'])
        self.assertIsNone(result['measured_watchdog_latency_ms'])
        control=[(t,c,p) for t,c,p in self.bus.history if c!=0x7FF]
        self.assertTrue(all(c==4 for _,c,_ in control))
        self.assertEqual(sum(p==b'\xff'*7+b'\xfc' for _,_,p in control),1)
        self.assertEqual(control[-1][2],b'\xff'*7+b'\xfd')
        mits=[(t,p) for t,c,p in control if p[:7]!=b'\xff'*7]
        last=mits[-1][0]
        self.assertFalse(any(last<t<last+.249 for t,_,_ in self.bus.history))
        for _,p in mits:
            self.assertEqual(((p[3]&15)<<8)|p[4],0)
            self.assertEqual((p[5]<<4)|(p[6]>>4),0)
            torque=(((p[6]&15)<<8)|p[7])*108/4095-54
            self.assertLess(abs(torque),.014)
        for _,cid,p in self.bus.history:
            if cid==0x7FF:
                self.assertIn(p[0],(1,2,3,4));self.assertIn(p[2],(0x33,0xCC))

    def test_production_window_audit_callback_accepts_frame_kind(self):
        handle=io.StringIO();published=[]
        event=make_event_writer(handle,published.append)
        self.check.event=event
        self.check.endpoint.audit=event
        result=self.check.run()
        self.assertTrue(result['watchdog_d_observed'],result)
        rows=[json.loads(line) for line in handle.getvalue().splitlines()]
        accepted=[r for r in rows if r['kind']=='CONTROL_ACCEPTED']
        self.assertTrue(accepted)
        self.assertEqual(accepted[0]['frame_kind'],'mit')
        self.assertTrue(any(r['frame_kind']=='enable' for r in accepted))
        self.assertTrue(any(r['frame_kind']=='disable' for r in accepted))

    def test_no_trip_does_not_pass(self):
        self.bus.trip=False
        result=self.check.run()
        self.assertFalse(result['watchdog_d_observed'])
        self.assertIn('error',result)

    def test_cancel_before_enable_sends_no_enable(self):
        self.stop.set();result=self.check.run()
        self.assertIn('error',result)
        self.assertFalse(self.check.enable_attempted)
        self.assertEqual(self.bus.frames,[])

    def test_disabled_watchdog_prevents_enable(self):
        original=self.reader.state.feed
        def feed(cid,payload,now):
            if payload[2:4]==bytes([0x33,9]):payload=payload[:4]+b'\0'*4
            original(cid,payload,now)
        self.reader.state.feed=feed
        result=self.check.run()
        self.assertIn('error',result)
        self.assertFalse(any(cid==4 for _,cid,_ in self.bus.history))

    def test_cancel_after_enable_attempts_disable(self):
        event=self.check.event
        def cancel(kind,**data):
            event(kind,**data)
            if kind=='PHASE' and '短时使能' in data.get('message',''):self.stop.set()
        self.check.event=cancel
        result=self.check.run()
        self.assertIn('error',result)
        self.assertEqual(self.bus.history[-2][2],b'\xff'*7+b'\xfd')

    def test_position_departure_aborts_and_disables(self):
        feedback=self.bus.feedback
        def moved(i):
            p=feedback(i)
            if i==4 and self.bus.states[4]==1:
                raw=int.from_bytes(p[1:3],'big')+300
                p=p[:1]+raw.to_bytes(2,'big')+p[3:]
            return p
        self.bus.feedback=moved
        result=self.check.run()
        self.assertIn('error',result)
        self.assertFalse(result['watchdog_d_observed'])
        self.assertTrue(any(p==b'\xff'*7+b'\xfd' for _,_,p in self.bus.history))


if __name__=='__main__':unittest.main()
