"""Byte-level synthetic USB2CAN integration: no serial import/open."""
import struct
import unittest
from . import test_actuation as fixtures
from .actuation_io import ControlSession, load_mapping
from .test_reader_fixture import make_reader


class Bus:
    def __init__(self,mapping):
        self.mapping=mapping;self.buf=bytearray();self.frames=[]
        self.states={i:0 for i in range(1,5)};self.closed=False

    @property
    def in_waiting(self):return len(self.buf)

    def reset_input_buffer(self):self.buf.clear()

    def read(self,n):
        result=bytes(self.buf[:n]);del self.buf[:n];return result

    def feedback(self,i):
        raw=round((self.mapping[i]['zero_offset_rad']+12.5)*65535/25)
        return bytes([self.states[i]*16+i,raw>>8,raw&255,0x7f,0xf7,0xff,25,25])

    def write(self,frame):
        self.frames.append(frame);cid=int.from_bytes(frame[13:17],'little');p=frame[21:29]
        i=p[0] if cid==0x7FF else cid
        if cid!=0x7FF and p[:7]==b'\xff'*7:
            self.states[i]=1 if p[-1]==0xFC else 0
        self.buf.extend(bytes([0xAA,0x11,8,0,0,0,0])+self.feedback(i)+b'\x55')
        return len(frame)

    def close(self):self.closed=True


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.fixture=fixtures.Tests();self.fixture.setUp()
        self.mapping=load_mapping();self.bus=Bus(self.mapping)
        self.reader=make_reader('SYNTHETIC_PORT',self.mapping,serial_factory=lambda *a,**k:self.bus)
        for i in range(1,5):
            for rid,v in {7:0,8:i,9:2000,10:1,21:12.5,22:45.,23:54.}.items():
                payload=bytes([i,0,0x33,rid])+struct.pack('<I' if rid<21 else '<f',v)
                self.reader.state.feed(0,payload,self.fixture.now)
            self.reader.state.feed(0,self.bus.feedback(i),self.fixture.now)
        self.session=ControlSession(self.reader,self.fixture.envelopes,self.fixture.acceptance,
                                    self.fixture.audit,lambda:self.fixture.now)

    def tearDown(self):self.session.close()

    def cycle(self):
        self.fixture.now+=.010001
        return self.session.cycle()

    def test_unknown_transition_does_not_send_further_queries(self):
        self.session.start()
        before=len(self.bus.frames)
        def become_unknown(snapshot,now):self.session.controller.state='UNKNOWN'
        self.session.controller.tick=become_unknown
        self.assertEqual(self.cycle(),'UNKNOWN')
        self.assertEqual(len(self.bus.frames),before)

    def test_wire_to_hold_and_confirmed_global_stop(self):
        self.session.start()
        for _ in range(600):
            if self.cycle()=='HOLD':break
        self.assertEqual(self.session.controller.state,'HOLD')
        self.assertEqual(self.bus.states,{1:1,2:1,3:1,4:1})
        self.session.stop()
        self.assertEqual(self.session.controller.state,'STOPPING')
        self.cycle()
        self.assertEqual(self.session.controller.state,'DISABLED')
        for f in self.bus.frames:
            cid=int.from_bytes(f[13:17],'little')
            if cid==0x7FF:
                self.assertIn(f[21],(1,2,3,4));self.assertEqual(f[23],0xCC)
            else:self.assertIn(cid,(1,2,3,4))

    def test_old_mapping_rejected_before_enable(self):
        self.reader.state.mapping={i:dict(m) for i,m in self.mapping.items()}
        self.reader.state.mapping[3]['zero_offset_rad']+=.1
        with self.assertRaises(ValueError):self.session.start()
        self.assertEqual(self.bus.frames,[])

    def test_close_reports_unknown_until_feedback_verification(self):
        self.session.start();self.cycle();self.session.close()
        self.assertTrue(self.bus.closed)
        self.assertEqual(self.session.controller.state,'UNKNOWN')


if __name__=='__main__':unittest.main()
