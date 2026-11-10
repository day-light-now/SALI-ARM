"""Offline checks only. No serial ports are opened."""
import math
import unittest
from unittest.mock import Mock, patch
import threading
import tempfile
from pathlib import Path
import j3_return_reference as m


def state(q=None,code=0,v=0):
    return dict(position_rad=m.REFERENCE+.10 if q is None else q,
                velocity_rad_s=v,state_code=code,driver_temp_c=27,motor_temp_c=26)


class Tests(unittest.TestCase):
    def test_configuration(self):
        # The old one-shot return tool must reject the new manual zero; do not
        # update its frozen target just to make a historical test pass.
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            config=root/'src/slaihand_bringup/config/hardware_mapping.yaml'
            config.parent.mkdir(parents=True)
            for zero,expected in ((m.REFERENCE,True),(m.REFERENCE+.1,False)):
                config.write_text('  slaihand_joint3:\n    can_id: 3\n    hardware_sign: 1\n'
                    +f'    zero_offset_rad: {zero}\n  slaihand_joint4:\n',encoding='utf-8')
                with patch.object(m,'WORKSPACE',root):
                    self.assertEqual(m.reference_matches(),expected)

    def test_legacy_return_blocked_by_current_recalibration(self):
        self.assertFalse(m.reference_matches())
    def test_requests_and_whitelist(self):
        self.assertEqual(m.request(),bytes.fromhex('03 00 cc 00 00 00 00 00'))
        for rid in m.READS:m.validate_send(0x7FF,m.request(rid))
        for data in (m.ENABLE,m.DISABLE,m.torque_packet(0)):m.validate_send(3,data)
        for cid,data in [(1,m.ENABLE),(3,bytes.fromhex('ff ff ff ff ff ff ff fe')),
                         (3,bytes.fromhex('ff ff ff ff ff ff ff fb')),
                         (0x7FF,bytes.fromhex('03 00 55 09 d0 07 00 00'))]:
            with self.assertRaises(ValueError):m.validate_send(cid,data)
        with self.assertRaises(ValueError):m.request(80)
    def test_usb(self):
        data=m.torque_packet(0);p=m.usb_packet(3,data)
        self.assertEqual(len(p),30);self.assertEqual(p[13:15],b'\x03\x00')
        self.assertEqual(p[21:29],data)
    def test_fragmented_stream(self):
        packet=bytes.fromhex('aa 11 08 00 00 00 00 03 7c a0 7f f7 ff 1b 1a 55')
        s=m.Stream();self.assertEqual(s.feed(b'garbage'+packet[:6]),[])
        self.assertEqual(s.feed(packet[6:]+packet),[(0,packet[7:15])]*2)
    def test_torque_quantization_bound(self):
        for i in range(601):
            p=m.torque_packet(-.3+i*.001)
            raw=(p[6]<<8)|p[7]
            self.assertLessEqual(abs(raw*108/4095-54),m.CAP)
            m.validate_send(3,p)
        for bad in [float('nan'),float('inf'),.301,-.301]:
            with self.assertRaises(ValueError):m.torque_packet(bad)
    def test_reference_path(self):
        g=m.Guard(m.REFERENCE+.1)
        q,vd,tau=g.command(0,state());self.assertAlmostEqual(q,g.start)
        q,vd,tau=g.command(5,state());self.assertAlmostEqual(q,m.REFERENCE);self.assertEqual(vd,0)
        for i in range(601):
            q,v,t=g.command(i*.01,state())
            self.assertTrue(m.REFERENCE-1e-12<=q<=g.start+1e-12)
            self.assertLessEqual(abs(v),.04);self.assertLessEqual(abs(t),m.CAP)
        with self.assertRaises(ValueError):m.Guard(m.REFERENCE+.2)
    def test_fault_guards(self):
        g=m.Guard(m.REFERENCE+.1)
        g.check(state(code=1),.01,0)
        for s,age,t in [(state(code=13),.01,0),(state(code=1),.1,0),
                        (state(code=1,v=.2),.01,0),(state(q=m.REFERENCE-.1,code=1),.01,0),
                        (state(code=1),.01,7),({**state(code=1),'motor_temp_c':60},.01,0)]:
            with self.assertRaises(RuntimeError):g.check(s,age,t)
    def test_cancel_before_enable(self):
        link=Mock();link.prepare.return_value=m.Guard(m.REFERENCE+.1)
        stop=threading.Event();stop.set()
        with self.assertRaises(RuntimeError):m.run_return(link,stop,lambda *x:None)
        link.send.assert_not_called()
    def test_already_at_zero_does_not_enable(self):
        link=Mock();link.prepare.return_value=m.Guard(m.REFERENCE)
        self.assertIn('no enable',m.run_return(link,threading.Event(),lambda *x:None))
        link.send.assert_not_called()
    def test_prepare_refuses_mismatched_register(self):
        l=m.Link.__new__(m.Link);l.regs={**m.EXPECTED,9:0};l.send=Mock()
        with self.assertRaises(RuntimeError):l.prepare(threading.Event())
        self.assertNotIn(m.ENABLE,[call.args[-1] for call in l.send.call_args_list])
    def test_cleanup_disables_after_enable_attempt(self):
        l=m.Link.__new__(m.Link);l.enable_attempted=True;l.latest=None;l.at=None
        l.serial=Mock();l.log=Mock();l.send=Mock();l.event=Mock()
        def receive(): l.latest=state(code=0)
        l.receive=receive
        with patch.object(m.time,'sleep'):
            self.assertEqual(l.finish(),[])
        self.assertEqual([c.args for c in l.send.call_args_list][:3],[(3,m.DISABLE)]*3)
        l.serial.close.assert_called_once()

    def test_active_sequence_with_fake_clock(self):
        class Clock:
            t=100.
            def now(self):return self.t
            def sleep(self,delta):self.t+=delta
        clock=Clock()
        class Stop:
            def is_set(self):return False
            def wait(self,delta):clock.sleep(delta);return False
        class FakeLink:
            def __init__(self):
                self.latest=state();self.at=clock.t;self.sent=[];self.enabled=False
                self.start=clock.t
            def prepare(self,stop):return m.Guard(self.latest['position_rad'])
            def send(self,cid,payload):
                m.validate_send(cid,payload);self.sent.append((cid,payload))
                if payload==m.ENABLE:self.enabled=True;self.start=clock.t
            def receive(self):
                u=min(1.,(clock.t-self.start)/5.)
                s=10*u**3-15*u**4+6*u**5
                self.latest=state(q=m.REFERENCE+.1*(1-s),code=1 if self.enabled else 0)
                self.at=clock.t
            def event(self,**data):pass
        l=FakeLink()
        with patch.object(m.time,'monotonic',clock.now),patch.object(m.time,'sleep',clock.sleep):
            result=m.run_return(l,Stop(),lambda *x:None)
        self.assertIn('Trial finished',result)
        self.assertEqual(sum(p==m.ENABLE for _,p in l.sent),1)
        self.assertTrue(all(cid==3 for cid,_ in l.sent))
        self.assertGreater(len(l.sent),200);self.assertLess(len(l.sent),400)
        self.assertLess(clock.t-100,6.1)


if __name__=='__main__':unittest.main()
