"""Offline tests: never opens a physical serial port."""
import math
import unittest
from unittest.mock import Mock,patch
import four_axis_monitor as m


def populate(s,i):
    for rid,value in {7:0,8:i,9:0,10:1,21:12.5,22:45.,23:54.}.items():
        payload=bytes([i,0,0x33,rid])+m.struct.pack('<I' if rid in (7,8,9,10) else '<f',value)
        s.feed(0,payload,1.)


class Tests(unittest.TestCase):
    def test_mapping(self):
        d=m.load_mapping();self.assertEqual(set(d),{1,2,3,4})
        self.assertEqual([d[i]['hardware_sign'] for i in m.IDS],[-1,1,1,-1])
    def test_allowlist(self):
        for i in m.IDS:
            for rid in (None,)+m.REGS:
                p=m.usb_packet(i,rid)
                self.assertEqual(len(p),30);self.assertEqual(p[13:15],b'\xff\x07')
                self.assertEqual(p[21],i);self.assertIn(p[23],(0x33,0xCC))
        for i,rid in [(5,None),(0,8),(1,3),(3,80),(1,0x55)]:
            with self.assertRaises(ValueError):m.request(i,rid)
    def test_fragmentation(self):
        p=bytes.fromhex('aa 11 08 00 00 00 00 04 78 be 7f f7 fe 1a 19 55')
        s=m.Stream();self.assertEqual(s.feed(b'x'+p[:4]),[])
        self.assertEqual(s.feed(p[4:]+p),[(0,p[7:15])]*2)
    def test_decode_mapping(self):
        # Historical 2026-10-30 trace has its own calibration fixture; live
        # manual re-zero must not change the meaning of this captured trace.
        mapping=m.load_mapping()
        mapping[3]['zero_offset_rad']=-0.4289692530708784
        mapping[4]['zero_offset_rad']=-0.47665369649805456
        s=m.State(mapping);populate(s,3);populate(s,4)
        s.feed(0,bytes.fromhex('03 7c a0 7f f7 ff 1b 1a'),2.)
        s.feed(0,bytes.fromhex('04 78 be 7f f7 fe 1a 19'),2.)
        self.assertAlmostEqual(s.feedback[3]['model_deg'],5.704661041,places=7)
        self.assertAlmostEqual(s.feedback[4]['model_deg'],13.289018824,places=7)
        self.assertTrue(s.fresh(3,2.2));self.assertFalse(s.fresh(3,3.1))
    def test_conflicting_id(self):
        s=m.State(m.load_mapping());populate(s,1)
        s.feed(0,bytes.fromhex('01 00 33 0a 02 00 00 00'),2.)
        self.assertFalse(s.ready(1));self.assertIn(10,s.conflicts[1])
    def test_j5_ignored(self):
        s=m.State(m.load_mapping())
        s.feed(0,bytes.fromhex('05 7f f6 7f f7 ff 1c 1b'),1.)
        self.assertNotIn(5,s.feedback);self.assertIn('忽略',s.notice)
    def test_no_decode_without_ranges(self):
        s=m.State(m.load_mapping())
        s.feed(0,bytes.fromhex('03 7c a0 7f f7 ff 1b 1a'),1.)
        self.assertEqual(s.feedback,{})
    def test_live_reader_only_reads_and_close_no_transmit(self):
        serial=Mock();serial.in_waiting=0;serial.read.return_value=b''
        serial.write.side_effect=lambda p:len(p)
        mapping=m.load_mapping()
        with patch.object(m.Path,'open',return_value=Mock()) as opened:
            r=m.Reader('FAKE',mapping,serial_factory=lambda *a,**kw:serial)
            for j in range(200):r.tick(j*.06)
            sent=[call.args[0] for call in serial.write.call_args_list]
            self.assertTrue(sent)
            for p in sent:
                self.assertEqual(p[13:15],b'\xff\x07')
                self.assertIn(p[21],m.IDS);self.assertIn(p[23],(0x33,0xCC))
            n=len(sent);r.close();self.assertEqual(serial.write.call_count,n)


if __name__=='__main__':unittest.main()
