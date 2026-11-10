"""Clock-domain and explicit read-schedule invariants; fake Reader only."""
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from .clock import now, CLOCK_NAME, describe


class TimingTests(unittest.TestCase):
    def test_one_high_resolution_monotonic_clock(self):
        self.assertIs(now,time.perf_counter)
        self.assertTrue(describe()['monotonic'])
        self.assertFalse(describe()['adjustable'])

    def test_explicit_rates_and_observation_cache_have_no_serial_access(self):
        from .adapters import ReadOnlyWorker,load_mapping
        from four_axis_monitor import State
        mapping=load_mapping()
        class FakeReader:
            instance=None
            def __init__(self,port,config):
                self.state=State(config)
                self.tx=self.rx=0
                self.next_send=0.
                self.path='synthetic-clock-probe'
                self.stamps=[]
                self.closed=False
                FakeReader.instance=self
            def tick(self,stamp):
                if stamp>=self.next_send:
                    self.tx+=1
                    self.stamps.append(stamp)
                    self.next_send=stamp+.05
            def close(self):self.closed=True
        with patch('slai_console.adapters.Reader',FakeReader),patch('serial.Serial',side_effect=AssertionError('No serial')):
            worker=ReadOnlyWorker('FAKE',mapping)
            self.assertEqual(worker.interval,.05)
            with self.assertRaises(ValueError):worker.set_interval(.001)
            worker.set_interval(.01)
            worker.start()
            try:
                end=now()+2
                while now()<end and worker.peek().get('tx',0)<3:
                    time.sleep(.005)
                sample=worker.peek()
                self.assertEqual(sample['clock'],CLOCK_NAME)
                self.assertEqual(sample['query_interval_s'],.01)
                self.assertGreaterEqual(sample['tx'],3)
                sample['rows'][5]='mutated copy'
                self.assertNotIn(5,worker.peek()['rows'])
            finally:
                worker.requestInterruption()
                self.assertTrue(worker.wait(3000))
            self.assertTrue(FakeReader.instance.closed)
            self.assertEqual(worker.peek()['rows'],{})
            self.assertTrue(all(b-a>=.0099 for a,b in zip(FakeReader.instance.stamps,FakeReader.instance.stamps[1:])))


if __name__=='__main__':
    unittest.main()
