import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/slaihand_bringup/scripts'))
from point_to_point import StableEndpoint,PointToPointSession,effective_document
from control_panel_model import preview_payload

class EndpointTests(unittest.TestCase):
    def test_continuous_error_and_velocity(self):
        v=StableEndpoint([0.]*4)
        for i in range(62):v.observe(i*.01,[.001]*4,[.01]*4)
        self.assertTrue(v.passed(.61))
        self.assertTrue(v.result(.61)['passed'])
    def test_cached_feedback_does_not_accumulate(self):
        v=StableEndpoint([0.]*4)
        for _ in range(100):v.observe(0.,[0.]*4,[0.]*4)
        self.assertFalse(v.passed(.5))
    def test_speed_resets_stability(self):
        v=StableEndpoint([0.]*4)
        for i in range(62):v.observe(i*.01,[0.]*4,[0.]*4)
        v.observe(.62,[0.]*4,[.051,0,0,0])
        self.assertFalse(v.passed(.62))
    def test_static_bias_cannot_pass(self):
        v=StableEndpoint([0.]*4)
        for i in range(200):v.observe(i*.01,[math.radians(.6),0,0,0],[0.]*4)
        self.assertFalse(v.passed(1.99))
    def test_feedback_gap_resets_stability(self):
        v=StableEndpoint([0.]*4)
        for i in range(62):v.observe(i*.01,[0.]*4,[0.]*4)
        v.observe(.8,[0.]*4,[0.]*4)
        self.assertFalse(v.passed(.8))
    def test_empty_is_unknown_not_zero_error(self):
        self.assertIsNone(StableEndpoint([0.]*4).result(0)['max_error_deg'])

class RetimingTests(unittest.TestCase):
    def session(self):return PointToPointSession([dict(arrival_s=1.,end_s=2.,target_rad=[0.]*4)],2.)
    def test_large_lag_pauses_official_clock(self):
        s=self.session()
        self.assertEqual(s.observe(.5,1,[0.]*4,[0.]*4,[.04,0,0,0],1),0.)
    def test_endpoint_wait_then_timeout(self):
        s=self.session()
        s.observe(1.9,1,[.02,0,0,0],[0.]*4,[0.]*4,1)
        with self.assertRaisesRegex(RuntimeError,'ENDPOINT_NOT_SETTLED'):
            s.observe(1.9,4.01,[.02,0,0,0],[0.]*4,[0.]*4,4.01)
    def test_stale_feedback_stops(self):
        with self.assertRaisesRegex(RuntimeError,'STALE'):
            self.session().observe(0,1,[0.]*4,[0.]*4,[0.]*4,0)
    def test_phase_regression_rejected(self):
        s=self.session();s.observe(.5,1,[0.]*4,[0.]*4,[0.]*4,1)
        with self.assertRaisesRegex(RuntimeError,'REGRESSED'):s.observe(.4,1.01,[0.]*4,[0.]*4,[0.]*4,1.01)
    def test_preview_adjustment_explicit_and_mode_bound(self):
        d=dict(coordinates='relative_start_deg',waypoints=[dict(degrees=[10,0,0,0],move_s=2,dwell_s=.5)])
        p=preview_payload(d,[0.]*4,[(-4,4)]*4,mode='point_to_point')
        self.assertEqual(p['execution_mode'],'point_to_point')
        self.assertEqual(p['frozen']['waypoints'][0]['dwell_s'],1.)
        self.assertEqual(p['budget']['retiming_allowance_s'],6.)
        self.assertEqual(d['waypoints'][0]['dwell_s'],.5)

if __name__=='__main__':unittest.main()
