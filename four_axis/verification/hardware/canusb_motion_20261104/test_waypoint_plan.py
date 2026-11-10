import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/slaihand_bringup/scripts'))
from joint_waypoint_plan import compile_plan, assess_waypoints


class Plans(unittest.TestCase):
    bounds=[tuple(map(math.radians,b)) for b in [(-80,200),(-10,190),(-90,90),(-50,90)]]
    def plan(self, degrees=None, mode='relative_start_deg', move=4, dwell=1):
        return dict(coordinates=mode,waypoints=[dict(degrees=degrees or [2,1,-1,2],move_s=move,dwell_s=dwell)])
    def test_independent_relative(self):
        p,w=compile_plan(self.plan(),[.1]*4,self.bounds)
        self.assertEqual(len(p),3)
        self.assertAlmostEqual(p[1][1][2],.1-math.radians(1))
        self.assertEqual(w[0]['end_s'],5)
    def test_full_absolute_limits(self):
        p,_=compile_plan(self.plan([200,190,90,90],'absolute_deg',1000),[0]*4,self.bounds)
        self.assertEqual(p[-1][1],[b[1] for b in self.bounds])
    def test_limit_rejected(self):
        with self.assertRaises(ValueError):compile_plan(self.plan([201,0,0,0]),[0]*4,self.bounds)
    def test_quintic_peak_rejected(self):
        with self.assertRaises(ValueError):compile_plan(self.plan([10,0,0,0],move=.2),[0]*4,self.bounds)
    def test_ten_degrees_in_four_seconds_allowed(self):
        points,_=compile_plan(self.plan([10,0,0,0],move=4),[0]*4,self.bounds)
        self.assertAlmostEqual(points[1][1][0],math.radians(10))
    def test_exact_speed_boundary(self):
        move=1.875*math.radians(10)/1.
        compile_plan(self.plan([10,0,0,0],move=move),[0]*4,self.bounds)
        with self.assertRaises(ValueError):compile_plan(self.plan([10,0,0,0],move=move-.001),[0]*4,self.bounds)
    def test_second_segment_checked(self):
        d=self.plan();d['waypoints'].append(dict(degrees=[-10,0,0,0],move_s=.25,dwell_s=1))
        with self.assertRaises(ValueError):compile_plan(d,[0]*4,self.bounds)
    def test_invalid_values(self):
        for v in (float('nan'),float('inf'),True,'2'):
            with self.subTest(v=v),self.assertRaises(ValueError):compile_plan(self.plan([v,0,0,0]),[0]*4,self.bounds)
    def test_missing_axes(self):
        with self.assertRaises(ValueError):compile_plan(self.plan([1,2,3]),[0]*4,self.bounds)
    def test_ambiguous_coordinates(self):
        with self.assertRaises(ValueError):compile_plan(self.plan(mode='relative'),[0]*4,self.bounds)
    def test_dwell_required(self):
        with self.assertRaises(ValueError):compile_plan(self.plan(dwell=0),[0]*4,self.bounds)
    def test_action_success_without_motion_fails(self):
        _,w=compile_plan(self.plan(),[0]*4,self.bounds)
        samples=[(4.7+i*.01,[0]*4) for i in range(30)]
        self.assertFalse(assess_waypoints(w,samples)[0]['passed'])
    def test_no_samples_not_zero_error(self):
        _,w=compile_plan(self.plan(),[0]*4,self.bounds)
        self.assertIsNone(assess_waypoints(w,[])[0]['max_error_deg'])
    def test_arrival_observed(self):
        _,w=compile_plan(self.plan(),[0]*4,self.bounds)
        samples=[(4.7+i*.01,w[0]['target_rad']) for i in range(30)]
        self.assertTrue(assess_waypoints(w,samples)[0]['passed'])


if __name__=='__main__':unittest.main()
