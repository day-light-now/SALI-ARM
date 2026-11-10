"""Calculation-only endpoint/model regression; no motor interface."""
import unittest
import numpy as np
from gravity_demo import calculate, DATA, LIMITS
from gravity_sim_bridge import validate_status,NAMES

class DemoTests(unittest.TestCase):
    def test_vendor_motor_mass_targets(self):
        targets=sorted(c['target_mass_kg'] for c in DATA['motor_corrections'])
        self.assertEqual(targets,[.362,.921,.921,1.375,1.375])
    def test_zero_and_limits(self):
        for q in [[0]*4,[p[0]+.001 for p in LIMITS],[p[1]-.001 for p in LIMITS]]:
            self.assertTrue(np.isfinite(calculate({'degrees':q})['torque']).all())
    def test_mass_sensitivity(self):
        a=calculate({'degrees':[10,20,-5,5]})['torque']
        b=calculate({'degrees':[10,20,-5,5],'scale':1.3})['torque']
        np.testing.assert_allclose(b,np.array(a)*1.3)
    def test_reject_invalid(self):
        for p in [{'degrees':[0,0,0,60]}, {'degrees':[0,0,0,float('nan')]},
                  {'degrees':[0,0,0,True]}, {'degrees':[0]*4,'scale':2}]:
            with self.assertRaises(ValueError):calculate(p)
    def test_horizontal_j1_and_gravity_sign(self):
        a=np.array(calculate({'degrees':[0]*4,'gravity':'+y'})['torque'])
        b=np.array(calculate({'degrees':[0]*4,'gravity':'-y'})['torque'])
        self.assertGreater(abs(a[0]),.01)
        np.testing.assert_allclose(a,-b)
        with self.assertRaises(ValueError):calculate({'degrees':[0]*4,'gravity':'-z'})
    def test_sim_transport_guard(self):
        valid={'hardware_plugin':'slaihand_hardware/OpenArmJointFake','transport':'fake','joints':NAMES,'ready':True}
        self.assertEqual(validate_status(valid),valid)
        for patch in [{'transport':'can'},{'hardware_plugin':'OpenArmHW'},{'ready':False},{'joints':NAMES[:3]}]:
            with self.assertRaises(ValueError):validate_status({**valid,**patch})

if __name__=='__main__':unittest.main()
