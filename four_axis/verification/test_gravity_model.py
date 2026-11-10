"""Analytical and finite-difference tests with synthetic data, not SLAI calibration."""
import unittest
from pathlib import Path
import numpy as np
from gravity_model import GravityModel

class GravityTests(unittest.TestCase):
    def setUp(self):
        self.urdf=Path(__file__).parent.parent/'src/slaihand_description/urdf/slaihand_four_axis.urdf'
        self.properties={f'link{i}':{'mass_kg':.5+i*.1,'com_link_m':[.02,.01,.03]} for i in range(1,5)}
        self.model=GravityModel(self.urdf,self.properties,[0,0,-9.80665])

    def test_energy_gradient_matches_holding_torque(self):
        for q in [np.zeros(4),np.array([.2,.5,-.3,.2]),np.array([-.5,2.,.7,-1.])]:
            torque,_=self.model.evaluate(q)
            eps=1e-6; gradient=[]
            for i in range(4):
                delta=np.eye(4)[i]*eps
                gradient.append((self.model.evaluate(q+delta)[1]-self.model.evaluate(q-delta)[1])/(2*eps))
            np.testing.assert_allclose(torque,gradient,atol=1e-7)

    def test_vertical_j1_has_zero_gravity_torque(self):
        torque,_=self.model.evaluate([.2,.5,-.3,.2])
        self.assertAlmostEqual(torque[0],0.,places=10)

    def test_linear_mass_scaling(self):
        doubled={n:{**p,'mass_kg':p['mass_kg']*2} for n,p in self.properties.items()}
        other=GravityModel(self.urdf,doubled,[0,0,-9.80665])
        np.testing.assert_allclose(other.evaluate([.2,.5,-.3,.2])[0],2*self.model.evaluate([.2,.5,-.3,.2])[0])

    def test_zero_gravity(self):
        other=GravityModel(self.urdf,self.properties,[0,0,0])
        np.testing.assert_allclose(other.evaluate([.2,.5,-.3,.2])[0],0.)

    def test_invalid_input_rejected(self):
        for q in [[0,0,0], [0,0,0,2], [0,float('nan'),0,0]]:
            with self.assertRaises(ValueError): self.model.evaluate(q)
        with self.assertRaises(ValueError):
            GravityModel(self.urdf,{**self.properties,'link1':{'mass_kg':-1,'com_link_m':[0,0,0]}},[0,0,-9.8])

if __name__=='__main__': unittest.main()
