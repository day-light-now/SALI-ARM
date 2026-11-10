import sys
import unittest
import numpy as np
from model import Simulation, build, mujoco, URDF, MASS, HOME, WORKSPACE
sys.path.insert(0,str(WORKSPACE/'verification'))
from gravity_model import GravityModel
import json


class DynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.model = build()

    def test_gravity_matches_independent_model(self):
        sim = Simulation(self.model)
        audit = json.loads(MASS.read_text(encoding='utf-8'))
        reference = GravityModel(URDF,audit['links'],[0,9.80665,0])
        for degrees in [[0,0,0,0],[25,65,0,-25],[-30,100,20,-40]]:
            sim.data.qpos[:] = np.deg2rad(degrees)
            np.testing.assert_allclose(sim.gravity_torque(),reference.evaluate(sim.data.qpos)[0],atol=1e-8)

    def test_compensation_prevents_drift(self):
        sim = Simulation(self.model); sim.enter_demo(); sim.set_mode('float'); sim.step(2000)
        self.assertLess(np.max(np.abs(sim.data.qpos-HOME)),1e-6)

    def test_no_compensation_sags(self):
        sim = Simulation(self.model); sim.enter_demo(); sim.set_mode('off'); sim.step(2000)
        self.assertGreater(np.linalg.norm(sim.data.qpos-HOME),.05)

    def test_push_and_spring_return(self):
        sim = Simulation(self.model); sim.enter_demo(); sim.push(); sim.step(400)
        deviation = np.linalg.norm(sim.data.qpos-HOME)
        self.assertGreater(deviation,.01)
        sim.step(5000)
        self.assertLess(np.linalg.norm(sim.data.qpos-HOME),deviation*.15)
        self.assertLess(np.max(np.abs(sim.torque)),31)

    def test_float_does_not_return_to_old_target(self):
        sim = Simulation(self.model); sim.enter_demo(); sim.set_mode('float'); sim.push(); sim.step(400); sim.step(3000)
        self.assertGreater(np.linalg.norm(sim.data.qpos-HOME),.03)

    def test_force_rejection(self):
        sim = Simulation(self.model)
        for force in [[0,0,100],[0,0,float('nan')],[1,2]]:
            with self.assertRaises(ValueError): sim.push(force)
        with self.assertRaises(ValueError): sim.set_mode('real')

    def test_startup_and_reset_are_zero(self):
        sim = Simulation(self.model)
        np.testing.assert_array_equal(sim.data.qpos, np.zeros(4))
        np.testing.assert_array_equal(sim.target, np.zeros(4))
        sim.step(2000)
        self.assertLess(np.max(np.abs(sim.data.qpos)),1e-8)
        sim.enter_demo()
        np.testing.assert_allclose(sim.data.qpos, HOME)
        sim.push(); sim.step(100); sim.reset()
        np.testing.assert_array_equal(sim.data.qpos, np.zeros(4))
        np.testing.assert_array_equal(sim.target, np.zeros(4))
        np.testing.assert_array_equal(sim.data.qvel, np.zeros(4))
        self.assertEqual(sim.force_until, 0)

if __name__ == '__main__': unittest.main()
