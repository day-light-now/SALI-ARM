"""Offline policy, CAN transport and Qt integration tests. Never opens COM."""
import dataclasses
import math
import tempfile
import time
import unittest
from unittest.mock import patch
import numpy as np
from .core import (Audit, CommandGateway, Context, PreviewSlider, Rejected,
                   Ticket, encoder_rad, model_rad, minjerk, validate_waypoints)
from .clock import now as clock_now


def ready(**changes):
    return dataclasses.replace(Context(mode='sim',connected=True,enabled=True,fresh=True),**changes)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.preview = PreviewSlider()
        self.gateway = CommandGateway(self.preview)
        self.ticket = self.preview.edit(3,10.)

    def test_six_rules(self):
        cases = [(Ticket(0,((5,0.),)),ready(),'R1'),
                 (self.ticket,ready(mode='live'),'R2'),
                 (self.ticket,ready(fresh=False),'R3'),
                 (self.ticket,ready(enabled=False),'R4'),
                 (self.preview.edit(3,91.),ready(),'R5'),
                 (self.ticket,ready(),'R6')]
        for ticket,ctx,rule in cases:
            with self.subTest(rule=rule):
                self.assertTrue(self.gateway.reason(ticket,ctx).startswith(rule))
                with self.assertRaises(Rejected):
                    self.gateway.commit(ticket,ctx)

    def test_context_guards(self):
        for ctx in (ready(connected=False),ready(fault=True),ready(busy=True)):
            with self.assertRaises(Rejected):
                self.gateway.commit(self.ticket,ctx)

    def test_preview_is_local_and_one_shot(self):
        self.assertEqual(self.preview.state,'PREVIEW')
        self.assertEqual(self.gateway.commit(self.ticket,ready()),{3:10.})
        with self.assertRaises(Rejected):
            self.gateway.commit(self.ticket,ready())

    def test_invalidation_even_same_value(self):
        self.preview.clear()
        new = self.preview.edit(3,10.)
        self.assertNotEqual(new,self.ticket)
        with self.assertRaises(Rejected):
            self.gateway.commit(self.ticket,ready())

    def test_context_defaults_are_closed(self):
        self.assertTrue(self.gateway.reason(self.ticket,Context()).startswith('R3'))

    def test_single_motion_duration_is_bound_and_audited(self):
        self.assertEqual(self.ticket.duration,3.)
        altered=dataclasses.replace(self.ticket,duration=4.)
        with self.assertRaises(Rejected):
            self.gateway.commit(altered,ready())
        events=[]
        self.gateway.audit=lambda *a,**k:events.append(k)
        self.gateway.commit(self.ticket,ready())
        self.assertEqual(events[0]['duration'],3.)

    def test_trajectory_ticket_binds_entire_plan(self):
        ticket=self.preview.trajectory([[0,0,0,0],[10,20,30,40]],4.)
        changed=dataclasses.replace(ticket,plan=((0.,0.,0.,0.),(20.,20.,30.,40.)))
        with self.assertRaises(Rejected):
            self.gateway.commit(changed,ready())
        events=[]
        self.gateway.audit=lambda *a,**k:events.append(k)
        self.gateway.commit(ticket,ready())
        self.assertEqual(events[0]['waypoints'],ticket.plan)
        self.assertEqual(events[0]['duration'],4.)
        with self.assertRaises(Rejected):
            self.gateway.commit(ticket,ready())

    def test_trajectory_invalid_later_waypoint(self):
        with self.assertRaises(Rejected):
            self.preview.trajectory([[0,0,0,0],[0,0,100,0]],4.)

    def test_j5_cannot_enter_preview(self):
        for joint in (0,5,6):
            with self.assertRaises(Rejected):
                self.preview.edit(joint,0.)

    def test_nan_and_infinite_never_execute(self):
        for value in (math.nan,math.inf,-math.inf):
            with self.assertRaises(Rejected):
                self.preview.edit(1,value)

    def test_raw16_and_j3_reference(self):
        self.assertEqual(encoder_rad(0,12.5),-12.5)
        self.assertEqual(encoder_rad(65535,12.5),12.5)
        self.assertAlmostEqual(model_rad(0x7B9B,12.5,1,-.4289692530708784),0.,places=12)
        self.assertGreater(model_rad(0x7CA0,12.5,1,-.4289692530708784),0.)

    def test_j4_direction(self):
        self.assertGreater(model_rad(0x78BE,12.5,-1,-.47665369649805456),0.)

    def test_trajectory_validation(self):
        self.assertEqual(validate_waypoints([[0,0,0,0]],4),[(0.,0.,0.,0.)])
        for points,duration in (([],4),([[0]*5],4),([[0,191,0,0]],4),([[0]*4],0),([[math.nan]*4],4)):
            with self.assertRaises(Rejected):
                validate_waypoints(points,duration)
        self.assertEqual(minjerk(0),0)
        self.assertEqual(minjerk(1),1)
        self.assertAlmostEqual(minjerk(.5),.5)

    def test_audit_persists_command_id(self):
        with tempfile.TemporaryDirectory() as directory:
            audit=Audit(directory)
            audit('CMD','motion',command_id='abc')
            audit.close()
            self.assertIn('abc',audit.path.read_text(encoding='utf-8'))

    def test_audit_closes_on_normal_and_exceptional_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            with Audit(directory) as normal:
                normal('INFO','normal')
            self.assertTrue(normal.file.closed)
            with self.assertRaisesRegex(RuntimeError,'original failure'):
                with Audit(directory) as failed:
                    failed('INFO','before exception')
                    raise RuntimeError('original failure')
            self.assertTrue(failed.file.closed)
            self.assertIn('before exception',failed.path.read_text(encoding='utf-8'))


class PhysicalTransportTests(unittest.TestCase):
    def test_whitelist_all_packet_bytes(self):
        from .adapters import Reader, load_mapping
        class Port:
            in_waiting=0
            def __init__(self): self.frames=[]
            def reset_input_buffer(self): pass
            def read(self,n): return b''
            def write(self,data): self.frames.append(data);return len(data)
            def close(self): pass
        port=Port()
        r=Reader('FAKE',load_mapping(),serial_factory=lambda *a,**k:port)
        for i in range(150): r.tick(i*.1)
        before=len(port.frames)
        r.close()
        self.assertEqual(before,len(port.frames))
        self.assertGreater(before,100)
        for frame in port.frames:
            self.assertEqual(int.from_bytes(frame[13:17],'little'),0x7FF)
            self.assertIn(frame[21],(1,2,3,4))
            self.assertIn(frame[23],(0x33,0xCC))


class QtIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app=QApplication.instance() or QApplication([])

    def test_workstation_offline(self):
        from .app import Console
        from PySide6.QtTest import QSignalSpy
        from .adapters import SimAdapter
        from .adapters import mujoco
        real_renderer=mujoco.Renderer
        def delayed_renderer(*args,**kwargs):
            # Reproduce cold GL startup deterministically, independent of GPU cache.
            time.sleep(3.)
            return real_renderer(*args,**kwargs)
        with patch('serial.Serial',side_effect=AssertionError('No COM in offline tests')), \
             patch('slai_console.adapters.mujoco.Renderer',side_effect=delayed_renderer):
            w=Console()
            w.timer.stop()
            try:
                np.testing.assert_allclose(w.sim.sim.data.qpos,0,atol=1e-12)
                w.tick()
                self.assertFalse(w.execute.isEnabled())
                w.start_shadow()
                self.assertIsNone(w.shadow_worker)
                self.assertTrue(w.stop_shadow())
                w.edit_target(3,10.)
                np.testing.assert_allclose(w.sim.sim.target,0,atol=1e-12)
                # GL is owned exclusively by the render worker (including previews).
                w.execute_preview()
                self.assertTrue(w.sim.busy)
                for _ in range(500): w.sim.step(20)
                self.assertLess(abs(np.rad2deg(w.sim.sim.data.qpos[2])-10),.3)
                w.edit_target(2,30.)
                ticket=w.preview.ticket
                w.nav.setCurrentRow(0)
                self.assertFalse(w.preview.matches(ticket))
                w.add_waypoint()
                self.assertEqual(w.waypoints.columnCount(),6)
                points=w.read_waypoints()
                w.sim.trajectory(points,1.)
                with self.assertRaises(ValueError):
                    w.sim.trajectory(points,1.)
                w.stop()
                self.assertFalse(w.sim.busy)
                w.mode_box.setCurrentIndex(1)
                self.assertIsNone(w.worker)
                self.assertEqual(w.mode,'live')
                self.assertTrue(all(not s.isEnabled() for s in w.sliders))
                w.edit_target(1,20.)
                w.execute_preview()
                self.assertFalse(w.sim.busy)
                w.tick()
                self.assertEqual(w.overview.item(0,1).text(),'—')
                self.assertEqual(w.overview.item(4,5).text(),'禁发送')
                w.on_sample(dict(rows={1:{'model_deg':10}},tx=1,rx=1,at=time.monotonic()))
                self.assertEqual(w.fresh_rows(),{})
                class Stuck:
                    def requestInterruption(self): pass
                    def wait(self,timeout): return False
                w.worker=Stuck()
                w.mode_box.setCurrentIndex(0)
                self.assertEqual(w.mode,'live')
                self.assertEqual(w.mode_box.currentIndex(),1)
                w.worker=None
                w.mode_box.setCurrentIndex(0)
                self.assertTrue(all(s.isEnabled() for s in w.sliders))
                w.sim.sim.reset()
                w.set_dynamics(1)
                self.assertEqual(w.sim.sim.mode,'float')
                w.push_sim()
                w.sim.step(400)
                self.assertGreater(np.linalg.norm(w.sim.sim.data.qpos),.001)
                w.sim.stop()
                frames=QSignalSpy(w.render_worker.frame)
                errors=QSignalSpy(w.render_worker.failed)
                w.render_worker.submit(w.sim.sim.data.qpos,{3:10.},w.sim.cam)
                deadline=time.monotonic()+20.
                while frames.count()==0 and errors.count()==0 and time.monotonic()<deadline:
                    self.app.processEvents()
                    time.sleep(.02)  # Explicitly release GIL for the Python render worker.
                self.app.processEvents()
                self.assertEqual(errors.count(),0,str(errors.at(0)) if errors.count() else '')
                self.assertGreater(frames.count(),0,'Render worker did not deliver a frame within 20 seconds')
                self.assertEqual(frames.at(0)[0].shape,(360,640,3))
                self.assertGreater(w.render_ms,0)
                for i in range(6):
                    w.nav.setCurrentRow(i)
                    self.assertEqual(w.inspector.currentIndex(),i)
                self.assertIsNone(w.worker)
                w.sim_feedback_at=clock_now()-1
                self.assertFalse(w.context().fresh)
            finally:
                w.close()
                self.app.processEvents()
            self.assertFalse(w.render_worker.isRunning())


if __name__=='__main__':
    unittest.main()
