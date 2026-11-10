"""Real backend/HTTP reconnect callbacks, lifecycle mocked: never opens hardware."""
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import test_control_panel_callbacks as cb
from control_panel_connection import DriverConnection


class ConnectionCase(cb.CallbackCase):
    def tearDown(self):
        self.backend._connection_cancel.set()
        time.sleep(.12)
        super().tearDown()

    def wait_phase(self, phase):
        end=time.monotonic()+3
        while time.monotonic()<end:
            if self.backend._connection_state['phase']==phase:return
            time.sleep(.02)
        self.fail(str(self.backend._connection_state))

    def manager(self, success=True):
        manager=Mock()
        manager.inspect.return_value={'port_present':True,'launch_pids':[],'port_owner_pids':[]}
        def restart(reset_usb, cancelled, progress):
            if not success:raise RuntimeError('serial is occupied')
            def feed():
                for _ in range(12):
                    if cancelled.wait(.1):return
                    self.store(stage=0)
            threading.Thread(target=feed,daemon=True).start()
            return SimpleNamespace(poll=lambda:None),Path('/tmp/fake-driver.log')
        manager.restart.side_effect=restart
        self.backend._connection_manager=manager
        return manager

    def test_connection_check_is_readonly(self):
        manager=self.manager()
        self.assertEqual(self.call('GET','/api/connection')[0],200)
        manager.restart.assert_not_called()
        self.assertEqual(self.spawned,[])

    def test_unknown_requires_explicit_review(self):
        manager=self.manager();self.store(stage=9,valid=0)
        code,body=self.call('POST','/api/connection/reconnect',{})
        self.assertEqual((code,body['code']),(409,'POWER_REVIEW'))
        manager.restart.assert_not_called()

    def test_active_stage_refuses_reconnect(self):
        manager=self.manager();self.store(stage=5)
        self.assertEqual(self.call('POST','/api/connection/reconnect',{'ack_power_cycle':True})[0],409)
        manager.restart.assert_not_called()

    def test_reconnect_locks_motion_until_manual_reset(self):
        manager=self.manager();self.store(stage=9,valid=0)
        self.assertEqual(self.call('POST','/api/connection/reconnect',{'ack_power_cycle':True})[0],202)
        self.assertEqual(self.hold()[0],409)
        self.wait_phase('connected')
        self.assertTrue(self.gate()['locked'])
        self.assertEqual(self.hold()[0],409)
        self.assertEqual(self.call('POST','/api/reset',{})[0],202)
        self.assertTrue(self.gate()['locked'])
        self.store(stage=0);self.backend._check_stop_gate(time.monotonic())
        self.assertFalse(self.gate()['locked'])
        self.assertEqual(self.spawned,[])
        self.assertTrue(self.backend._reset_queue.empty())
        manager.cancel_owned_start.assert_not_called()

    def test_duplicate_and_stop_during_restart(self):
        manager=Mock();ready=threading.Event()
        def pending(*_):
            ready.set();self.backend._connection_cancel.wait(2)
            return SimpleNamespace(poll=lambda:None),Path('/tmp/fake.log')
        manager.restart.side_effect=pending;self.backend._connection_manager=manager
        self.assertEqual(self.call('POST','/api/connection/reconnect',{})[0],202)
        self.assertTrue(ready.wait(1))
        self.assertEqual(self.call('POST','/api/connection/reconnect',{'ack_power_cycle':True})[0],409)
        self.assertEqual(self.stop()[0],202)
        self.wait_phase('failed')
        self.assertTrue(self.gate()['locked'])
        manager.cancel_owned_start.assert_called_once()
        self.assertEqual(self.spawned,[])

    def test_failed_restart_stays_locked(self):
        self.manager(success=False)
        self.assertEqual(self.call('POST','/api/connection/reconnect',{})[0],202)
        self.wait_phase('failed')
        self.assertIn('occupied',self.backend._connection_state['message'])
        self.assertTrue(self.gate()['locked'])
        self.assertEqual(self.spawned,[])


class LifecycleCase(unittest.TestCase):
    def test_multiple_drivers_never_signalled_or_spawned(self):
        driver=DriverConnection('/tmp/test-connection')
        with patch('control_panel_connection.launches',return_value=[1,2]), \
             patch('control_panel_connection.os.kill') as kill, \
             patch('control_panel_connection.subprocess.Popen') as popen:
            with self.assertRaisesRegex(RuntimeError,'多个'):
                driver.restart(False,threading.Event(),lambda _:None)
            kill.assert_not_called();popen.assert_not_called()

    def test_occupied_port_never_spawns(self):
        driver=DriverConnection('/tmp/test-connection')
        with patch('control_panel_connection.launches',return_value=[]), \
             patch.object(driver,'owners',return_value=[123]), \
             patch('control_panel_connection.subprocess.Popen') as popen:
            with self.assertRaisesRegex(RuntimeError,'占用'):
                driver.restart(False,threading.Event(),lambda _:None)
            popen.assert_not_called()


if __name__=='__main__':unittest.main(verbosity=2)
