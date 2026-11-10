"""Separate calibration: full real parser/controller path, synthetic bus only."""
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from .test_holding_bias import LoadedPlant
from .test_watchdog_check import Clock
from .test_reader_fixture import make_reader
from .native_mit_console import NativeTrialRequest,run_native_trial
from .watchdog_check import make_event_writer
from .load_calibration import save_record,inventory
from .openarm_mit import NativeMITAcceptance
from four_axis_monitor import load_mapping


class Tests(unittest.TestCase):
    def run_point(self,joint):
        clock=Clock();mapping=load_mapping();bus=LoadedPlant(mapping,clock,joint=joint)
        reader=make_reader('FAKE_ONLY',mapping,serial_factory=lambda *a,**k:bus)
        stream=io.StringIO()
        try:
            with patch.dict('sys.modules',{'slai_console.gravity_feedforward':None,
                'verification.gravity_model':None,'mujoco':None}):
                result=run_native_trial(reader,NativeTrialRequest(True,True,True,f'cal-j{joint}',
                    'calibration_hold',joint=joint,holding_bias_acknowledged=True),
                    make_event_writer(stream,lambda row:None),threading.Event(),
                    connection_id='fake-cal',clock=clock.now,sleep=clock.sleep)
            return result,bus.history,stream.getvalue()
        finally:reader.close()

    def test_each_axis_hold_records_actual_pose_and_no_trajectory(self):
        for joint in (1,2,3,4):
            with self.subTest(joint=joint):
                result,history,audit=self.run_point(joint)
                self.assertEqual(result['reason'],'calibration_hold_complete',result)
                self.assertEqual(result['final_controller_state'],'DISABLED')
                point=result['calibration_record']
                self.assertTrue(point['accepted'],point)
                self.assertFalse(point['simulated_mass_used'])
                self.assertTrue(point['not_a_gravity_model'])
                self.assertGreater(point['holding_request_estimate_model_nm'],1.2)
                self.assertEqual(len(point['pose_rad']),4)
                self.assertNotIn('PLAN_ACCEPTED_NOT_SENT',audit)
                for _,cid,payload in history:
                    if cid==0x7ff:self.assertIn(payload[0],(1,2,3,4))
                    elif payload[:7]!=b'\xff'*7 or payload[-1]==0xfc:self.assertEqual(cid,joint)

    def test_archive_is_pose_scoped_deduplicated_and_not_auto_applied(self):
        result,_,_=self.run_point(2)
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);trial=d/'trial.json';trial.write_text(json.dumps(result),encoding='utf-8')
            path=d/'records.jsonl'
            record=save_record(result,trial,pose_label='pose-A',approach='positive',
                load_note='known test disturbance',conditions_confirmed=True,path=path)
            self.assertFalse(record['full_range_calibration_complete'])
            self.assertFalse(record['auto_apply_authorized'])
            self.assertEqual(len(inventory(path)[2]),1)
            with self.assertRaisesRegex(ValueError,'already'):
                save_record(result,trial,pose_label='same',approach='negative',load_note='same',
                    conditions_confirmed=True,path=path)
            with self.assertRaises(ValueError):
                save_record(result,trial,pose_label='other',approach='positive',load_note='test',
                    conditions_confirmed=False,path=d/'unconfirmed.jsonl')
            bad={**result,'fresh_stop_confirmation':False}
            with self.assertRaises(ValueError):
                save_record(bad,trial,pose_label='other',approach='positive',load_note='test',
                    conditions_confirmed=True,path=d/'bad.jsonl')

    def test_regular_j1_and_j5_are_not_opened_by_calibration_ticket(self):
        for joint in (1,5):
            with self.assertRaises(ValueError):
                NativeMITAcceptance(True,True,True,'test','test',joint=joint).validate()
        with self.assertRaises(ValueError):
            NativeMITAcceptance(True,True,True,'test','test',joint=5,
                holding_bias_acknowledged=True,calibration_only=True).validate()


if __name__=='__main__':unittest.main()
