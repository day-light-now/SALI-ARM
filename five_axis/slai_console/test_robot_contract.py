"""Cross-path regressions: current calibration, real consumers and old ranges."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from .robot_contract import (ROOT, MODEL_PATH, model_limits_deg, load_mapping,
                             operating_limits_deg, audit_contract)
from .core import (LIMITS, PreviewSlider, CommandGateway, Context, Rejected,
                   validate_waypoints)
from .five_axis_config import load_settings, template_settings, build_envelopes
from .five_axis_monitor import load_mapping5
from .auto_load_acquisition import build_plan


class Tests(unittest.TestCase):
    def test_full_offline_consistency_check_is_not_motion_authorization(self):
        result = audit_contract()
        self.assertEqual(result['status'], 'OFFLINE_CONSISTENT')
        self.assertFalse(result['hardware_motion_authorized'])
        self.assertFalse(result['j5_enabled'])

    def test_current_sign_zero_and_roundtrip(self):
        mapping = load_mapping()
        self.assertEqual(mapping[1]['hardware_sign'], 1)
        self.assertEqual(mapping[1]['zero_offset_rad'], 1.4574273289082171)
        self.assertEqual({i: load_mapping5()[i] for i in mapping}, mapping)
        from .native_mit_console import load_mapping as native_mapping
        self.assertEqual(native_mapping(), mapping)
        for i, item in mapping.items():
            q = math.radians(10.)
            raw = item['zero_offset_rad'] + item['hardware_sign'] * q
            self.assertAlmostEqual(item['hardware_sign'] * (raw-item['zero_offset_rad']), q)

    def test_model_limits_are_read_from_xml_not_a_second_table(self):
        self.assertEqual(LIMITS, model_limits_deg())
        self.assertEqual(LIMITS[0], (-80., 200.))
        with tempfile.TemporaryDirectory() as folder:
            model = ET.parse(MODEL_PATH)
            model.getroot().find("joint[@name='slaihand_joint1']/limit").set('upper', str(math.radians(180)))
            path = Path(folder)/'changed.urdf'; model.write(path)
            self.assertAlmostEqual(model_limits_deg(path)[0][1], 180.)

    def test_gateway_and_waypoints_use_current_sector(self):
        context = Context(mode='sim', connected=True, enabled=True, fresh=True)
        for target, accepted in ((-80., True), (170., True), (200., True), (-150., False), (201., False)):
            preview = PreviewSlider(); gateway = CommandGateway(preview)
            ticket = preview.edit(1, target)
            with self.subTest(target=target):
                if accepted:
                    self.assertEqual(gateway.commit(ticket, context)[1], target)
                    validate_waypoints([(target, 0., 0., 0.)], 3.)
                else:
                    with self.assertRaises(Rejected): gateway.commit(ticket, context)
                    with self.assertRaises(Rejected): validate_waypoints([(target, 0., 0., 0.)], 3.)

    def test_five_axis_template_and_runtime_config_match_current_sector(self):
        template = template_settings()
        self.assertFalse(template.user_confirmed)
        self.assertEqual((template.axes[1].travel_min_deg, template.axes[1].travel_max_deg), operating_limits_deg()[0])
        settings = load_settings()
        envelopes, active = build_envelopes(settings)
        self.assertEqual(active, (1, 2, 3, 4))
        self.assertIsNone(settings.axes[1].travel_min_deg)
        self.assertIsNone(settings.axes[1].travel_max_deg)
        self.assertAlmostEqual(envelopes[1].lower, math.radians(-80.))
        self.assertAlmostEqual(envelopes[1].upper, math.radians(200.))
        # A stale runtime JSON must fail, rather than silently clipping or mirroring.
        from dataclasses import replace
        settings.axes[1] = replace(settings.axes[1], travel_min_deg=-198., travel_max_deg=78.)
        with self.assertRaises(ValueError): build_envelopes(settings)

    def test_acquisition_retains_narrower_range_without_override(self):
        path = ROOT/'verification/hardware/load_calibration/measured_limits_20261102.json'
        settings = json.loads(path.read_text(encoding='utf-8'))['joints']['1']
        self.assertFalse(settings['measured_limit_override'])
        self.assertEqual(tuple(settings['mechanical_limits_deg']), LIMITS[0])
        plan = build_plan(joint=1, initial_pose_rad=(0., 0., 0., 0.),
            mechanical_limits_deg=settings['mechanical_limits_deg'],
            scan_limits_deg=settings['scan_limits_deg'], step_deg=10.,
            mapping_sha256='a'*64, physical_limits_confirmed=True,
            physical_limit_evidence='offline current-coordinate regression fixture')
        self.assertEqual(plan.scan_limits_deg, (-70., 70.))
        self.assertFalse(plan.measured_limit_override)
        self.assertTrue(all(-70 <= math.degrees(p.target_pose_rad[0]) <= 70 for p in plan.points))

    def test_ros_wrappers_have_no_second_limit_table(self):
        directory = MODEL_PATH.parent
        for filename in ('slaihand_four_axis.urdf.xacro', 'slaihand_five_axis.urdf.xacro'):
            root = ET.parse(directory/filename).getroot()
            includes = [n.get('filename') for n in root.findall('{http://www.ros.org/wiki/xacro}include')]
            self.assertTrue(any(v.endswith('/slaihand_four_axis.urdf') for v in includes))
            self.assertFalse(root.findall('.//command_interface/param'))
            self.assertFalse(any('lower' in n.attrib or 'upper' in n.attrib
                                 for n in root.iter() if n.tag.endswith('offline_joint')))


if __name__ == '__main__':
    unittest.main()
