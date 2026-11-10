"""Offline unit tests for the model-compensated plan converter."""
import importlib.util
import json
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('compensate_plan', HERE / 'compensate_plan.py')
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_plan(name):
    return read(HERE / 'plans' / name)


class ConverterTests(unittest.TestCase):
    def test_in_domain_config_regenerates_plan(self):
        plan = load_plan('V1sub_sag_20261110.json')
        desired = [w['degrees'] for w in plan['waypoints']]
        result = cp.compute(desired)
        self.assertEqual(result['problems'], [])
        compensated = result['compensated']
        self.assertLessEqual(abs(compensated[0]).max(), 1e-9)
        self.assertLessEqual(abs(compensated[-1]).max(), 1e-9)
        converted = load_plan('V1sub_sag_comp_20261110.json')
        expected = [w['degrees'] for w in converted['waypoints']]
        for row, target in zip(compensated, expected):
            for value, wanted in zip(row, target):
                self.assertAlmostEqual(float(value), float(wanted), places=6)

    def test_duration_and_endpoints_preserved(self):
        plan = load_plan('T1a_sag_20261110.json')
        converted = load_plan('T1a_sag_comp_20261110.json')
        original = sum(w['move_s'] + w['dwell_s'] for w in plan['waypoints'])
        after = sum(w['move_s'] + w['dwell_s'] for w in converted['waypoints'])
        self.assertEqual(original, after)
        self.assertLessEqual(max(abs(float(v)) for v in converted['waypoints'][0]['degrees']), 1e-9)
        self.assertLessEqual(max(abs(float(v)) for v in converted['waypoints'][-1]['degrees']), 1e-9)

    def test_out_of_domain_plans_are_rejected(self):
        for name in ('T2_sag_20261110.json', 'T2r_sag_20261110.json',
                     'V2_sag_20261110.json', 'JZ_20261110.json'):
            plan = load_plan(name)
            result = cp.compute([w['degrees'] for w in plan['waypoints']])
            self.assertTrue(result['problems'], '%s should be rejected' % name)

    def test_all_suite_plans_validate(self):
        import subprocess
        import sys
        manifest = read(HERE / 'test_suite_manifest_20261110.json')
        for entry in manifest['entries']:
            result = subprocess.run([sys.executable, str(HERE / 'validate_plan.py'),
                                     entry['plan']], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, entry['name'])


if __name__ == '__main__':
    unittest.main()
