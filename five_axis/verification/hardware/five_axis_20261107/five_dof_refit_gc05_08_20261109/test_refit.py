"""Holdout isolation, real data provenance, and actual-pose pair semantics."""
import unittest
import numpy as np
from refit import HERE, read, splits, fit, paired, FiveDofPotential, rigid_parameters


class RefitTests(unittest.TestCase):
    def test_real_data_split_and_independent_extraction_agree(self):
        poses = read(HERE/'static_windows.json')['poses']
        train, history, final = splits(poses)
        self.assertEqual((len(poses), int(sum(train)), int(sum(history)), int(sum(final))), (115, 75, 29, 11))
        self.assertFalse(np.any(train & (history | final)))
        self.assertEqual({p['session'] for p, k in zip(poses, final) if k}, {'GC08'})
        cross = read(HERE/'extraction_crosscheck.json')
        self.assertEqual(len(cross), 42)
        self.assertEqual(max(p['max_q_difference_deg'] for p in cross), 0.)
        self.assertEqual(max(p['max_effort_difference_nm'] for p in cross), 0.)

    def test_synthetic_gravity_direction_recovery_and_holdout_isolation(self):
        rng = np.random.default_rng(20261109)
        model = FiveDofPotential()
        q = rng.uniform(-.4, .7, (50, 5))
        x = np.stack([model.orbit_regressor(p) for p in q])
        base = np.stack([model.base(p) for p in q])
        h, _ = rigid_parameters([.08, .01, -.03, .015, .01, .5, -.4])
        direction = rng.choice([-1, 1], (50, 5))
        y = base + x@h
        y[:, 4] += .8*direction[:, 4]
        rows = np.ones((50, 5), dtype=bool)
        rows[35:] = False
        a, gravity, total = fit('free_orbit_J5_direction', x, base, y, direction, rows)
        np.testing.assert_allclose(gravity, base+x@h, atol=1e-10)
        np.testing.assert_allclose(total[35:], y[35:], atol=1e-10)
        corrupted = y.copy()
        corrupted[35:] += rng.normal(0, 1000, (15, 5))
        b, _, _ = fit('free_orbit_J5_direction', x, base, corrupted, direction, rows)
        np.testing.assert_array_equal(a['h_kg_m'], b['h_kg_m'])
        np.testing.assert_array_equal(a['nuisance_nm'], b['nuisance_nm'])

    def test_pair_predictions_average_both_actual_poses(self):
        data = read(HERE/'static_windows.json')['poses']
        # Deliberately nonlinear values at every actual feedback pose distinguish
        # averaging two predictions from evaluating one averaged target pose.
        gravity = np.array([[np.sin((j+1)*p['q_rad'][4]) for j in range(5)] for p in data])
        pairs = paired(data, gravity, gravity)
        self.assertEqual(len(pairs), 12)
        for pair in pairs:
            indices = [next(i for i, p in enumerate(data) if p['id'] == name) for name in pair['ids']]
            np.testing.assert_allclose(pair['gravity_pair_mean_prediction_nm'], gravity[indices].mean(axis=0))
            self.assertGreater(max(abs(v) for v in pair['pose_separation_deg']), .01)

    def test_no_candidate_was_promoted_and_primary_is_rigid(self):
        result = read(HERE/'refit_result.json')
        self.assertFalse(result['deployable'])
        self.assertIsNone(result['selected_for_deployment'])
        self.assertFalse(result['parameters_applied'])
        self.assertFalse(result['model_selection_on_GC08'])
        for r in result['models']:
            self.assertFalse(r['deployable'])
            if r['kind'].startswith('rigid'):
                h = np.array(r['h_kg_m']).reshape(3, 3)
                self.assertAlmostEqual(np.linalg.norm(h[1]), np.linalg.norm(h[2]), places=10)
                self.assertAlmostEqual(float(h[1]@h[2]), 0., places=10)

    def test_future_plans_absolute_zero_pairs_and_limits(self):
        from make_next_plans import previous, OUT
        manifest = read(OUT/'PLAN_MANIFEST.json')
        limits, _ = previous.bounds()
        self.assertEqual(len(manifest['plans']), 5)
        self.assertFalse(manifest['hardware_executed'])
        self.assertFalse(manifest['panel_imported'])
        for entry in manifest['plans']:
            path = OUT/entry['file']
            self.assertEqual(previous.sha(path), entry['sha256'])
            plan = read(path)
            check = previous.validate(plan, limits, [0.]*5)
            self.assertEqual(check['compiled_final_rad'], [0.]*5)
            self.assertGreaterEqual(check['session_reserve_s'], 30.)
            for pair in entry['paired_windows']:
                a, b = pair['waypoint_indices_zero_based']
                self.assertEqual(plan['waypoints'][a]['degrees'], plan['waypoints'][b]['degrees'])
                j = pair['axis_one_based']-1
                da = plan['waypoints'][a]['degrees'][j]-plan['waypoints'][a-1]['degrees'][j]
                db = plan['waypoints'][b]['degrees'][j]-plan['waypoints'][b-1]['degrees'][j]
                self.assertLess(da*db, 0.)


if __name__ == '__main__':
    unittest.main()
