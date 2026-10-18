import ast
import copy
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from sensor_array_bringup.consistency_validation import (
    MAPS_SOURCE, evaluate_trial, load_validation_config, make_validation_plan, maps_local, performance)
from sensor_array_bringup.validation_collect import ValidationCollector, ValidationStore, compare_runs, reanalyze
from sensor_array_bringup.validation_ros import main
from test_consistency import PACKAGE, SyntheticBackend, configuration


class ValidationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        common = configuration(self.root)
        backend = SyntheticBackend(common)
        sensors = copy.deepcopy(common['sensors'])
        for s in sensors:
            s['T'] = (backend.matrices[10] @ np.linalg.inv(backend.matrices[s['sensor_id']])).tolist()
        calibration = dict(schema_version=1, data_valid=True, nominal_ut_per_count=1.1,
                           reference_id=10, sensors=sensors, device_id=common['device_id'],
                           firmware_device_id=common['firmware_device_id'], sensor_mode=common['sensor_mode'],
                           calibration_id='synthetic_calibration')
        self.calibration_path = self.root / 'calibration.json'
        self.calibration_path.write_text(json.dumps(calibration))
        source = yaml.safe_load((PACKAGE / 'config/consistency_validation.yaml').read_text())
        source.update(calibration_file=str(self.calibration_path), firmware_device_id=123,
                      array_geometry=common['array_geometry'], output_root=str(self.root / 'runs'),
                      environment_label='room_background_A', temperature_c=25.,
                      startup_s=.5, capture_s=.5, settle_s=.01, max_capture_s=.5,
                      subwindow_s=.1, min_samples=10, clock_guard_s=.05, retries=1)
        self.path = self.root / 'validation.yaml'
        self.path.write_text(yaml.safe_dump(source))
        self.config = load_validation_config(self.path)

    def run_synthetic(self, backend=None):
        store = ValidationStore(self.config)
        self.addCleanup(store.close)
        store.write('raw_stream.csv', [dict(sensor_id=1, raw=[1, 2, 3])])
        result = ValidationCollector(self.config, backend or SyntheticBackend(self.config), store).run()
        store.close()
        return store.path, result

    def test_frozen_plan_and_no_hardware_cli_modes(self):
        p = make_validation_plan(self.config)
        self.assertEqual(len(p), 24)
        self.assertEqual(sum(len(t['axes']) == 1 for t in p), 18)
        self.assertEqual([t['commands_v'] for t in p[:6]],
                         [{'X': 1.}, {'X': -1.}, {'Y': .4}, {'Y': -.4}, {'Z': 1.}, {'Z': -1.}])
        self.assertEqual(p[-2]['commands_v'], {'Y': 1., 'Z': 2.5})
        with patch('sys.stdout', new_callable=io.StringIO) as output:
            main(['--config', str(self.path), '--check'])
            main(['--config', str(self.path), '--dry-run'])
        self.assertIn('24 independent', output.getvalue())
        self.assertFalse((self.root / 'runs').exists())

    def test_geometry_identity_temperature_and_voltage_rejected(self):
        source = yaml.safe_load(self.path.read_text())
        for delta in ({'temperature_c': None}, {'environment_label': ''}, {'firmware_device_id': 999},
                      {'sensor_mode': 'other'}):
            self.path.write_text(yaml.safe_dump(dict(source, **delta)))
            with self.assertRaises(ValueError):
                load_validation_config(self.path)
        self.path.write_text(yaml.safe_dump(source))
        geom = json.loads(Path(source['array_geometry']).read_text())
        geom['d_list'][0][0] += .001
        gpath = self.root / 'geometry.json'
        gpath.write_text(json.dumps(geom))
        self.path.write_text(yaml.safe_dump(dict(source, array_geometry=str(gpath))))
        with self.assertRaisesRegex(ValueError, 'geometry differs'):
            load_validation_config(self.path)
        source['channels']['X']['combination_v'] = 5.1
        self.path.write_text(yaml.safe_dump(source))
        with self.assertRaisesRegex(ValueError, 'voltage exceeds'):
            load_validation_config(self.path)

    def test_maps_known_tensor_uniform_field_and_noncentered_geometry(self):
        d = np.array([s['position_m'] for s in self.config['sensors']])
        x = np.array([[120., -30., 45.], [-30., -70., 20.], [45., 20., -50.]])
        for positions in (d, d + [.0003, -.0002, .0001]):
            b = np.array([10., 20., -30.]) + positions @ x.T
            m = maps_local(b, positions)
            np.testing.assert_allclose(m['maps_X_ut_m'], x, atol=1e-8)
            np.testing.assert_allclose(m['local_field_ut'], [10., 20., -30.], atol=1e-9)
        m = maps_local(np.tile([1500., -200., 100.], (12, 1)), d)
        self.assertLess(m['maps_gradient_frobenius_ut_m'], 1e-8)
        with self.assertRaises(ValueError):
            maps_local(np.zeros((12, 3)), np.zeros((12, 3)))

    @unittest.skipUnless(Path(MAPS_SOURCE).is_file(), 'external MaPS reference unavailable')
    def test_maps_matches_supplied_source(self):
        tree = ast.parse(Path(MAPS_SOURCE).read_text())
        tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in
                     ('null', 'MaPS_Estimator', 'compute_rho_hat_eigen')]
        namespace = dict(np=np, kabsch_solver=lambda a, b: np.eye(3))
        exec(compile(tree, MAPS_SOURCE, 'exec'), namespace)
        positions = np.array([s['position_m'] for s in self.config['sensors']]) + [.0003, -.0002, .0001]
        rng = np.random.RandomState(42)
        b = rng.normal(size=(12, 3)) + [1200., -900., 700.]
        sources = [dict(p_Ci=np.zeros(3), m_Ci=np.ones(3))] * 3
        _, _, detail = namespace['MaPS_Estimator'](positions.T, sources, [b.T] * 3)
        m = maps_local(b, positions)
        np.testing.assert_allclose(m['maps_X_ut_m'], detail['X_hat_locals'][0], rtol=1e-12, atol=1e-8)
        np.testing.assert_allclose(m['local_field_ut'], detail['b_hat_locals'][:, 0], atol=1e-9)

    def test_end_to_end_fixed_matrix_and_compact_reanalysis(self):
        path, result = self.run_synthetic()
        self.assertTrue(result['data_valid'])
        self.assertEqual(result['performance_pass'], {'long': None, 'short': None})
        self.assertEqual(len(result['trials']), 24)
        self.assertTrue((path / 'metrics.png').is_file())
        self.assertTrue((path / 'axis_residuals.png').is_file())
        self.assertTrue((path / 'maps_gradients.csv').is_file())
        for t in result['trials']:
            self.assertLess(t['increment']['after']['vector_rmse_ut'], 1e-8)
            self.assertGreater(t['total_on']['after']['vector_rmse_ut'], 1.)
            np.testing.assert_allclose(t['increment']['after']['sensor_residual_ut'][9], 0.)
        self.assertFalse((path / 'raw_stream.csv').exists())
        self.assertFalse((path / 'samples.csv').exists())
        self.assertTrue((path / 'short_measurements.json').is_file())
        storage = json.loads((path / 'storage.json').read_text())
        self.assertEqual({r['file'] for r in storage['raw_files_removed']}, {'raw_stream.csv', 'samples.csv'})
        # Derived output tables are not replay inputs: metrics must be rebuilt from measurements.
        (path / 'validation.csv').write_text('corrupted derived data\n')
        (path / 'short_validation.csv').write_text('corrupted derived data\n')
        replay = reanalyze(path)
        recovered = json.loads((replay / 'result.json').read_text())
        for a, b in zip(result['trials'], recovered['trials']):
            for q in ('increment', 'total_pre', 'total_on', 'total_post'):
                np.testing.assert_allclose(a[q]['after']['axis_rmse_ut'], b[q]['after']['axis_rmse_ut'], atol=1e-8)
                np.testing.assert_allclose(a[q]['after']['maps_X_ut_m'], b[q]['after']['maps_X_ut_m'], atol=1e-7)
        self.assertEqual(result['short_summary'].keys(), recovered['short_summary'].keys())
        from sensor_array_bringup.validation_collect import assert_metrics_equal
        assert_metrics_equal(result['short_summary'], recovered['short_summary'])
        self.assertFalse((replay / 'samples.csv').exists())
        comparison = compare_runs([path, replay])
        self.assertEqual(len(comparison), 24 * 4 * 2 * 2)
        self.assertEqual(len({r['calibration_sha256'] for r in comparison}), 1)
        (path / 'config.yaml').write_text((path / 'config.yaml').read_text() + '\nnotes: changed\n')
        with self.assertRaisesRegex(ValueError, 'hash'):
            reanalyze(path)

    def test_retry_after_missing_samples(self):
        class RetryBackend(SyntheticBackend):
            def __init__(self, config):
                super().__init__(config, failure='missing')
                self.enabled_count = 0

            def set_output(self, commands):
                self.enabled_count += 1
                if self.enabled_count > 1:
                    self.failure = None
                super().set_output(commands)
        with patch('sensor_array_bringup.validation_collect.plot_results'):
            path, result = self.run_synthetic(RetryBackend(self.config))
        self.assertTrue(result['data_valid'])
        self.assertEqual(result['trials'][0]['attempt'], 1)
        with (path / 'trial_summary.csv').open() as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(sum(r['accepted'] == 'false' for r in rows), 12)

    def test_legacy_raw_reanalysis_and_no_recreated_raw_stream(self):
        store = ValidationStore(self.config)
        try:
            with patch.object(store, 'cleanup_raw'), patch('sensor_array_bringup.validation_collect.plot_results'):
                result = ValidationCollector(self.config, SyntheticBackend(self.config), store).run()
            self.assertTrue((store.path / 'samples.csv').exists())
            # Simulate the pre-compact v1 format, which retained the complete sample CSV.
            result['schema_version'] = 1
            (store.path / 'result.json').write_text(json.dumps(result))
            (store.path / 'compact_manifest.json').unlink()
            with patch('sensor_array_bringup.validation_collect.plot_results'):
                replay = reanalyze(store.path)
            recovered = json.loads((replay / 'result.json').read_text())
            from sensor_array_bringup.validation_collect import assert_metrics_equal
            assert_metrics_equal(result['trials'], recovered['trials'])
            assert_metrics_equal(result['short_summary'], recovered['short_summary'])
            self.assertFalse((replay / 'samples.csv').exists())
            store.cleanup_raw()
            store.write('raw_stream.csv', [dict(raw=[1, 2, 3])])
            self.assertFalse((store.path / 'raw_stream.csv').exists())
        finally:
            store.close()

    def test_compact_verification_failure_preserves_full_samples(self):
        store = ValidationStore(self.config)
        try:
            with patch('sensor_array_bringup.validation_collect.assert_metrics_equal', side_effect=ValueError('replay mismatch')):
                with self.assertRaisesRegex(ValueError, 'replay mismatch'):
                    ValidationCollector(self.config, SyntheticBackend(self.config), store).run()
            self.assertTrue((store.path / 'samples.csv').exists())
            self.assertFalse((store.path / 'result.json').exists())
            self.assertFalse((store.path / 'storage.json').exists())
        finally:
            store.close()

    def test_failures_always_attempt_shutdown_and_never_save_success(self):
        for failure in ('timeout', 'interrupt', 'missing', 'shutdown'):
            backend = SyntheticBackend(self.config, failure=failure)
            store = ValidationStore(self.config)
            try:
                with self.assertRaises((RuntimeError, KeyboardInterrupt)):
                    ValidationCollector(self.config, backend, store).run()
                self.assertGreater(backend.off_count, 0)
                self.assertFalse((store.path / 'result.json').exists())
                self.assertTrue((store.path / 'failure.json').exists())
                if failure != 'shutdown':
                    self.assertTrue((store.path / 'samples.csv').exists())
            finally:
                store.close()

    def test_stream_failure_and_final_shutdown_failure(self):
        class StreamFailure(SyntheticBackend):
            def read_samples(self, start, end):
                if self.outputs:
                    raise RuntimeError('sensor stream timeout')
                return super().read_samples(start, end)

        class FinalShutdownFailure(SyntheticBackend):
            def all_off(self):
                super().all_off()
                if self.off_count == 50:
                    raise RuntimeError('final shutdown not confirmed')

        for backend in (StreamFailure(self.config), FinalShutdownFailure(self.config)):
            store = ValidationStore(self.config)
            try:
                with self.assertRaises(RuntimeError):
                    ValidationCollector(self.config, backend, store).run()
                self.assertFalse((store.path / 'result.json').exists())
                self.assertTrue((store.path / 'failure.json').exists())
            finally:
                store.close()

    @unittest.skipUnless((PACKAGE.parents[1] / 'data/consistency/run_20261017T083604.813944Z/calibration.json').is_file(),
                         'recorded physical calibration unavailable')
    def test_existing_run_summary_residual_regression(self):
        from sensor_array_bringup.consistency_collect import read_csv
        root = PACKAGE.parents[1] / 'data/consistency/run_20261017T083604.813944Z'
        original = json.loads((root / 'calibration.json').read_text())
        c = dict(self.config, sensors=original['sensors'], reference_id=original['reference_id'])
        rows = list(read_csv(root / 'trial_summary.csv'))
        for expected in original['validation']:
            actual = evaluate_trial([r for r in rows if r['accepted'] and r['trial_id'] == expected['trial_id']], c)
            for stage in ('before', 'after'):
                for key in ('vector_rmse_ut', 'max_pairwise_ut'):
                    self.assertAlmostEqual(actual['increment'][stage][key], expected[stage][key], places=9)
                x = np.array(actual['increment'][stage]['maps_X_ut_m'])
                np.testing.assert_allclose(x, x.T, atol=1e-10)
                self.assertAlmostEqual(float(np.trace(x)), 0., places=8)

    def test_performance_thresholds_and_short_rejection(self):
        with patch('sensor_array_bringup.validation_collect.plot_results'):
            _, result = self.run_synthetic()
        c = copy.deepcopy(self.config)
        c['performance_limits'] = {'vector_rmse_ut': .1}
        self.assertFalse(performance(result['trials'], [], c)['long'])
        self.assertFalse(performance(result['trials'], [{'accepted': False}], c)['short'])


if __name__ == '__main__':
    unittest.main()
