import copy
from collections import deque
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml

from sensor_array_bringup.consistency import (
    IDS, analyze_trials, build_delta, choose_reference, fit_relative, load_config,
    make_plan, metrics, validate_config, window_statistics)
from sensor_array_bringup.consistency_collect import Collector, RunStore, read_csv, refit
from sensor_array_bringup.consistency_ros import DeviceClock, RosBackend, main


PACKAGE = Path(__file__).resolve().parents[1]


def configuration(root):
    source = yaml.safe_load((PACKAGE / 'config/consistency_calibration.yaml').read_text())
    source.update(firmware_device_id=123, axes_confirmed=True,
                  array_geometry=str(PACKAGE.parent / 'sensor_array_config/config/array_01/ak/array.json'),
                  output_root=str(root), startup_s=.5, capture_s=.5, settle_s=.01,
                  max_capture_s=.5, subwindow_s=.1, min_samples=10,
                  clock_guard_s=.05, repeats=1, retries=1)
    for entry in source['channels'].values():
        entry.update(levels_v=[.1, .2, .3, .4, .5], max_abs_offset_v=.6)
    path = Path(root) / 'experiment.yaml'
    path.write_text(yaml.safe_dump(source))
    return load_config(path)


class SyntheticBackend:
    """Virtual clock + independent invertible sensor responses and linear drift."""
    def __init__(self, config, failure=None):
        self.config = config
        self.time = 100.
        self.outputs = {}
        self.off_count = 0
        self.failure = failure
        self.matrices = {sid: np.array([[1 + sid * .01, .01, -.01], [.02, 1, .01], [0, -.01, 1]]) for sid in IDS}

    def now(self):
        return self.time

    def prepare(self):
        pass

    def sleep(self, seconds):
        self.time += seconds

    def all_off(self):
        self.off_count += 1
        self.outputs = {}
        if self.failure == 'shutdown':
            raise RuntimeError('cannot turn off')

    def set_output(self, commands):
        if self.failure == 'timeout':
            raise RuntimeError('device timeout')
        if self.failure == 'interrupt':
            raise KeyboardInterrupt()
        self.outputs = dict(commands)

    def read_samples(self, start, end):
        rows = []
        field = np.array([self.outputs.get(a, 0) * 1000 for a in 'XYZ'])
        for sid in IDS:
            sensor = self.config['sensors'][sid - 1]
            for t in np.arange(start + .005, end - .001, .01):
                vector = self.matrices[sid] @ field + sid * np.array([3., -1., 2.]) + t * np.array([.05, -.02, .01])
                raw = np.asarray(sensor['Q']).T @ vector / 1.1
                rows.append(dict(sensor_id=sid, bus=sensor['bus'], mask=sensor['mask'],
                                 round_seq=1, sample_seq=None, t_device=t - 10, t_host=t + .001,
                                 t_arrival=t + .002, t=float(t), raw=raw.tolist(), x=vector.tolist(),
                                 drdy=1, err=0, dor=0, comm_ok=True,
                                 valid=not (self.failure == 'missing' and self.outputs and sid == 12), reason=''))
        return rows


class ConsistencyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = configuration(self.temp.name)

    def test_plan_five_per_axis_polarities_and_independent_validation(self):
        config = copy.deepcopy(self.config)
        config['repeats'] = 3
        config['channels']['Y']['levels_v'] = [.11, .21, .31, .41, .51]
        trials = make_plan(config)
        self.assertEqual(len(trials), 102)
        self.assertEqual(len([t for t in trials if t['split'] == 'train']), 90)
        self.assertEqual(trials, make_plan(config))
        self.assertEqual({abs(t['commands_v']['Y']) for t in trials if t['split'] == 'train' and t['axes'] == 'Y'}, {.11, .21, .31, .41, .51})
        self.assertEqual({t['axes'] for t in trials if t['split'] == 'validation'}, {'X', 'Y', 'Z', 'XY', 'XZ', 'YZ'})
        for axis in 'XYZ':
            for level in range(1, 6):
                states = [t for t in trials if t['split'] == 'train' and t['axes'] == axis and t['level'] == level]
                self.assertEqual(sorted(t['polarity'] for t in states), [-1, -1, -1, 1, 1, 1])

    def test_placeholder_and_invalid_configs_rejected(self):
        # The checked-in experiment may already contain real hardware values.
        # Construct missing-ID cases rather than requiring its YAML to be blank.
        changes = [dict(axes_confirmed=False), dict(firmware_device_id=None),
                   dict(firmware_device_id=True), dict(reference_id=255),
                   dict(capture_s=.1), dict(min_samples=1), dict(retries=-1),
                   dict(max_condition=float('nan')), dict(generator_namespace='relative')]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config(dict(self.config, **change))
        for values in ([1, 2, 3], [.1, .2, .2, .4, .5], [.1, .2, -.3, .4, .5], [.1, .2, .3, .4, 2], [None] * 5):
            config = copy.deepcopy(self.config)
            config['channels']['X']['levels_v'] = values
            with self.subTest(values=values), self.assertRaises(ValueError):
                validate_config(config)
        config = copy.deepcopy(self.config)
        config['channels']['Z']['channel'] = 1
        with self.assertRaises(ValueError):
            validate_config(config)

    def test_linear_background_interpolation_at_asymmetric_times(self):
        background = lambda t: np.array([3, -10, 2]) + np.array([.2, 3, -1]) * t
        delta = build_delta(dict(t=1., mean=background(1)), dict(t=3., mean=background(3) + [10, 20, 30]),
                            dict(t=8., mean=background(8)))
        np.testing.assert_allclose(delta['delta'], [10, 20, 30], atol=1e-12)
        self.assertAlmostEqual(delta['alpha'], 2 / 7)
        with self.assertRaises(ValueError):
            build_delta(dict(t=1, mean=[0]*3), dict(t=9, mean=[0]*3), dict(t=8, mean=[0]*3))
        with self.assertRaises(ValueError):
            build_delta(dict(t=1, mean=[0]*3), dict(t=2, mean=[0]*3), dict(t=3, mean=[100]*3), 1)

    def test_svd_recovery_rank_condition_and_nonuniform_field(self):
        rng = np.random.RandomState(9)
        fields = rng.normal(size=(60, 3))
        matrices = {sid: np.eye(3) + rng.normal(scale=.03, size=(3, 3)) for sid in IDS}
        delta = {sid: fields @ matrices[sid].T for sid in IDS}
        result = fit_relative(delta, 5)
        for sid in IDS:
            np.testing.assert_allclose(result[sid]['T'], matrices[5] @ np.linalg.inv(matrices[sid]), atol=1e-12)
        # Applying fixed matrices preserves a spatially varying input field.
        corrected = [np.array(result[sid]['T']) @ matrices[sid] @ [sid, 2, 3] for sid in IDS]
        self.assertGreater(np.linalg.norm(corrected[0] - corrected[-1]), 1)
        for bad in (np.ones((10, 3)), np.column_stack((np.arange(10), np.arange(10)**2, np.zeros(10))),
                    np.diag([1., 1., 1e-5])):
            with self.assertRaises(ValueError):
                fit_relative({sid: bad for sid in IDS}, 1, max_condition=1000)
        delta[2] = np.ones((2, 3))
        with self.assertRaises(ValueError):
            fit_relative(delta, 1)

    def test_statistics_flags_gap_noise_dor_and_reference(self):
        backend = SyntheticBackend(self.config)
        records = [r for r in backend.read_samples(100, 100.4) if r['sensor_id'] == 1]
        records[0]['drdy'] = 0
        records[1]['err'] = 1
        records[2]['dor'] = 1
        records[3]['comm_ok'] = False
        stats = window_statistics(records, 100, 100.4, self.config)
        self.assertEqual(stats['n'], len(records) - 3)
        self.assertGreater(stats['dor_fraction'], 0)
        with self.assertRaises(ValueError):
            window_statistics(records[:10] + records[-5:], 100, 101, self.config)
        strict = dict(self.config, max_slope_ut_s=.001)
        with self.assertRaisesRegex(ValueError, 'drift'):
            window_statistics(records, 100, 100.4, strict)
        all_stats = {sid: copy.deepcopy(stats) for sid in IDS}
        all_stats[3]['n'] += 1
        self.assertEqual(choose_reference(all_stats), 3)
        self.assertEqual(choose_reference(all_stats, 9), 9)

    def test_collect_full_run_and_raw_refit_and_alignment(self):
        store = RunStore(self.config)
        backend = SyntheticBackend(self.config)
        try:
            result = Collector(self.config, backend, store).run()
        finally:
            store.close()
        self.assertEqual(len(result['training_trial_ids']), 30)
        self.assertEqual(len(result['validation']), 12)
        self.assertIsNone(result['performance_pass'])
        self.assertFalse(backend.outputs)
        self.assertTrue((store.path / 'short_validation.csv').exists())
        for entry in result['validation']:
            self.assertLess(entry['after']['vector_rmse_ut'], 1e-8)
        for sid in IDS:
            expected = backend.matrices[result['reference_id']] @ np.linalg.inv(backend.matrices[sid])
            np.testing.assert_allclose(result['sensors'][sid - 1]['T'], expected, atol=1e-10)
        shuffled = list(reversed(store.summaries))
        self.assertEqual(analyze_trials(self.config, shuffled, result['reference_id'])['training_trial_ids'], result['training_trial_ids'])
        for broken in (shuffled[:-1], shuffled + shuffled[:1]):
            with self.assertRaises(ValueError):
                analyze_trials(self.config, broken, result['reference_id'])
        # Offline refit actually reconstructs vectors from raw counts + saved Q.
        replay_path = refit(store.path)
        replay = json.loads((replay_path / 'calibration.json').read_text())
        for a, b in zip(result['sensors'], replay['sensors']):
            np.testing.assert_allclose(a['T'], b['T'], atol=1e-10)
        short = list(read_csv(replay_path / 'short_validation.csv'))
        self.assertTrue(short)
        self.assertTrue(all(r['accepted'] for r in short))
        source = yaml.safe_load((store.path / 'config.yaml').read_text())
        source['sensor_mode'] = 'changed after collection'
        (store.path / 'config.yaml').write_text(yaml.safe_dump(source))
        with self.assertRaisesRegex(ValueError, 'hash mismatch'):
            refit(store.path)

    def test_failures_close_outputs_preserve_records_no_artifact(self):
        for failure in ('missing', 'timeout', 'interrupt', 'shutdown'):
            with self.subTest(failure=failure):
                store = RunStore(self.config)
                backend = SyntheticBackend(self.config, failure)
                try:
                    with self.assertRaises((RuntimeError, KeyboardInterrupt)):
                        Collector(self.config, backend, store).run()
                finally:
                    store.close()
                self.assertGreater(backend.off_count, 0)
                self.assertFalse(backend.outputs)
                self.assertTrue((store.path / 'report.md').exists())
                self.assertFalse((store.path / 'calibration.json').exists())
                events = list(read_csv(store.path / 'events.csv'))
                if failure == 'missing':
                    self.assertEqual(len([e for e in events if e['event'] == 'RETRY']), 2)
                if failure == 'shutdown':
                    self.assertTrue(any(e['event'] == 'SHUTDOWN_FAILED' for e in events))

    def test_sensor_adapter_preserves_signed_raw_flags_and_checks_address(self):
        import threading
        backend = RosBackend.__new__(RosBackend)
        backend.lock = threading.RLock()
        backend.config = self.config
        backend.expected = {s['sensor_id']: s for s in self.config['sensors']}
        backend.gate = SimpleNamespace(accept=lambda *a: True, mismatch=False)
        backend.clock = DeviceClock(.1, .05, .25)
        backend.clock.offset = 100.
        backend.now = lambda: 100.2
        backend.rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 10.)))
        backend.buffer = deque()
        backend.fatal = None
        archive, events = [], []
        backend.store = SimpleNamespace(write=lambda name, rows: archive.extend(rows), event=lambda *a: events.append(a))
        sample = SimpleNamespace(sensor_id=1, bus=1, mask=2, hx=-1, hy=2, hz=-3,
                                 drdy=1, err=0, dor=1, read_done_offset_us=0)
        msg = SimpleNamespace(has_stats=False, stats=None, ak=[sample], round_seq=1,
                              device_tick_us=200000, device_tick_unwrapped_us=200000,
                              header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 10.)), errors=[])
        backend._on_round(msg)
        self.assertEqual(archive[0]['raw'], [-1, 2, -3])
        self.assertTrue(archive[0]['valid'])
        self.assertEqual(archive[0]['dor'], 1)
        self.assertIsNone(archive[0]['sample_seq'])
        np.testing.assert_allclose(archive[0]['x'], np.array(self.config['sensors'][0]['Q']) @ [-1.1, 2.2, -3.3])
        self.assertAlmostEqual(archive[0]['t'], 100.2)
        msg.round_seq = 2
        msg.device_tick_us = msg.device_tick_unwrapped_us = 300000
        sample.bus = 3
        backend.now = lambda: 100.3
        backend._on_round(msg)
        self.assertFalse(archive[-1]['valid'])
        self.assertIn('address mismatch', backend.fatal)

    def test_smoke_test_collects_only_lowest_x_polarities_without_fit(self):
        store = RunStore(self.config)
        backend = SyntheticBackend(self.config)
        try:
            self.assertIsNone(Collector(self.config, backend, store).run(smoke_test=True))
        finally:
            store.close()
        self.assertEqual(len(store.summaries), 24)
        self.assertEqual({r['axes'] for r in store.summaries}, {'X'})
        self.assertEqual({r['polarity'] for r in store.summaries}, {-1, 1})
        self.assertEqual({abs(r['commands_v']['X']) for r in store.summaries}, {.1})
        self.assertFalse((store.path / 'calibration.json').exists())
        self.assertFalse(backend.outputs)

    def test_sample_count_extension_is_bounded_and_recorded(self):
        config = dict(self.config, min_samples=50, max_capture_s=.8)
        store = RunStore(config)
        backend = SyntheticBackend(config)
        try:
            stats, _ = Collector(config, backend, store).capture(
                dict(trial_id=0, split='startup'), 0, 'STARTUP_CAPTURE', .5)
        finally:
            store.close()
        self.assertTrue(all(s['n'] >= 50 for s in stats.values()))
        self.assertGreater(store.windows[0]['end'] - store.windows[0]['start'], .4)
        self.assertTrue(any(r['event'] == 'EXTEND_WINDOW' for r in read_csv(store.path / 'events.csv')))

    def test_check_and_dry_run_need_no_ros_or_writes(self):
        config_path = Path(self.temp.name) / 'experiment.yaml'
        before = set(Path(self.temp.name).iterdir())
        with patch('builtins.print') as output:
            main(['--config', str(config_path), '--check'])
            main(['--config', str(config_path), '--dry-run'])
        self.assertGreater(output.call_count, 40)
        self.assertEqual(before, set(Path(self.temp.name).iterdir()))


class ClockAndDriverTests(unittest.TestCase):
    def lock_clock(self):
        clock = DeviceClock(.4, .05, .25)
        for seq in range(6):
            clock.observe(seq * .1, 100 + seq * .1 + (seq % 2) * .001, seq)
        self.assertAlmostEqual(clock.offset, 100.)
        return clock

    def test_clock_wrap_sequence_delay_restart_and_bad_time(self):
        clock = self.lock_clock()
        self.assertAlmostEqual(clock.observe(.6, 100.61, 6), 100.6)
        for device_t, host_t, seq in ((.5, 100.7, 5), (.4, 100.7, 7), (.7, 101.2, 7), (.7, 100.6, 7), (float('nan'), 100.8, 8)):
            with self.subTest(seq=seq), self.assertRaises(RuntimeError):
                clock = self.lock_clock()
                clock.observe(device_t, host_t, seq)
        clock = DeviceClock(.1, .05, .25)
        clock.observe(4294.9, 5000, 0xffffffff)
        clock.observe(4295.0, 5000.1, 0)  # unfolded tick and modular sequence both progress

    def driver(self):
        backend = RosBackend.__new__(RosBackend)
        import threading
        backend.lock = threading.RLock()
        backend.config = {'command_timeout_s': .01, 'output_tolerance_v': .002,
                          'channels': {a: dict(channel=i, max_abs_offset_v=1.) for i, a in enumerate('XYZ', 1)}}
        backend.status = {ch: deque(maxlen=4) for ch in (1, 2, 3)}
        backend.types = {f: SimpleNamespace for f in ('output_en', 'offset', 'amplitude', 'waveform', 'sync_output')}
        backend.check = lambda: None
        backend.store = SimpleNamespace(event=lambda *a: None)
        backend.rospy = SimpleNamespace(logerr=lambda *a: None)
        sent = []
        def pub(ch, field):
            def publish(msg):
                sent.append((ch, field, msg.data))
                value = msg.data - 1 if field == 'waveform' and ch != 1 else msg.data
                status = SimpleNamespace(valid=True, output_enabled=False, waveform=0, offset=0., amplitude=0.)
                setattr(status, 'output_enabled' if field == 'output_en' else field, value)
                for _ in range(2):
                    backend.status[ch].append((backend.now() + .000001, status))
            return SimpleNamespace(get_num_connections=lambda: 1, publish=publish)
        backend.publishers = {(ch, field): pub(ch, field) for ch in (1, 2, 3)
                              for field in ('output_en', 'offset', 'amplitude', 'waveform')}
        backend.sync = SimpleNamespace(get_num_connections=lambda: 1, publish=lambda msg: sent.append((0, 'sync', msg.data)))
        return backend, sent

    def test_prepare_waits_for_initialization_before_first_command(self):
        backend, sent = self.driver()
        backend.config['connection_timeout_s'] = 30.
        backend.rospy.get_param = lambda *a: False
        backend.clock = SimpleNamespace(offset=100., anchors=[])
        backend.gate = SimpleNamespace(verified=True)
        elapsed = [0.]
        backend.now = lambda: elapsed[0]
        commands = []
        backend.all_off = lambda: commands.append(elapsed[0])

        def sleep(seconds):
            before = elapsed[0]
            elapsed[0] += seconds
            # Model topics connected immediately, but initialization takes
            # longer than command_timeout. Polling starts only after 12 s.
            for poll in (12., 13.):
                if before < poll <= elapsed[0]:
                    for ch in (1, 2, 3):
                        backend.status[ch].append((poll, SimpleNamespace(valid=True)))

        with patch('sensor_array_bringup.consistency_ros.time.sleep', side_effect=sleep):
            backend.prepare()
        self.assertEqual(len(commands), 1)
        self.assertGreaterEqual(commands[0], 13.)
        self.assertFalse(sent)

    def test_status_readiness_rejects_missing_stale_or_invalid_polls(self):
        backend, _ = self.driver()
        for ch in (1, 2, 3):
            backend.status[ch].extend([(1., SimpleNamespace(valid=True)),
                                      (2., SimpleNamespace(valid=True))])
        self.assertTrue(backend.status_ready(0.))
        self.assertFalse(backend.status_ready(1.5))
        backend.status[3].append((3., SimpleNamespace(valid=False)))
        self.assertFalse(backend.status_ready(0.))

    def test_driver_native_waveform_negative_offset_and_readback(self):
        backend, sent = self.driver()
        with patch('sensor_array_bringup.consistency_ros.time.sleep'):
            backend.set_output({'Y': -.3})
        self.assertIn((2, 'waveform', 6), sent)
        self.assertIn((2, 'offset', -.3), sent)
        self.assertIn((2, 'amplitude', 0.), sent)
        self.assertEqual([r for r in sent if r[1] == 'output_en' and r[2]], [(2, 'output_en', True)])
        # An old status or a single new poll cannot acknowledge a command.
        self.assertFalse(backend.status_matches(2, 'output_en', True, backend.now() + 1))
        backend.status[2].clear()
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            backend.wait(lambda: backend.status_matches(2, 'output_en', True, 0), .001, 'readback')

    def test_all_off_attempts_every_channel_even_after_first_failure(self):
        backend, sent = self.driver()
        calls = []
        def command(ch, *args, **kwargs):
            calls.append(ch)
            if ch == 1:
                raise RuntimeError('disconnected')
        backend.command = command
        with patch('sensor_array_bringup.consistency_ros.time.sleep'), self.assertRaises(RuntimeError):
            backend.all_off()
        self.assertEqual(calls, [1, 2, 3])

    def test_gradient_rank_and_no_improvement_division_claim(self):
        positions = np.zeros((12, 3))
        self.assertIsNone(metrics(np.ones((12, 3)), positions)['gradient_frobenius_ut_m'])
        positions = np.random.RandomState(1).normal(size=(12, 3))
        gradient = np.diag([1., 2., 3.])
        measured = positions @ gradient.T + [10, 20, 30]
        self.assertAlmostEqual(metrics(measured, positions)['gradient_frobenius_ut_m'], np.linalg.norm(gradient))


if __name__ == '__main__':
    unittest.main()
