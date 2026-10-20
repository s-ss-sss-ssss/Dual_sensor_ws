import copy
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
import yaml
from unittest.mock import Mock, patch

from magnetic_scan.configuration import load_config, load_points
from magnetic_scan.geometry import aligned_target, inverse, offset_target, residual, transform
from magnetic_scan.engine import AlignmentEngine, Engine, ScanError
from magnetic_scan.initial_pose import load_initial_pose, save_initial_pose
from magnetic_scan.mock import MockMotion, MockVision, MockSampler
from magnetic_scan.recording import Recorder
from magnetic_scan.sampling import validate_round, statistics
from magnetic_scan.bringup import render

PACKAGE = Path(__file__).resolve().parents[1]


class ScanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = load_config(PACKAGE / 'config/experiment.example.yaml')
        self.config['output_dir'] = self.temp.name
        self.config['initial_pose_file'] = str(Path(self.temp.name) / 'initial_pose.json')
        self.points = load_points(self.config['points_file'])

    def engine(self, vision_class=MockVision, sampler_class=MockSampler, cancelled=lambda: False):
        self.motion = MockMotion(self.config)
        alignment = AlignmentEngine(self.config, self.motion, MockVision(self.motion),
                                    Recorder(self.config, {'points': []}))
        self.assertEqual(alignment.run(), 'COMPLETE')
        self.vision = vision_class(self.motion)
        self.recorder = Recorder(self.config, self.points)
        self.addCleanup(self.recorder.events.close)
        self.addCleanup(self.recorder.points.close)
        return Engine(self.config, self.points, self.motion, self.vision,
                      sampler_class(self.motion, self.config), self.recorder, cancelled=cancelled)

    def results(self):
        return [json.loads(s) for s in (self.recorder.path / 'results.jsonl').read_text().splitlines()]

    def test_alignment_preserves_axial_distance_and_roll(self):
        source = transform(dict(translation_m=[.2, .3, .4], quaternion_xyzw=[0, 0, .70710678, .70710678]))
        sensor = source.copy()
        sensor[:3, 3] += source[:3, :3] @ [.003, -.005, .1]
        rotation = np.diag([1., -1., -1.])
        target = aligned_target(source, sensor, rotation, [.05, .2])
        np.testing.assert_allclose(inverse(source) @ target, transform(dict(
            translation_m=[0, 0, .1], quaternion_xyzw=[1, 0, 0, 0])), atol=1e-12)
        with self.assertRaises(ValueError):
            aligned_target(source, source, rotation, [.05, .2])

    def test_alignment_saves_explicit_distance_and_all_active_joints(self):
        self.config['alignment']['target_distance_m'] = .12
        motion = MockMotion(self.config)
        recorder = Recorder(self.config, {'points': []})
        self.assertEqual(AlignmentEngine(self.config, motion, MockVision(motion), recorder).run(), 'COMPLETE')
        saved = load_initial_pose(self.config)
        self.assertEqual(len(saved['joints']), 13)
        error = residual(transform(saved['relative_pose']), transform(dict(
            translation_m=[0, 0, .12], quaternion_xyzw=[1, 0, 0, 0])))
        self.assertLessEqual(error[0], self.config['parameters']['position_tolerance_m'])
        self.assertLessEqual(error[1], self.config['parameters']['angle_tolerance_rad'])

    def test_alignment_does_not_require_points_or_sampling(self):
        c = copy.deepcopy(self.config)
        del c['points_file'], c['sampling']
        path = Path(self.temp.name) / 'alignment.yaml'
        path.write_text(yaml.safe_dump(c))
        self.assertEqual(load_config(path, alignment=True)['alignment']['target_distance_m'], .1)

    def test_failed_alignment_preserves_existing_start(self):
        self.engine()
        path = Path(self.config['initial_pose_file'])
        before = path.read_bytes()
        motion = MockMotion(self.config)
        vision = Mock()
        vision.observe.side_effect = ScanError('vision unavailable')
        recorder = Recorder(self.config, {'points': []})
        self.assertEqual(AlignmentEngine(self.config, motion, vision, recorder).run(), 'FAILED')
        self.assertEqual(path.read_bytes(), before)
        with patch('magnetic_scan.initial_pose.os.replace', side_effect=OSError('disk error')):
            with self.assertRaises(OSError):
                save_initial_pose(self.config, load_initial_pose(self.config))
        self.assertEqual(path.read_bytes(), before)

    def test_missing_invalid_or_mismatched_start_never_moves_or_samples(self):
        for invalid in ('missing', 'joint', 'setup'):
            e = self.engine()
            path = Path(self.config['initial_pose_file'])
            saved = load_initial_pose(self.config)
            if invalid == 'missing':
                path.unlink()
            else:
                if invalid == 'joint':
                    saved['joints'].pop('diana7_joint_7')
                else:
                    saved['setup']['source_arm'] = 'arm2'
                path.write_text(json.dumps(saved))
            e.motion.restore_joints = Mock()
            e.sampler.collect = Mock()
            self.assertEqual(e.run(), 'FAILED')
            e.motion.restore_joints.assert_not_called()
            e.sampler.collect.assert_not_called()

    def test_start_check_rejects_relative_error_without_correction(self):
        for translation, angle in ((.01, 0), (0, .1)):
            e = self.engine()
            original = self.vision.observe
            def wrong_pose(check):
                obs = original(check)
                obs['sensor'][0, 3] += translation
                if angle:
                    obs['sensor'][:3, :3] = transform(dict(translation_m=[0, 0, 0],
                        quaternion_xyzw=[0, 0, np.sin(angle / 2), np.cos(angle / 2)]))[:3, :3] @ obs['sensor'][:3, :3]
                return obs
            self.vision.observe = wrong_pose
            self.motion.move_to = Mock()
            e.sampler.collect = Mock()
            self.assertEqual(e.run(), 'FAILED')
            self.motion.move_to.assert_not_called()
            e.sampler.collect.assert_not_called()

    def test_restore_failure_never_observes_or_samples(self):
        e = self.engine()
        e.motion.restore_joints = Mock(side_effect=ScanError('restoration failed'))
        e.vision.observe = Mock()
        e.sampler.collect = Mock()
        self.assertEqual(e.run(), 'FAILED')
        e.vision.observe.assert_not_called()
        e.sampler.collect.assert_not_called()

    def test_all_false_points_only_observe_once_at_start(self):
        e = self.engine()
        for point in self.points['points']:
            point.pop('vision_check', None)
        self.motion.actual[:3, 3] += [.02, .01, -.01]
        self.assertEqual(e.run(), 'COMPLETE')
        self.assertEqual(self.vision.stamp, 1)
        self.assertTrue(all(row['visual_measurement'] is None for row in self.results()))
        self.assertTrue((self.recorder.path / 'initial_pose.json').is_file())

    def test_alignment_launch_has_no_scan_or_sensor_include(self):
        import xml.etree.ElementTree as ET
        for arm in ('arm1', 'arm2'):
            c = copy.deepcopy(self.config)
            c['source_arm'] = arm
            c['hardware']['source_tool'] = 'calibrated_em40250'
            xml = ET.fromstring(render(c, vision=True, alignment=True))
            self.assertIsNotNone(xml.find("node[@name='magnetic_align']/param[@name='alignment'][@value='true']"))
            self.assertIsNotNone(xml.find("node[@name='scan_joint_states']/param[@name='alignment'][@value='true']"))
            includes = [node.get('file') for node in xml.findall('include')]
            self.assertFalse(any('scan.launch' in name or 'array.launch' in name for name in includes))
            self.assertTrue(any('localization_tagslam.launch' in name for name in includes))

    def test_arm2_experiment_geometry_and_three_arm_launch(self):
        import xml.etree.ElementTree as ET
        c = load_config(PACKAGE / 'config/experiment.arm2.yaml', hardware=True)
        self.assertEqual(c['source_arm'], 'arm2')
        self.assertEqual(c['hardware']['sensor_tool'], 'calibrated_magnetmeter_array_dual')
        self.assertEqual(c['robot_sensor_frame'], 'diana7_magnetmeter_array_dual_tcp_link')
        self.assertEqual(c['hardware']['source_tool'], 'calibrated_em4098')
        self.assertEqual(c['sensor_tracking_frame'], 'sensor_array_calib')
        self.assertEqual(c['sensor_tcp_frame'], 'sensor_array')
        expected = transform(dict(translation_m=[0, 0, .2171],
                                 quaternion_xyzw=[0, 0, -2**-.5, 2**-.5]))
        np.testing.assert_allclose(c['visual_profiles']['sensor']['matrix'], expected, atol=1e-10)
        for alignment in (True, False):
            xml = ET.fromstring(render(c, vision=True, alignment=alignment))
            fr5 = [node for node in xml.findall('include') if 'fr5_hardware' in node.get('file')]
            self.assertEqual([node.find("arg[@name='arm_id']").get('value') for node in fr5],
                             ['arm2', 'arm1'])
            vision = [node for node in xml.findall('include') if 'localization_tagslam' in node.get('file')][0]
            args = {node.get('name'): node.get('value') for node in vision.findall('arg')}
            self.assertEqual(args, dict(diana7_tool='none', arm1_tool='none',
                                       arm2_tool='calibrated_em4098', sensor_tool='calibrated_magnetmeter_array_dual',
                                       visual_tool_transforms_file=c['vision']['transforms_file'],
                                       sensor_tool_profile='default', arm2_tool_profile='default',
                                       sensor_tracking_frame='sensor_array_calib', sensor_tcp_frame='sensor_array',
                                       arm2_tracking_frame='arm2_flange_calib', arm2_tcp_frame='arm2_em_tcp'))
            controllers = yaml.safe_load(xml.find("rosparam[@param='/move_group/controller_list']").text)
            self.assertEqual(len(controllers), 3)

    def test_launch_defaults_use_physical_config_and_do_not_override_source_arm(self):
        import xml.etree.ElementTree as ET
        for name in ('align', 'experiment', 'scan'):
            root = ET.parse(PACKAGE / 'launch' / (name + '.launch')).getroot()
            self.assertTrue(root.find("arg[@name='config']").get('default').endswith('/experiment.arm2.yaml'))
            self.assertEqual(root.find("arg[@name='source_arm']").get('default'), '')
            if name != 'scan':
                self.assertEqual(root.find("arg[@name='start_vision']").get('default'), 'true')

    def test_offset_is_rotated_and_not_accumulated(self):
        origin = transform(dict(translation_m=[1, 2, 3], quaternion_xyzw=[0, 0, .70710678, .70710678]))
        np.testing.assert_allclose(offset_target(origin, [.01, 0, 0])[:3, 3], [1, 2.01, 3])
        np.testing.assert_allclose(offset_target(origin, [0, 0, 0]), origin)

    def test_mock_full_run_with_nonzero_link_offset(self):
        e = self.engine()
        self.assertEqual(e.run(), 'COMPLETE')
        rows = self.results()
        self.assertEqual([r['id'] for r in rows], ['origin', 'x_plus', 'return'])
        np.testing.assert_allclose(self.motion.actual, e.origin, atol=1e-12)
        self.assertIsNone(rows[1]['visual_measurement'])
        self.assertGreater(rows[2]['stage'], rows[0]['stage'])
        self.assertTrue((self.recorder.path / 'summary.json').is_file())
        self.assertIn('ak:1', rows[0]['samples']['sensors'])

    def test_checkpoint_compensates_bias_without_moving_origin(self):
        class ChangingBiasVision(MockVision):
            def observe(self, check):
                # Introduce drift after initial alignment and the first point.
                obs = super().observe(check)
                if self.stamp == 3:
                    self.motion.bias[:3, 3] += [.0008, 0, 0]
                    self.motion.actual[:3, 3] -= [.0008, 0, 0]
                    obs['sensor'] = self.motion.actual.copy()
                return obs
        self.points['points'][1]['vision_check'] = True
        e = self.engine(ChangingBiasVision)
        self.assertEqual(e.run(), 'COMPLETE')
        for row in self.results():
            np.testing.assert_allclose(row['target_vision'], offset_target(e.origin, row['offset_m']), atol=1e-12)
        np.testing.assert_allclose(self.motion.actual, e.origin, atol=1e-12)

    def test_source_motion_fails_and_preserves_completed_points(self):
        class MovingSource(MockVision):
            def observe(self, check):
                obs = super().observe(check)
                if self.stamp >= 3:
                    obs['source'][0, 3] += .01
                return obs
        self.points['points'][1]['vision_check'] = True
        e = self.engine(MovingSource)
        self.assertEqual(e.run(), 'FAILED')
        rows = self.results()
        self.assertEqual(rows[0]['state'], 'complete')
        self.assertEqual(rows[-1]['state'], 'failed')
        self.assertTrue(self.motion.stopped)
        self.assertIn('fixed source moved', rows[-1]['reason'])

    def test_visual_timeout_and_stop_are_recorded(self):
        class NoVision(MockVision):
            def observe(self, check):
                raise ScanError('fresh visual observations timed out')
        self.assertEqual(self.engine(NoVision).run(), 'FAILED')
        self.assertEqual(self.engine(cancelled=lambda: True).run(), 'STOPPED')

    def test_correction_nonconvergence(self):
        class FrozenVision(MockVision):
            def __init__(self, motion):
                super().__init__(motion)
                self.frozen = motion.actual.copy()
            def observe(self, check):
                result = super().observe(check)
                result['sensor'] = self.frozen.copy()
                return result
        self.points['points'][1]['vision_check'] = True
        self.assertEqual(self.engine(FrozenVision).run(), 'FAILED')
        summary = json.loads((self.recorder.path / 'summary.json').read_text())
        self.assertIn('did not converge', summary['reason'])

    def test_sample_failure_preserves_partial_point(self):
        class NoSamples(MockSampler):
            def collect(self, seconds, check):
                raise ScanError('insufficient valid rounds')
        self.assertEqual(self.engine(sampler_class=NoSamples).run(), 'FAILED')
        self.assertEqual(self.results()[0]['state'], 'failed')

    def test_failed_motion_is_not_followed_by_sampling(self):
        e = self.engine()
        def failed(*args):
            raise ScanError('incomplete Cartesian trajectory: 0.9')
        self.motion.move_to = failed
        self.assertEqual(e.run(), 'FAILED')
        self.assertTrue(self.motion.stopped)

    def test_points_validation_and_repeated_positions(self):
        doc = copy.deepcopy(self.points)
        doc['points'][1]['id'] = doc['points'][0]['id']
        path = Path(self.temp.name) / 'points.json'
        path.write_text(json.dumps(doc))
        with self.assertRaises(ValueError):
            load_points(path)
        doc = copy.deepcopy(self.points)
        doc['points'][1]['offset_m'][0] = float('nan')
        path.write_text(json.dumps(doc))
        with self.assertRaises(ValueError):
            load_points(path)
        self.assertEqual(len(load_points(self.config['points_file'])['points']), 3)

    def test_hardware_requires_real_geometry_and_inactive_state(self):
        with self.assertRaisesRegex(ValueError, 'geometry_confirmed'):
            load_config(self.config['config_path'], hardware=True)
        c = copy.deepcopy(self.config)
        c['hardware']['geometry_confirmed'] = True
        c['hardware']['source_tool'] = 'calibrated_em40250'
        path = Path(self.temp.name) / 'hardware.yaml'
        path.write_text(yaml.safe_dump(c))
        with self.assertRaisesRegex(ValueError, 'stationary joints'):
            load_config(path, hardware=True)

    def test_dual_launch_selects_only_requested_hardware_and_controllers(self):
        import xml.etree.ElementTree as ET
        for arm in ('arm1', 'arm2'):
            c = copy.deepcopy(self.config)
            c['source_arm'] = arm
            c['hardware']['source_tool'] = 'calibrated_em40250'
            xml = ET.fromstring(render(c))
            fr5 = [n for n in xml.findall('include') if 'fr5_hardware' in n.get('file')]
            self.assertEqual(len(fr5), 1)
            self.assertEqual(fr5[0].find("arg[@name='arm_id']").get('value'), arm)
            ctl = yaml.safe_load(xml.find("rosparam[@param='/move_group/controller_list']").text)
            self.assertEqual([i['name'] for i in ctl], ['diana7/position_trajectory_controller', arm + '/' + arm + '_controller'])
            self.assertFalse(any('triple_arm_bringup' in n.get('file') for n in xml.findall('include')))

    def test_round_quality_and_statistics(self):
        row = dict(sensor_id=1, x=1., y=2., z=3., drdy=1, err=0, dor=0, overflow=0)
        record = dict(identity_verified=True, errors=[], ak=[row], tmag=[])
        expected = {'ak': [1]}
        vectors, reason = validate_round(record, expected)
        self.assertIsNone(reason)
        self.assertEqual(statistics([vectors, vectors])['ak:1']['std'], [0, 0, 0])
        for key in ('err', 'dor', 'overflow'):
            bad = copy.deepcopy(record)
            bad['ak'][0][key] = 1
            self.assertEqual(validate_round(bad, expected)[1], 'sensor_status')
        self.assertEqual(validate_round(record, {'ak': [1, 2]})[1], 'missing_sensor')
        bad = copy.deepcopy(record)
        bad['ak'].append(row)
        self.assertEqual(validate_round(bad, expected)[1], 'duplicate_id')
        record['identity_verified'] = False
        self.assertEqual(validate_round(record, expected)[1], 'identity_unverified')


if __name__ == '__main__':
    unittest.main()
