"""TCP geometry and migration safeguards, without a ROS master."""
import copy
import json
import tempfile
import unittest
from itertools import chain, repeat
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped

from magnetic_scan.configuration import load_config
from magnetic_scan.engine import AlignmentEngine, Engine, ScanError
from magnetic_scan.geometry import inverse, rotation_from_matrix
from magnetic_scan.initial_pose import load_initial_pose
from magnetic_scan.mock import MockMotion, MockVision, MockSampler
from magnetic_scan.recording import Recorder
from magnetic_scan.ros_adapters import RosMotion, RosVision
from magnetic_scan.tool_geometry import fixed_chain, xyz_rpy

PACKAGE = Path(__file__).resolve().parents[1]


def message(parent, child, matrix, stamp=0):
    msg = TransformStamped()
    msg.header.frame_id, msg.child_frame_id = parent, child
    msg.header.stamp = rospy.Time.from_sec(stamp)
    t, q = msg.transform.translation, msg.transform.rotation
    t.x, t.y, t.z = matrix[:3, 3]
    q.x, q.y, q.z, q.w = rotation_from_matrix(matrix[:3, :3]).as_quat()
    return msg


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.c = load_config(PACKAGE / 'config/experiment.arm2.yaml')
        self.c['output_dir'] = self.temp.name
        self.c['initial_pose_file'] = str(Path(self.temp.name) / 'start.json')
        self.c['parameters']['vision_timeout_s'] = .03

    def vision(self):
        vision = object.__new__(RosVision)
        vision.c, vision.p = self.c, self.c['parameters']
        vision.joints = Mock()
        vision.joints.assert_still = Mock()
        vision.buffer = Mock()
        return vision

    def static_lookup(self, parent, child, stamp):
        p = next(p for p in self.c['visual_profiles'].values() if p['child'] == child)
        return message(parent, child, np.array(p['matrix']))

    def test_static_profile_checks_accept_real_tf2_static_timestamp(self):
        vision = self.vision()
        buffer = tf2_ros.Buffer(debug=False)
        for p in self.c['visual_profiles'].values():
            buffer.set_transform_static(message(p['parent'], p['child'], np.array(p['matrix']), 100), 'test')
        vision.buffer.lookup_transform.side_effect = buffer.lookup_transform_core
        vision.validate_geometry(lambda: None)

    def test_missing_mismatched_or_dynamic_visual_tcp_is_rejected(self):
        for failure in ('missing', 'offset', 'dynamic'):
            vision = self.vision()
            def lookup(parent, child, stamp):
                if failure == 'missing':
                    raise tf2_ros.LookupException('missing TCP')
                msg = self.static_lookup(parent, child, stamp)
                if failure == 'offset':
                    msg.transform.translation.z *= -1
                else:
                    msg.header.stamp = rospy.Time(100)
                return msg
            vision.buffer.lookup_transform.side_effect = lookup
            with self.assertRaises(ScanError):
                vision.validate_geometry(lambda: None)

    def test_visual_tcp_is_consumed_without_a_second_offset(self):
        vision = self.vision()
        vision.last_stamps = (0., 0.)
        vision.p = dict(vision.p, vision_min_observations=2, vision_window_s=.05, vision_timeout_s=.3)
        sensor = xyz_rpy([.2, .3, .4], [.1, .2, .3])
        source = xyz_rpy([.1, .2, .3], [0, 0, .4])
        calls = [0]
        def lookup(parent, child, stamp):
            if parent != self.c['vision_frame']:
                return self.static_lookup(parent, child, stamp)
            is_sensor = child == self.c['sensor_tcp_frame']
            if is_sensor:
                calls[0] += 1
            return message(parent, child, sensor if is_sensor else source, 1000 + .01 * calls[0])
        vision.buffer.lookup_transform.side_effect = lookup
        with patch('rospy.Time.now', side_effect=chain([rospy.Time(1000)], repeat(rospy.Time.from_sec(1000.1)))):
            obs = vision.observe(lambda: None)
        np.testing.assert_allclose(obs['sensor'], sensor, atol=1e-12)
        np.testing.assert_allclose(obs['source'], source, atol=1e-12)

    def test_urdf_fixed_chain_rejects_missing_or_movable_tcp(self):
        description = '''<robot><link name="ee"/><link name="mount"/><link name="tcp"/>
          <joint name="a" type="fixed"><parent link="ee"/><child link="mount"/>
          <origin xyz="0 0 .005"/></joint>
          <joint name="b" type="fixed"><parent link="mount"/><child link="tcp"/>
          <origin xyz="0 0 .2146"/></joint></robot>'''
        np.testing.assert_allclose(fixed_chain(description, 'ee', 'tcp'), xyz_rpy([0, 0, .2196], [0, 0, 0]))
        for xml, child in ((description, 'absent'), (description.replace('type="fixed"', 'type="revolute"'), 'tcp')):
            with self.assertRaises(ValueError):
                fixed_chain(xml, 'ee', child)

    def test_robot_pose_queries_complete_tf_chain_and_checks_age(self):
        motion = object.__new__(RosMotion)
        motion.c, motion.p, motion.buffer = self.c, self.c['parameters'], Mock()
        expected = xyz_rpy([.1, .2, .3], [.3, .2, .1])
        with patch('rospy.Time.now', return_value=rospy.Time(100)):
            for stamp in (99, 100):
                motion.buffer.lookup_transform.return_value = message('world', self.c['robot_sensor_frame'], expected, stamp)
                if stamp == 99:
                    with self.assertRaisesRegex(ScanError, 'stale robot TCP'):
                        motion.sensor_pose()
                else:
                    np.testing.assert_allclose(motion.sensor_pose(), expected, atol=1e-12)
        self.assertEqual(motion.buffer.lookup_transform.call_args[0][:2], ('world', self.c['robot_sensor_frame']))

    def test_complete_robot_tf_and_urdf_fixed_chain_preflight(self):
        motion = object.__new__(RosMotion)
        motion.c, motion.p = self.c, self.c['parameters']
        ee, tcp = self.c['planning_link'], self.c['robot_sensor_frame']
        description = ('<robot><link name="%s"/><link name="%s"/>'
                       '<joint name="tool" type="fixed"><parent link="%s"/>'
                       '<child link="%s"/><origin xyz="0 0 .2196"/></joint></robot>') % (ee, tcp, ee, tcp)
        world_from_ee = xyz_rpy([.1, .2, .3], [.1, .2, .3])
        fixed = xyz_rpy([0, 0, .2196], [0, 0, 0])
        for failure in (None, 'mismatch', 'missing'):
            buffer = tf2_ros.Buffer(debug=False)
            buffer.set_transform(message('world', ee, world_from_ee, 100), 'test')
            if failure != 'missing':
                actual = fixed.copy()
                if failure == 'mismatch':
                    actual[2, 3] += .001
                buffer.set_transform_static(message(ee, tcp, actual), 'test')
            motion.buffer = Mock()
            motion.buffer.lookup_transform.side_effect = buffer.lookup_transform_core
            with patch('rospy.get_param', return_value=description), patch('rospy.Time.now', return_value=rospy.Time(100)):
                if failure:
                    with self.assertRaises(ScanError):
                        motion.validate_geometry(lambda: None)
                else:
                    motion.validate_geometry(lambda: None)
                    np.testing.assert_allclose(motion.sensor_pose(), world_from_ee @ fixed, atol=1e-12)

    def test_desynchronized_visual_bodies_cannot_form_a_window(self):
        vision = self.vision()
        vision.last_stamps = (0., 0.)
        vision.p = dict(vision.p, vision_min_observations=2, vision_window_s=.001)
        calls = [0]
        def lookup(parent, child, stamp):
            if parent != self.c['vision_frame']:
                return self.static_lookup(parent, child, stamp)
            calls[0] += 1
            value = 1000 + .001 * calls[0] + (.2 if child == self.c['source_tcp_frame'] else 0)
            return message(parent, child, np.eye(4), value)
        vision.buffer.lookup_transform.side_effect = lookup
        with patch('rospy.Time.now', side_effect=chain([rospy.Time(1000)], repeat(rospy.Time.from_sec(1000.4)))):
            with self.assertRaisesRegex(ScanError, 'paired visual observations timed out'):
                vision.observe(lambda: None)

    def test_legacy_config_fields_require_migration(self):
        for key in ('planning_link_to_sensor', 'tracking_to_sensor', 'tracking_to_source'):
            c = copy.deepcopy(self.c)
            c[key] = {}
            path = Path(self.temp.name) / 'old.yaml'
            path.write_text(yaml.safe_dump(c))
            with self.assertRaisesRegex(ValueError, 'migrate legacy extrinsics'):
                load_config(path)

    def aligned(self, fixed=None):
        transforms = None if fixed is None else {(self.c['planning_link'], self.c['robot_sensor_frame']): fixed}
        motion = MockMotion(self.c, transforms=transforms)
        vision = MockVision(motion)
        recorder = Recorder(self.c, {'points': []})
        self.assertEqual(AlignmentEngine(self.c, motion, vision, recorder).run(), 'COMPLETE')
        return motion, vision

    def test_changed_geometry_profile_or_old_start_blocks_joint_restoration(self):
        for failure in ('fixed', 'profile', 'old'):
            motion, vision = self.aligned()
            if failure == 'fixed':
                motion.transforms[(self.c['planning_link'], self.c['robot_sensor_frame'])][2, 3] += .001
            elif failure == 'profile':
                # Same file path and transforms: metadata still identifies a changed profile.
                self.c['visual_profiles']['sensor']['profile']['description'] += ' changed'
            else:
                path = Path(self.c['initial_pose_file'])
                saved = json.loads(path.read_text()); saved['version'] = 1
                path.write_text(json.dumps(saved))
            motion.restore_joints = Mock()
            sampler = MockSampler(motion, self.c); sampler.collect = Mock()
            engine = Engine(self.c, {'points': []}, motion, vision, sampler, Recorder(self.c, {'points': []}))
            self.assertEqual(engine.run(), 'FAILED')
            motion.restore_joints.assert_not_called()
            sampler.collect.assert_not_called()

    def test_geometry_failure_blocks_alignment_and_saved_start_restoration(self):
        motion, vision = self.aligned()
        vision.transforms.clear()
        motion.restore_joints = Mock(); motion.move_to = Mock()
        for align in (False, True):
            recorder = Recorder(self.c, {'points': []})
            engine = (AlignmentEngine(self.c, motion, vision, recorder) if align else
                      Engine(self.c, {'points': []}, motion, vision, MockSampler(motion, self.c), recorder))
            self.assertEqual(engine.run(), 'FAILED')
        motion.restore_joints.assert_not_called()
        motion.move_to.assert_not_called()

    def test_rotated_tool_target_converts_to_ee_once_and_saves_evidence(self):
        fixed = xyz_rpy([.01, .02, .2196], [.1, -.2, .4])
        motion, vision = self.aligned(fixed)
        points = {'points': [dict(id='one', offset_m=[.002, 0, 0], vision_check=True)]}
        recorder = Recorder(self.c, points)
        engine = Engine(self.c, points, motion, vision, MockSampler(motion, self.c), recorder)
        self.assertEqual(engine.run(), 'COMPLETE')
        row = json.loads((recorder.path / 'results.jsonl').read_text())
        np.testing.assert_allclose(row['command_planning_link'],
                                   motion.bias @ np.array(row['target_vision']) @ inverse(fixed), atol=1e-12)
        saved = load_initial_pose(self.c)
        self.assertEqual(saved['version'], 2)
        np.testing.assert_allclose(saved['setup']['resolved_geometry']['fixed_transform'], fixed)
        self.assertTrue((recorder.path / 'geometry.json').is_file())


if __name__ == '__main__':
    unittest.main()
