"""Adapter contracts without a ROS master or hardware connections."""
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import rospy
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_array_bringup.msg import SensorCorrectionRound, SensorCorrectionSample

from magnetic_scan.configuration import load_config
from magnetic_scan.initial_pose import joint_names
from magnetic_scan.engine import ScanError
from magnetic_scan.joint_states import StateMerger
from magnetic_scan.ros_adapters import RosMotion, RosVision
from magnetic_scan.geometry import rotation_from_matrix
from magnetic_scan.ros_recording import RosSampler


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config(Path(__file__).resolve().parents[1] / 'config/experiment.example.yaml')

    def test_merger_requires_both_fresh_arms_and_rejects_repeated_stamp(self):
        for source in ('arm1', 'arm2'):
            inactive = 'arm2' if source == 'arm1' else 'arm1'
            fixed = {inactive + '_joint_%d' % i: .2 for i in range(1, 7)}
            merger = StateMerger(source, fixed, .5)
            for arm, count in (('diana7', 7), (source, 6)):
                names = [arm + '_joint_%d' % i for i in range(1, count + 1)]
                self.assertTrue(merger.update(arm, 100., 10., names, [0.] * count, []))
                self.assertFalse(merger.update(arm, 100., 10.3, names, [1.] * count, []))
                if arm == 'diana7':
                    self.assertIsNone(merger.combined(10., 100.))
            merged = merger.combined(10.1, 100.1)
            self.assertEqual(len(merged['name']), 19)
            self.assertEqual(merged['position'][merged['name'].index(inactive + '_joint_1')], .2)
            self.assertEqual(merged['velocity'], [])
            self.assertIsNone(merger.combined(10.6, 100.6))

    def test_three_arm_merger_requires_real_feedback_from_arm1(self):
        merger = StateMerger('arm2', {}, .5, connect_inactive_arm=True)
        for arm, count in (('diana7', 7), ('arm2', 6)):
            merger.update(arm, 100., 10., [arm + '_joint_%d' % i for i in range(1, count + 1)],
                          [.1] * count, [])
        self.assertIsNone(merger.combined(10., 100.))
        merger.update('arm1', 100., 10., ['arm1_joint_%d' % i for i in range(1, 7)], [.3] * 6, [])
        merged = merger.combined(10., 100.)
        self.assertEqual(len(merged['name']), 19)
        self.assertEqual(merged['position'][merged['name'].index('arm1_joint_1')], .3)
        self.assertIsNone(merger.combined(10.6, 100.6))

    def test_launch_arguments_resolve_with_noetic_substitutions(self):
        package = Path(__file__).resolve().parents[1]
        for name in ('align', 'experiment'):
            config = ROSLaunchConfig()
            XmlLoader().load(str(package / 'launch' / (name + '.launch')), config, verbose=False)
            args = config.nodes[0].args
            self.assertIn('experiment.arm2.yaml', args)
            self.assertIn('--vision', args)
            self.assertNotIn('--source-arm', args)
            self.assertNotIn('--sensors', args)
            config = ROSLaunchConfig()
            overrides = ['source_arm:=arm1', 'start_vision:=false']
            if name == 'experiment':
                overrides.append('start_sensors:=true')
            XmlLoader().load(str(package / 'launch' / (name + '.launch')), config,
                             argv=overrides, verbose=False)
            args = config.nodes[0].args
            self.assertIn('--source-arm arm1', args)
            self.assertNotIn('--vision', args)
            if name == 'experiment':
                self.assertIn('--sensors', args)

    def test_partial_cartesian_plan_is_never_executed(self):
        motion = object.__new__(RosMotion)
        motion.c, motion.p = self.c, self.c['parameters']
        motion.joints = Mock()
        motion.group = Mock()
        current = PoseStamped()
        current.pose.orientation.w = 1.
        current.pose.position.x = .01
        motion.group.get_current_pose.return_value = current
        motion.group.compute_cartesian_path.return_value = (Mock(), .99)
        motion.client = Mock()
        with self.assertRaisesRegex(ScanError, 'incomplete Cartesian'):
            motion.move_to(np.eye(4), lambda: None)
        motion.client.send_goal.assert_not_called()
        motion.group.retime_trajectory.assert_not_called()

    def test_noop_origin_does_not_execute_zero_duration_trajectory(self):
        motion = object.__new__(RosMotion)
        motion.c, motion.p = self.c, self.c['parameters']
        motion.joints, motion.group, motion.client = Mock(), Mock(), Mock()
        current = PoseStamped()
        current.pose.orientation.w = 1.
        motion.group.get_current_pose.return_value = current
        motion.move_to(np.eye(4), lambda: None)
        motion.client.send_goal.assert_not_called()
        motion.group.compute_cartesian_path.assert_not_called()

    def restoration_motion(self):
        motion = object.__new__(RosMotion)
        motion.c, motion.p = self.c, self.c['parameters']
        motion.source_group, motion.group = Mock(), Mock()
        motion.joints = Mock()
        names = joint_names(self.c)
        actual = dict.fromkeys(names, 0.)
        motion.joints.snapshot.side_effect = lambda: dict(name=names, position=[actual[n] for n in names])
        for group in (motion.source_group, motion.group):
            trajectory = SimpleNamespace(joint_trajectory=SimpleNamespace(
                points=[SimpleNamespace(time_from_start=rospy.Duration(1))]))
            group.plan.return_value = (True, trajectory, .1, None)
        return motion, actual

    def test_restore_source_before_sensor_and_verify_measured_angles(self):
        motion, actual = self.restoration_motion()
        order = []
        def execute(plan, check):
            check()
            group = motion.source_group if not order else motion.group
            order.append('source' if not order else 'sensor')
            actual.update(group.set_joint_value_target.call_args[0][0])
        motion.execute = Mock(side_effect=execute)
        desired = dict.fromkeys(actual, .1)
        motion.restore_joints(desired, lambda: None)
        self.assertEqual(order, ['source', 'sensor'])
        self.assertEqual(actual, desired)

    def test_restore_planning_failure_does_not_execute(self):
        motion, actual = self.restoration_motion()
        motion.source_group.plan.return_value = (False, None, .1, None)
        motion.execute = Mock()
        with self.assertRaisesRegex(ScanError, 'planning failed'):
            motion.restore_joints(dict.fromkeys(actual, .1), lambda: None)
        motion.execute.assert_not_called()
        motion.group.plan.assert_not_called()

    def test_restore_rejects_execution_that_does_not_reach_target(self):
        motion, actual = self.restoration_motion()
        motion.execute = Mock()
        with self.assertRaisesRegex(ScanError, 'outside tolerance'):
            motion.restore_joints(dict.fromkeys(actual, .1), lambda: None)
        motion.group.plan.assert_not_called()

    def test_restore_already_at_target_skips_planning(self):
        motion, actual = self.restoration_motion()
        motion.execute = Mock()
        motion.restore_joints(dict(actual), lambda: None)
        motion.execute.assert_not_called()
        motion.source_group.plan.assert_not_called()
        motion.group.plan.assert_not_called()

    def test_restore_stop_prevents_planning(self):
        motion, actual = self.restoration_motion()
        def stopped():
            raise ScanError('stop requested')
        with self.assertRaisesRegex(ScanError, 'stop requested'):
            motion.restore_joints(dict.fromkeys(actual, .1), stopped)
        motion.source_group.plan.assert_not_called()

    def test_repeated_or_stale_tf_cannot_form_a_visual_window(self):
        for stamp in (999., 1000.):
            vision = object.__new__(RosVision)
            vision.c, vision.p = self.c, dict(self.c['parameters'], vision_timeout_s=.03,
                                             vision_window_s=.001, vision_min_observations=2)
            vision.joints = SimpleNamespace(assert_still=lambda: None)
            vision.last_stamps = (0., 0.)
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.from_sec(stamp)
            msg.transform.rotation.w = 1.
            vision.buffer = Mock()
            def lookup(parent, child, stamp):
                if parent == self.c['vision_frame']:
                    return msg
                profile = next(p for p in self.c['visual_profiles'].values() if p['child'] == child)
                static = TransformStamped()
                matrix = np.array(profile['matrix'])
                static.transform.translation.x, static.transform.translation.y, static.transform.translation.z = matrix[:3, 3]
                q = rotation_from_matrix(matrix[:3, :3]).as_quat()
                static.transform.rotation.x, static.transform.rotation.y, static.transform.rotation.z, static.transform.rotation.w = q
                return static
            vision.buffer.lookup_transform.side_effect = lookup
            with patch('rospy.Time.now', return_value=rospy.Time(1000)):
                with self.assertRaisesRegex(ScanError, 'visual observations timed out'):
                    vision.observe(lambda: None)

    def sampler(self):
        sampler = object.__new__(RosSampler)
        sampler.c = self.c
        sampler.p = dict(self.c['parameters'], sample_timeout_s=.04, min_rounds=2)
        sampler.joints = SimpleNamespace(assert_still=lambda: None)
        sampler.lock = threading.Lock()
        sampler.queue = deque()
        sampler.active, sampler.overflow = False, False
        return sampler

    def record(self, seq):
        msg = SensorCorrectionRound()
        msg.header.stamp = rospy.Time(1000)
        msg.identity_verified = True
        msg.round_seq = seq
        msg.device_tick_unwrapped_us = seq
        msg.ak = [SensorCorrectionSample(sensor_id=i, x=1., y=2., z=3., drdy=1) for i in range(1, 13)]
        return msg

    def test_sampler_records_valid_data_and_rejects_duplicate_rounds(self):
        sampler = self.sampler()
        seq = [0]
        def next_round():
            seq[0] += 1
            sampler.callback(self.record(seq[0]))
        with patch('rospy.Time.now', return_value=rospy.Time(1000)):
            result = sampler.collect(.001, next_round)
        self.assertGreaterEqual(result['valid_rounds'], 2)
        self.assertEqual(result['sensors']['ak:1']['mean'], [1., 2., 3.])
        sampler = self.sampler()
        with patch('rospy.Time.now', return_value=rospy.Time(1000)):
            with self.assertRaisesRegex(ScanError, 'insufficient valid rounds: 1'):
                sampler.collect(.001, lambda: sampler.callback(self.record(1)))
        self.assertFalse(sampler.active)


if __name__ == '__main__':
    unittest.main()
