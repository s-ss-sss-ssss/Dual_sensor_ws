"""Adapter contracts without a ROS master or hardware connections."""
import threading
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_array_bringup.msg import SensorCorrectionRound, SensorCorrectionSample

from magnetic_scan.configuration import load_config
from magnetic_scan.engine import ScanError
from magnetic_scan.joint_states import StateMerger
from magnetic_scan.ros_adapters import RosMotion, RosVision
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
            vision.buffer.lookup_transform.return_value = msg
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
