"""Message-level tests use generated ROS messages, without a ROS master."""
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

try:
    from maggrad_protocol.msg import RoundSample, AkSample, TmagSample
    from sensor_array_bringup.sensor_correction_node import SensorCorrectionNode
except ImportError:
    SensorCorrectionNode = None

from sensor_array_bringup.sensor_correction import AffineSensorCorrection, DeviceIdentityGate


@unittest.skipIf(SensorCorrectionNode is None, 'source built workspace to test ROS messages')
class NodeTests(unittest.TestCase):
    def node(self, check=True):
        with tempfile.TemporaryDirectory() as directory:
            for family, bias in (('ak', [1, 2, 3]), ('tmag', [10, 20, 30])):
                path = Path(directory) / 'board' / family / 'affine.json'
                path.parent.mkdir(parents=True)
                path.write_text(json.dumps({'sensors': [{'sensor_id': 1,
                    'D_i': [[2, 0, 0], [0, 2, 0], [0, 0, 2]], 'e_i': bias}]}))
            node = SensorCorrectionNode.__new__(SensorCorrectionNode)
            node.model = AffineSensorCorrection(directory, 'board', 0.5)
        node.gate = DeviceIdentityGate(0xFFFFFFFF, check)
        node.device_id = 'board'
        node.publisher = Mock()
        node.received = False
        node.unknown = node.missing = 0
        return node

    @patch('sensor_array_bringup.sensor_correction_node.rospy.logwarn_throttle')
    @patch('sensor_array_bringup.sensor_correction_node.rospy.logerr_throttle')
    def test_gate_and_float_message(self, _error, _warning):
        node = self.node()
        msg = RoundSample(round_seq=123)
        node.on_round(msg)
        node.publisher.publish.assert_not_called()
        msg.has_stats = True
        msg.stats.device_id = 0xFFFFFFFF
        msg.ak = [AkSample(sensor_id=1, hx=2, hy=4, hz=6, bus=3, mask=4)]
        msg.tmag = [TmagSample(sensor_id=255), TmagSample(sensor_id=2)]
        node.on_round(msg)
        result = node.publisher.publish.call_args.args[0]
        self.assertEqual((result.ak[0].x, result.ak[0].y, result.ak[0].z), (3., 6., 9.))
        self.assertEqual((result.ak[0].channel, result.ak[0].address), (3, 4))
        self.assertEqual(result.round_seq, 123)
        self.assertTrue(result.identity_verified)
        self.assertEqual(result.firmware_device_id, 0xFFFFFFFF)
        self.assertEqual(result.skipped_unknown_id, 1)
        self.assertEqual(result.skipped_missing_sensor_correction, 1)
        result.serialize(io.BytesIO())
        self.assertEqual(msg.ak[0].hx, 2)
        node.publisher.reset_mock()
        msg.stats.device_id = 1
        node.on_round(msg)
        msg.stats.device_id = 0xFFFFFFFF
        node.on_round(msg)
        node.publisher.publish.assert_not_called()

    def test_mixed_family_round_uses_independent_coefficients(self):
        node = self.node(False)
        msg = RoundSample(ak=[AkSample(sensor_id=1, hx=2, hy=4, hz=6)],
                          tmag=[TmagSample(sensor_id=1, x=2, y=4, z=6)])
        node.on_round(msg)
        result = node.publisher.publish.call_args.args[0]
        self.assertEqual((result.ak[0].x, result.ak[0].y, result.ak[0].z), (3., 6., 9.))
        self.assertEqual((result.tmag[0].x, result.tmag[0].y, result.tmag[0].z), (12., 24., 36.))
        self.assertEqual(result.skipped_missing_sensor_correction, 0)
        result.serialize(io.BytesIO())

    def test_microsecond_timing_is_preserved_without_phase(self):
        node = self.node(False)
        msg = RoundSample(round_seq=42, device_tick_us=0xFFFFFFF0,
                          device_tick_unwrapped_us=0x1FFFFFFF0,
                          ak=[AkSample(sensor_id=1, read_done_offset_us=123)],
                          tmag=[TmagSample(sensor_id=1, read_done_offset_us=456)])
        msg.header.stamp.secs = 123
        msg.header.stamp.nsecs = 456789
        msg.header.frame_id = 'board'
        msg.phase_age_us = [0xFFFFFFFF] * 3
        node.on_round(msg)
        result = node.publisher.publish.call_args.args[0]
        self.assertEqual(result.header, msg.header)
        self.assertEqual(result.round_seq, 42)
        self.assertEqual(result.device_tick_us, 0xFFFFFFF0)
        self.assertEqual(result.device_tick_unwrapped_us, 0x1FFFFFFF0)
        self.assertFalse(result.has_phase)
        self.assertEqual(list(result.phase_age_us), list(msg.phase_age_us))
        self.assertEqual(list(result.phase_valid), list(msg.phase_valid))
        self.assertEqual(result.ak[0].read_done_offset_us, 123)
        self.assertEqual(result.tmag[0].read_done_offset_us, 456)
        self.assertEqual(result.device_tick_unwrapped_us + result.ak[0].read_done_offset_us,
                         0x20000006B)
        self.assertEqual(msg.device_tick_us, 0xFFFFFFF0)
        result.serialize(io.BytesIO())

    @patch('sensor_array_bringup.sensor_correction_node.rospy.logwarn_throttle')
    def test_disabled_gate_empty_round(self, _warning):
        node = self.node(False)
        msg = RoundSample(tmag=[TmagSample(sensor_id=255)])
        node.on_round(msg)
        result = node.publisher.publish.call_args.args[0]
        self.assertFalse(result.identity_verified)
        self.assertFalse(result.has_device_id)
        self.assertEqual(result.tmag, [])
        self.assertEqual(result.skipped_unknown_id, 1)
        result.serialize(io.BytesIO())


if __name__ == '__main__':
    unittest.main()
