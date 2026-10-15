import unittest
from threading import Lock
from unittest.mock import patch

try:
    from sensor_array_bringup.msg import SensorCorrectionRound, SensorCorrectionSample
    from sensor_array_bringup.sensor_visualization_node import (
        SensorVisualizationNode, strongest_sensor)
except ImportError:
    SensorVisualizationNode = None


def sample(sensor_id, x, y=0, z=0):
    return SensorCorrectionSample(sensor_id=sensor_id, x=x, y=y, z=z)


@unittest.skipIf(SensorVisualizationNode is None, 'source built workspace to test ROS messages')
class VisualizationTests(unittest.TestCase):
    def node(self):
        node = SensorVisualizationNode.__new__(SensorVisualizationNode)
        node.field_range = 5.0
        node.topic = 'maggrad/sample_corr'
        node.latest = None
        node.lock = Lock()
        return node

    def test_selects_by_vector_magnitude_across_families_and_id_boundaries(self):
        msg = SensorCorrectionRound(
            ak=[sample(0, 100), sample(1, -3, 4), sample(13, 100)],
            tmag=[sample(12, 3, -4, 12), sample(255, 100)])
        magnitude, family, selected = strongest_sensor(msg)
        self.assertEqual((magnitude, family, selected.sensor_id), (13., 'tmag', 12))
        self.assertEqual((selected.x, selected.y, selected.z), (3., -4., 12.))

    def test_empty_invalid_and_outside_ids(self):
        self.assertIsNone(strongest_sensor(SensorCorrectionRound()))
        self.assertIsNone(strongest_sensor(SensorCorrectionRound(
            ak=[sample(0, 10), sample(1, float('nan')), sample(12, float('inf'))])))

    @patch('sensor_array_bringup.sensor_visualization_node.rospy.logwarn_throttle')
    @patch('sensor_array_bringup.sensor_visualization_node.rospy.loginfo')
    def test_latest_round_and_range_boundary(self, info, warning):
        node = self.node()
        node.on_round(SensorCorrectionRound(round_seq=1, ak=[sample(1, 3, 4)]))
        warning.assert_not_called()  # Equality is within range.
        node.on_round(SensorCorrectionRound(round_seq=2, tmag=[sample(12, -6)]))
        warning.assert_called_once()
        self.assertEqual(warning.call_args.args[-2:], (6., 5.))
        node.on_timer(None)
        self.assertEqual(info.call_args.args[2:6], (2, 'TMAG', 12, 6.))
        node.on_round(SensorCorrectionRound(round_seq=3))
        node.on_timer(None)
        self.assertIn('no finite', info.call_args.args[0])

    @patch('sensor_array_bringup.sensor_visualization_node.rospy')
    def test_subscription_and_five_hz_timer(self, ros):
        ros.get_param.side_effect = lambda name, default: default
        node = SensorVisualizationNode()
        ros.Duration.assert_called_once_with(0.2)
        ros.Timer.assert_called_once_with(ros.Duration.return_value, node.on_timer)
        ros.Subscriber.assert_called_once_with(
            'maggrad/sample_corr', SensorCorrectionRound, node.on_round, queue_size=1)


if __name__ == '__main__':
    unittest.main()
