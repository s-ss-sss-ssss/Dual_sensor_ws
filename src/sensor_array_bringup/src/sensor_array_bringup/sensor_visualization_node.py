"""Display the strongest corrected magnetic vector among sensor IDs 1..12."""
import math
from threading import Lock

import rospy
from sensor_array_bringup.msg import SensorCorrectionRound

from .configuration import validate_field_range


def strongest_sensor(msg):
    """Return (magnitude, family, sample), or None for an empty valid subset."""
    strongest = None
    for family in ('ak', 'tmag'):
        for sample in getattr(msg, family):
            if not 1 <= sample.sensor_id <= 12:
                continue
            magnitude = math.hypot(sample.x, sample.y, sample.z)
            if not math.isfinite(magnitude):
                continue
            if strongest is None or magnitude > strongest[0]:
                strongest = (magnitude, family, sample)
    return strongest


class SensorVisualizationNode:
    def __init__(self):
        self.field_range = validate_field_range(rospy.get_param('~field_range', 1000.0))
        self.topic = rospy.get_param('~input_topic', 'maggrad/sample_corr')
        self.latest = None
        self.lock = Lock()
        self.subscriber = rospy.Subscriber(
            self.topic, SensorCorrectionRound, self.on_round, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.on_timer)

    def on_round(self, msg):
        strongest = strongest_sensor(msg)
        # Inspect every received round so short over-range events are also reported.
        if strongest is not None and strongest[0] > self.field_range:
            magnitude, family, sample = strongest
            rospy.logwarn_throttle(
                1.0, '%s %s ID=%2d over range: |B|=%.6g > %.6g',
                msg.device_id, family.upper(), sample.sensor_id, magnitude, self.field_range)
        with self.lock:
            self.latest = (msg, strongest)

    def on_timer(self, _event):
        with self.lock:
            latest = self.latest
        if latest is None:
            rospy.loginfo('Waiting for corrected samples on %s', self.topic)
            return
        msg, strongest = latest
        if strongest is None:
            rospy.loginfo('%s round=%d: no finite corrected samples with ID 1..12',
                          msg.device_id, msg.round_seq)
            return
        magnitude, family, sample = strongest
        rospy.loginfo('%s round=%d %s ID=%2d |B|=%.6g X=%.6g Y=%.6g Z=%.6g',
                      msg.device_id, msg.round_seq, family.upper(), sample.sensor_id,
                      magnitude, sample.x, sample.y, sample.z)


def main():
    rospy.init_node('sensor_visualization')
    try:
        node = SensorVisualizationNode()
    except (ValueError, TypeError) as error:
        rospy.logfatal('Invalid visualization configuration: %s', error)
        raise SystemExit(1)
    rospy.spin()
