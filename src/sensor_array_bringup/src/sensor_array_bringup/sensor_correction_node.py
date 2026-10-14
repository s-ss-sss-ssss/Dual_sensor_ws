"""Online sensor_correction; raw protocol messages remain unchanged."""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from maggrad_protocol.msg import RoundSample
from sensor_array_bringup.msg import SensorCorrectionRound, SensorCorrectionSample

from .sensor_correction import AffineSensorCorrection, DeviceIdentityGate


class SensorCorrectionNode:
    def __init__(self):
        self.device_id = rospy.get_param('~device_id')
        self.model = AffineSensorCorrection(
            rospy.get_param('~sensor_correction_dir'), self.device_id,
            rospy.get_param('~raw_scale', 1.0))
        self.gate = DeviceIdentityGate(rospy.get_param('~firmware_device_id'),
                                       rospy.get_param('~check_device_id', True))
        self.unknown = 0
        self.missing = 0
        self.received = False
        topic = rospy.get_param('~input_topic')
        self.publisher = rospy.Publisher(rospy.get_param('~output_topic', topic + '_corr'),
                                         SensorCorrectionRound, queue_size=5)
        self.diagnostics = rospy.Publisher(rospy.get_param('~diagnostics_topic', 'diagnostics'),
                                           DiagnosticArray, queue_size=5)
        self.subscriber = rospy.Subscriber(topic, RoundSample, self.on_round, queue_size=512)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.on_diagnostics)

    def on_round(self, msg):
        self.received = True
        if not self.gate.accept(msg.has_stats, msg.stats.device_id if msg.has_stats else None):
            if self.gate.mismatch:
                rospy.logerr_throttle(5.0, 'Device ID mismatch: expected %s, observed %s; '
                                     'sensor_correction blocked until restart',
                                     self.gate.expected, self.gate.observed)
            else:
                rospy.logwarn_throttle(5.0, 'Sensor correction waiting for STATS device ID')
            return
        result = SensorCorrectionRound()
        for field in ('header', 'round_seq', 'device_tick_ms', 'device_tick_unwrapped_ms',
                      'has_phase', 'phase_age_us', 'phase_valid', 'has_icm', 'icm',
                      'errors', 'has_stats', 'stats', 'complete', 'missing_sensor_mask'):
            setattr(result, field, getattr(msg, field))
        result.device_id = self.device_id
        result.identity_verified = self.gate.verified and not self.gate.mismatch
        result.has_device_id = self.gate.observed is not None
        if result.has_device_id:
            result.firmware_device_id = self.gate.observed
        for family in ('ak', 'tmag'):
            for sample in getattr(msg, family):
                if sample.sensor_id == 0xFF:
                    result.skipped_unknown_id += 1
                    continue
                raw = ((sample.hx, sample.hy, sample.hz) if family == 'ak'
                       else (sample.x, sample.y, sample.z))
                vector = self.model.apply(family, sample.sensor_id, raw)
                if vector is None:
                    result.skipped_missing_sensor_correction += 1
                    continue
                corrected = SensorCorrectionSample()
                corrected.sensor_id = sample.sensor_id
                corrected.read_done_offset_us = sample.read_done_offset_us
                corrected.drdy = sample.drdy
                corrected.x, corrected.y, corrected.z = map(float, vector)
                if family == 'ak':
                    corrected.channel, corrected.address = sample.bus, sample.mask
                    corrected.err, corrected.dor = sample.err, sample.dor
                else:
                    corrected.channel, corrected.address = sample.ch_mask, sample.addr
                    corrected.overflow = sample.overflow
                getattr(result, family).append(corrected)
        self.unknown = result.skipped_unknown_id
        self.missing = result.skipped_missing_sensor_correction
        if self.unknown or self.missing:
            rospy.logwarn_throttle(5.0, 'Sensor correction skipped sensors: unknown IDs=%s, '
                                  'missing coefficients=%s', self.unknown, self.missing)
        # Empty rounds retain timing and explicit skipped counts, never fake vectors.
        self.publisher.publish(result)

    def on_diagnostics(self, _event):
        status = DiagnosticStatus(name=rospy.get_name() + ': sensor_correction',
                                  hardware_id=self.device_id)
        if self.gate.enabled and self.gate.mismatch:
            status.level, status.message = DiagnosticStatus.ERROR, 'Device ID mismatch (latched)'
        elif self.gate.enabled and not self.gate.verified:
            status.level, status.message = DiagnosticStatus.WARN, 'Waiting for device ID'
        elif not self.received:
            status.level, status.message = DiagnosticStatus.WARN, 'No protocol samples'
        elif self.unknown or self.missing:
            status.level, status.message = DiagnosticStatus.WARN, 'Sensors skipped during sensor_correction'
        else:
            status.level, status.message = DiagnosticStatus.OK, 'Sensor correction active'
        status.values = [KeyValue(key=key, value=str(value)) for key, value in (
            ('check_device_id', self.gate.enabled), ('expected_device_id', self.gate.expected),
            ('observed_device_id', self.gate.observed), ('raw_scale', self.model.raw_scale),
            ('skipped_unknown_id', self.unknown), ('skipped_missing_sensor_correction', self.missing))]
        report = DiagnosticArray()
        report.header.stamp = rospy.Time.now()
        report.status = [status]
        self.diagnostics.publish(report)


def main():
    rospy.init_node('sensor_correction')
    try:
        node = SensorCorrectionNode()
    except (OSError, ValueError, KeyError, TypeError) as error:
        rospy.logfatal('Invalid sensor_correction configuration: %s', error)
        raise SystemExit(1)
    rospy.spin()
