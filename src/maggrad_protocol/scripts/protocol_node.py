#!/usr/bin/env python3
"""ROS Noetic adapter for the ROS-independent MagGrad ROUND decoder."""

import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from serial_frame_bridge.msg import RawFrame
from maggrad_protocol.decoder import Decoder, FrameError, INVALID_PHASE
from maggrad_protocol.msg import (AkSample, ErrorRecord, IcmSample, RoundSample,
                                  StatsRecord, TmagSample)


def to_message(frame, round_data):
    msg = RoundSample()
    msg.header = frame.header  # Bridge receipt time; not device time.
    msg.round_seq = round_data.round_seq
    msg.device_tick_us = round_data.device_tick_us
    msg.device_tick_unwrapped_us = round_data.device_tick_unwrapped_us
    if round_data.phase is not None:
        msg.has_phase = True
        msg.phase_age_us = round_data.phase
        msg.phase_valid = [age != INVALID_PHASE for age in round_data.phase]
    else:
        msg.phase_age_us = [INVALID_PHASE] * 3
        msg.phase_valid = [False] * 3
    # TODO: Confirm address-pair -> logical ID mappings; match the C++ placeholder.
    msg.ak = [AkSample(*values, 0xFF) for values in round_data.ak]
    msg.tmag = [TmagSample(*values, 0xFF) for values in round_data.tmag]
    if round_data.icm is not None:
        msg.has_icm = True
        msg.icm = IcmSample(*round_data.icm)
    msg.errors = [ErrorRecord(*values) for values in round_data.errors]
    if round_data.stats is not None:
        msg.has_stats = True
        msg.stats = StatsRecord(*round_data.stats)
    msg.complete = round_data.complete
    msg.missing_sensor_mask = round_data.missing_sensor_mask
    return msg


class ProtocolNode:
    def __init__(self):
        self.decoder = Decoder(rospy.get_param('~expected_ak', []),
                               rospy.get_param('~expected_tmag', []),
                               rospy.get_param('~expect_icm', False))
        self.configured = bool(self.decoder.expected_ak or self.decoder.expected_tmag
                               or self.decoder.expect_icm)
        self.publisher = rospy.Publisher('~sample', RoundSample, queue_size=5)
        self.diagnostics = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=5)
        self.last_received = None
        self.last_report = time.monotonic()
        self.last_count = 0
        self.last_bad = 0
        self.latest_stats = None
        self.subscriber = rospy.Subscriber(
            rospy.get_param('~raw_frame_topic', '/serial_frame_bridge/raw_frame'),
            RawFrame, self.on_frame, queue_size=512)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.on_diagnostics)

    def on_frame(self, frame):
        try:
            decoded = self.decoder.decode(bytes(frame.data))
        except FrameError as exc:
            rospy.logwarn_throttle(5.0, 'Discarding malformed MagGrad ROUND: %s', str(exc))
            return
        self.last_received = time.monotonic()
        if decoded.stats is not None:
            self.latest_stats = decoded.stats
        self.publisher.publish(to_message(frame, decoded))

    def on_diagnostics(self, _event):
        now = time.monotonic()
        counters = self.decoder.counters
        interval = max(now - self.last_report, 0.001)
        rate = (counters.frames - self.last_count) / interval
        age = float('inf') if self.last_received is None else now - self.last_received
        status = DiagnosticStatus()
        status.name = rospy.get_name() + ': protocol'
        status.hardware_id = 'maggrad_v2'
        bad = counters.malformed - self.last_bad
        if age > 1.0:
            status.level, status.message = DiagnosticStatus.ERROR, 'No recent valid rounds'
        elif age > 0.1 or bad:
            status.level, status.message = DiagnosticStatus.WARN, 'Stale rounds or malformed frames'
        elif not self.configured:
            status.level, status.message = DiagnosticStatus.WARN, 'Expected sensors not configured'
        else:
            status.level, status.message = DiagnosticStatus.OK, 'Receiving rounds'
        fields = vars(counters).copy()
        fields.update(rate_hz='%.2f' % rate, last_round_age_s='%.3f' % age,
                      expected_ak=len(self.decoder.expected_ak),
                      expected_tmag=len(self.decoder.expected_tmag),
                      expect_icm=self.decoder.expect_icm)
        if self.latest_stats is not None:
            fields['firmware_skipped'] = self.latest_stats[3]
            fields['firmware_errors'] = self.latest_stats[4]
        status.values = [KeyValue(str(key), str(value)) for key, value in fields.items()]
        report = DiagnosticArray()
        report.header.stamp = rospy.Time.now()
        report.status = [status]
        self.diagnostics.publish(report)
        self.last_report, self.last_count, self.last_bad = now, counters.frames, counters.malformed


if __name__ == '__main__':
    rospy.init_node('maggrad_protocol')
    try:
        ProtocolNode()
    except ValueError as exc:
        rospy.logfatal('Invalid expected sensor configuration: %s', exc)
        raise
    rospy.spin()
