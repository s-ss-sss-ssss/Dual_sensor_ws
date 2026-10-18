"""Thread-safe rosbag capture and stationary sampling."""
import threading
import time
from collections import Counter, deque
import rosbag
import rospy
from genpy import Message, Time, Duration
from sensor_array_bringup.msg import SensorCorrectionRound
from maggrad_protocol.msg import RoundSample
from serial_frame_bridge.msg import RawFrame
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from .engine import ScanError
from .sampling import validate_round, statistics


def message_dict(value):
    if isinstance(value, (Time, Duration)):
        return dict(secs=value.secs, nsecs=value.nsecs)
    if isinstance(value, Message):
        return {slot: message_dict(getattr(value, slot)) for slot in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [message_dict(v) for v in value]
    return value


class BagCapture:
    def __init__(self, config, path):
        self.lock = threading.Lock()
        self.error = None
        self.closed = False
        self.bag = rosbag.Bag(str(path / 'capture.bag'), 'w')
        topics = {'/tf': TFMessage, '/tf_static': TFMessage, '/joint_states': JointState,
                  config['sampling']['raw_topic']: RawFrame,
                  config['sampling']['corrected_topic']: SensorCorrectionRound}
        protocol_topic = config['sampling'].get('protocol_topic')
        if protocol_topic:
            topics[protocol_topic] = RoundSample
        self.subs = [rospy.Subscriber(topic, cls, self.callback, callback_args=topic, queue_size=1000)
                     for topic, cls in topics.items()]

    def callback(self, msg, topic):
        with self.lock:
            if self.closed or self.error:
                return
            try:
                self.bag.write(topic, msg, rospy.Time.now())
            except Exception as error:
                self.error = str(error)

    def health(self):
        if self.error:
            raise ScanError('rosbag recording failed: ' + self.error)

    def close(self):
        for sub in self.subs:
            sub.unregister()
        with self.lock:
            self.closed = True
            self.bag.close()


class RosSampler:
    def __init__(self, config, joints):
        self.c, self.p, self.joints = config, config['parameters'], joints
        self.lock = threading.Lock()
        self.queue = deque()
        self.active = False
        self.overflow = False
        self.sub = rospy.Subscriber(config['sampling']['corrected_topic'], SensorCorrectionRound,
                                    self.callback, queue_size=1000)

    def callback(self, msg):
        with self.lock:
            if self.active:
                if len(self.queue) >= 10000:
                    self.overflow = True
                else:
                    self.queue.append((time.monotonic(), msg))

    def collect(self, seconds, check):
        start = time.monotonic()
        if seconds > self.p['sample_timeout_s']:
            raise ScanError('point sample_s exceeds sample_timeout_s')
        records, vectors, seen = [], [], set()
        rejected = Counter()
        with self.lock:
            self.queue.clear()
            self.overflow = False
            self.active = True
        try:
            while time.monotonic() - start < self.p['sample_timeout_s']:
                check()
                self.joints.assert_still()
                with self.lock:
                    batch = list(self.queue)
                    self.queue.clear()
                    overflow = self.overflow
                if overflow:
                    raise ScanError('stationary sample queue overflow')
                for received, msg in batch:
                    age = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
                    if not 0 <= age <= self.p['joint_max_age_s']:
                        rejected['stale_round'] += 1
                        continue
                    key = (msg.device_tick_unwrapped_us, msg.round_seq)
                    if key in seen:
                        rejected['duplicate_round'] += 1
                        continue
                    seen.add(key)
                    record = message_dict(msg)
                    values, reason = validate_round(record, self.c['sampling']['expected'],
                                                    self.c['sampling'].get('require_identity_verified', True))
                    if values is None:
                        rejected[reason] += 1
                        continue
                    records.append(record)
                    vectors.append(values)
                elapsed = time.monotonic() - start
                if elapsed >= seconds and len(vectors) >= self.p['min_rounds']:
                    return dict(valid_rounds=len(vectors), duration_s=elapsed,
                                rejected=dict(rejected), rounds=records, sensors=statistics(vectors))
                time.sleep(.01)
            raise ScanError('insufficient valid rounds: %d; rejected=%s' % (len(vectors), dict(rejected)))
        finally:
            with self.lock:
                self.active = False

    def close(self):
        self.sub.unregister()
