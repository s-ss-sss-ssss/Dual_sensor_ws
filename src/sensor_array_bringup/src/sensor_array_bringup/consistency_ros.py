"""ROS1 adapters for AK raw rounds and the existing FY8300 topic API."""
from collections import deque
import math
import threading
import time

import numpy as np


class DeviceClock:
    """Unit-slope arrival-envelope mapping; explicit uncertainty, never a sync claim.

    Anchor uses the latest sensor-read timestamp in each round. Fixed after lock
    so a delayed frame cannot shift already assigned windows. Transport changes
    and oscillator drift beyond configured guards invalidate the run.
    """
    def __init__(self, lock_s, guard_s, residual_limit_s):
        self.lock_s, self.guard, self.limit = lock_s, guard_s, residual_limit_s
        self.anchors = []
        self.offset = None
        self.previous = None

    def observe(self, device_t, host_t, seq):
        if not math.isfinite(device_t) or not math.isfinite(host_t):
            raise RuntimeError('nonfinite device/host timestamp')
        if self.previous is not None:
            old_t, old_seq = self.previous
            step = (seq - old_seq) & 0xffffffff
            if device_t <= old_t or step == 0 or step >= 0x80000000:
                raise RuntimeError('duplicate/out-of-order round or device restart; restart calibration')
        self.previous = (device_t, seq)
        anchor = host_t - device_t
        if self.offset is None:
            self.anchors.append((host_t, anchor))
            if len(self.anchors) >= 5 and host_t - self.anchors[0][0] >= self.lock_s:
                offsets = [a[1] for a in self.anchors]
                if max(offsets) - min(offsets) > self.limit:
                    raise RuntimeError('transport jitter exceeds clock residual limit')
                self.offset = min(offsets)
        else:
            residual = anchor - self.offset
            if residual < -self.guard or residual > self.limit:
                raise RuntimeError('clock mapping/transport residual out of bounds: %.6fs' % residual)
        return None if self.offset is None else device_t + self.offset


class RosBackend:
    def __init__(self, config, store):
        import rospy
        from maggrad_protocol.msg import RoundSample
        from signal_generator.msg import ChannelStatus
        from std_msgs.msg import Bool, Float32, UInt8
        from .sensor_correction import DeviceIdentityGate
        self.rospy, self.config, self.store = rospy, config, store
        self.types = dict(output_en=Bool, offset=Float32, amplitude=Float32, waveform=UInt8, sync_output=Bool)
        prefix = config.get('generator_namespace', '/fy8300').rstrip('/')
        self.publishers = {(ch, field): rospy.Publisher('%s/ch%s/%s' % (prefix, ch, field), self.types[field], queue_size=1)
                           for ch in (1, 2, 3) for field in ('output_en', 'offset', 'amplitude', 'waveform')}
        self.sync = rospy.Publisher(prefix + '/sync_output', Bool, queue_size=1)
        self.lock = threading.RLock()
        self.status = {ch: deque(maxlen=4) for ch in (1, 2, 3)}
        self.buffer = deque()
        self.clock = DeviceClock(config['clock_lock_s'], config['clock_guard_s'], config['clock_residual_limit_s'])
        self.gate = DeviceIdentityGate(config['firmware_device_id'], True)
        self.last_round = None
        self.fatal = None
        self.cancelled = False
        self.closed = False
        self.expected = {s['sensor_id']: s for s in config['sensors']}
        self.subscribers = [rospy.Subscriber('%s/ch%s/status' % (prefix, ch), ChannelStatus,
                                            self.on_status, callback_args=ch, queue_size=10) for ch in (1, 2, 3)]
        topic = '/%s/maggrad/sample' % config['device_id']
        self.subscribers.append(rospy.Subscriber(topic, RoundSample, self.on_round, queue_size=512))

    @staticmethod
    def now():
        return time.monotonic()

    def check(self):
        if self.cancelled:
            raise KeyboardInterrupt('manual stop')
        if self.rospy.is_shutdown():
            raise RuntimeError('ROS shut down before outputs could be confirmed off')
        if self.fatal:
            raise RuntimeError(self.fatal)
        if self.last_round is not None and self.now() - self.last_round > self.config['stream_timeout_s']:
            raise RuntimeError('sensor stream timeout')

    def sleep(self, duration):
        deadline = self.now() + duration
        while self.now() < deadline:
            self.check()
            time.sleep(min(.02, max(0., deadline - self.now())))
        self.check()

    def on_status(self, msg, ch):
        with self.lock:
            self.status[ch].append((self.now(), msg))

    def on_round(self, msg):
        with self.lock:
            if self.closed:
                return
            try:
                self._on_round(msg)
            except Exception as error:
                self.fatal = '%s: %s' % (type(error).__name__, error)
                self.store.event('SENSOR_ADAPTER_FAILED', self.fatal)

    def _on_round(self, msg):
        received = self.now()
        self.last_round = received
        if not self.gate.accept(msg.has_stats, msg.stats.device_id if msg.has_stats else None):
            if self.gate.mismatch:
                raise RuntimeError('STATS hardware ID mismatch (latched)')
            return
        if not msg.ak:
            return
        device_times = [(msg.device_tick_unwrapped_us + s.read_done_offset_us) * 1e-6 for s in msg.ak]
        # header is host ROS receipt time, not device time. Convert its age to monotonic.
        age = self.rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        if not math.isfinite(age) or age < -.01 or age > self.config['stream_timeout_s']:
            raise RuntimeError('invalid/stale host receipt header')
        host_receipt = received - age
        with self.lock:
            self.clock.observe(max(device_times), host_receipt, msg.round_seq)
            offset = self.clock.offset
            seen, archive = set(), []
            for sample, device_t in zip(msg.ak, device_times):
                sid = sample.sensor_id
                expected = self.expected.get(sid)
                valid_address = expected is not None and sample.bus == expected['bus'] and sample.mask == expected['mask']
                reason = ''
                if not valid_address or sid in seen:
                    reason = 'unknown/duplicate sensor or address mismatch'
                seen.add(sid)
                raw = [sample.hx, sample.hy, sample.hz]
                mapped = None if offset is None else device_t + offset
                vector = (np.asarray(expected['Q']) @ (np.asarray(raw) * 1.1)).tolist() if expected else None
                if any(v <= -32768 or v >= 32767 for v in raw):
                    reason = 'raw sensor clipping'
                record = dict(sensor_id=sid, bus=sample.bus, mask=sample.mask,
                              round_seq=msg.round_seq, sample_seq=None, device_tick_us=msg.device_tick_us,
                              device_tick_unwrapped_us=msg.device_tick_unwrapped_us,
                              read_done_offset_us=sample.read_done_offset_us,
                              t_device=device_t, t_host=host_receipt, t_arrival=received, t=mapped,
                              raw=raw, x=vector, drdy=int(sample.drdy), err=int(sample.err), dor=int(sample.dor),
                              comm_ok=True, valid=not reason and mapped is not None, reason=reason)
                archive.append(record)
                if mapped is not None:
                    self.buffer.append(record)
                if not valid_address or reason.startswith('unknown'):
                    self.fatal = reason
            self.store.write('raw_stream.csv', archive)
            if msg.errors:
                self.store.event('DEVICE_ERRORS', [dict(source=e.source, code=e.code, detail=e.detail) for e in msg.errors])
            # Keep all potentially active capture samples; raw_stream retains everything.
            keep_since = received - self.config['max_capture_s'] - self.config['stream_timeout_s'] - 5
            while self.buffer and self.buffer[0]['t'] < keep_since:
                self.buffer.popleft()

    def read_samples(self, start, end):
        self.check()
        with self.lock:
            return [s.copy() for s in self.buffer if start <= s['t'] <= end]

    def wait(self, predicate, timeout, description, shutdown=False):
        deadline = self.now() + timeout
        while self.now() < deadline:
            if not shutdown:
                self.check()
            if predicate():
                return
            time.sleep(.02)
        raise RuntimeError('timeout: ' + description)

    def prepare(self):
        if self.rospy.get_param('/use_sim_time', False):
            raise RuntimeError('hardware calibration requires wall ROS time (/use_sim_time=false)')
        self.wait(lambda: all(p.get_num_connections() > 0 for p in list(self.publishers.values()) + [self.sync]),
                  self.config['connection_timeout_s'], 'FY8300 topic subscribers')
        # FY8300 advertises its topics before init() finishes writing initial
        # settings. Status polling starts only afterwards; connections alone
        # must not start the shorter per-command acknowledgement timeout.
        ready_after = self.now()
        self.store.event('WAIT_GENERATOR_READY', {})
        self.wait(lambda: self.status_ready(ready_after), self.config['connection_timeout_s'],
                  'FY8300 initialization and two valid status polls per channel')
        self.store.event('GENERATOR_READY', {})
        # Hardware readback is used as acknowledgement; publish alone is never success.
        self.all_off()
        self.wait(lambda: self.clock.offset is not None and self.gate.verified,
                  self.config['connection_timeout_s'], 'hardware ID and device clock mapping')
        self.store.event('CLOCK_LOCKED', dict(offset_s=self.clock.offset,
                                             anchors=self.clock.anchors, method='minimum arrival offset, unit slope'))

    def status_ready(self, after):
        with self.lock:
            for ch in (1, 2, 3):
                fresh = [msg for t, msg in self.status[ch] if t > after]
                if len(fresh) < 2 or not all(msg.valid for msg in fresh[-2:]):
                    return False
            return True

    def status_matches(self, ch, field, expected, after):
        with self.lock:
            # Two fresh matching polls protect against an already in-flight status read.
            fresh = [(t, m) for t, m in self.status[ch] if t > after]
            if len(fresh) < 2:
                return False
            for _, msg in fresh[-2:]:
                if not msg.valid:
                    return False
                actual = getattr(msg, 'output_enabled' if field == 'output_en' else field)
                if field in ('offset', 'amplitude'):
                    if not math.isfinite(actual) or abs(actual - expected) > self.config['output_tolerance_v']:
                        return False
                elif actual != expected:
                    return False
            return True

    def command(self, ch, field, value, shutdown=False):
        if not shutdown:
            self.check()
        publisher = self.publishers[(ch, field)]
        if publisher.get_num_connections() == 0:
            raise RuntimeError('FY8300 disconnected: ch%s/%s' % (ch, field))
        sent = self.now()
        self.store.event('COMMAND', dict(channel=ch, field=field, value=value), sent)
        publisher.publish(self.types[field](data=value))
        # Node returns device-native waveform number: DC=6 on CH1, DC=5 on CH2/3.
        expected = value - 1 if field == 'waveform' and ch != 1 and value >= 6 else value
        self.wait(lambda: self.status_matches(ch, field, expected, sent), self.config['command_timeout_s'],
                  'ch%s/%s readback=%s' % (ch, field, expected), shutdown=shutdown)
        self.store.event('READBACK_CONFIRMED', dict(channel=ch, field=field, value=expected))

    def all_off(self):
        # Cleanup ignores cancellation, stream errors and identity mismatch.
        if self.sync.get_num_connections() == 0:
            raise RuntimeError('FY8300 sync subscriber unavailable; cannot confirm shutdown')
        self.sync.publish(self.types['sync_output'](data=False))
        time.sleep(.6)  # Driver serializes each command with a 0.5s delay.
        errors = []
        for ch in (1, 2, 3):
            try:
                self.command(ch, 'output_en', False, shutdown=True)
            except Exception as error:
                errors.append(str(error))
        if errors:
            self.rospy.logerr('ALL_OFF FAILED: %s', '; '.join(errors))
            raise RuntimeError('; '.join(errors))
        self.store.event('ALL_OFF_CONFIRMED', {})

    def set_output(self, commands):
        self.all_off()
        for axis, value in commands.items():
            entry = self.config['channels'][axis]
            if abs(value) > entry['max_abs_offset_v']:
                raise RuntimeError('voltage exceeds configured limit')
            ch = entry['channel']
            self.command(ch, 'waveform', 6)
            self.command(ch, 'amplitude', 0.)
            self.command(ch, 'offset', float(value))
        for axis in commands:
            self.command(self.config['channels'][axis]['channel'], 'output_en', True)
        # Detect persisted hardware sync modes that inadvertently enable other axes.
        desired = {entry['channel']: axis in commands for axis, entry in self.config['channels'].items()}
        for ch, enabled in desired.items():
            if not self.status_matches(ch, 'output_en', enabled, self.now() - self.config['command_timeout_s']):
                raise RuntimeError('unexpected FY8300 output state; check persisted sync modes')


def main(argv=None):
    import argparse
    import signal
    from .consistency import load_config, make_plan
    from .consistency_collect import Collector, RunStore, refit
    parser = argparse.ArgumentParser(description='AK09973D relative consistency calibration; results only')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--config', help='experiment YAML')
    group.add_argument('--refit', help='completed run directory, recompute from samples.csv')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true', help='validate without ROS, hardware, or file writes')
    mode.add_argument('--dry-run', action='store_true', help='print complete state schedule without ROS or writes')
    mode.add_argument('--smoke-test', action='store_true', help='collect only X lowest-level +/- trials; no fitted artifact')
    # roslaunch adds __name/__log; parse only those explicitly permitted remappings.
    args, extra = parser.parse_known_args(argv)
    if any(not a.startswith(('__name:=', '__log:=', '__ns:=')) for a in extra):
        parser.error('unrecognized arguments: ' + ' '.join(extra))
    if args.refit:
        if args.check or args.dry_run or args.smoke_test:
            parser.error('--check/--dry-run/--smoke-test require --config')
        print(refit(args.refit))
        return
    config = load_config(args.config)
    plan = make_plan(config)
    print('Configuration valid: %s training + %s validation trials.' % (
        sum(t['split'] == 'train' for t in plan), sum(t['split'] == 'validation' for t in plan)))
    if args.check:
        return
    if args.dry_run:
        print('STARTUP: all_off -> settle -> %.3fs check -> freeze reference (no warmup).' % config['startup_s'])
        for trial in plan:
            print('%s: %s %s; OFF_PRE_SETTLE -> OFF_PRE_CAPTURE -> ON_SETTLE -> ON_CAPTURE -> '
                  'OFF_POST_SETTLE -> OFF_POST_CAPTURE -> PROCESS -> ACCEPT/RETRY' %
                  (trial['trial_id'], trial['split'], trial['commands_v']))
        print('FIT -> independent validation -> short-window report -> all_off -> result files.')
        return
    import rospy
    rospy.init_node('consistency_calibration', disable_signals=True)
    store = RunStore(config)
    backend = None
    old_handlers = {}
    try:
        backend = RosBackend(config, store)
        def stop(_signum, _frame):
            backend.cancelled = True
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, stop)
        print('Run directory: %s' % store.path, flush=True)
        Collector(config, backend, store).run(smoke_test=args.smoke_test)
        print('Calibration results: %s' % store.path)
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        # Stop callbacks before closing shared log streams.
        if backend is not None:
            for subscriber in backend.subscribers:
                subscriber.unregister()
            with backend.lock:
                backend.closed = True
                store.close()
        else:
            store.close()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        rospy.signal_shutdown('calibration finished')


if __name__ == '__main__':
    main()
