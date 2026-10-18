"""Mock plant with a deliberately biased command frame and static field."""
import argparse
import json
import numpy as np
from .configuration import load_config, load_points
from .engine import Engine
from .geometry import inverse, transform
from .recording import Recorder


class MockMotion:
    def __init__(self, config):
        self.config = config
        self.link_to_sensor = transform(config['planning_link_to_sensor'])
        self.actual = np.eye(4)
        self.actual[:3, 3] = [.003, -.002, .1]
        self.actual[:3, :3] = np.diag([1., -1., -1.])
        self.bias = np.eye(4)
        self.bias[:3, 3] = [.001, -.001, .0005]
        self.commands = []
        self.stopped = False

    def sensor_pose(self):
        return self.bias @ self.actual

    def move_to(self, command, check):
        check()
        self.commands.append(command.copy())
        self.actual = inverse(self.bias) @ command @ self.link_to_sensor

    def wait_still(self, check):
        check()

    def wait(self, seconds, check):
        check()

    def snapshot(self):
        return dict(sensor_planning=self.sensor_pose(), joint_state={'mock': True})

    def stop(self):
        self.stopped = True


class MockVision:
    def __init__(self, motion):
        self.motion = motion
        self.source = np.eye(4)
        self.stamp = 0.

    def observe(self, check):
        check()
        self.stamp += 1.
        return dict(stamp=self.stamp, sensor=self.motion.actual.copy(), source=self.source.copy(),
                    observation_count=10, mock=True)


class MockSampler:
    def __init__(self, motion, config):
        self.motion, self.config = motion, config

    def collect(self, seconds, check):
        check()
        b = (np.array([10., 20., 30.]) + np.diag([2., 3., -5.]) @ self.motion.actual[:3, 3]).tolist()
        n = self.config['parameters']['min_rounds']
        return dict(mock=True, valid_rounds=n, duration_s=seconds, sensors={
            '%s:%s' % (f, sid): dict(mean=b, std=[0., 0., 0.], count=n)
            for f, ids in self.config['sampling']['expected'].items() for sid in ids})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--source-arm', choices=['arm1', 'arm2'])
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    c = load_config(args.config, args.source_arm)
    points = load_points(c['points_file'])
    if args.check:
        print('Valid configuration and %d points (no hardware started)' % len(points['points']))
        return
    r = Recorder(c, points)
    m = MockMotion(c)
    state = Engine(c, points, m, MockVision(m), MockSampler(m, c), r).run()
    print(json.dumps(dict(state=state, output_dir=str(r.path))))
    if state != 'COMPLETE':
        raise SystemExit(1)
