"""Mock plant with a deliberately biased command frame and static field."""
import argparse
import json
import numpy as np
from .tool_geometry import xyz_rpy
from .configuration import load_config, load_points
from .engine import AlignmentEngine, Engine
from .geometry import inverse, rotation_from_matrix, rotation_matrix, transform
from scipy.spatial.transform import Rotation
from .recording import Recorder


class MockMotion:
    def __init__(self, config, transforms=None):
        self.config = config
        tcp = config['sensor_tool_config']['tcp']
        self.transforms = transforms if transforms is not None else {
            (config['planning_link'], config['robot_sensor_frame']): xyz_rpy(tcp['xyz'], tcp['rpy'])}
        self.geometry_mode = 'mock_tf'
        self.link_to_sensor = self.transforms[(config['planning_link'], config['robot_sensor_frame'])].copy()
        self.actual = np.eye(4)
        self.actual[:3, 3] = [.003, -.002, .1]
        self.actual[:3, :3] = np.diag([1., -1., -1.])
        self.bias = np.eye(4)
        self.bias[:3, 3] = [.001, -.001, .0005]
        self.commands = []
        self.stopped = False

    def validate_geometry(self, check):
        check()
        self.link_to_sensor = self.transforms[(self.config['planning_link'], self.config['robot_sensor_frame'])].copy()
        self.sensor_pose()

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
        # Synthetic reversible joint model for exercising saved-start restoration.
        positions = list(self.actual[:3, 3]) + list(rotation_from_matrix(self.actual[:3, :3]).as_rotvec()) + [0.] * 7
        names = ['diana7_joint_%d' % i for i in range(1, 8)] + [
            self.config['source_arm'] + '_joint_%d' % i for i in range(1, 7)]
        return dict(sensor_planning=self.sensor_pose(),
                    joint_state=dict(mock=True, name=names, position=positions))

    def restore_joints(self, targets, check):
        check()
        positions = [targets['diana7_joint_%d' % i] for i in range(1, 8)]
        self.actual[:3, 3] = positions[:3]
        self.actual[:3, :3] = rotation_matrix(Rotation.from_rotvec(positions[3:6]))

    def stop(self):
        self.stopped = True


class MockVision:
    def __init__(self, motion, transforms=None):
        self.motion = motion
        self.transforms = transforms if transforms is not None else {
            (p['parent'], p['child']): np.array(p['matrix'])
            for p in motion.config['visual_profiles'].values()}
        self.source = np.eye(4)
        self.stamp = 0.

    def validate_geometry(self, check):
        from .engine import ScanError
        check()
        for p in self.motion.config['visual_profiles'].values():
            actual = self.transforms.get((p['parent'], p['child']))
            if actual is None or not np.allclose(actual, p['matrix'], atol=1e-9, rtol=0):
                raise ScanError('mock visual TF missing or profile mismatch')

    def observe(self, check):
        self.validate_geometry(check)
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
    parser.add_argument('--align', action='store_true', help='determine and save the mock initial pose')
    args = parser.parse_args()
    c = load_config(args.config, args.source_arm, alignment=args.align)
    points = {'points': []} if args.align else load_points(c['points_file'])
    if args.check:
        print('Valid configuration and %d points (no hardware started)' % len(points['points']))
        return
    r = Recorder(c, points)
    m = MockMotion(c)
    if args.align:
        state = AlignmentEngine(c, m, MockVision(m), r).run()
    else:
        state = Engine(c, points, m, MockVision(m), MockSampler(m, c), r).run()
    print(json.dumps(dict(state=state, output_dir=str(r.path))))
    if state != 'COMPLETE':
        raise SystemExit(1)
