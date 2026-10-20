"""ROS adapters. No hardware is contacted until instantiated by scan_node."""
import threading
import time
from collections import deque
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from .tool_geometry import fixed_chain
from .engine import ScanError
from .geometry import average, inverse, residual, transform, rotation_from_matrix


def pose_matrix(pose):
    return transform(dict(translation_m=[pose.position.x, pose.position.y, pose.position.z],
                          quaternion_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))


def tf_matrix(msg):
    t = msg.transform
    return transform(dict(translation_m=[t.translation.x, t.translation.y, t.translation.z],
                          quaternion_xyzw=[t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))


def sleep_checked(seconds, check):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        check()
        time.sleep(min(.02, max(0., end - time.monotonic())))
    check()


def require_match(actual, expected, label):
    position, angle = residual(actual, expected)
    if position > 1e-7 or angle > 1e-6:
        raise ScanError(label + ' disagrees with configured geometry/profile')


def wait_for_geometry(validate, seconds, check):
    end = time.monotonic() + seconds
    while True:
        check()
        try:
            return validate()
        except tf2_ros.TransformException as error:
            if time.monotonic() >= end:
                raise ScanError('geometry TF unavailable; timed out: ' + str(error))
            time.sleep(.02)


class JointMonitor:
    def __init__(self, config):
        self.p = config['parameters']
        self.names = ['diana7_joint_%d' % i for i in range(1, 8)] + [
            config['source_arm'] + '_joint_%d' % i for i in range(1, 7)]
        if config['hardware'].get('connect_inactive_arm', False):
            inactive = 'arm2' if config['source_arm'] == 'arm1' else 'arm1'
            self.names += [inactive + '_joint_%d' % i for i in range(1, 7)]
        self.lock = threading.Lock()
        self.latest = None
        self.previous = None
        self.history = deque(maxlen=1000)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=100)

    def callback(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        values = dict(zip(msg.name, msg.position))
        if not set(self.names) <= set(values) or not all(np.isfinite(values[n]) for n in self.names):
            return
        if msg.velocity and (len(msg.velocity) != len(msg.name) or not np.isfinite(msg.velocity).all()):
            return
        stamp = msg.header.stamp.to_sec()
        now = time.monotonic()
        vector = np.array([values[n] for n in self.names])
        with self.lock:
            if self.previous is not None and stamp <= self.previous[0]:
                return
            speed = float('inf')
            if self.previous is not None:
                speed = float(np.max(np.abs(vector - self.previous[1])) / (stamp - self.previous[0]))
            if len(msg.velocity) == len(msg.name):
                velocities = dict(zip(msg.name, msg.velocity))
                speed = max(speed, max(abs(velocities[n]) for n in self.names))
            self.previous = (stamp, vector)
            self.latest = dict(stamp=stamp, received=now, name=msg.name, position=msg.position,
                               velocity=msg.velocity, speed=speed)
            self.history.append((now, speed))

    def snapshot(self):
        with self.lock:
            item = dict(self.latest) if self.latest else None
        if item is None or time.monotonic() - item['received'] > self.p['joint_max_age_s'] or not (
                0 <= rospy.Time.now().to_sec() - item['stamp'] <= self.p['joint_max_age_s']):
            raise ScanError('missing/stale active joint states')
        item.pop('received')
        item.pop('speed')
        return item

    def wait_still(self, check):
        end = time.monotonic() + self.p['motion_timeout_s']
        while time.monotonic() < end:
            check()
            try:
                self.snapshot()
                now = time.monotonic()
                with self.lock:
                    history = list(self.history)
                window = self.p['stationary_window_s']
                recent = [(t, v) for t, v in history if t >= now - window]
                if recent and history[0][0] <= now - window and all(
                        v <= self.p['joint_velocity_rad_s'] for _, v in recent):
                    return
            except ScanError:
                pass
            time.sleep(.02)
        raise ScanError('joint states did not become fresh and stationary')

    def assert_still(self):
        self.snapshot()
        with self.lock:
            speed = self.latest['speed']
        if speed > self.p['joint_velocity_rad_s']:
            raise ScanError('active arm moved during stationary observation/sampling')

    def close(self):
        self.sub.unregister()


class RosMotion:
    def __init__(self, config, joints):
        import actionlib
        import moveit_commander
        from moveit_msgs.msg import ExecuteTrajectoryAction
        self.c, self.p, self.joints = config, config['parameters'], joints
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('diana7')
        self.source_group = moveit_commander.MoveGroupCommander(config['source_arm'])
        self.group.set_pose_reference_frame(config['planning_frame'])
        self.group.set_end_effector_link(config['planning_link'])
        self.client = actionlib.SimpleActionClient('/execute_trajectory', ExecuteTrajectoryAction)
        self.command_lock = threading.Lock()
        self.aborted = threading.Event()
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.geometry_mode = 'robot_tf'

    def validate_geometry(self, check):
        try:
            self.link_to_sensor = fixed_chain(rospy.get_param('/robot_description'),
                                             self.c['planning_link'], self.c['robot_sensor_frame'])
        except (ValueError, KeyError) as error:
            raise ScanError(str(error))
        def validate():
            msg = self.buffer.lookup_transform(self.c['planning_link'], self.c['robot_sensor_frame'], rospy.Time(0))
            require_match(tf_matrix(msg), self.link_to_sensor, 'robot fixed TCP TF')
            self.sensor_pose()
        wait_for_geometry(validate, self.p['vision_timeout_s'], check)

    def sensor_pose(self):
        msg = self.buffer.lookup_transform(self.c['planning_frame'], self.c['robot_sensor_frame'], rospy.Time(0))
        if not 0 <= rospy.Time.now().to_sec() - msg.header.stamp.to_sec() <= self.p['joint_max_age_s']:
            raise ScanError('missing/stale robot TCP TF')
        return tf_matrix(msg)

    def move_to(self, target, check):
        check()
        self.joints.snapshot()
        current = pose_matrix(self.group.get_current_pose(self.c['planning_link']).pose)
        position_error, angle_error = residual(current, target)
        if position_error <= 1e-7 and angle_error <= 1e-6:
            return  # an origin/duplicate point needs sampling, not a zero-duration trajectory
        self.group.set_start_state_to_current_state()
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target[:3, 3]
        q = rotation_from_matrix(target[:3, :3]).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        # Keyword avoids the jump_threshold/avoid_collisions positional ambiguity across versions.
        plan, fraction = self.group.compute_cartesian_path(
            [pose], self.p['cartesian_step_m'], avoid_collisions=True)
        check()
        if fraction < 1. - 1e-9 or not plan.joint_trajectory.points:
            raise ScanError('incomplete Cartesian trajectory: %.6f' % fraction)
        plan = self.group.retime_trajectory(self.robot.get_current_state(), plan,
                                           velocity_scaling_factor=self.p['velocity_scaling'],
                                           acceleration_scaling_factor=self.p['acceleration_scaling'])
        if not plan.joint_trajectory.points or plan.joint_trajectory.points[-1].time_from_start.to_sec() <= 0:
            raise ScanError('trajectory retiming failed')
        self.execute(plan, check)

    def restore_joints(self, targets, check):
        for group, prefix, count in ((self.source_group, self.c['source_arm'], 6),
                                     (self.group, 'diana7', 7)):
            check()
            self.joints.wait_still(check)
            desired = {prefix + '_joint_%d' % i: targets[prefix + '_joint_%d' % i]
                       for i in range(1, count + 1)}
            snapshot = self.joints.snapshot()
            actual = dict(zip(snapshot['name'], snapshot['position']))
            if any(abs(actual[n] - value) > self.p['initial_joint_tolerance_rad']
                   for n, value in desired.items()):
                group.set_start_state_to_current_state()
                group.set_max_velocity_scaling_factor(self.p['velocity_scaling'])
                group.set_max_acceleration_scaling_factor(self.p['acceleration_scaling'])
                group.set_planning_time(self.p['motion_timeout_s'])
                group.set_goal_joint_tolerance(self.p['initial_joint_tolerance_rad'])
                group.set_joint_value_target(desired)
                planned = group.plan()
                # Noetic returns (success, trajectory, planning_time, error_code).
                if isinstance(planned, tuple):
                    success, plan = planned[:2]
                    if not success:
                        raise ScanError('joint restoration planning failed: ' + prefix)
                else:
                    plan = planned
                check()
                if (not plan.joint_trajectory.points or
                        plan.joint_trajectory.points[-1].time_from_start.to_sec() <= 0):
                    raise ScanError('empty/invalid joint restoration plan: ' + prefix)
                self.execute(plan, check)
                self.joints.wait_still(check)
            snapshot = self.joints.snapshot()
            actual = dict(zip(snapshot['name'], snapshot['position']))
            if any(abs(actual[n] - value) > self.p['initial_joint_tolerance_rad']
                   for n, value in desired.items()):
                raise ScanError('restored joint angles outside tolerance: ' + prefix)
        # Check both arms together after all movement, including the held source.
        snapshot = self.joints.snapshot()
        actual = dict(zip(snapshot['name'], snapshot['position']))
        if any(abs(actual[n] - value) > self.p['initial_joint_tolerance_rad'] for n, value in targets.items()):
            raise ScanError('active joints changed during restoration')

    def execute(self, plan, check):
        from moveit_msgs.msg import ExecuteTrajectoryGoal, MoveItErrorCodes
        if not self.client.wait_for_server(rospy.Duration(2.)):
            raise ScanError('execute_trajectory action unavailable')
        check()
        with self.command_lock:
            check()
            if self.aborted.is_set():
                raise ScanError('motion adapter was stopped')
            self.client.send_goal(ExecuteTrajectoryGoal(trajectory=plan))
        end = time.monotonic() + self.p['motion_timeout_s']
        try:
            while not self.client.wait_for_result(rospy.Duration(.05)):
                check()
                self.joints.snapshot()
                if time.monotonic() > end:
                    raise ScanError('trajectory execution timed out')
            check()
            result = self.client.get_result()
            if result is None or result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise ScanError('trajectory execution failed')
        except Exception:
            self.stop()
            raise

    def wait_still(self, check):
        self.joints.wait_still(check)

    def wait(self, seconds, check):
        def still_check():
            check()
            self.joints.assert_still()
        sleep_checked(seconds, still_check)

    def snapshot(self):
        return dict(sensor_planning=self.sensor_pose(), joint_state=self.joints.snapshot())

    def stop(self):
        with self.command_lock:
            self.aborted.set()
            self.client.cancel_all_goals()
            self.group.stop()
            self.group.clear_pose_targets()
            if hasattr(self, 'source_group'):
                self.source_group.stop()
                self.source_group.clear_pose_targets()


class RosVision:
    def __init__(self, config, joints):
        self.c, self.p, self.joints = config, config['parameters'], joints
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.last_stamps = (0., 0.)

    def validate_geometry(self, check):
        def validate():
            for profile in self.c['visual_profiles'].values():
                msg = self.buffer.lookup_transform(profile['parent'], profile['child'], rospy.Time(0))
                if msg.header.stamp.to_sec() != 0:
                    raise ScanError('visual tool TCP must be a static TF')
                require_match(tf_matrix(msg), np.array(profile['matrix']), 'visual static TCP TF')
        wait_for_geometry(validate, self.p['vision_timeout_s'], check)

    def observe(self, check):
        start = time.monotonic()
        end = start + self.p['vision_timeout_s']
        samples = deque()
        after = rospy.Time.now().to_sec()  # insist on post-settle observations
        self.validate_geometry(check)
        while time.monotonic() < end:
            check()
            self.joints.assert_still()
            try:
                sensor = self.buffer.lookup_transform(self.c['vision_frame'], self.c['sensor_tcp_frame'], rospy.Time(0))
                source = self.buffer.lookup_transform(self.c['vision_frame'], self.c['source_tcp_frame'], rospy.Time(0))
                stamps = (sensor.header.stamp.to_sec(), source.header.stamp.to_sec())
                now = rospy.Time.now().to_sec()
                if (all(t > old and t >= after and 0 <= now - t <= self.p['vision_max_age_s']
                        for t, old in zip(stamps, self.last_stamps)) and
                        abs(stamps[0] - stamps[1]) <= self.p['vision_pair_skew_s']):
                    self.last_stamps = stamps
                    samples.append((time.monotonic(), stamps, tf_matrix(sensor),
                                    tf_matrix(source)))
                    while samples and samples[0][0] < time.monotonic() - self.p['vision_window_s'] * 1.2:
                        samples.popleft()
                    if len(samples) >= self.p['vision_min_observations'] and samples[-1][0] - samples[0][0] >= self.p['vision_window_s']:
                        sensor_mean = average([s[2] for s in samples])
                        source_mean = average([s[3] for s in samples])
                        deviations = [residual(s[k], mean) for k, mean in ((2, sensor_mean), (3, source_mean)) for s in samples]
                        if all(p <= self.p['vision_stability_m'] and r <= self.p['vision_stability_rad'] for p, r in deviations):
                            return dict(stamp=min(stamps), sensor=sensor_mean, source=source_mean,
                                        observation_count=len(samples), source_stamp=stamps[1], sensor_stamp=stamps[0],
                                        sensor_observations=[dict(stamp=s[1][0], transform=s[2]) for s in samples],
                                        source_observations=[dict(stamp=s[1][1], transform=s[3]) for s in samples])
            except tf2_ros.TransformException:
                pass
            time.sleep(.01)
        raise ScanError('fresh stable paired visual observations timed out')
