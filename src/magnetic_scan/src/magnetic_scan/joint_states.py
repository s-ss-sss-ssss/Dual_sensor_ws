"""Merge fresh real active states with explicitly measured inactive geometry."""
import threading
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from .configuration import load_config


class StateMerger:
    def __init__(self, source_arm, inactive, max_age):
        self.source_arm, self.inactive, self.max_age = source_arm, inactive, max_age
        self.latest = {}

    def update(self, arm, stamp, received, names, positions, velocities):
        required = [arm + '_joint_%d' % i for i in range(1, 8 if arm == 'diana7' else 7)]
        if len(names) != len(positions) or len(names) != len(set(names)):
            return False
        position = dict(zip(names, positions))
        velocity = dict(zip(names, velocities)) if len(velocities) == len(names) else {}
        if not set(required) <= set(position) or not all(np.isfinite(position[n]) for n in required):
            return False
        if arm in self.latest and stamp <= self.latest[arm]['stamp']:
            return False
        self.latest[arm] = dict(stamp=stamp, received=received,
                                positions={n: position[n] for n in required},
                                velocities={n: velocity[n] for n in required} if velocity else {})
        return True

    def combined(self, now, ros_now):
        arms = ('diana7', self.source_arm)
        if any(arm not in self.latest for arm in arms):
            return None
        rows = [self.latest[arm] for arm in arms]
        if any(now - r['received'] > self.max_age or not 0 <= ros_now - r['stamp'] <= self.max_age for r in rows):
            return None
        positions, velocities = dict(self.inactive), {n: 0. for n in self.inactive}
        for row in rows:
            positions.update(row['positions'])
            velocities.update(row['velocities'])
        names = sorted(positions)
        return dict(stamp=min(r['stamp'] for r in rows), name=names,
                    position=[positions[n] for n in names],
                    velocity=[velocities[n] for n in names] if set(velocities) == set(positions) else [])


def main():
    rospy.init_node('scan_joint_states')
    c = load_config(rospy.get_param('~config'), rospy.get_param('~source_arm'), hardware=True)
    inactive = 'arm2' if c['source_arm'] == 'arm1' else 'arm1'
    merger = StateMerger(c['source_arm'], c['hardware']['inactive_joint_positions'][inactive],
                         c['parameters']['joint_max_age_s'])
    # Validate measured inactive geometry against the loaded URDF joint limits.
    import xml.etree.ElementTree as ET
    robot = ET.fromstring(rospy.get_param('/robot_description'))
    for joint in robot.findall('joint'):
        if joint.get('name') in merger.inactive:
            limit = joint.find('limit')
            if limit is not None and 'lower' in limit.attrib and not (
                    float(limit.get('lower')) <= merger.inactive[joint.get('name')] <= float(limit.get('upper'))):
                raise ValueError('inactive joint outside URDF limits: ' + joint.get('name'))
    lock = threading.Lock()
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def callback(msg, arm):
        with lock:
            merger.update(arm, msg.header.stamp.to_sec(), time.monotonic(), msg.name, msg.position, msg.velocity)

    subs = [rospy.Subscriber('/' + arm + '/joint_states', JointState, callback, callback_args=arm, queue_size=20)
            for arm in ('diana7', c['source_arm'])]
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        with lock:
            combined = merger.combined(time.monotonic(), rospy.Time.now().to_sec())
        if combined:
            msg = JointState()
            msg.header.stamp = rospy.Time.from_sec(combined.pop('stamp'))
            msg.name, msg.position, msg.velocity = combined['name'], combined['position'], combined['velocity']
            pub.publish(msg)
        else:
            rospy.logwarn_throttle(5., 'Waiting for fresh joint states from both connected arms')
        rate.sleep()
