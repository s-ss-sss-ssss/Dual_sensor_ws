"""Explicit service controlled execution; launch never starts a scan."""
import json
import sys
import threading
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from .configuration import load_config, load_points
from .engine import AlignmentEngine, Engine
from .initial_pose import SETUP_KEYS
from .recording import Recorder
from .ros_adapters import JointMonitor, RosMotion, RosVision
from .ros_recording import BagCapture, RosSampler


class ScanNode:
    def __init__(self):
        self.path = rospy.get_param('~config')
        self.source_arm = rospy.get_param('~source_arm', None)
        self.alignment = rospy.get_param('~alignment', False)
        self.config = load_config(self.path, self.source_arm, hardware=True, alignment=self.alignment)
        if not self.alignment:
            load_points(self.config['points_file'])
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.worker = None
        self.motion = None
        self.output = None
        self.publisher = rospy.Publisher('~status', String, queue_size=10, latch=True)
        self.set_status('READY', None)
        self.start_service = rospy.Service('~start', Trigger, self.start)
        self.stop_service = rospy.Service('~stop', Trigger, self.stop)
        rospy.on_shutdown(self.shutdown)

    def set_status(self, state, point):
        self.publisher.publish(String(data=json.dumps(dict(state=state, point_id=point, output_dir=self.output))))

    def start(self, _request):
        with self.lock:
            if self.worker and self.worker.is_alive():
                return TriggerResponse(False, 'scan already running')
            self.cancel.clear()
            self.worker = threading.Thread(target=self.run, daemon=True)
            self.worker.start()
        return TriggerResponse(True, 'scan started; monitor ~status')

    def stop(self, _request):
        self.cancel.set()
        if self.motion:
            self.motion.stop()
        return TriggerResponse(True, 'stop requested; data will be saved')

    def run(self):
        recorder = None
        capture = None
        joints = None
        sampler = None
        vision = None
        engine_ran = False
        state = 'FAILED'
        try:
            c = load_config(self.path, self.source_arm, hardware=True, alignment=self.alignment)
            for key in SETUP_KEYS:
                if c.get(key) != self.config.get(key):
                    raise ValueError('physical setup changed; restart experiment nodes before scanning: ' + key)
            points = {'points': []} if self.alignment else load_points(c['points_file'])
            recorder = Recorder(c, points)
            self.output = str(recorder.path)
            self.set_status('INITIALIZING', None)
            if not self.alignment:
                capture = BagCapture(c, recorder.path)
            joints = JointMonitor(c)
            self.motion = RosMotion(c, joints)
            vision = RosVision(c, joints)
            options = dict(cancelled=lambda: self.cancel.is_set() or rospy.is_shutdown(),
                           status=self.set_status)
            if self.alignment:
                engine = AlignmentEngine(c, self.motion, vision, recorder, **options)
            else:
                sampler = RosSampler(c, joints)
                engine = Engine(c, points, self.motion, vision, sampler, recorder,
                                health=capture.health, **options)
            engine_ran = True
            state = engine.run()
        except Exception as error:
            rospy.logerr('Scan initialization failed: %s', error)
            if recorder and not engine_ran:
                recorder.finish('FAILED', str(error))
        finally:
            try:
                if self.motion:
                    self.motion.stop()
                if sampler:
                    sampler.close()
                if joints:
                    joints.close()
                if vision:
                    vision.listener.unregister()
                if self.motion and hasattr(self.motion, 'listener'):
                    self.motion.listener.unregister()
                if capture:
                    capture.close()
            except Exception as error:
                state = 'FAILED'
                rospy.logerr('Scan cleanup failed: %s', error)
            self.motion = None
            self.set_status(state, None)

    def shutdown(self):
        self.stop(None)
        if self.worker:
            self.worker.join(timeout=5.)


def main():
    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('magnetic_scan')
    try:
        node = ScanNode()
        rospy.spin()
    finally:
        moveit_commander.roscpp_shutdown()
