"""Portable, validated starting joints and physical relative pose."""
import json
import math
import os
import tempfile
import time
from pathlib import Path
from .geometry import inverse, rotation_from_matrix, transform


SETUP_KEYS = ('source_arm', 'planning_frame', 'planning_link', 'vision_frame',
              'sensor_tracking_frame', 'source_tracking_frame', 'sensor_tcp_frame',
              'source_tcp_frame', 'robot_sensor_frame', 'sensor_tool_config',
              'source_tool_config', 'visual_profiles', 'hardware', 'source_to_sensor_quaternion_xyzw',
              'alignment', 'center_distance_range_m')


def setup(config):
    return dict({key: config.get(key) for key in SETUP_KEYS},
                resolved_geometry=config['resolved_geometry'])


def joint_names(config):
    return ['diana7_joint_%d' % i for i in range(1, 8)] + [
        config['source_arm'] + '_joint_%d' % i for i in range(1, 7)]


def make_initial_pose(config, observation, snapshot):
    state = snapshot['joint_state']
    joints = dict(zip(state['name'], state['position']))
    relative = inverse(observation['source']) @ observation['sensor']
    return dict(version=2, created_at=time.time(), setup=setup(config),
                joints={name: joints[name] for name in joint_names(config)},
                relative_pose=dict(translation_m=relative[:3, 3].tolist(),
                                   quaternion_xyzw=rotation_from_matrix(relative[:3, :3]).as_quat().tolist()),
                target_distance_m=config['alignment']['target_distance_m'])


def validate_initial_pose(config, doc):
    if not isinstance(doc, dict) or type(doc.get('version')) is not int or doc['version'] != 2:
        raise ValueError('initial pose requires version=2; repeat alignment to migrate')
    if doc.get('setup') != setup(config):
        raise ValueError('initial pose physical setup mismatch; repeat alignment')
    joints = doc.get('joints')
    if not isinstance(joints, dict) or set(joints) != set(joint_names(config)) or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
            for v in joints.values()):
        raise ValueError('initial pose requires all 13 finite active joint angles in radians')
    transform(doc['relative_pose'])
    if doc.get('target_distance_m') != config['alignment']['target_distance_m']:
        raise ValueError('initial pose target distance mismatch')
    return doc


def load_initial_pose(config):
    with open(config['initial_pose_file'], encoding='utf-8') as stream:
        return validate_initial_pose(config, json.load(stream))


def save_initial_pose(config, doc):
    validate_initial_pose(config, doc)
    path = Path(config['initial_pose_file'])
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(doc, stream, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
