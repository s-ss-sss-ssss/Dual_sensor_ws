"""Resolve tool/profile sources and validate the robot's fixed TCP chain."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from .geometry import transform


def package_path(name):
    import rospkg
    return Path(rospkg.RosPack().get_path(name))


def xyz_rpy(xyz, rpy):
    return transform(dict(translation_m=xyz,
                          quaternion_xyzw=Rotation.from_euler('xyz', rpy).as_quat().tolist()))


def resolve_tools(c, config_dir):
    h = c['hardware']
    name = h['sensor_tool']
    tool_path = package_path('tools_description') / 'tools' / name / 'config/tool.yaml'
    with tool_path.open() as stream:
        tool = yaml.safe_load(stream)
    tcp = tool['urdf_package']['tcp_link_name']
    c['robot_sensor_frame'] = 'diana7_' + tcp
    c['sensor_tool_config'] = tool
    source_path = package_path('tools_description') / 'tools' / h['source_tool'] / 'config/tool.yaml'
    with source_path.open() as stream:
        c['source_tool_config'] = yaml.safe_load(stream)
    v = c.setdefault('vision', {})
    path = Path(v.get('transforms_file', str(package_path('zlab_robots_calibration') / 'config/visual_tool_transforms.yaml'))).expanduser()
    path = path if path.is_absolute() else config_dir / path
    v['transforms_file'] = str(path.resolve())
    with path.open() as stream:
        profiles = yaml.safe_load(stream)['visual_tool_transforms']
    resolved = {}
    for role, tool_name, parent, child in (
            ('sensor', name, c['sensor_tracking_frame'], c['sensor_tcp_frame']),
            ('source', h['source_tool'], c['source_tracking_frame'], c['source_tcp_frame'])):
        profile_name = v.setdefault(role + '_profile', 'default')
        try:
            profile = profiles[tool_name][profile_name]
        except (KeyError, TypeError):
            raise ValueError('missing visual profile: %s/%s' % (tool_name, profile_name))
        matrix = (transform(dict(translation_m=profile['translation'],
                                 quaternion_xyzw=profile['rotation_quaternion']))
                  if 'rotation_quaternion' in profile
                  else xyz_rpy(profile['translation'], profile['rotation_rpy']))
        resolved[role] = dict(tool=tool_name, profile_name=profile_name, profile=profile,
                              parent=parent, child=child, matrix=matrix.tolist())
    c['visual_profiles'] = resolved
    if 'vision_launch_args' in c:
        raise ValueError('migrate vision_launch_args to vision.transforms_file and sensor_profile/source_profile')


def fixed_chain(description, parent, child):
    root = ET.fromstring(description)
    links = {link.get('name') for link in root.findall('link')}
    if parent not in links or child not in links or parent == child:
        raise ValueError('robot TCP or planning link missing/invalid in URDF')
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    result = np.eye(4)
    seen = set()
    while child != parent:
        if child in seen or child not in joints:
            raise ValueError('robot TCP is not a descendant of planning link')
        seen.add(child)
        joint = joints[child]
        if joint.get('type') != 'fixed':
            raise ValueError('planning link to robot TCP must be a fixed URDF joint chain')
        origin = joint.find('origin')
        xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
        rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
        result = xyz_rpy(xyz, rpy) @ result
        child = joint.find('parent').get('link')
    return result
