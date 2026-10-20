"""Strict point semantics and explicit physical setup configuration."""
import copy
import json
import math
from pathlib import Path
import yaml
from .geometry import transform
from .tool_geometry import resolve_tools

DEFAULTS = dict(position_tolerance_m=0.0005, angle_tolerance_rad=math.radians(.5),
                correction_step_m=.005, correction_step_rad=math.radians(2), max_iterations=10,
                vision_timeout_s=30., vision_window_s=1., vision_min_observations=10,
                vision_max_age_s=.5, vision_pair_skew_s=.1,
                vision_stability_m=.0005, vision_stability_rad=math.radians(.5),
                settle_s=1., sample_s=1., min_rounds=10, sample_timeout_s=10.,
                cartesian_step_m=.001, velocity_scaling=.05, acceleration_scaling=.05,
                motion_timeout_s=60., joint_max_age_s=.5, stationary_window_s=.5,
                joint_velocity_rad_s=.005, source_tolerance_m=.0005,
                source_tolerance_rad=math.radians(.5), initial_joint_tolerance_rad=.005)


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('%s must be finite and positive' % name)
    return value


def load_points(path):
    with open(path, encoding='utf-8') as stream:
        doc = json.load(stream)
    if doc.get('version') != 1 or doc.get('frame') != 'initial_sensor':
        raise ValueError('points require version=1 and frame=initial_sensor')
    if not isinstance(doc.get('points'), list) or not doc['points']:
        raise ValueError('points must be a nonempty list')
    seen = set()
    for point in doc['points']:
        if not isinstance(point, dict) or set(point) - {'id', 'offset_m', 'vision_check', 'settle_s', 'sample_s'}:
            raise ValueError('unsupported point fields')
        name = point.get('id')
        if not isinstance(name, str) or not name.strip() or name in seen:
            raise ValueError('point IDs must be nonempty and unique')
        seen.add(name)
        xyz = point.get('offset_m')
        if not isinstance(xyz, list) or len(xyz) != 3 or any(
                isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in xyz):
            raise ValueError('offset_m must contain three finite numbers')
        if type(point.get('vision_check', False)) is not bool:
            raise ValueError('vision_check must be boolean')
        for key in ('settle_s', 'sample_s'):
            if key in point:
                positive(point[key], key)
    return doc


def load_config(path, source_arm=None, hardware=False, alignment=False):
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        c = yaml.safe_load(stream)
    if not isinstance(c, dict):
        raise ValueError('experiment config must be a mapping')
    c = copy.deepcopy(c)
    legacy = {'planning_link_to_sensor', 'tracking_to_sensor', 'tracking_to_source'} & set(c)
    if legacy:
        raise ValueError('migrate legacy extrinsics to tool TF and visual profiles: ' + ', '.join(sorted(legacy)))
    c['source_arm'] = source_arm or c.get('source_arm', 'arm1')
    if c['source_arm'] not in ('arm1', 'arm2'):
        raise ValueError('source_arm must be arm1 or arm2')
    c['parameters'] = dict(DEFAULTS, **c.get('parameters', {}))
    for key, value in c['parameters'].items():
        if key not in DEFAULTS:
            raise ValueError('unknown parameter ' + key)
        positive(value, key)
    for key in ('max_iterations', 'vision_min_observations', 'min_rounds'):
        if type(c['parameters'][key]) is not int:
            raise ValueError(key + ' must be integer')
    for key in ('velocity_scaling', 'acceleration_scaling'):
        if c['parameters'][key] > 1:
            raise ValueError(key + ' must be <= 1')
    if c['parameters']['vision_min_observations'] < 2:
        raise ValueError('vision_min_observations must be >= 2')
    if c['parameters']['sample_timeout_s'] < c['parameters']['sample_s']:
        raise ValueError('sample_timeout_s must be >= sample_s')
    for key in (('output_dir', 'initial_pose_file') if alignment else
                ('points_file', 'output_dir', 'initial_pose_file')):
        p = Path(c[key]).expanduser()
        c[key] = str(p if p.is_absolute() else path.parent / p)
    c['config_path'] = str(path)
    for key in ('planning_frame', 'planning_link', 'vision_frame', 'sensor_tracking_frame', 'source_tracking_frame',
                'sensor_tcp_frame', 'source_tcp_frame'):
        if not isinstance(c.get(key), str) or not c[key].strip():
            raise ValueError('missing frame/link: ' + key)
        c[key] = c[key].format(source_arm=c['source_arm'])
    resolve_tools(c, path.parent)
    rotation = c.get('source_to_sensor_quaternion_xyzw', [1., 0., 0., 0.])
    transform(dict(translation_m=[0, 0, 0], quaternion_xyzw=rotation))
    c['source_to_sensor_quaternion_xyzw'] = rotation
    bounds = c.get('center_distance_range_m')
    if not isinstance(bounds, list) or len(bounds) != 2:
        raise ValueError('center_distance_range_m must be explicitly configured')
    if not 0 < positive(bounds[0], 'minimum distance') < positive(bounds[1], 'maximum distance'):
        raise ValueError('invalid center distance range')
    distance = positive(c.get('alignment', {}).get('target_distance_m'), 'alignment.target_distance_m')
    if not bounds[0] <= distance <= bounds[1]:
        raise ValueError('alignment.target_distance_m outside center_distance_range_m')
    if not alignment:
        validate_sampling(c)
    if hardware:
        validate_hardware(c, path)
    return c


def validate_sampling(c):
    expected = c['sampling']['expected']
    if not isinstance(expected, dict) or not expected or set(expected) - {'ak', 'tmag'}:
        raise ValueError('sampling.expected requires ak and/or tmag ID lists')
    for family, ids in expected.items():
        if not isinstance(ids, list) or not ids or any(type(i) is not int or not 0 <= i < 255 for i in ids) or len(set(ids)) != len(ids):
            raise ValueError('invalid expected sensor IDs for ' + family)
    for key in ('corrected_topic', 'raw_topic'):
        if not isinstance(c['sampling'].get(key), str) or not c['sampling'][key].startswith('/'):
            raise ValueError('sampling.' + key + ' must be an absolute topic')
    if type(c['sampling'].get('require_identity_verified', True)) is not bool:
        raise ValueError('require_identity_verified must be boolean')


def validate_hardware(c, path):
    h = c['hardware']
    if not h.get('geometry_confirmed', False):
        raise ValueError('hardware.geometry_confirmed must be true after verifying physical transforms')
    for key in ('sensor_tool', 'source_tool', 'inactive_tool', 'diana7_ip'):
        if not isinstance(h.get(key), str) or not h[key].strip():
            raise ValueError('hardware.' + key + ' must be configured')
    if type(h.get('connect_inactive_arm', False)) is not bool:
        raise ValueError('hardware.connect_inactive_arm must be boolean')
    if 'layout_config' in h:
        p = Path(h['layout_config']).expanduser()
        h['layout_config'] = str(p if p.is_absolute() else path.parent / p)
    inactive = 'arm2' if c['source_arm'] == 'arm1' else 'arm1'
    if h.get('connect_inactive_arm', False):
        return
    joints = h.get('inactive_joint_positions', {}).get(inactive, {})
    if set(joints) != {inactive + '_joint_' + str(i) for i in range(1, 7)} or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in joints.values()):
        raise ValueError('configure all six measured stationary joints of ' + inactive)
