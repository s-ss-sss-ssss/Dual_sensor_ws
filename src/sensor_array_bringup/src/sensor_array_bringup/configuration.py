"""Single-board configuration, independent of ROS and sensor_array_config."""
import math
from pathlib import Path
import re

import yaml


_REQUIRED = {'device_id', 'firmware_device_id', 'port', 'sensor_correction_dir'}
_OPTIONAL = {'check_device_id', 'baud_rate', 'raw_scale',
             'expected_ak', 'expected_tmag', 'expect_icm'}
_FIND = re.compile(r'\$\(find\s+([A-Za-z][A-Za-z0-9_]*)\)')


def _string(value, name):
    if not isinstance(value, str) or not value.strip() or value != value.strip():
        raise ValueError('%s must be a nonempty string without surrounding whitespace' % name)
    if any(ord(char) < 32 for char in value):
        raise ValueError('%s must not contain control characters' % name)
    return value


def _integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('%s must be an integer in %d..%d' % (name, minimum, maximum))
    return value


def _boolean(value, name):
    if type(value) is not bool:
        raise ValueError('%s must be a YAML boolean' % name)
    return value


def _pairs(value, name):
    if not isinstance(value, list):
        raise ValueError('%s must be a list of [uint8, uint8] pairs' % name)
    result = []
    for pair in value:
        if not isinstance(pair, list) or len(pair) != 2:
            raise ValueError('%s entries must be [uint8, uint8] pairs' % name)
        item = [_integer(item, name, 0, 255) for item in pair]
        if item in result:
            raise ValueError('%s entries must be unique' % name)
        result.append(item)
    return result


def _device_id(value):
    value = _string(value, 'device_id')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value):
        raise ValueError('device_id must be a single ROS name and directory name')
    return value


def _sensor_correction_path(value, base, device_id):
    value = _string(value, 'sensor_correction_dir')
    resolver = None

    def find_package(match):
        nonlocal resolver
        # Only package expressions require ROS package discovery.
        import rospkg
        if resolver is None:
            resolver = rospkg.RosPack()
        try:
            return resolver.get_path(match.group(1))
        except rospkg.ResourceNotFound as error:
            raise ValueError('sensor_correction_dir package not found: %s' % match.group(1)) from error

    value = _FIND.sub(find_package, value)
    if '$(' in value:
        raise ValueError('sensor_correction_dir only supports $(find pkg) substitutions')
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    path = path.resolve()
    if not path.is_dir():
        raise ValueError('sensor_correction_dir does not exist or is not a directory: %s' % path)
    device_path = path / device_id
    if not device_path.is_dir():
        raise ValueError('sensor_correction_dir/device_id does not exist or is not a directory: %s' % device_path)
    return str(path)


def load_configuration(config_file):
    """Validate one board YAML and return normalized device settings.

    Sensor correction contents are owned by the sensor_correction node; this loader only
    checks the root and device directories. No ports are opened and no files are written.
    """
    path = Path(config_file).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        source = yaml.safe_load(stream)
    if not isinstance(source, dict):
        raise ValueError('configuration must be a single-board mapping')
    missing = _REQUIRED - source.keys()
    unknown = source.keys() - _REQUIRED - _OPTIONAL
    if missing:
        raise ValueError('missing required fields: %s' % ', '.join(sorted(missing)))
    if unknown:
        raise ValueError('unknown fields: %s (use one YAML per board)' % ', '.join(map(str, unknown)))
    result = {
        'device_id': _device_id(source['device_id']),
        'firmware_device_id': _integer(source['firmware_device_id'], 'firmware_device_id', 0, 0xFFFFFFFF),
        'check_device_id': _boolean(source.get('check_device_id', True), 'check_device_id'),
        'port': _string(source['port'], 'port'),
        'baud_rate': _integer(source.get('baud_rate', 115200), 'baud_rate', 1, 0x7FFFFFFF),
    }
    scale = source.get('raw_scale', 1.0)
    if type(scale) not in (int, float):
        raise ValueError('raw_scale must be a finite scalar')
    try:
        scale = float(scale)
    except OverflowError as error:
        raise ValueError('raw_scale must be a finite scalar') from error
    if not math.isfinite(scale):
        raise ValueError('raw_scale must be a finite scalar')
    result['raw_scale'] = scale
    result['sensor_correction_dir'] = _sensor_correction_path(
        source['sensor_correction_dir'], path.parent, result['device_id'])
    result['expected_ak'] = _pairs(source.get('expected_ak', []), 'expected_ak')
    result['expected_tmag'] = _pairs(source.get('expected_tmag', []), 'expected_tmag')
    result['expect_icm'] = _boolean(source.get('expect_icm', False), 'expect_icm')
    if len(result['expected_ak']) + len(result['expected_tmag']) + result['expect_icm'] > 64:
        raise ValueError('at most 64 expected sensors are supported')
    # Values passed as roslaunch arguments must not introduce substitutions.
    for key in ('device_id', 'port', 'sensor_correction_dir'):
        if '$(' in result[key]:
            raise ValueError('%s must not contain unresolved ROS substitutions' % key)
    return result
