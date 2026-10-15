"""ROS-independent affine correction, indexed by family and logical sensor ID."""
import json
import math
from pathlib import Path
import re

import numpy as np


# AK: signed int16 counts -> microtesla, applied independently to X/Y/Z.
AK_UT_PER_COUNT = 1.1


class AffineSensorCorrection:
    def __init__(self, config_dir, device_id, raw_scale=1.0):
        self.raw_scale = float(raw_scale)
        if not math.isfinite(self.raw_scale):
            raise ValueError('raw_scale must be finite')
        if not isinstance(device_id, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', device_id):
            raise ValueError('device_id must be a single ROS/directory name')
        directory = Path(config_dir).expanduser() / device_id
        if not directory.is_dir():
            raise ValueError('device correction directory does not exist: %s' % directory)
        self.parameters = {}
        self.families = {}
        for family_dir in sorted(directory.iterdir()):
            if not family_dir.is_dir():
                continue
            family = family_dir.name
            path = family_dir / 'affine.json'
            if not path.is_file():
                raise ValueError('missing affine.json for family %s: %s' % (family, path))
            self._load_family(family, path)
            self.families[family] = str(path)
        if not self.families:
            raise ValueError('device correction directory has no family subdirectories: %s' % directory)

    def _load_family(self, family, path):
        with path.open(encoding='utf-8') as stream:
            payload = json.load(stream)
        if not isinstance(payload, dict) or not isinstance(payload.get('sensors'), list):
            raise ValueError('%s must contain a sensors list' % path)
        if not payload['sensors']:
            raise ValueError('%s sensors list must not be empty' % path)
        for entry in payload['sensors']:
            sid = entry['sensor_id']
            key = (family, sid)
            if type(sid) is not int or not 0 <= sid < 255 or key in self.parameters:
                raise ValueError('%s: sensor_id must be unique within family and in 0..254' % path)
            matrix = np.asarray(entry['D_i'], dtype=float)
            bias = np.asarray(entry['e_i'], dtype=float)
            if matrix.shape != (3, 3) or bias.shape != (3,):
                raise ValueError('%s: D_i must be 3x3 and e_i must contain 3 values' % path)
            if not np.isfinite(matrix).all() or not np.isfinite(bias).all():
                raise ValueError('%s: sensor correction coefficients must be finite' % path)
            self.parameters[key] = (matrix, bias)

    def apply(self, family, sensor_id, raw):
        if sensor_id == 0xFF or (family, sensor_id) not in self.parameters:
            return None
        # Protocol already decodes signed int16; no offset or unsigned conversion.
        scaled = np.asarray(raw, dtype=float) * self.raw_scale
        if family == 'ak':
            scaled *= AK_UT_PER_COUNT
        # TODO: confirm TMAG physical-unit conversion; retain its existing scale.
        matrix, bias = self.parameters[(family, sensor_id)]
        result = matrix @ scaled + bias
        if not np.isfinite(result).all():
            raise ValueError('non-finite corrected sample')
        return result


class DeviceIdentityGate:
    """Fail closed until STATS verifies identity; latch a mismatch until restart."""
    def __init__(self, expected, enabled=True):
        if type(enabled) is not bool:
            raise ValueError('check_device_id must be boolean')
        if isinstance(expected, str) and expected.isdecimal():
            expected = int(expected)
        if type(expected) is not int or not 0 <= expected <= 0xFFFFFFFF:
            raise ValueError('firmware_device_id must be uint32 or a decimal string')
        self.expected = expected
        self.enabled = enabled
        self.verified = False
        self.mismatch = False
        self.observed = None

    def accept(self, has_stats, device_id=None):
        if has_stats:
            self.observed = device_id
            if device_id != self.expected:
                self.mismatch = True
            elif not self.mismatch:
                self.verified = True
        return not self.enabled or (self.verified and not self.mismatch)
