"""ROS-independent AK relative consistency calibration and experiment planning.

Vectors are nominal array-frame microtesla. No absolute bias is estimated.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import random

import numpy as np
import yaml


IDS = tuple(range(1, 13))
PHASES = ('OFF_PRE_CAPTURE', 'ON_CAPTURE', 'OFF_POST_CAPTURE')


def positive(value, name, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('%s must be a finite number' % name)
    if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
        raise ValueError('%s must be %s' % (name, 'nonnegative' if allow_zero else 'positive'))
    return value


def integer(value, name, minimum=1):
    if type(value) is not int or value < minimum:
        raise ValueError('%s must be an integer >= %s' % (name, minimum))
    return value


def validate_config(source):
    config = copy.deepcopy(source)
    if not isinstance(config, dict):
        raise ValueError('configuration must be a mapping')
    import re
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', str(config.get('device_id', ''))):
        raise ValueError('device_id must be a single ROS name')
    firmware_id = config.get('firmware_device_id')
    if isinstance(firmware_id, str) and firmware_id.isdecimal():
        firmware_id = int(firmware_id)
    if type(firmware_id) is not int or not 0 <= firmware_id <= 0xffffffff:
        raise ValueError('firmware_device_id must be the measured uint32 STATS ID')
    config['firmware_device_id'] = firmware_id
    namespace = config.get('generator_namespace', '/fy8300')
    if not isinstance(namespace, str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace):
        raise ValueError('generator_namespace must be an absolute ROS namespace')
    if config.get('axes_confirmed') is not True:
        raise ValueError('verify physical Q axes, then set axes_confirmed: true')
    for name in ('sensor_mode', 'array_geometry', 'output_root'):
        if not isinstance(config.get(name), str) or not config[name]:
            raise ValueError('%s is required' % name)
    channels = config.get('channels', {})
    if set(channels) != set('XYZ'):
        raise ValueError('channels must define X, Y and Z')
    mapped = []
    for axis, entry in channels.items():
        channel = entry.get('channel')
        if type(channel) is not int or channel not in (1, 2, 3):
            raise ValueError('%s channel must be 1, 2 or 3' % axis)
        mapped.append(channel)
        levels = entry.get('levels_v')
        if not isinstance(levels, list) or len(levels) != 5:
            raise ValueError('%s levels_v must contain exactly five measured positive voltages' % axis)
        for value in levels:
            positive(value, axis + '.levels_v')
        if any(a >= b for a, b in zip(levels, levels[1:])):
            raise ValueError('%s levels_v must be strictly increasing' % axis)
        limit = positive(entry.get('max_abs_offset_v'), axis + '.max_abs_offset_v')
        validation = entry.setdefault('validation_v', (levels[1] + levels[2]) / 2)
        positive(validation, axis + '.validation_v')
        if max(levels + [validation]) > limit:
            raise ValueError('%s voltage exceeds configured limit' % axis)
    if sorted(mapped) != [1, 2, 3]:
        raise ValueError('X/Y/Z must map to distinct channels')
    defaults = dict(startup_s=3., capture_s=3., settle_s=1., max_capture_s=15.,
                    subwindow_s=.5, min_samples=500, retries=2, repeats=3, seed=20261015,
                    command_timeout_s=10., stream_timeout_s=3., connection_timeout_s=30.,
                    clock_guard_s=.25, clock_residual_limit_s=.25, clock_lock_s=1.,
                    output_tolerance_v=.002, rcond=1e-10, max_condition=1000.,
                    reference_id=None, short_window_s=.1,
                    max_std_ut=None, max_slope_ut_s=None, max_background_change_ut=None,
                    max_validation_rmse_ut=None, max_validation_pairwise_ut=None)
    for name, value in defaults.items():
        config.setdefault(name, value)
    for name in ('startup_s', 'capture_s', 'settle_s', 'max_capture_s', 'subwindow_s',
                 'command_timeout_s', 'stream_timeout_s', 'connection_timeout_s',
                 'clock_guard_s', 'clock_residual_limit_s', 'clock_lock_s',
                 'output_tolerance_v', 'rcond', 'max_condition', 'short_window_s'):
        positive(config[name], name)
    for name in ('min_samples', 'repeats'):
        integer(config[name], name, 2 if name == 'min_samples' else 1)
    for name in ('retries', 'seed'):
        integer(config[name], name, 0)
    for name in ('max_std_ut', 'max_slope_ut_s', 'max_background_change_ut',
                 'max_validation_rmse_ut', 'max_validation_pairwise_ut'):
        if config[name] is not None:
            positive(config[name], name)
    if config['max_capture_s'] < max(config['startup_s'], config['capture_s']):
        raise ValueError('max_capture_s must cover startup_s and capture_s')
    if min(config['capture_s'], config['startup_s']) <= 2 * config['clock_guard_s'] + 2 * config['subwindow_s']:
        raise ValueError('capture windows must cover guards and two subwindows')
    if config['reference_id'] is not None and (type(config['reference_id']) is not int or config['reference_id'] not in IDS):
        raise ValueError('reference_id must be null or 1..12')
    return config


def load_config(path):
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        config = validate_config(yaml.safe_load(stream))
    for key in ('array_geometry', 'output_root'):
        value = Path(config[key]).expanduser()
        config[key] = str((path.parent / value).resolve() if not value.is_absolute() else value.resolve())
    with Path(config['array_geometry']).open(encoding='utf-8') as stream:
        geometry = json.load(stream)
    q = {}
    for group in geometry['R_CORR']:
        matrix = np.asarray(group['matrix'], dtype=float).reshape(3, 3)
        if not np.isfinite(matrix).all() or not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(matrix), 1):
            raise ValueError('Q must be a finite proper rotation')
        for sid in group['sensor_ids']:
            if type(sid) is not int or sid in q or sid not in IDS:
                raise ValueError('invalid or repeated geometry sensor ID')
            q[sid] = matrix.tolist()
    if set(q) != set(IDS):
        raise ValueError('Q must explicitly cover all 12 sensors')
    positions = np.asarray(geometry['d_list'], dtype=float)
    if positions.shape != (12, 3) or not np.isfinite(positions).all():
        raise ValueError('d_list must contain 12 finite positions in ID order, meters')
    config['sensors'] = [dict(sensor_id=sid, bus=(sid - 1) // 6 + 1,
                              mask=1 << ((sid - 1) % 6 + 1), Q=q[sid],
                              position_m=positions[sid - 1].tolist()) for sid in IDS]
    return config


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, allow_nan=False).encode()).hexdigest()


def make_plan(config):
    trials = []
    rng = random.Random(config['seed'])
    for repeat in range(config['repeats']):
        states = []
        for axis in 'XYZ':
            for level, voltage in enumerate(config['channels'][axis]['levels_v'], 1):
                for sign in (1, -1):
                    states.append(dict(split='train', repeat=repeat + 1, axes=axis,
                                       level=level, polarity=sign, commands_v={axis: sign * voltage}))
        rng.shuffle(states)
        trials.extend(states)
    for axes in ('X', 'Y', 'Z', 'XY', 'XZ', 'YZ'):
        for sign in (1, -1):
            trials.append(dict(split='validation', repeat=1, axes=axes, level='intermediate',
                               polarity=sign, commands_v={a: sign * config['channels'][a]['validation_v'] for a in axes}))
    for index, trial in enumerate(trials, 1):
        trial['trial_id'] = index
    return trials


def window_statistics(samples, start, end, config):
    """Samples: dict(t, raw, drdy, err, dor, valid), all in mapped monotonic time."""
    good = [s for s in samples if start <= s['t'] <= end and s.get('valid', True)
            and s.get('comm_ok', True) and s['drdy'] and not s['err']]
    if len(good) < config['min_samples']:
        raise ValueError('insufficient effective samples: %s' % len(good))
    times = np.array([s['t'] for s in good])
    values = np.array([s['x'] for s in good], dtype=float)
    if not np.isfinite(values).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('nonfinite or nonincreasing sensor samples')
    width = config['subwindow_s']
    # Every full subwindow must have coverage; counts alone cannot pass a long gap.
    bins = int((end - start) / width + 1e-8)
    means, centers = [], []
    for index in range(bins):
        selected = (times >= start + index * width) & (times < start + (index + 1) * width)
        if not selected.any():
            raise ValueError('missing subwindow coverage')
        means.append(values[selected].mean(axis=0))
        centers.append(times[selected].mean())
    if bins < 2 or times[0] - start > width or end - times[-1] > width:
        raise ValueError('insufficient time coverage')
    if np.max(np.diff(times)) > width:
        raise ValueError('sample gap exceeds subwindow duration')
    centers = np.asarray(centers)
    means = np.asarray(means)
    slope = np.linalg.lstsq(np.column_stack((centers - centers.mean(), np.ones(bins))), means, rcond=None)[0][0]
    std = values.std(axis=0, ddof=1)
    if config['max_std_ut'] is not None and max(std) > config['max_std_ut']:
        raise ValueError('window noise threshold exceeded')
    if config['max_slope_ut_s'] is not None and max(abs(slope)) > config['max_slope_ut_s']:
        raise ValueError('window drift threshold exceeded')
    return dict(n=len(good), mean=values.mean(axis=0).tolist(), t=float(times.mean()),
                covariance=np.cov(values.T).tolist(), std=std.tolist(), slope=slope.tolist(),
                subwindow_spread=means.std(axis=0).tolist(),
                endpoint_change=(means[-1] - means[0]).tolist(),
                effective_rate_hz=(len(good) - 1) / (times[-1] - times[0]),
                dor_fraction=sum(bool(s['dor']) for s in good) / len(good),
                start=start, end=end)


def choose_reference(statistics, requested=None):
    if set(statistics) != set(IDS):
        raise ValueError('startup must cover all 12 sensors')
    if requested is not None:
        return requested
    return min(IDS, key=lambda sid: (-statistics[sid]['n'],
               np.linalg.norm(statistics[sid]['subwindow_spread']),
               np.linalg.norm(statistics[sid]['std']), sid))


def build_delta(pre, on, post, max_background_change=None):
    if not pre['t'] < on['t'] < post['t']:
        raise ValueError('background interpolation requires pre < on < post')
    before, after = np.array(pre['mean']), np.array(post['mean'])
    if max_background_change is not None and np.linalg.norm(after - before) > max_background_change:
        raise ValueError('background change threshold exceeded')
    alpha = (on['t'] - pre['t']) / (post['t'] - pre['t'])
    background = (1 - alpha) * before + alpha * after
    return dict(alpha=alpha, background=background.tolist(), delta=(np.array(on['mean']) - background).tolist())


def fit_relative(delta, reference_id, rcond=1e-10, max_condition=1000.):
    if set(delta) != set(IDS) or reference_id not in delta:
        raise ValueError('training must cover all 12 sensors and reference')
    target = np.asarray(delta[reference_id], dtype=float)
    if target.ndim != 2 or target.shape[1] != 3 or len(target) < 3 or not np.isfinite(target).all():
        raise ValueError('invalid reference training data')
    result = {}
    for sid in IDS:
        design = np.asarray(delta[sid], dtype=float)
        if design.shape != target.shape or not np.isfinite(design).all():
            raise ValueError('invalid or misaligned training data')
        solution, _, rank, singular = np.linalg.lstsq(design, target, rcond=rcond)
        if rank != 3:
            raise ValueError('insufficient independent excitation directions')
        condition = float(singular[0] / singular[-1])
        if condition > max_condition:
            raise ValueError('poor excitation conditioning: sensor %s' % sid)
        matrix = np.eye(3) if sid == reference_id else solution.T
        residual = design @ matrix.T - target
        result[sid] = dict(T=matrix.tolist(), singular_values=singular.tolist(), condition=condition,
                           training_vector_rmse_ut=float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))))
    return result


def metrics(vectors, positions):
    vectors = np.asarray(vectors, dtype=float)
    centered = vectors - vectors.mean(axis=0)
    position = np.asarray(positions, dtype=float)
    design = position - position.mean(axis=0)
    gradient = None
    if np.linalg.matrix_rank(design) == 3:
        gradient = float(np.linalg.norm(np.linalg.lstsq(design, centered, rcond=None)[0], 'fro'))
    return dict(vector_rmse_ut=float(np.sqrt(np.mean(np.sum(centered ** 2, axis=1)))),
                max_pairwise_ut=float(np.linalg.norm(vectors[:, None] - vectors[None, :], axis=2).max()),
                gradient_frobenius_ut_m=gradient)


def align_trials(config, summaries):
    """Align by explicit trial + attempt + sensor IDs; never by CSV row order."""
    groups = {}
    expected = {t['trial_id']: t for t in make_plan(config)}
    for row in summaries:
        if not row['accepted']:
            continue
        planned = expected.get(row['trial_id'])
        if planned is None or any(row[k] != planned[k] for k in ('split', 'axes', 'level', 'polarity', 'repeat', 'commands_v')):
            raise ValueError('trial metadata does not match frozen experiment plan')
        key = (row['trial_id'], row['attempt'])
        group = groups.setdefault(key, {})
        sid = row['sensor_id']
        if sid not in IDS or sid in group:
            raise ValueError('duplicate or invalid trial sensor')
        group[sid] = row
    accepted, seen = [], set()
    for (trial_id, _), group in sorted(groups.items()):
        if set(group) != set(IDS) or trial_id in seen:
            raise ValueError('incomplete or duplicate accepted trial')
        seen.add(trial_id)
        if len({group[s]['split'] for s in IDS}) != 1:
            raise ValueError('mixed trial splits')
        accepted.append(group)
    if seen != set(expected):
        raise ValueError('experiment plan is incomplete')
    train = [g for g in accepted if g[1]['split'] == 'train']
    validation = [g for g in accepted if g[1]['split'] == 'validation']
    if not train or not validation:
        raise ValueError('training and independent validation are required')
    return train, validation


def analyze_trials(config, summaries, reference_id):
    train, validation = align_trials(config, summaries)
    fitted = fit_relative({sid: [g[sid]['delta'] for g in train] for sid in IDS}, reference_id,
                          config['rcond'], config['max_condition'])
    positions = [s['position_m'] for s in config['sensors']]
    evaluated = []
    for group in validation:
        raw = np.array([group[s]['delta'] for s in IDS])
        corrected = np.array([np.array(fitted[s]['T']) @ raw[s - 1] for s in IDS])
        before, after = metrics(raw, positions), metrics(corrected, positions)
        entry = {k: group[1][k] for k in ('trial_id', 'axes', 'level', 'polarity', 'repeat')}
        entry.update(before=before, after=after, improvement_fraction={
            key: (1 - after[key] / value if value is not None and value > 1e-12 and after[key] is not None else None)
            for key, value in before.items()})
        entry['sensor_residual_ut'] = {sid: float(np.linalg.norm(corrected[sid - 1] - corrected[reference_id - 1])) for sid in IDS}
        evaluated.append(entry)
    performance = None
    thresholds = {'vector_rmse_ut': config['max_validation_rmse_ut'],
                  'max_pairwise_ut': config['max_validation_pairwise_ut']}
    if any(v is not None for v in thresholds.values()):
        performance = all(entry['after'][key] <= limit for entry in evaluated
                          for key, limit in thresholds.items() if limit is not None)
    return dict(schema_version=1, reference_id=reference_id, sensor_ids=list(IDS),
                nominal_ut_per_count=1.1, input_frame='nominal array frame after Q',
                output_frame='reference sensor scale, nominal array frame',
                output_quantity='background-subtracted field increment, nominal uT',
                fit_method='SVD ordinary least squares without intercept',
                config_sha256=config_hash(config), device_id=config['device_id'],
                firmware_device_id=config['firmware_device_id'], sensor_mode=config['sensor_mode'],
                sensors=[dict(s, **fitted[s['sensor_id']]) for s in config['sensors']],
                training_trial_ids=[g[1]['trial_id'] for g in train],
                validation=evaluated, data_valid=True, performance_pass=performance,
                temperature_range_c=None,
                limitations=['Relative calibration retains reference distortion.',
                             'No absolute bias, AC phase or temperature correction.',
                             'Timing mapping is estimated from host arrivals, not hardware synchronization.'])
