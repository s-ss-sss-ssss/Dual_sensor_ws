"""Fixed-calibration validation and MaPS local quantities (no ROS dependency)."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from .consistency import IDS, PHASES, build_delta, positive, validate_config


MAPS_METHOD = 'MaPS local estimator: null(D), Pw=I-w1T, symmetric traceless S, column-major vec; v1'
MAPS_SOURCE = '/home/zhang/embedded_array_ws/src/gels_localization/scripts/maps_estimator.py'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_validation_config(path):
    path = Path(path).expanduser().resolve()
    source = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(source, dict):
        raise ValueError('configuration must be a mapping')
    def resolve(value):
        p = Path(value).expanduser()
        return str((path.parent / p).resolve() if not p.is_absolute() else p.resolve())
    calibration_path = resolve(source['calibration_file'])
    calibration = json.loads(Path(calibration_path).read_text())
    validate_calibration(calibration)
    if not isinstance(source.get('environment_label'), str) or not source['environment_label'].strip():
        raise ValueError('environment_label must be filled in for this environment')
    temperature = source.get('temperature_c')
    if isinstance(temperature, bool) or not isinstance(temperature, (float, int)) or not np.isfinite(temperature):
        raise ValueError('temperature_c must be a manually measured finite temperature')
    original_channels = copy.deepcopy(source['channels'])
    # Reuse common acquisition validation without changing its five-level contract.
    common = copy.deepcopy(source)
    for axis in 'XYZ':
        entry = common['channels'][axis]
        levels = entry['levels_v']
        if not isinstance(levels, list) or len(levels) != 3:
            raise ValueError('%s requires three increasing positive levels_v' % axis)
        for v in levels:
            positive(v, axis + '.levels_v')
        if not levels[0] < levels[1] < levels[2]:
            raise ValueError('%s levels_v must be strictly increasing' % axis)
        combo = entry.setdefault('combination_v', levels[1])
        positive(combo, axis + '.combination_v')
        original_channels[axis]['combination_v'] = combo
        entry['levels_v'] = np.linspace(levels[0], levels[-1], 5).tolist()
        entry['validation_v'] = combo
    common['reference_id'] = calibration['reference_id']
    common.setdefault('settle_s', 3.)
    config = validate_config(common)
    if config['short_window_s'] > config['capture_s'] - 2 * config['clock_guard_s']:
        raise ValueError('short_window_s must fit inside the guarded capture window')
    config['channels'] = original_channels
    for key in ('array_geometry', 'output_root'):
        config[key] = resolve(config[key])
    if config['device_id'] != calibration['device_id'] or config['firmware_device_id'] != calibration['firmware_device_id']:
        raise ValueError('calibration device identity mismatch')
    if config['sensor_mode'] != calibration['sensor_mode']:
        raise ValueError('sensor_mode differs from frozen calibration; confirm hardware mode')
    geometry = json.loads(Path(config['array_geometry']).read_text())
    q = {}
    for group in geometry['R_CORR']:
        matrix = np.asarray(group['matrix'], dtype=float).reshape(3, 3)
        for sid in group['sensor_ids']:
            if type(sid) is not int or sid in q or sid not in IDS:
                raise ValueError('invalid or duplicate geometry sensor ID')
            q[sid] = matrix
    sensors = calibration['sensors']
    positions = np.asarray(geometry['d_list'], dtype=float)
    if set(q) != set(IDS) or positions.shape != (12, 3):
        raise ValueError('geometry must cover all 12 sensors')
    for s in sensors:
        sid = s['sensor_id']
        if not np.array_equal(q[sid], s['Q']) or not np.array_equal(positions[sid - 1], s['position_m']):
            raise ValueError('current geometry differs from frozen calibration: sensor %s' % sid)
    thresholds = config.setdefault('performance_limits', {})
    if not isinstance(thresholds, dict):
        raise ValueError('performance_limits must be a mapping')
    for k, v in thresholds.items():
        if k not in ('axis_rmse_ut', 'vector_rmse_ut', 'max_pairwise_ut', 'maps_gradient_frobenius_ut_m'):
            raise ValueError('unknown performance limit: ' + k)
        if v is not None:
            positive(v, k)
    config.update(calibration_file=calibration_path, calibration_sha256=digest(calibration_path),
                  calibration=calibration, sensors=copy.deepcopy(sensors),
                  geometry_sha256=digest(config['array_geometry']), maps_method=MAPS_METHOD,
                  maps_source=MAPS_SOURCE,
                  maps_source_sha256=digest(MAPS_SOURCE) if Path(MAPS_SOURCE).is_file() else None)
    # Check gradient identifiability before touching equipment.
    maps_local(np.zeros((12, 3)), [s['position_m'] for s in sensors])
    return config


def validate_calibration(c):
    if c.get('schema_version') != 1 or c.get('data_valid') is not True:
        raise ValueError('requires a completed, data-valid schema v1 calibration')
    if c.get('nominal_ut_per_count') != 1.1 or c.get('reference_id') not in IDS:
        raise ValueError('unsupported calibration scale/reference')
    if [s['sensor_id'] for s in c['sensors']] != list(IDS):
        raise ValueError('calibration sensors must cover IDs 1..12 in order')
    for s in c['sensors']:
        sid = s['sensor_id']
        if s['bus'] != (sid - 1) // 6 + 1 or s['mask'] != 1 << ((sid - 1) % 6 + 1):
            raise ValueError('unsupported calibration sensor address')
        for key in ('Q', 'T'):
            m = np.asarray(s[key], dtype=float)
            if m.shape != (3, 3) or not np.isfinite(m).all() or np.linalg.matrix_rank(m) != 3:
                raise ValueError('invalid calibration matrix ' + key)
            if key == 'Q' and (not np.allclose(m @ m.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(m), 1)):
                raise ValueError('Q must be a proper rotation')
        pos = np.asarray(s['position_m'], dtype=float)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise ValueError('invalid frozen position')
    if not np.allclose(c['sensors'][c['reference_id'] - 1]['T'], np.eye(3)):
        raise ValueError('reference correction must be identity')


def make_validation_plan(config):
    plan = []
    for level in range(3):
        for axis in 'XYZ':
            for sign in (1, -1):
                plan.append(dict(axes=axis, level=level + 1, polarity=sign,
                                 commands_v={axis: sign * config['channels'][axis]['levels_v'][level]}))
    for axes in ('XY', 'XZ', 'YZ'):
        for sign in (1, -1):
            plan.append(dict(axes=axes, level='combination', polarity=sign,
                             commands_v={a: sign * config['channels'][a]['combination_v'] for a in axes}))
    return [dict(t, trial_id=i, split='validation', repeat=1) for i, t in enumerate(plan, 1)]


def maps_local(vectors, positions):
    """Same local estimator as maps_estimator.py; B in uT, D in metres."""
    b = np.asarray(vectors, dtype=float).T
    d = np.asarray(positions, dtype=float).T
    if d.ndim != 2 or d.shape[0] != 3 or b.shape != d.shape or not np.isfinite(b).all() or not np.isfinite(d).all():
        raise ValueError('finite matching Nx3 vectors and positions required')
    n = d.shape[1]
    _, singular, vh = np.linalg.svd(d, full_matrices=True)
    rank = int(np.sum(singular > 1e-10 * singular[0])) if singular.size else 0
    q = vh[rank:].T
    ones = np.ones((n, 1))
    if q.size == 0:
        if np.linalg.norm(d @ ones) >= 1e-10:
            raise ValueError('cannot construct MaPS local-field weight')
        w = ones / n
    else:
        g = q.T @ ones
        g2 = float((g.T @ g).item())
        if g2 < 1e-10:
            raise ValueError('MaPS local field is not identifiable')
        w = q @ g / g2
    s = np.array([[1,0,0,0,0], [0,1,0,0,0], [0,0,1,0,0],
                  [0,1,0,0,0], [0,0,0,1,0], [0,0,0,0,1],
                  [0,0,1,0,0], [0,0,0,0,1], [-1,0,0,-1,0]], dtype=float)
    design = np.kron(d.T, np.eye(3)) @ s
    if np.linalg.matrix_rank(design, tol=1e-10) < 5:
        raise ValueError('MaPS gradient is not identifiable: rank(C) < 5')
    x5 = np.linalg.pinv(design) @ (b @ (np.eye(n) - w @ ones.T)).reshape(-1, order='F')
    x = (s @ x5).reshape(3, 3, order='F')
    return dict(local_field_ut=(b @ w).ravel().tolist(), maps_X_ut_m=x.tolist(),
                maps_x5_ut_m=x5.tolist(), maps_gradient_frobenius_ut_m=float(np.linalg.norm(x)),
                maps_design_condition=float(np.linalg.cond(design)))


def field_metrics(vectors, config):
    vectors = np.asarray(vectors, dtype=float)
    centered = vectors - vectors.mean(axis=0)
    residual = vectors - vectors[config['reference_id'] - 1]
    result = dict(axis_rmse_ut=np.sqrt(np.mean(centered ** 2, axis=0)).tolist(),
                  vector_rmse_ut=float(np.sqrt(np.mean(np.sum(centered ** 2, axis=1)))),
                  max_pairwise_ut=float(np.linalg.norm(vectors[:, None] - vectors[None, :], axis=2).max()),
                  reference_field_ut=vectors[config['reference_id'] - 1].tolist(),
                  reference_field_norm_ut=float(np.linalg.norm(vectors[config['reference_id'] - 1])),
                  sensor_residual_ut=residual.tolist(), sensor_residual_norm_ut=np.linalg.norm(residual, axis=1).tolist())
    result.update(maps_local(vectors, [s['position_m'] for s in config['sensors']]))
    return result


def evaluate_vectors(vectors, config):
    corrected = [np.asarray(s['T']) @ v for s, v in zip(config['sensors'], vectors)]
    return dict(before=field_metrics(vectors, config), after=field_metrics(corrected, config))


def evaluate_trial(rows, config):
    rows = sorted(rows, key=lambda r: r['sensor_id'])
    if [r['sensor_id'] for r in rows] != list(IDS):
        raise ValueError('trial must contain every sensor exactly once')
    result = {k: rows[0][k] for k in ('trial_id', 'attempt', 'axes', 'level', 'polarity', 'commands_v')}
    for quantity in ('increment', 'total_pre', 'total_on', 'total_post'):
        vectors = [r['delta'] if quantity == 'increment' else r[quantity[6:]]['mean'] for r in rows]
        result[quantity] = evaluate_vectors(vectors, config)
    result['background_change_ut'] = [float(np.linalg.norm(np.array(r['post']['mean']) - r['pre']['mean'])) for r in rows]
    return result


def evaluate_short(records, pre, post, start, end, trial_id, phase, config):
    width = config['short_window_s']
    results = []
    for index in range(int((end - start) / width + 1e-8)):
        measurements = []
        for sid in IDS:
            good = [r for r in records if r['sensor_id'] == sid and r['valid'] and r.get('comm_ok', True)
                    and r['drdy'] and not r['err'] and start + index * width <= r['t'] < start + (index + 1) * width]
            measurements.append(dict(sensor_id=sid, n=len(good),
                                     mean=np.mean([r['x'] for r in good], axis=0).tolist() if good else None,
                                     t=float(np.mean([r['t'] for r in good])) if good else None))
        results.append(evaluate_short_measurements(dict(trial_id=trial_id, phase=phase,
                       short_window_index=index, duration_s=width, measurements=measurements), pre, post, config))
    return results


def evaluate_short_measurements(snapshot, pre, post, config):
    """Exact current metrics from window means; no decimation of short windows."""
    measurements = snapshot['measurements']
    if [m['sensor_id'] for m in measurements] != list(IDS):
        raise ValueError('short-window measurements must cover IDs 1..12')
    for m in measurements:
        if type(m['n']) is not int or m['n'] < 0:
            raise ValueError('invalid short-window sample count')
        if m['n'] and (np.shape(m['mean']) != (3,) or not np.isfinite(m['mean']).all() or not np.isfinite(m['t'])):
            raise ValueError('invalid short-window measurement')
    reason = 'missing sensor samples' if any(not m['n'] for m in measurements) else ''
    row = dict(snapshot, accepted=not reason, reason=reason, total=None, increment=None)
    if not reason:
        row['total'] = evaluate_vectors([m['mean'] for m in measurements], config)
        if snapshot['phase'] == 'ON_CAPTURE':
            delta = [build_delta(pre[m['sensor_id']], m, post[m['sensor_id']])['delta'] for m in measurements]
            row['increment'] = evaluate_vectors(delta, config)
    return row


def summarize_short(rows):
    summary = {}
    for phase in PHASES:
        for quantity in ('increment', 'total'):
            selected = [r[quantity] for r in rows if r['phase'] == phase and r['accepted'] and r.get(quantity) is not None]
            if not selected:
                continue
            group = {}
            for stage in ('before', 'after'):
                group[stage] = {key: dict(zip(('median', 'p95', 'max'), np.percentile(
                    [r[stage][key] for r in selected], [50, 95, 100], axis=0).tolist()))
                    for key in ('axis_rmse_ut', 'vector_rmse_ut', 'max_pairwise_ut', 'maps_gradient_frobenius_ut_m')}
            summary[phase + '/' + quantity] = dict(count=len(selected), **group)
    return summary


def performance(trials, short, config):
    limits = {k: v for k, v in config.get('performance_limits', {}).items() if v is not None}
    if not limits:
        return dict(long=None, short=None)
    long_values = [t[q]['after'] for t in trials for q in ('increment', 'total_pre', 'total_on', 'total_post')]
    short_values = [r[q]['after'] for r in short if r['accepted'] for q in ('increment', 'total') if r.get(q) is not None]
    def passed(values):
        return bool(values) and all(np.max(v[k]) <= limit for v in values for k, limit in limits.items())
    return dict(long=passed(long_values), short=all(r['accepted'] for r in short) and passed(short_values))
