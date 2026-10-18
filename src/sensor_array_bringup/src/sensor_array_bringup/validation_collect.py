"""Independent validation acquisition, evidence output and bounded-memory replay."""
from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import tempfile

import numpy as np
import yaml

from .consistency import IDS, PHASES, build_delta, config_hash, window_statistics
from .consistency_collect import Collector, RunStore, dump_json, read_csv
from .consistency_validation import (digest, evaluate_short, evaluate_trial, make_validation_plan,
                                     evaluate_short_measurements, performance, summarize_short, validate_calibration)


QUANTITIES = ('increment', 'total_pre', 'total_on', 'total_post')


class ValidationStore(RunStore):
    def __init__(self, config):
        super().__init__(config)
        self.raw_closed = False
        dump_json(self.path / 'calibration_snapshot.json', config['calibration'])
        dump_json(self.path / 'manifest.json', dict(config_sha256=config_hash(config),
                  calibration_snapshot_sha256=digest(self.path / 'calibration_snapshot.json')))

    def write(self, name, rows):
        with self.lock:
            if self.raw_closed and name in ('raw_stream.csv', 'samples.csv'):
                return
            super().write(name, rows)

    def cleanup_raw(self):
        """Called only after successful shutdown, compact replay check and publication."""
        with self.lock:
            self.raw_closed = True
            removed, errors = [], []
            for name in ('raw_stream.csv', 'samples.csv'):
                stream = self.streams.pop(name, None)
                self.writers.pop(name, None)
                try:
                    if stream is not None:
                        stream.close()
                    path = self.path / name
                    if path.exists():
                        size = path.stat().st_size
                        path.unlink()
                        removed.append(dict(file=name, bytes=size))
                except OSError as error:
                    errors.append('%s: %s' % (name, error))
            status = dict(raw_files_removed=removed, cleanup_errors=errors,
                          offline_source='platform statistics and every short-window measurement mean')
            dump_json(self.path / 'storage.json', status)
            self.event('RAW_CLEANUP_FAILED' if errors else 'RAW_CLEANUP_COMPLETE', status)
            if errors:
                print('Raw cleanup incomplete: %s' % '; '.join(errors), flush=True)

    def fail(self, reason):
        self.event('FAILED', str(reason))
        dump_json(self.path / 'failure.json', dict(data_valid=False, reason=str(reason)))
        (self.path / 'report.md').write_text('# 验证未完成\n\n%s\n\n已有诊断与采样记录已保留，不宣称验证通过。\n' % reason)

    def save_validation(self, config, trials, short, source_run=None):
        result = dict(schema_version=2, validation_id=self.run_id,
                      created_utc=datetime.now(timezone.utc).isoformat(), data_valid=True,
                      calibration_id=config['calibration']['calibration_id'],
                      calibration_sha256=config['calibration_sha256'], geometry_sha256=config['geometry_sha256'],
                      config_sha256=config_hash(config), reference_id=config['reference_id'],
                      environment_label=config['environment_label'], temperature_c=config['temperature_c'],
                      temperature_source='manual environment measurement; not sensor die temperature',
                      notes=config.get('notes', ''), maps_method=config['maps_method'],
                      maps_source=config['maps_source'], maps_source_sha256=config['maps_source_sha256'],
                      source_run=source_run, performance_pass=performance(trials, short, config),
                      trials=trials, short_summary=summarize_short(short),
                      short_windows_rejected=sum(not r['accepted'] for r in short),
                      limitations=['Fixed relative calibration; reference common distortion remains.',
                                   'Total-field correction does not estimate absolute sensor bias.',
                                   'Apparent gradient includes real spatial gradients of the environment.',
                                   'Short windows share long-platform background estimates.',
                                   'Host-estimated device timing, not hardware synchronization.'])
        self.write('validation.csv', trials)
        self.write('short_validation.csv', [{k: v for k, v in r.items() if k != 'measurements'} for r in short])
        snapshots = [{k: r[k] for k in ('trial_id', 'phase', 'short_window_index', 'duration_s', 'measurements')} for r in short]
        dump_json(self.path / 'short_measurements.json', snapshots)
        files = ('trial_summary.csv', 'short_measurements.json', 'windows.json', 'startup.json')
        dump_json(self.path / 'compact_manifest.json', dict(schema_version=1,
                  sha256={name: digest(self.path / name) for name in files}))
        replay_trials, replay_short, _ = rebuild_compact(self.path, config)
        assert_metrics_equal(trials, replay_trials)
        assert_metrics_equal(short, replay_short)
        assert_metrics_equal(summarize_short(short), summarize_short(replay_short))
        if performance(trials, short, config) != performance(replay_trials, replay_short, config):
            raise ValueError('compact replay performance mismatch')
        residuals, gradients = [], []
        for t in trials:
            identity = {k: t[k] for k in ('trial_id', 'axes', 'level', 'polarity', 'commands_v')}
            for quantity in QUANTITIES:
                for stage in ('before', 'after'):
                    m = t[quantity][stage]
                    gradients.append(dict(identity, quantity=quantity, stage=stage,
                                          X_ut_m=m['maps_X_ut_m'], x5_ut_m=m['maps_x5_ut_m'],
                                          frobenius_ut_m=m['maps_gradient_frobenius_ut_m']))
                    for sid in IDS:
                        e = m['sensor_residual_ut'][sid - 1]
                        residuals.append(dict(identity, quantity=quantity, stage=stage, sensor_id=sid,
                                              residual_x_ut=e[0], residual_y_ut=e[1], residual_z_ut=e[2],
                                              residual_norm_ut=m['sensor_residual_norm_ut'][sid - 1]))
        self.write('sensor_residuals.csv', residuals)
        self.write('maps_gradients.csv', gradients)
        plot_results(self.path, trials)
        self.report(result)
        # Result file is the completion marker; only produced after shutdown/plots/report.
        dump_json(self.path / 'result.json', result)
        self.event('COMPLETE', dict(data_valid=True))
        return result

    def report(self, result):
        lines = ['# 独立一致性验证', '',
                 '环境：%s；人工环境温度：%s °C；参考传感器：%s。' %
                 (result['environment_label'], result['temperature_c'], result['reference_id']),
                 '固定标定：%s；24组采集完整，未重新拟合或修改在线参数。' % result['calibration_id'],
                 '性能门限结果（长窗 / 短窗）：%s。null 表示仅报告、未作性能验收。' % result['performance_pass'],
                 '', '残差是相对固定参考的三轴差；阵列RMSE是相对阵列均值的离散程度，均不代表绝对场强误差。',
                 'X采用MaPS对称无迹约束，单位μT/m；环境真实梯度也会进入结果，不能全部归因于传感器误差。',
                 '总场使用 T @ B，未消除绝对零偏；增量使用 T @ (B − 插值背景)。',
                 '人工温度不是芯片温度；100ms短窗共用长平台背景。',
                 '噪声、斜率、背景变化门限未配置时仅记录，不自动证明平台稳定。',
                 '详细质量统计见 trial_summary.csv，前后背景变化见 validation.csv。',
                 '成功完成后清理原始全流和逐样本文件；平台统计与全部短窗测量均值保留，供相同窗口的指标重算。',
                 '精简记录不支持重新划分采样窗口或更换逐样本筛选规则。',
                 '', '![Residuals and MaPS gradients](metrics.png)',
                 '', '![Component residuals versus field strength](axis_residuals.png)', '']
        for quantity in QUANTITIES:
            lines.extend(['## ' + quantity, '',
                          '| 状态 | 参考场模长 μT | 阵列RMSE 前→后 μT | 校正后X/Y/Z RMSE μT | 最大差 前→后 μT | MaPS ‖X‖F 前→后 μT/m |',
                          '| --- | --- | --- | --- | --- | --- |'])
            for t in result['trials']:
                b, a = t[quantity]['before'], t[quantity]['after']
                lines.append('| %s / %s / %s / %s | %.2f | %.3f → %.3f | %s | %.3f → %.3f | %.2f → %.2f |' % (
                    t['trial_id'], t['axes'], t['level'], t['polarity'], a['reference_field_norm_ut'],
                    b['vector_rmse_ut'], a['vector_rmse_ut'], ' / '.join('%.3f' % x for x in a['axis_rmse_ut']),
                    b['max_pairwise_ut'], a['max_pairwise_ut'], b['maps_gradient_frobenius_ut_m'],
                    a['maps_gradient_frobenius_ut_m']))
            lines.append('')
        lines.extend(['## 100ms短窗汇总', '', '被拒绝的短窗：%s。' % result['short_windows_rejected'], ''])
        for name, group in result['short_summary'].items():
            a = group['after']
            lines.append('- %s：%s窗；校正后向量RMSE（中位数/95分位/最大）%s μT；MaPS梯度范数 %s μT/m。' %
                         (name, group['count'], a['vector_rmse_ut'], a['maps_gradient_frobenius_ut_m']))
        (self.path / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def plot_results(path, trials):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for quantity, column in (('increment', 0), ('total_on', 1)):
        for field_axis in 'XYZ':
            selected = [t for t in trials if t['axes'] == field_axis]
            selected.sort(key=lambda t: t[quantity]['after']['reference_field_norm_ut'])
            x = [t[quantity]['after']['reference_field_norm_ut'] for t in selected]
            for stage, style in (('before', '--'), ('after', '-')):
                axes[0, column].plot(x, [t[quantity][stage]['vector_rmse_ut'] for t in selected],
                                     style + '.', label=field_axis + ' ' + stage)
                axes[1, column].plot(x, [t[quantity][stage]['maps_gradient_frobenius_ut_m'] for t in selected],
                                     style + '.', label=field_axis + ' ' + stage)
        axes[0, column].set_title(quantity)
        axes[0, column].set_ylabel('Array vector RMSE (uT)')
        axes[1, column].set_ylabel('MaPS gradient Frobenius norm (uT/m)')
        for row in range(2):
            axes[row, column].set_xlabel('Reference field magnitude (uT)')
            axes[row, column].grid(True)
            axes[row, column].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(str(Path(path) / 'metrics.png'), dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for row, quantity in enumerate(('increment', 'total_on')):
        for column, excitation in enumerate('XYZ'):
            ax = axes[row, column]
            selected = sorted((t for t in trials if t['axes'] == excitation),
                              key=lambda t: t[quantity]['after']['reference_field_norm_ut'])
            fields = [t[quantity]['after']['reference_field_norm_ut'] for t in selected]
            for component, color in enumerate(('tab:blue', 'tab:orange', 'tab:green')):
                for stage, style in (('before', '--'), ('after', '-')):
                    ax.plot(fields, [t[quantity][stage]['axis_rmse_ut'][component] for t in selected],
                            style + '.', color=color, label='XYZ'[component] + ' ' + stage)
            ax.set_title(quantity + ': excitation ' + excitation)
            ax.set_xlabel('Reference field magnitude (uT)')
            ax.set_ylabel('Component array RMSE (uT)')
            ax.grid(True)
            ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(str(Path(path) / 'axis_residuals.png'), dpi=150)
    plt.close(fig)


class ValidationCollector(Collector):
    def __init__(self, config, backend, store):
        super().__init__(config, backend, store)
        self.phase_records = {}

    def capture(self, trial, attempt, phase, duration):
        stats, records = super().capture(trial, attempt, phase, duration)
        self.phase_records[phase] = (stats, records)
        return stats, records

    def run(self):
        c, b = self.config, self.backend
        trials, short = [], []
        try:
            try:
                b.prepare()
                b.all_off()
                b.sleep(c['settle_s'])
                stats, _ = self.capture(dict(trial_id=0, split='startup'), 0, 'STARTUP_CAPTURE', c['startup_s'])
                self.store.reference_id = c['reference_id']
                dump_json(self.store.path / 'startup.json', dict(reference_id=c['reference_id'], statistics=stats))
                for trial in make_validation_plan(c):
                    for attempt in range(c['retries'] + 1):
                        self.phase_records.clear()
                        self.state('TRIAL', dict(trial, attempt=attempt))
                        try:
                            rows = self.collect_trial(trial, attempt)
                        except ValueError as error:
                            self.store.summary([dict(trial, attempt=attempt, sensor_id=sid, accepted=False,
                                                     reason=str(error), pre=self.partial.get('pre', {}).get(sid),
                                                     on=self.partial.get('on', {}).get(sid), post=self.partial.get('post', {}).get(sid),
                                                     alpha=None, background=None, delta=None) for sid in IDS])
                            self.state('RETRY', str(error))
                            b.all_off()
                            if attempt == c['retries']:
                                raise RuntimeError('trial %s exhausted retries: %s' % (trial['trial_id'], error)) from error
                            continue
                        self.store.summary(rows)
                        trials.append(evaluate_trial(rows, c))
                        pre, post = self.phase_records[PHASES[0]][0], self.phase_records[PHASES[2]][0]
                        for phase in PHASES:
                            stats, records = self.phase_records[phase]
                            short.extend(evaluate_short(records, pre, post, stats[1]['start'], stats[1]['end'],
                                                        trial['trial_id'], phase, c))
                        self.validation_windows.clear()
                        self.state('ACCEPT', dict(trial_id=trial['trial_id'], attempt=attempt))
                        break
            finally:
                # Cleanup also runs after cancellation and before publishing any success artifact.
                b.all_off()
            result = self.store.save_validation(c, trials, short)
            self.store.cleanup_raw()
            return result
        except BaseException as error:
            self.store.fail('%s: %s' % (type(error).__name__, error))
            raise


def assert_metrics_equal(expected, actual):
    """Reject any loss in current metrics before raw files become disposable."""
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or expected.keys() != actual.keys():
            raise ValueError('compact replay keys mismatch')
        for key in expected:
            assert_metrics_equal(expected[key], actual[key])
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            raise ValueError('compact replay length mismatch')
        for a, b in zip(expected, actual):
            assert_metrics_equal(a, b)
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        if not np.isclose(expected, actual, rtol=1e-12, atol=1e-9):
            raise ValueError('compact replay numeric mismatch')
    elif expected != actual:
        raise ValueError('compact replay value mismatch')


def rebuild_compact(path, config):
    """Recompute every existing metric from full-resolution window statistics."""
    path = Path(path)
    manifest = json.loads((path / 'compact_manifest.json').read_text())
    files = ('trial_summary.csv', 'short_measurements.json', 'windows.json', 'startup.json')
    if manifest.get('schema_version') != 1 or set(manifest['sha256']) != set(files):
        raise ValueError('unsupported compact measurements manifest')
    if any(digest(path / name) != manifest['sha256'][name] for name in files):
        raise ValueError('compact measurements hash mismatch')
    if json.loads((path / 'startup.json').read_text())['reference_id'] != config['reference_id']:
        raise ValueError('compact reference mismatch')
    rows = list(read_csv(path / 'trial_summary.csv'))
    plan = make_validation_plan(config)
    accepted_rows, groups, trials = [], {}, []
    for t in plan:
        group = sorted((r for r in rows if r['accepted'] and r['trial_id'] == t['trial_id']), key=lambda r: r['sensor_id'])
        if [r['sensor_id'] for r in group] != list(IDS) or len({r['attempt'] for r in group}) != 1:
            raise ValueError('compact trial is incomplete or duplicated')
        for r in group:
            if any(r[k] != v for k, v in t.items()):
                raise ValueError('compact trial metadata mismatch')
            r.update(build_delta(r['pre'], r['on'], r['post'], config['max_background_change_ut']))
        accepted_rows.extend(group)
        groups[t['trial_id']] = group
        trials.append(evaluate_trial(group, config))
    if sum(bool(r['accepted']) for r in rows) != len(accepted_rows):
        raise ValueError('unexpected compact trial')
    windows = {}
    for w in json.loads((path / 'windows.json').read_text()):
        if w['trial_id'] in groups and w['attempt'] == groups[w['trial_id']][0]['attempt']:
            key = (w['trial_id'], w['phase'])
            if key in windows:
                raise ValueError('duplicate compact window')
            windows[key] = w
    if set(windows) != {(t['trial_id'], ph) for t in plan for ph in PHASES}:
        raise ValueError('incomplete compact windows')
    expected = {(tid, ph, i) for (tid, ph), w in windows.items()
                for i in range(int((w['end'] - w['start']) / config['short_window_s'] + 1e-8))}
    seen, short = set(), []
    for snapshot in json.loads((path / 'short_measurements.json').read_text()):
        key = (snapshot['trial_id'], snapshot['phase'], snapshot['short_window_index'])
        if key in seen or key not in expected or snapshot['duration_s'] != config['short_window_s']:
            raise ValueError('duplicate or invalid compact short window')
        seen.add(key)
        group = groups[snapshot['trial_id']]
        pre = {r['sensor_id']: r['pre'] for r in group}
        post = {r['sensor_id']: r['post'] for r in group}
        short.append(evaluate_short_measurements(snapshot, pre, post, config))
    if seen != expected:
        raise ValueError('incomplete compact short windows')
    short.sort(key=lambda r: (r['trial_id'], PHASES.index(r['phase']), r['short_window_index']))
    return trials, short, accepted_rows


def reanalyze(run_path):
    run_path = Path(run_path).expanduser().resolve()
    c = yaml.safe_load((run_path / 'config.yaml').read_text())
    manifest = json.loads((run_path / 'manifest.json').read_text())
    original = json.loads((run_path / 'result.json').read_text())
    if original.get('data_valid') is not True or original['config_sha256'] != config_hash(c) or manifest['config_sha256'] != config_hash(c):
        raise ValueError('requires complete validation with matching frozen config hash')
    if original.get('schema_version', 1) >= 2 and not (run_path / 'compact_manifest.json').is_file():
        raise ValueError('compact measurements manifest missing')
    if manifest['calibration_snapshot_sha256'] != digest(run_path / 'calibration_snapshot.json'):
        raise ValueError('calibration snapshot hash mismatch')
    if json.loads((run_path / 'calibration_snapshot.json').read_text()) != c['calibration']:
        raise ValueError('embedded calibration differs from snapshot')
    validate_calibration(c['calibration'])
    metadata = {}
    for row in read_csv(run_path / 'trial_summary.csv'):
        if row['accepted']:
            key = (row['trial_id'], row['attempt'], row['sensor_id'])
            if key in metadata:
                raise ValueError('duplicate trial summary')
            metadata[key] = row
    plan = make_validation_plan(c)
    accepted = {}
    for t in plan:
        group = [r for k, r in metadata.items() if k[0] == t['trial_id']]
        if len(group) != 12 or {r['sensor_id'] for r in group} != set(IDS) or len({r['attempt'] for r in group}) != 1:
            raise ValueError('incomplete or duplicate accepted trial')
        if any(any(r[k] != v for k, v in t.items()) for r in group):
            raise ValueError('trial metadata does not match frozen plan')
        accepted[t['trial_id']] = group[0]['attempt']
    if len(metadata) != 24 * 12:
        raise ValueError('unexpected accepted trial metadata')
    windows = {}
    for w in json.loads((run_path / 'windows.json').read_text()):
        if accepted.get(w['trial_id']) != w['attempt']:
            continue
        key = (w['trial_id'], w['phase'])
        if key in windows:
            raise ValueError('duplicate window')
        windows[key] = w
    if set(windows) != {(t['trial_id'], ph) for t in plan for ph in PHASES}:
        raise ValueError('incomplete window metadata')
    sensors = {s['sensor_id']: s for s in c['sensors']}
    store = ValidationStore(c)
    trials, short = [], []
    try:
        store.event('REANALYZE_SOURCE', str(run_path))
        dump_json(store.path / 'startup.json', json.loads((run_path / 'startup.json').read_text()))
        if (run_path / 'compact_manifest.json').is_file():
            trials, short, rows = rebuild_compact(run_path, c)
            store.summary(rows)
            for w in json.loads((run_path / 'windows.json').read_text()):
                store.add_window(**w)
            store.save_validation(c, trials, short, source_run=str(run_path))
            store.cleanup_raw()
            return store.path
        with tempfile.TemporaryDirectory(prefix='ak_validation_') as scratch:
            db = sqlite3.connect(str(Path(scratch) / 'samples.sqlite'))
            try:
                db.execute('CREATE TABLE samples (trial INTEGER, phase TEXT, t REAL, payload TEXT)')
                batch = []
                for r in read_csv(run_path / 'samples.csv'):
                    if accepted.get(r['trial_id']) != r['attempt']:
                        continue
                    s = sensors.get(r['sensor_id'])
                    if s is None or r['bus'] != s['bus'] or r['mask'] != s['mask']:
                        raise ValueError('raw sample identity/address mismatch')
                    raw = np.asarray(r['raw'], dtype=float)
                    if raw.shape != (3,) or not np.isfinite(raw).all():
                        raise ValueError('invalid raw vector')
                    r['x'] = (np.asarray(s['Q']) @ (raw * c['calibration']['nominal_ut_per_count'])).tolist()
                    if np.any(raw <= -32768) or np.any(raw >= 32767):
                        r['valid'] = False
                    batch.append((r['trial_id'], r['phase'], r['t'], json.dumps(r, allow_nan=False)))
                    if len(batch) >= 1000:
                        db.executemany('INSERT INTO samples VALUES (?,?,?,?)', batch)
                        batch.clear()
                db.executemany('INSERT INTO samples VALUES (?,?,?,?)', batch)
                db.execute('CREATE INDEX window_key ON samples(trial,phase,t)')
                for t in plan:
                    phases = {}
                    for phase in PHASES:
                        w = windows[(t['trial_id'], phase)]
                        records = [json.loads(r[0]) for r in db.execute(
                            'SELECT payload FROM samples WHERE trial=? AND phase=? ORDER BY t', (t['trial_id'], phase))]
                        stats = {sid: window_statistics([r for r in records if r['sensor_id'] == sid], w['start'], w['end'], c) for sid in IDS}
                        phases[phase] = (stats, records)
                        store.add_window(**w)
                        store.record_samples(records, w)
                    pre, on, post = [phases[ph][0] for ph in PHASES]
                    rows = [dict(t, attempt=accepted[t['trial_id']], sensor_id=sid, accepted=True, reason='',
                                 pre=pre[sid], on=on[sid], post=post[sid],
                                 **build_delta(pre[sid], on[sid], post[sid], c['max_background_change_ut'])) for sid in IDS]
                    store.summary(rows)
                    trials.append(evaluate_trial(rows, c))
                    for phase in PHASES:
                        w = windows[(t['trial_id'], phase)]
                        short.extend(evaluate_short(phases[phase][1], pre, post, w['start'], w['end'], t['trial_id'], phase, c))
            finally:
                db.close()
        store.save_validation(c, trials, short, source_run=str(run_path))
        store.cleanup_raw()
        return store.path
    except BaseException as error:
        store.fail(str(error))
        raise
    finally:
        store.close()


def compare_runs(paths):
    """Return aligned rows without pooling distinct calibrations/geometries."""
    rows = []
    for path in paths:
        path = Path(path).expanduser().resolve()
        r = json.loads((path / 'result.json').read_text())
        if r.get('data_valid') is not True:
            raise ValueError('comparison requires complete valid runs')
        for t in r['trials']:
            for quantity in QUANTITIES:
                for stage in ('before', 'after'):
                    m = t[quantity][stage]
                    rows.append(dict(run=str(path), environment_label=r['environment_label'], temperature_c=r['temperature_c'],
                                     calibration_sha256=r['calibration_sha256'], geometry_sha256=r['geometry_sha256'],
                                     axes=t['axes'], polarity=t['polarity'], commands_v=t['commands_v'],
                                     quantity=quantity, stage=stage, reference_field_ut=m['reference_field_ut'],
                                     reference_field_norm_ut=m['reference_field_norm_ut'],
                                     axis_rmse_ut=m['axis_rmse_ut'], vector_rmse_ut=m['vector_rmse_ut'],
                                     max_pairwise_ut=m['max_pairwise_ut'], maps_X_ut_m=m['maps_X_ut_m'],
                                     maps_gradient_frobenius_ut_m=m['maps_gradient_frobenius_ut_m']))
    return sorted(rows, key=lambda r: (r['calibration_sha256'], r['geometry_sha256'], r['axes'],
                                      json.dumps(r['commands_v'], sort_keys=True), r['quantity'], r['stage'],
                                      r['temperature_c'], r['environment_label'], r['run']))
