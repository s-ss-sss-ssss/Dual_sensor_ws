"""Calibration state machine, durable run records, and offline replay."""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import threading

import numpy as np
import yaml

from .consistency import (IDS, PHASES, align_trials, analyze_trials, build_delta, choose_reference,
                          config_hash, make_plan, window_statistics)


def dump_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


class RunStore:
    """CSV payloads use JSON for arrays/objects; files are flushed per record batch."""
    def __init__(self, config):
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.run_id = 'run_' + stamp
        self.path = Path(config['output_root']) / self.run_id
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / 'config.yaml').write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding='utf-8')
        self.streams = {}
        self.writers = {}
        self.summaries = []
        self.windows = []
        self.reference_id = None
        self.lock = threading.RLock()

    def write(self, name, rows):
        with self.lock:
            self._write(name, rows)

    def _write(self, name, rows):
        for row in rows:
            if name not in self.streams:
                stream = (self.path / name).open('x', newline='', encoding='utf-8')
                self.streams[name] = stream
                self.writers[name] = csv.DictWriter(stream, fieldnames=list(row))
                self.writers[name].writeheader()
            self.writers[name].writerow({k: json.dumps(v, ensure_ascii=False, allow_nan=False)
                                        if isinstance(v, (list, dict, bool)) else v for k, v in row.items()})
        if name in self.streams:
            self.streams[name].flush()

    def event(self, kind, detail, t=None):
        self.write('events.csv', [dict(t_monotonic=time.monotonic() if t is None else t,
                                       event=kind, detail=detail)])

    def add_window(self, start, end, trial_id, split, phase, attempt):
        self.windows.append(dict(start=start, end=end, trial_id=trial_id, split=split,
                                 phase=phase, attempt=attempt))
        # Saved independently so late-arriving samples can always be reassigned offline.
        dump_json(self.path / 'windows.json', self.windows)

    def record_samples(self, records, window):
        rows = []
        for s in records:
            row = dict(run_id=self.run_id, trial_id=window['trial_id'], split=window['split'],
                       phase=window['phase'], attempt=window['attempt'])
            row.update(s)
            rows.append(row)
        self.write('samples.csv', rows)

    def summary(self, rows):
        self.summaries.extend(rows)
        self.write('trial_summary.csv', rows)

    def save_result(self, result):
        result.update(calibration_id=self.run_id, created_utc=datetime.now(timezone.utc).isoformat())
        self.write('validation.csv', result['validation'])
        dump_json(self.path / 'calibration.json', result)
        performance = {True: '通过已配置门限', False: '未通过已配置门限', None: '未设置性能门限，仅报告改善'}[result['performance_pass']]
        lines = ['# AK09973D 相对一致性标定', '', '参考传感器：%s；训练 trial：%s；独立验证 trial：%s。' % (
            result['reference_id'], len(result['training_trial_ids']), len(result['validation'])),
            '', '数据与求解有效；性能评价：%s。' % performance,
            '', '仅输出结果，未修改在线校正参数。参考传感器的公共畸变仍保留，未辨识绝对零偏。',
            '参考选择仅依据启动短窗，不代表长期重复性。温度缺测。',
            '设备时间通过主机到达时间估计映射，非硬件同步；边界保护和残差阈值见配置。',
            '', '| 传感器 | 条件数 | 训练向量 RMSE (μT) |', '| --- | --- | --- |']
        lines.extend('| %s | %.5g | %.5g |' % (s['sensor_id'], s['condition'], s['training_vector_rmse_ut']) for s in result['sensors'])
        lines.extend(['', '| 验证 trial / 方向 / 极性 | RMSE 前→后 (μT) | 最大通道差 前→后 (μT) | 伪梯度 前→后 (μT/m) |',
                      '| --- | --- | --- | --- |'])
        for e in result['validation']:
            before, after = e['before'], e['after']
            lines.append('| %s / %s / %s | %.5g → %.5g | %.5g → %.5g | %s → %s |' % (
                e['trial_id'], e['axes'], e['polarity'], before['vector_rmse_ut'], after['vector_rmse_ut'],
                before['max_pairwise_ut'], after['max_pairwise_ut'], before['gradient_frobenius_ut_m'], after['gradient_frobenius_ut_m']))
        lines.extend(['', '短窗指标见 short_validation.csv；短窗共用平台背景，不是完全独立重复。',
                      '噪声、漂移和背景门限为 null 时对应检查仅报告统计，不宣称平台稳定性已达项目指标。',
                      '失败、重采、原始样本与状态回读见 events.csv、trial_summary.csv 和 samples.csv。'])
        (self.path / 'report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def fail(self, reason):
        self.event('FAILED', str(reason))
        (self.path / 'report.md').write_text('# 标定未完成\n\n%s\n\n未生成标定参数；原始记录已保留。\n' % reason, encoding='utf-8')

    def close(self):
        with self.lock:
            for stream in self.streams.values():
                stream.close()


class Collector:
    """Backend supplies monotonic now/sleep, all_off, set_output, read_samples.

    read_samples(start,end) returns raw records incl. x and mapped t. The backend
    archives transition/invalid/late samples as well, without using arrival phase.
    """
    def __init__(self, config, backend, store):
        self.config, self.backend, self.store = config, backend, store
        self.validation_windows = {}

    def state(self, name, detail=None):
        self.store.event(name, detail or {}, self.backend.now())

    def capture(self, trial, attempt, phase, duration):
        c, b = self.config, self.backend
        start = b.now()
        self.state(phase, dict(trial_id=trial['trial_id'], attempt=attempt))
        self.store.add_window(start + c['clock_guard_s'], start + duration - c['clock_guard_s'],
                              trial['trial_id'], trial['split'], phase, attempt)
        window = self.store.windows[-1]
        # Boundaries are fixed before sampling, extended only at the right edge.
        elapsed = duration
        while True:
            b.sleep(max(0., start + elapsed - b.now()))
            end = start + elapsed - c['clock_guard_s']
            window['end'] = end
            records = b.read_samples(window['start'], end)
            stats = {}
            reasons = []
            for sid in IDS:
                try:
                    stats[sid] = window_statistics([s for s in records if s['sensor_id'] == sid], window['start'], end, c)
                except ValueError as error:
                    reasons.append('%s: %s' % (sid, error))
            if not reasons or elapsed >= c['max_capture_s']:
                dump_json(self.store.path / 'windows.json', self.store.windows)
                self.store.record_samples(records, window)
                if reasons:
                    self.state('WINDOW_REJECTED', dict(window=window, reasons=reasons, statistics=stats))
                    raise ValueError('; '.join(reasons))
                return stats, records
            # Only insufficient data warrants extension. Drift/noise requires a new trial.
            if any('insufficient effective samples' not in reason for reason in reasons):
                self.store.record_samples(records, window)
                self.state('WINDOW_REJECTED', dict(window=window, reasons=reasons, statistics=stats))
                raise ValueError('; '.join(reasons))
            elapsed = min(c['max_capture_s'], elapsed + c['subwindow_s'])
            self.state('EXTEND_WINDOW', dict(phase=phase, duration_s=elapsed))

    def collect_trial(self, trial, attempt):
        b, c = self.backend, self.config
        self.partial = {}
        b.all_off()
        self.state('OFF_PRE_SETTLE')
        b.sleep(c['settle_s'])
        pre, _ = self.capture(trial, attempt, PHASES[0], c['capture_s'])
        self.partial['pre'] = pre
        b.set_output(trial['commands_v'])
        self.state('ON_SETTLE')
        b.sleep(c['settle_s'])
        on, records = self.capture(trial, attempt, PHASES[1], c['capture_s'])
        self.partial['on'] = on
        b.all_off()
        self.state('OFF_POST_SETTLE')
        b.sleep(c['settle_s'])
        post, _ = self.capture(trial, attempt, PHASES[2], c['capture_s'])
        self.partial['post'] = post
        self.state('PROCESS_TRIAL')
        rows = []
        for sid in IDS:
            delta = build_delta(pre[sid], on[sid], post[sid], c['max_background_change_ut'])
            rows.append(dict(trial, attempt=attempt, sensor_id=sid, accepted=True, reason='',
                             pre=pre[sid], on=on[sid], post=post[sid], **delta))
        if trial['split'] == 'validation':
            self.validation_windows[trial['trial_id']] = (records, pre, post, on[1]['start'], on[1]['end'])
        return rows

    def short_validation(self, result):
        from .consistency import metrics
        positions = [s['position_m'] for s in self.config['sensors']]
        matrices = {s['sensor_id']: np.array(s['T']) for s in result['sensors']}
        rows = []
        width = self.config['short_window_s']
        for trial_id, (records, pre, post, start, end) in self.validation_windows.items():
            count = int((end - start) / width + 1e-8)
            for index in range(count):
                vectors, reason = [], ''
                for sid in IDS:
                    valid = [r for r in records if r['sensor_id'] == sid and r['valid'] and r.get('comm_ok', True) and r['drdy'] and not r['err']
                             and start + index * width <= r['t'] < start + (index + 1) * width]
                    if not valid:
                        reason = 'missing sensor samples'
                        break
                    on = dict(t=float(np.mean([r['t'] for r in valid])), mean=np.mean([r['x'] for r in valid], axis=0).tolist())
                    vectors.append(build_delta(pre[sid], on, post[sid])['delta'])
                before = after = None
                if not reason:
                    before = metrics(vectors, positions)
                    after = metrics([matrices[sid] @ vectors[sid - 1] for sid in IDS], positions)
                rows.append(dict(trial_id=trial_id, short_window_index=index, duration_s=width,
                                 accepted=not reason, reason=reason, before=before, after=after))
        self.store.write('short_validation.csv', rows)

    def run(self, smoke_test=False):
        c, b = self.config, self.backend
        try:
            b.prepare()
            b.all_off()
            b.sleep(c['settle_s'])
            self.state('STARTUP_CHECK', 'Preheated manually; no warmup wait.')
            stats, _ = self.capture(dict(trial_id=0, split='startup'), 0, 'STARTUP_CAPTURE', c['startup_s'])
            reference = choose_reference(stats, c['reference_id'])
            self.store.reference_id = reference
            dump_json(self.store.path / 'startup.json', dict(reference_id=reference, statistics=stats))
            self.state('REFERENCE_FROZEN', dict(sensor_id=reference))
            plan = make_plan(c)
            if smoke_test:
                voltage = c['channels']['X']['levels_v'][0]
                plan = [dict(trial_id=i, split='smoke', axes='X', level=1, polarity=sign, repeat=1,
                             commands_v={'X': sign * voltage}) for i, sign in enumerate((1, -1), 1)]
            for trial in plan:
                for attempt in range(c['retries'] + 1):
                    self.state('TRIAL', dict(trial, attempt=attempt))
                    try:
                        rows = self.collect_trial(trial, attempt)
                    except ValueError as error:
                        self.store.summary([dict(trial, attempt=attempt, sensor_id=sid, accepted=False,
                                                  reason=str(error), **{name: self.partial.get(name, {}).get(sid)
                                                                       for name in ('pre', 'on', 'post')},
                                                  alpha=None, background=None, delta=None) for sid in IDS])
                        self.state('RETRY', str(error))
                        b.all_off()
                        if attempt == c['retries']:
                            raise RuntimeError('trial %s exhausted retries: %s' % (trial['trial_id'], error)) from error
                    else:
                        self.store.summary(rows)
                        self.state('ACCEPT', dict(trial_id=trial['trial_id'], attempt=attempt))
                        break
            if smoke_test:
                b.all_off()
                (self.store.path / 'report.md').write_text(
                    '# 单轴最低档正负试采完成\n\n仅采集 X 轴最低档正负两个 trial，未拟合标定矩阵。\n'
                    '请核对实际输出电平、极性与传感器响应后再执行完整实验。\n', encoding='utf-8')
                self.state('SMOKE_COMPLETE', {})
                return None
            result = analyze_trials(c, self.store.summaries, reference)
            self.short_validation(result)
            # Confirm output shutdown before publishing the successful artifact.
            b.all_off()
            self.store.save_result(result)
            self.state('COMPLETE', dict(config_sha256=config_hash(c)))
            return result
        except BaseException as error:
            self.store.fail('%s: %s' % (type(error).__name__, error))
            raise
        finally:
            try:
                b.all_off()
            except BaseException as error:
                self.store.event('SHUTDOWN_FAILED', str(error))
                # A shutdown error must be visible to callers, even during cancellation.
                self.store.fail('ALL_OFF FAILED: %s' % error)
                (self.store.path / 'calibration.json').unlink(missing_ok=True)
                raise RuntimeError('ALL_OFF FAILED: %s' % error) from error


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            yield {key: _decode(value) for key, value in row.items()}


def _decode(value):
    try:
        return json.loads(value)
    except (ValueError, TypeError):
        return value


def refit(run_path):
    """Recompute raw CSV with bounded memory, even when rows are out of order."""
    import sqlite3
    import tempfile
    from .consistency import validate_config
    run_path = Path(run_path)
    with (run_path / 'config.yaml').open(encoding='utf-8') as stream:
        config = validate_config(yaml.safe_load(stream))
    original = json.loads((run_path / 'calibration.json').read_text())
    if original['config_sha256'] != config_hash(config):
        raise ValueError('source configuration hash mismatch')
    reference = json.loads((run_path / 'startup.json').read_text())['reference_id']
    if reference != original['reference_id']:
        raise ValueError('source reference ID mismatch')
    windows = json.loads((run_path / 'windows.json').read_text())
    metadata = {}
    for row in read_csv(run_path / 'trial_summary.csv'):
        key = (row['trial_id'], row['attempt'], row['sensor_id'])
        if key in metadata:
            raise ValueError('duplicate trial summary row')
        metadata[key] = row
    accepted_keys = {(r['trial_id'], r['attempt']) for r in metadata.values() if r['accepted']}
    # Verify the complete frozen plan and sensor alignment before expensive I/O.
    align_trials(config, list(metadata.values()))
    sensors = {s['sensor_id']: s for s in config['sensors']}
    store = RunStore(config)
    try:
        store.event('REFIT_SOURCE', str(run_path.resolve()))
        dump_json(store.path / 'startup.json', dict(reference_id=reference))
        with tempfile.TemporaryDirectory(prefix='ak_refit_') as scratch:
            db = sqlite3.connect(str(Path(scratch) / 'samples.sqlite'))
            try:
                db.execute('CREATE TABLE samples (trial INTEGER, attempt INTEGER, phase TEXT, t REAL, payload TEXT)')
                batch = []
                for row in read_csv(run_path / 'samples.csv'):
                    key = (row['trial_id'], row['attempt'])
                    if key not in accepted_keys:
                        continue
                    sensor = sensors.get(row['sensor_id'])
                    if sensor is None or row['bus'] != sensor['bus'] or row['mask'] != sensor['mask']:
                        raise ValueError('raw sample identity/address mismatch')
                    raw = np.asarray(row['raw'], dtype=float)
                    if raw.shape != (3,) or not np.isfinite(raw).all():
                        raise ValueError('invalid raw vector')
                    row['x'] = (np.asarray(sensor['Q']) @ (raw * 1.1)).tolist()
                    batch.append((key[0], key[1], row['phase'], row['t'], json.dumps(row, allow_nan=False)))
                    if len(batch) >= 1000:
                        db.executemany('INSERT INTO samples VALUES (?,?,?,?,?)', batch)
                        batch.clear()
                db.executemany('INSERT INTO samples VALUES (?,?,?,?,?)', batch)
                db.execute('CREATE INDEX window_key ON samples(trial,attempt,phase,t)')
                def samples(key):
                    return [json.loads(r[0]) for r in db.execute(
                        'SELECT payload FROM samples WHERE trial=? AND attempt=? AND phase=? ORDER BY t', key)]
                stats, seen = {}, set()
                for w in windows:
                    key = (w['trial_id'], w['attempt'], w['phase'])
                    if key[:2] not in accepted_keys:
                        continue
                    if key in seen:
                        raise ValueError('duplicate capture window')
                    seen.add(key)
                    records = samples(key)
                    stats[key] = {sid: window_statistics([r for r in records if r['sensor_id'] == sid],
                                                        w['start'], w['end'], config) for sid in IDS}
                    store.add_window(**{k: w[k] for k in ('start', 'end', 'trial_id', 'split', 'phase', 'attempt')})
                    store.record_samples(records, w)
                rows = []
                for (trial_id, attempt, sid), row in metadata.items():
                    if row['accepted']:
                        pre, on, post = [stats[(trial_id, attempt, phase)][sid] for phase in PHASES]
                        rows.append(dict(row, pre=pre, on=on, post=post,
                                         **build_delta(pre, on, post, config['max_background_change_ut'])))
                result = analyze_trials(config, rows, reference)
                store.summary(rows)
                # Compute short windows one validation platform at a time.
                collector = Collector(config, None, store)
                for w in windows:
                    if w['split'] == 'validation' and w['phase'] == 'ON_CAPTURE' and (w['trial_id'], w['attempt']) in accepted_keys:
                        key = (w['trial_id'], w['attempt'])
                        collector.validation_windows = {key[0]: (samples((*key, PHASES[1])), stats[(*key, PHASES[0])],
                                                                  stats[(*key, PHASES[2])], w['start'], w['end'])}
                        collector.short_validation(result)
                store.save_result(result)
            finally:
                db.close()
        return store.path
    except BaseException as error:
        store.fail('%s: %s' % (type(error).__name__, error))
        raise
    finally:
        store.close()
