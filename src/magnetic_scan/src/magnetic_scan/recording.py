"""Append-only experiment evidence, including partially completed points."""
import json
import time
from pathlib import Path
from uuid import uuid4
import numpy as np
import yaml


def serializable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class Recorder:
    def __init__(self, config, points):
        self.path = Path(config['output_dir']) / (time.strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8])
        self.path.mkdir(parents=True)
        (self.path / 'config.yaml').write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')
        (self.path / 'points.json').write_text(json.dumps(points, indent=2), encoding='utf-8')
        self.events = (self.path / 'events.jsonl').open('a', encoding='utf-8')
        self.points = (self.path / 'results.jsonl').open('a', encoding='utf-8')

    def event(self, kind, **data):
        self.events.write(json.dumps(dict(time=time.time(), event=kind, **data), default=serializable, allow_nan=False) + '\n')
        self.events.flush()

    def point(self, data):
        self.points.write(json.dumps(data, default=serializable, allow_nan=False) + '\n')
        self.points.flush()

    def finish(self, state, reason):
        self.event('finished', state=state, reason=reason)
        (self.path / 'summary.json').write_text(json.dumps(dict(state=state, reason=reason), indent=2), encoding='utf-8')
        self.events.close()
        self.points.close()
