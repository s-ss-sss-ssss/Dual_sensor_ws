"""Pure validation/statistics for corrected magnetic rounds."""
import numpy as np


def validate_round(record, expected, require_identity=True):
    if require_identity and not record['identity_verified']:
        return None, 'identity_unverified'
    if record['errors']:
        return None, 'device_error'
    vectors = {}
    for family, ids in expected.items():
        rows = record[family]
        by_id = {}
        for row in rows:
            sid = row['sensor_id']
            if sid in by_id:
                return None, 'duplicate_id'
            by_id[sid] = row
        if not set(ids) <= set(by_id):
            return None, 'missing_sensor'
        for sid in ids:
            row = by_id[sid]
            xyz = [row['x'], row['y'], row['z']]
            if not np.isfinite(xyz).all():
                return None, 'nonfinite_field'
            if not row['drdy'] or any(row.get(k, 0) for k in ('err', 'dor', 'overflow')):
                return None, 'sensor_status'
            vectors['%s:%s' % (family, sid)] = xyz
    return vectors, None


def statistics(vectors):
    return {key: dict(mean=np.mean([v[key] for v in vectors], axis=0).tolist(),
                      std=np.std([v[key] for v in vectors], axis=0, ddof=1).tolist() if len(vectors) > 1 else [0., 0., 0.],
                      count=len(vectors)) for key in vectors[0]}
