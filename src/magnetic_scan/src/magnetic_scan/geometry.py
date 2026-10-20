"""SE(3) transforms: T_A_B maps coordinates from B into A, SI units."""
import numpy as np
from scipy.spatial.transform import Rotation


def rotation_from_matrix(matrix):
    return Rotation.from_matrix(matrix) if hasattr(Rotation, "from_matrix") else Rotation.from_dcm(matrix)


def rotation_matrix(rotation):
    return rotation.as_matrix() if hasattr(rotation, "as_matrix") else rotation.as_dcm()


def transform(spec):
    p = np.asarray(spec['translation_m'], dtype=float)
    q = np.asarray(spec['quaternion_xyzw'], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('transform requires finite translation_m[3] and quaternion_xyzw[4]')
    if np.linalg.norm(q) < 1e-10:
        raise ValueError('zero quaternion')
    t = np.eye(4)
    t[:3, :3] = rotation_matrix(Rotation.from_quat(q))
    t[:3, 3] = p
    return t


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def residual(actual, target):
    return (float(np.linalg.norm(actual[:3, 3] - target[:3, 3])),
            float(np.linalg.norm(rotation_from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec())))


def average(transforms):
    out = np.eye(4)
    out[:3, 3] = np.mean([t[:3, 3] for t in transforms], axis=0)
    # scipy 1.3 in Noetic may not provide Rotation.mean; use Markley averaging.
    qs = rotation_from_matrix(np.asarray(transforms)[:, :3, :3]).as_quat()
    _, eigenvectors = np.linalg.eigh(qs.T @ qs)
    out[:3, :3] = rotation_matrix(Rotation.from_quat(eigenvectors[:, -1]))
    return out


def aligned_target(source, sensor, relative_rotation, distance_range, target_distance=None):
    axis = source[:3, 2]
    distance = float(axis @ (sensor[:3, 3] - source[:3, 3]))
    if not distance_range[0] <= distance <= distance_range[1] or distance <= 0:
        raise ValueError('initial axial distance %.6f m is outside configured positive range' % distance)
    if target_distance is not None:
        if not distance_range[0] <= target_distance <= distance_range[1]:
            raise ValueError('target distance outside configured range')
        distance = target_distance
    out = np.eye(4)
    out[:3, :3] = source[:3, :3] @ relative_rotation
    out[:3, 3] = source[:3, 3] + axis * distance
    return out


def offset_target(origin, offset):
    out = origin.copy()
    out[:3, 3] += origin[:3, :3] @ np.asarray(offset)
    return out


def limited_step(actual, target, max_translation, max_rotation):
    out = actual.copy()
    delta = target[:3, 3] - actual[:3, 3]
    out[:3, 3] += delta * min(1., max_translation / max(np.linalg.norm(delta), 1e-12))
    rot = rotation_from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()
    rot *= min(1., max_rotation / max(np.linalg.norm(rot), 1e-12))
    out[:3, :3] = rotation_matrix(Rotation.from_rotvec(rot)) @ actual[:3, :3]
    return out
