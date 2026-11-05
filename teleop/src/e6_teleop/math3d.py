"""SI transforms. Public quaternions use xyzw; MuJoCo uses wxyz."""
import numpy as np


def unit(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid quaternion")
    return q / np.linalg.norm(q)


def multiply(a, b):
    av, aw = np.asarray(a)[:3], a[3]
    bv, bw = np.asarray(b)[:3], b[3]
    return unit(np.r_[aw * bv + bw * av + np.cross(av, bv), aw * bw - av @ bv])


def inverse(q):
    q = unit(q)
    return np.r_[-q[:3], q[3]]


def from_rotvec(v):
    v = np.asarray(v, dtype=float)
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return unit(np.r_[v / 2, 1.0])
    return np.r_[v / angle * np.sin(angle / 2), np.cos(angle / 2)]


def to_rotvec(q):
    q = unit(q)
    if q[3] < 0:
        q = -q
    norm = np.linalg.norm(q[:3])
    if norm < 1e-12:
        return q[:3] * 2
    return q[:3] * (2 * np.arctan2(norm, q[3]) / norm)


def matrix(q):
    x, y, z, w = unit(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def integrate(position, quaternion, linear, angular, dt, frame):
    q = unit(quaternion)
    dp = np.asarray(linear) * dt
    dq = from_rotvec(np.asarray(angular) * dt)
    if frame == "tool":
        return np.asarray(position) + matrix(q) @ dp, multiply(q, dq)
    return np.asarray(position) + dp, multiply(dq, q)
