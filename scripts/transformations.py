"""The parts of ROS 1's tf.transformations that this assignment uses.

ROS 2 does not ship tf.transformations. The functions below have the same
names, arguments and results as their ROS 1 counterparts, including the
quaternion order [x, y, z, w], so code written for tf.transformations works
unchanged after

    import transformations as tr

Only the default Euler convention, axes='sxyz' (roll about x, then pitch
about y, then yaw about z, all about the fixed axes), is implemented.
"""
import math

import numpy as np

_EPS = np.finfo(float).eps * 4.0


def _check_axes(axes):
    if axes != 'sxyz':
        raise NotImplementedError(f"only axes='sxyz' is implemented, not {axes!r}")


def quaternion_matrix(quaternion):
    """Return the 4x4 homogeneous rotation matrix of quaternion [x, y, z, w]."""
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0),
        (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0),
        (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
        (0.0, 0.0, 0.0, 1.0)), dtype=np.float64)


def quaternion_multiply(quaternion1, quaternion0):
    """Return quaternion1 * quaternion0, i.e. rotate by quaternion0 first."""
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array((
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0), dtype=np.float64)


def quaternion_conjugate(quaternion):
    """Return the conjugate [-x, -y, -z, w] of quaternion [x, y, z, w]."""
    return np.array((-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]),
                    dtype=np.float64)


def quaternion_inverse(quaternion):
    """Return the inverse of quaternion [x, y, z, w]."""
    return quaternion_conjugate(quaternion) / np.dot(quaternion, quaternion)


def euler_from_matrix(matrix, axes='sxyz'):
    """Return (roll, pitch, yaw) of the rotation in a 3x3 or 4x4 matrix."""
    _check_axes(axes)
    M = np.array(matrix, dtype=np.float64)[:3, :3]
    cy = math.sqrt(M[0, 0] * M[0, 0] + M[1, 0] * M[1, 0])
    if cy > _EPS:
        roll = math.atan2(M[2, 1], M[2, 2])
        pitch = math.atan2(-M[2, 0], cy)
        yaw = math.atan2(M[1, 0], M[0, 0])
    else:
        roll = math.atan2(-M[1, 2], M[1, 1])
        pitch = math.atan2(-M[2, 0], cy)
        yaw = 0.0
    return roll, pitch, yaw


def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return (roll, pitch, yaw) of quaternion [x, y, z, w]."""
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion [x, y, z, w] for roll ai, pitch aj, yaw ak."""
    _check_axes(axes)
    ci, si = math.cos(ai / 2.0), math.sin(ai / 2.0)
    cj, sj = math.cos(aj / 2.0), math.sin(aj / 2.0)
    ck, sk = math.cos(ak / 2.0), math.sin(ak / 2.0)
    cc, cs, sc, ss = ci * ck, ci * sk, si * ck, si * sk
    return np.array((
        cj * sc - sj * cs,
        cj * ss + sj * cc,
        cj * cs - sj * sc,
        cj * cc + sj * ss), dtype=np.float64)
