"""scripts/transformations.py must agree with ROS 1's tf.transformations."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import transformations as tr  # noqa: E402

Q1 = [0.1, 0.2, 0.3, 0.9]  # deliberately not unit length
Q2 = [0.4, -0.5, 0.6, 0.5]


# Expected values below were computed with tf.transformations on ROS Noetic.
def test_matches_ros1_tf_transformations():
    np.testing.assert_allclose(tr.quaternion_multiply(Q1, Q2), [0.68, -0.29, 0.56, 0.33], atol=1e-15)
    np.testing.assert_allclose(tr.euler_from_quaternion(Q1),
                               [0.3392926144540446, 0.32128858926481035, 0.6989996140390011], atol=1e-15)
    np.testing.assert_allclose(tr.quaternion_from_euler(0.1, -0.2, 0.3),
                               [0.06407134770607116, -0.09115754934299071,
                                0.1534393020242226, 0.9818561728660808], atol=1e-15)
    np.testing.assert_allclose(tr.quaternion_inverse(Q2),
                               [-0.39215686274509803, 0.49019607843137253,
                                -0.5882352941176471, 0.49019607843137253], atol=1e-15)
    np.testing.assert_allclose(tr.quaternion_matrix(Q1)[:3, :3],
                               [[0.7263157894736842, -0.5263157894736841, 0.44210526315789467],
                                [0.6105263157894736, 0.7894736842105264, -0.06315789473684211],
                                [-0.3157894736842105, 0.3157894736842105, 0.8947368421052633]], atol=1e-15)


def test_quaternion_order_is_xyzw():
    # 90 degrees of yaw: x axis -> y axis
    R = tr.quaternion_matrix([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    np.testing.assert_allclose(R[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    np.testing.assert_allclose(R[3], [0, 0, 0, 1])


def test_upside_down_laser():
    # the bag's laser is mounted upside down, [1, 0, 0, 0] relative to base_link
    np.testing.assert_allclose(tr.quaternion_matrix([1.0, 0, 0, 0])[:3, :3], np.diag([1, -1, -1]), atol=1e-12)


def test_multiply_composes_matrices():
    a = tr.quaternion_from_euler(0.1, -0.2, 0.3)
    b = tr.quaternion_from_euler(-0.4, 0.5, 2.0)
    np.testing.assert_allclose(tr.quaternion_matrix(tr.quaternion_multiply(a, b)),
                               tr.quaternion_matrix(a) @ tr.quaternion_matrix(b), atol=1e-12)


@pytest.mark.parametrize('rpy', [(0.1, -0.2, 0.3), (0.0, 0.0, -3.1), (math.pi, 0.0, 1.0)])
def test_euler_round_trip(rpy):
    np.testing.assert_allclose(tr.euler_from_quaternion(tr.quaternion_from_euler(*rpy)), rpy, atol=1e-12)


def test_inverse():
    q = tr.quaternion_from_euler(0.3, 0.2, 0.1)
    np.testing.assert_allclose(tr.quaternion_multiply(q, tr.quaternion_inverse(q)), [0, 0, 0, 1], atol=1e-12)


def test_other_axes_rejected():
    with pytest.raises(NotImplementedError):
        tr.euler_from_quaternion(Q1, axes='rzyx')
