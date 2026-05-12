"""Pin Pose's xyzw → R conversion to match the daemon's Eigen result.

The daemon crosses the wire boundary by constructing
``Eigen::Quaterniond(w, x, y, z).toRotationMatrix()`` from the capnp
``targetQuatXyzw`` field. ``Pose(p, q).R`` must produce the same rotation
matrix. Eigen's formula is well-defined and reproduced inline below as
``_eigen_q_to_R_oracle`` — independent of whatever Python library
``geometry.py`` happens to use under the hood (scipy today, possibly
something else tomorrow).

If anyone changes ``geometry.py`` in a way that breaks parity with the
daemon, this test fails fast with a numerical mismatch.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from fr3_stack.geometry import Pose


def _eigen_q_to_R_oracle(q_xyzw: np.ndarray) -> np.ndarray:
    """Reference: Eigen::Quaterniond(w, x, y, z).toRotationMatrix() with xyzw input.

    Frozen here so changes to fr3_stack's quaternion library never silently
    diverge from the C++ daemon. Same formula scipy / Eigen / ROS tf2 use.
    """
    q = np.asarray(q_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


@pytest.mark.parametrize("name,quat_xyzw", [
    ("identity",   [0.0, 0.0, 0.0, 1.0]),
    ("x180",       [1.0, 0.0, 0.0, 0.0]),
    ("y180",       [0.0, 1.0, 0.0, 0.0]),
    ("z180",       [0.0, 0.0, 1.0, 0.0]),
    ("z90",        [0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4)]),
    ("xyzw_45deg", [0.1, 0.2, 0.3, 0.4]),  # not unit; will be normalized below
])
def test_pose_R_matches_eigen_oracle_corner_cases(name, quat_xyzw):
    q = np.asarray(quat_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    got = Pose(np.zeros(3), q).R
    ref = _eigen_q_to_R_oracle(q)
    assert np.allclose(got, ref, atol=1e-12), (
        f"{name}: mismatch\n got=\n{got}\n ref=\n{ref}"
    )


def test_pose_R_fuzz_against_eigen_oracle():
    """200 random unit quats — Pose.R must match the Eigen formula to 1e-12."""
    rng = np.random.default_rng(2026_05_09)
    for _ in range(200):
        q = rng.normal(size=4)
        q = q / np.linalg.norm(q)
        got = Pose(np.zeros(3), q).R
        ref = _eigen_q_to_R_oracle(q)
        assert np.allclose(got, ref, atol=1e-12)
