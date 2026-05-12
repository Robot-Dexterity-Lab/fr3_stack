"""Tests for ``fr3_stack.geometry`` — Pose, Transform, composition rules."""
from __future__ import annotations

import math

import numpy as np
import pytest

from fr3_stack.geometry import Pose, Transform


# ============================================================================
# Pose basics (carried over from the old test_robot_client.py Pose section)
# ============================================================================

def test_pose_normalises_inputs():
    p = Pose.from_xyz_quat([0.1, 0.2, 0.3], [0, 0, 0, 1])
    assert isinstance(p.pos, np.ndarray) and p.pos.shape == (3,)
    assert isinstance(p.quat, np.ndarray) and p.quat.shape == (4,)


def test_pose_R_matches_quaternion_z90():
    s, c = math.sin(math.pi / 4), math.cos(math.pi / 4)
    p = Pose.from_xyz_quat([0, 0, 0], [0, 0, s, c])
    R = p.R
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)
    assert np.allclose(R @ [0, 1, 0], [-1, 0, 0], atol=1e-9)


def test_pose_is_frozen():
    p = Pose.from_xyz_quat([0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(Exception):
        p.pos = np.zeros(3)             # type: ignore[misc]


def test_transform_is_frozen():
    t = Transform.translation([0.1, 0.2, 0.3])
    with pytest.raises(Exception):
        t.pos = np.zeros(3)             # type: ignore[misc]


# ============================================================================
# Identity & inverse
# ============================================================================

def test_pose_identity():
    p = Pose.identity()
    assert np.allclose(p.pos, 0)
    assert np.allclose(p.quat, [0, 0, 0, 1])
    assert np.allclose(p.matrix, np.eye(4))


def test_transform_identity_is_neutral():
    """Pose @ Transform.identity() == Pose."""
    p = Pose.from_xyz_quat([0.5, 0.1, 0.4], [0, 0, math.sin(0.3), math.cos(0.3)])
    p2 = p @ Transform.identity()
    assert p.approx_equal(p2)


def test_transform_inverse_round_trip():
    """t @ t.inverse() == identity."""
    t = Transform.from_axis_angle([0.3, 0.7, 0.5], math.radians(40), pos=[0.1, -0.2, 0.05])
    composed = t @ t.inverse()
    assert np.allclose(composed.matrix, np.eye(4), atol=1e-12)


# ============================================================================
# Composition operator (the load-bearing one)
# ============================================================================

def test_pose_at_transform_translation_in_base_frame():
    """Pose @ Transform.translation([dx,dy,dz]) translates in the Pose's frame.

    With identity orientation, the offset is in base; a 90°-z rotation
    swaps which base axis the local-x corresponds to.
    """
    p = Pose.from_xyz_quat([1, 2, 3], [0, 0, 0, 1])
    out = p @ Transform.translation([0.1, 0.0, 0.0])
    assert np.allclose(out.pos, [1.1, 2.0, 3.0])

    s, c = math.sin(math.pi / 4), math.cos(math.pi / 4)
    p = Pose.from_xyz_quat([1, 2, 3], [0, 0, s, c])
    out = p @ Transform.translation([0.1, 0.0, 0.0])
    # Local +x = base +y after a 90° z rotation.
    assert np.allclose(out.pos, [1.0, 2.1, 3.0])


def test_matmul_matches_homogeneous_matrix_product():
    """Core invariant: (p @ t).matrix ≈ p.matrix @ t.matrix.

    This is the daemon-parity contract — Eigen Affine3d composition on the
    C++ side does exactly this matrix product.
    """
    rng = np.random.default_rng(42)
    for _ in range(20):
        p_pos = rng.uniform(-1, 1, 3)
        p_q = rng.normal(size=4); p_q /= np.linalg.norm(p_q)
        t_pos = rng.uniform(-0.5, 0.5, 3)
        t_q = rng.normal(size=4); t_q /= np.linalg.norm(t_q)

        p = Pose.from_xyz_quat(p_pos, p_q)
        t = Transform(t_pos, t_q)

        out = p @ t
        ref = p.matrix @ t.matrix
        assert np.allclose(out.matrix, ref, atol=1e-12), (
            f"Pose @ Transform does not match matrix product:\n"
            f"got=\n{out.matrix}\nref=\n{ref}"
        )


def test_transform_at_transform_matches_matrix_product():
    rng = np.random.default_rng(7)
    for _ in range(20):
        a_pos = rng.uniform(-1, 1, 3); a_q = rng.normal(size=4); a_q /= np.linalg.norm(a_q)
        b_pos = rng.uniform(-1, 1, 3); b_q = rng.normal(size=4); b_q /= np.linalg.norm(b_q)
        a, b = Transform(a_pos, a_q), Transform(b_pos, b_q)
        assert np.allclose((a @ b).matrix, a.matrix @ b.matrix, atol=1e-12)


def test_inv_compose_round_trip():
    """p1 @ p1.inv_compose(p2) == p2."""
    rng = np.random.default_rng(11)
    for _ in range(10):
        p1 = Pose.from_xyz_quat(
            rng.uniform(-1, 1, 3),
            (lambda q: q / np.linalg.norm(q))(rng.normal(size=4)),
        )
        p2 = Pose.from_xyz_quat(
            rng.uniform(-1, 1, 3),
            (lambda q: q / np.linalg.norm(q))(rng.normal(size=4)),
        )
        rel = p1.inv_compose(p2)
        # rot_tol < ~1e-7 hits float precision after a few quat products.
        assert (p1 @ rel).approx_equal(p2, pos_tol=1e-9, rot_tol=1e-7)


# ============================================================================
# Composition type rules
# ============================================================================

def test_pose_at_pose_raises():
    p1 = Pose.identity()
    p2 = Pose.from_xyz_quat([0.1, 0, 0], [0, 0, 0, 1])
    with pytest.raises(TypeError, match="Pose @ Pose"):
        _ = p1 @ p2


def test_transform_at_pose_raises():
    t = Transform.translation([0.1, 0, 0])
    p = Pose.identity()
    with pytest.raises(TypeError, match="Transform @ Pose"):
        _ = t @ p


# ============================================================================
# Constructors
# ============================================================================

def test_transform_translation():
    t = Transform.translation([0.1, 0.2, 0.3])
    assert np.allclose(t.pos, [0.1, 0.2, 0.3])
    assert np.allclose(t.quat, [0, 0, 0, 1])


def test_transform_from_axis_angle_z():
    t = Transform.from_axis_angle([0, 0, 1], math.pi / 2)
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert np.allclose(t.quat, [0, 0, s, c])


def test_pose_from_matrix_round_trip():
    """from_matrix(matrix) should round-trip pos and orientation."""
    s, c = math.sin(0.7), math.cos(0.7)
    p = Pose.from_xyz_quat([0.5, 0.1, 0.4], [0, 0, s, c])
    p2 = Pose.from_matrix(p.matrix)
    # rot_tol < ~1e-7 is below float precision after a R→quat→R round-trip.
    assert p.approx_equal(p2, pos_tol=1e-12, rot_tol=1e-7)


# ============================================================================
# Composition algebra (sanity checks via the public Transform API)
# ============================================================================

def test_transform_compose_with_identity_is_noop():
    t = Transform.from_axis_angle([0, 0, 1], math.radians(30), pos=[0.1, 0, 0])
    assert np.allclose((t @ Transform.identity()).matrix, t.matrix)
    assert np.allclose((Transform.identity() @ t).matrix, t.matrix)


def test_transform_z90_twice_equals_z180():
    z90  = Transform.from_axis_angle([0, 0, 1], math.pi / 2)
    z180 = Transform.from_axis_angle([0, 0, 1], math.pi)
    assert np.allclose((z90 @ z90).matrix, z180.matrix, atol=1e-12)
