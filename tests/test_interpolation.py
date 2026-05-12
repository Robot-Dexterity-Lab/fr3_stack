"""Tests for ``fr3_stack.PoseTrajectoryInterpolator`` and basic
``InterpolationController`` shape checks.

The pure-Python interpolator is fully unit-testable. The
``InterpolationController`` involves a subprocess + ZMQ + manager IPC, which
is finicky in pytest; we cover only the construction/argument-validation
surface here. End-to-end validation belongs on real hardware (or against a
running daemon binary), not in CI.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from fr3_stack import (
    InterpolationController,
    Pose,
    PoseTrajectoryInterpolator,
)


# =============================================================================
# PoseTrajectoryInterpolator
# =============================================================================

def test_single_anchor_holds():
    p = Pose.from_xyz_quat([0.5, 0.0, 0.4], [0, 0, 0, 1])
    interp = PoseTrajectoryInterpolator.from_pose(0.0, p)
    pos, quat = interp(0.0)
    assert np.allclose(pos, p.pos)
    assert np.allclose(quat, p.quat)
    pos, quat = interp(100.0)        # extrapolation = clamp to last
    assert np.allclose(pos, p.pos)


def test_strict_increasing_required():
    with pytest.raises(ValueError, match="strictly increasing"):
        PoseTrajectoryInterpolator(
            [0.0, 0.0, 1.0],
            [[0, 0, 0]] * 3,
            [[0, 0, 0, 1]] * 3,
        )


def test_length_mismatch_rejected():
    with pytest.raises(ValueError, match="length mismatch"):
        PoseTrajectoryInterpolator(
            [0.0, 1.0],
            [[0, 0, 0]],
            [[0, 0, 0, 1]],
        )


def test_position_linear_midpoint():
    """Position interp is exact linear: 50% point is the average."""
    interp = PoseTrajectoryInterpolator(
        times      = [0.0, 1.0],
        positions  = [[0, 0, 0], [1, 2, 3]],
        quats_xyzw = [[0, 0, 0, 1], [0, 0, 0, 1]],
    )
    pos, _ = interp(0.5)
    assert np.allclose(pos, [0.5, 1.0, 1.5])


def test_quaternion_slerp_midpoint_z90():
    """SLERP from identity to z90 at t=0.5 should produce z45."""
    interp = PoseTrajectoryInterpolator(
        times      = [0.0, 1.0],
        positions  = [[0, 0, 0], [0, 0, 0]],
        quats_xyzw = [[0, 0, 0, 1], [0, 0, math.sin(math.pi/4), math.cos(math.pi/4)]],
    )
    _, quat = interp(0.5)
    # Expected z45: [0, 0, sin(π/8), cos(π/8)]
    expected = np.array([0, 0, math.sin(math.pi/8), math.cos(math.pi/8)])
    # SLERP is sign-correct; allow tiny numerical noise.
    assert np.allclose(np.abs(quat), np.abs(expected), atol=1e-9)


def test_quaternion_signs_normalised_for_continuous_slerp():
    """Adjacent waypoints with opposite-sign quats must not produce a long-way SLERP.

    [0,0,0,-1] and [0,0,0,1] are the same rotation but on opposite sides of the
    quaternion sphere. The interpolator should detect this and flip signs.
    """
    interp = PoseTrajectoryInterpolator(
        times      = [0.0, 1.0],
        positions  = [[0, 0, 0]] * 2,
        quats_xyzw = [[0, 0, 0, -1], [0, 0, 0, 1]],
    )
    _, quat = interp(0.5)
    # Should remain identity-ish; if SLERP went the long way it'd be ~180°.
    R_id = np.eye(3)
    from scipy.spatial.transform import Rotation
    R_mid = Rotation.from_quat(quat).as_matrix()
    assert np.allclose(R_mid, R_id, atol=1e-9)


def test_drive_to_waypoint_history_preserved():
    """drive_to_waypoint keeps t < curr_time history so old samples re-evaluate
    to the same value (idempotency for already-published targets)."""
    interp = PoseTrajectoryInterpolator(
        times      = [0.0, 1.0, 2.0],
        positions  = [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
        quats_xyzw = [[0, 0, 0, 1]] * 3,
    )
    # Sample at t=0.3 before the rewrite
    pos_before, _ = interp(0.3)

    # Now reroute future plan starting at curr_time=0.5
    target = Pose.from_xyz_quat([5, 5, 5], [0, 0, 0, 1])
    new = interp.drive_to_waypoint(target, time=2.0, curr_time=0.5)

    # Old sample re-evaluates the same.
    pos_after, _ = new(0.3)
    assert np.allclose(pos_before, pos_after)
    # Future is the new linear ramp.
    pos_at_target, _ = new(2.0)
    assert np.allclose(pos_at_target, [5, 5, 5])


def test_drive_to_waypoint_rejects_past_target():
    interp = PoseTrajectoryInterpolator.from_pose(
        0.0, Pose.from_xyz_quat([0, 0, 0], [0, 0, 0, 1])
    )
    with pytest.raises(ValueError, match="must be > curr_time"):
        interp.drive_to_waypoint(
            Pose.from_xyz_quat([1, 0, 0], [0, 0, 0, 1]),
            time      = 0.0,
            curr_time = 0.5,
        )


def test_schedule_waypoint_drops_stale_future():
    """A new schedule with target before last_waypoint_time wipes stale plans."""
    interp = PoseTrajectoryInterpolator(
        times      = [0.0, 5.0],
        positions  = [[0, 0, 0], [10, 0, 0]],
        quats_xyzw = [[0, 0, 0, 1]] * 2,
    )
    # Schedule a closer waypoint at t=2.0; the old t=5.0 plan must drop.
    new = interp.schedule_waypoint(
        Pose.from_xyz_quat([3, 0, 0], [0, 0, 0, 1]),
        time               = 2.0,
        curr_time          = 1.0,
        last_waypoint_time = 5.0,
    )
    # At t=2.0 we're at the new target.
    pos, _ = new(2.0)
    assert np.allclose(pos, [3, 0, 0])
    # No waypoint after t=2.0.
    assert new.last_time == pytest.approx(2.0)


def test_schedule_waypoint_falls_back_when_in_past():
    """schedule_waypoint with time <= curr_time falls back to drive_to_waypoint."""
    interp = PoseTrajectoryInterpolator.from_pose(
        0.0, Pose.from_xyz_quat([0, 0, 0], [0, 0, 0, 1])
    )
    new = interp.schedule_waypoint(
        Pose.from_xyz_quat([1, 0, 0], [0, 0, 0, 1]),
        time               = 0.5,
        curr_time          = 1.0,
        last_waypoint_time = 0.0,
    )
    # Just verify it didn't raise and produced a valid trajectory.
    pos, _ = new(2.0)
    assert pos.shape == (3,)


# =============================================================================
# InterpolationController — argument validation only (no subprocess)
# =============================================================================

def test_controller_rejects_zero_frequency():
    with pytest.raises(ValueError, match="frequency"):
        InterpolationController("127.0.0.1", frequency=0)


def test_controller_rejects_bad_stiffness_shape():
    with pytest.raises(ValueError, match="stiffness"):
        InterpolationController("127.0.0.1", stiffness=[1, 2, 3])


def test_controller_rejects_bad_joints_init_shape():
    with pytest.raises(ValueError, match="joints_init"):
        InterpolationController("127.0.0.1", joints_init=[1, 2, 3])


def test_controller_servoL_rejects_too_short_duration():
    """duration < 1/freq would alias the interpolator."""
    ctl = InterpolationController("127.0.0.1", frequency=200.0)   # dt = 5 ms
    pose = Pose.from_xyz_quat([0.5, 0, 0.4], [0, 0, 0, 1])
    with pytest.raises(ValueError, match="duration"):
        ctl.servoL(pose, duration=0.001)   # 1 ms < 5 ms
