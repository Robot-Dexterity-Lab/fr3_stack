"""Tests for ``fr3_stack.RobotAgent`` against the shared FakeDaemon.

RobotAgent is the thin reset/observe/step wrapper for policy rollouts.
Composes an Arm (which composes a Robot). These tests verify the core
loop shape and the sticky / rate-limit behaviors that the wrapper adds on
top of Arm. Underlying streaming + Arm semantics are covered in
``test_robot_client.py`` and ``test_arm.py``.
"""
from __future__ import annotations

import time

import numpy as np
import pytest

from fr3_stack import Pose, RobotAgent


# ============================================================================
# Helpers
# ============================================================================

HOME = Pose.from_xyz_quat([0.5, 0.0, 0.4], [0, 0, 0, 1])


@pytest.fixture
def agent(daemon):
    a = RobotAgent(
        "127.0.0.1",
        home_pose=HOME,
        control_hz=100.0,           # 10 ms tick — fast for tests
        default_stiffness=[200, 200, 200, 20, 20, 20],
        reset_duration=0.05,        # 50 ms — fast reset for tests
        cmd_port=daemon.cmd_port,
        state_port=daemon.state_port,
    )
    a.connect()
    # Pre-publish state so observe() at end of reset() / step() doesn't time out.
    daemon.publish_until_received(
        a.robot, pos=(0.5, 0.0, 0.4), quat_xyzw=(0, 0, 0, 1),
    )
    yield a
    a.close()


# ============================================================================
# Construction / lifecycle
# ============================================================================

def test_agent_rejects_nonpositive_hz(daemon):
    with pytest.raises(ValueError, match="control_hz"):
        RobotAgent("127.0.0.1", home_pose=HOME, control_hz=0,
                   cmd_port=daemon.cmd_port, state_port=daemon.state_port)


def test_agent_rejects_nonpositive_reset_duration(daemon):
    with pytest.raises(ValueError, match="reset_duration"):
        RobotAgent("127.0.0.1", home_pose=HOME, reset_duration=0,
                   cmd_port=daemon.cmd_port, state_port=daemon.state_port)


def test_agent_context_manager(daemon):
    with RobotAgent("127.0.0.1", home_pose=HOME,
                    cmd_port=daemon.cmd_port, state_port=daemon.state_port) as a:
        a.arm.relax()
        with daemon.recv_command() as cmd:
            assert cmd is not None
            assert cmd.config.which() == "idle"


def test_agent_escape_hatches(agent):
    from fr3_stack import Arm, Robot
    assert isinstance(agent.arm,   Arm)
    assert isinstance(agent.robot, Robot)


# ============================================================================
# reset()
# ============================================================================

def test_reset_emits_moveto_to_home(agent, daemon):
    obs = agent.reset()

    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "moveTo"
        mt = cmd.config.moveTo
        assert list(mt.targetPos)      == [0.5, 0.0, 0.4]
        assert list(mt.targetQuatXyzw) == [0.0, 0.0, 0.0, 1.0]
        assert mt.runTime == pytest.approx(0.05)
    # Returns the observation post-home.
    assert obs is not None
    assert np.allclose(obs.pose.pos, [0.5, 0.0, 0.4])


def test_reset_reapplies_default_stiffness(agent, daemon):
    """A bad policy may have left K hot. reset() must wipe it back to default."""
    # Simulate prior policy override.
    agent.arm.set_stiffness(K=[1, 2, 3, 4, 5, 6])
    assert agent.robot._cart_cache["K"] == [1, 2, 3, 4, 5, 6]

    agent.reset()
    assert agent.robot._cart_cache["K"] == [200, 200, 200, 20, 20, 20]


# ============================================================================
# step()
# ============================================================================

def test_step_emits_cartesian_impedance(agent, daemon):
    target = Pose.from_xyz_quat([0.6, 0.0, 0.4], [0, 0, 0, 1])
    agent.reset()
    with daemon.recv_command():     # consume the reset moveTo
        pass

    agent.step(target)
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "cartesianImpedance"
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos) == [0.6, 0.0, 0.4]


def test_step_stiffness_is_sticky(agent, daemon):
    target = Pose.from_xyz_quat([0.6, 0.0, 0.4], [0, 0, 0, 1])
    agent.reset()
    with daemon.recv_command():
        pass

    agent.step(target, stiffness=[111, 222, 333, 11, 22, 33])
    with daemon.recv_command() as cmd:
        assert list(cmd.config.cartesianImpedance.k) == [111, 222, 333, 11, 22, 33]

    # Next step without stiffness= reuses cached values.
    agent.step(Pose.from_xyz_quat([0.7, 0.0, 0.4], [0, 0, 0, 1]))
    with daemon.recv_command() as cmd:
        assert list(cmd.config.cartesianImpedance.k) == [111, 222, 333, 11, 22, 33]


def test_step_rate_limits_to_control_hz(agent, daemon):
    """Three steps at 100 Hz must take ≥ ~20 ms (2 inter-step gaps)."""
    target = Pose.from_xyz_quat([0.6, 0.0, 0.4], [0, 0, 0, 1])
    agent.reset()
    with daemon.recv_command():
        pass

    t0 = time.monotonic()
    for _ in range(3):
        agent.step(target)
        with daemon.recv_command():
            pass
    elapsed = time.monotonic() - t0
    # At 100 Hz we have 2 sleeps of 10 ms + the last observe. Allow some slack.
    assert elapsed >= 0.018, f"rate limiter not enforcing dt: {elapsed:.4f}s for 3 steps"


def test_step_returns_observation(agent, daemon):
    target = Pose.from_xyz_quat([0.6, 0.0, 0.4], [0, 0, 0, 1])
    agent.reset()
    with daemon.recv_command():
        pass

    obs = agent.step(target)
    # Observation has the expected fields populated.
    assert obs.pose is not None
    assert obs.q.shape  == (7,)
    assert obs.dq.shape == (7,)
    assert obs.wrench.shape == (6,)
    assert isinstance(obs.has_ft, bool)
