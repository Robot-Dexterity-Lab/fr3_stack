"""Tests for ``fr3_stack.Arm`` against the shared ``FakeDaemon``.

Arm is the Pose-centric facade. Composes a Robot, exposes 11 attributes:
``observe / send / move_to / hold / relax / set_stiffness / use_profile``,
plus ``connect/close/__enter__/__exit__`` and the ``robot`` escape hatch.

These tests focus on Arm's specific behavior — sticky-cache semantics,
last_error → RuntimeError, observe() field correctness, on-the-wire packet
shape from ``send``. Underlying Robot streaming is covered separately in
``test_robot_client.py``.
"""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from fr3_stack import Arm, Pose, Transform


# ============================================================================
# Helpers
# ============================================================================

@pytest.fixture
def arm(daemon):
    """An ``Arm`` connected to the test ``FakeDaemon`` (from conftest)."""
    a = Arm("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    a.connect()
    yield a
    a.close()


# ============================================================================
# Lifecycle
# ============================================================================

def test_arm_context_manager(daemon):
    with Arm("127.0.0.1", cmd_port=daemon.cmd_port,
             state_port=daemon.state_port) as a:
        a.relax()
        with daemon.recv_command() as cmd:
            assert cmd is not None
            assert cmd.config.which() == "idle"


def test_arm_robot_escape_hatch_is_robot(arm):
    from fr3_stack import Robot
    assert isinstance(arm.robot, Robot)


# ============================================================================
# observe()
# ============================================================================

def test_observe_returns_typed_observation(arm, daemon):
    seen = daemon.publish_until_received(
        arm.robot,
        controller="cartesian_impedance",
        pos=(0.5, 0.1, 0.4),
        quat_xyzw=(0, 0, 0, 1),
        q=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        wrench_ext=(1, 2, 3, 0.1, 0.2, 0.3),
        timestamp=42.0,
    )
    assert seen
    obs = arm.observe()
    assert isinstance(obs.pose, Pose)
    assert np.allclose(obs.pose.pos, [0.5, 0.1, 0.4])
    assert np.allclose(obs.q, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    assert np.allclose(obs.wrench, [1, 2, 3, 0.1, 0.2, 0.3])
    assert obs.has_ft is False
    assert obs.timestamp == pytest.approx(42.0)


def test_observe_picks_ft_wrench_when_available(arm, daemon):
    seen = daemon.publish_until_received(
        arm.robot,
        wrench_ext=(0, 0, 0, 0, 0, 0),
        wrench_ft=(10, 20, 30, 0.4, 0.5, 0.6),
    )
    assert seen
    obs = arm.observe()
    assert obs.has_ft is True
    assert np.allclose(obs.wrench, [10, 20, 30, 0.4, 0.5, 0.6])


def test_observe_raises_on_last_error(arm, daemon):
    seen = daemon.publish_until_received(
        arm.robot,
        last_error="protective stop: joint 3 limit",
    )
    assert seen
    with pytest.raises(RuntimeError, match="protective stop"):
        arm.observe()


def test_observe_arrays_are_independent_copies(arm, daemon):
    daemon.publish_until_received(arm.robot, pos=(1, 2, 3))
    obs = arm.observe()
    obs.q[0] = 999.0   # mutate the returned view
    obs2 = arm.observe()
    assert obs2.q[0] != 999.0


# ============================================================================
# send()
# ============================================================================

def test_send_emits_cartesian_impedance(arm, daemon):
    target = Pose.from_xyz_quat([0.5, 0.0, 0.4], [0, 0, 0, 1])
    arm.send(target)
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "cartesianImpedance"
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos)      == [0.5, 0.0, 0.4]
        assert list(ci.targetQuatXyzw) == [0.0, 0.0, 0.0, 1.0]


def test_send_stiffness_is_sticky(arm, daemon):
    """send(stiffness=K) updates the cache; subsequent send picks up K."""
    target1 = Pose.from_xyz_quat([0.5, 0, 0.4], [0, 0, 0, 1])
    arm.send(target1, stiffness=[111, 222, 333, 11, 22, 33])
    with daemon.recv_command() as cmd:
        assert list(cmd.config.cartesianImpedance.k) == [111, 222, 333, 11, 22, 33]

    # Second call without stiffness=, cache should still hold the override.
    target2 = Pose.from_xyz_quat([0.6, 0, 0.4], [0, 0, 0, 1])
    arm.send(target2)
    with daemon.recv_command() as cmd:
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos) == [0.6, 0.0, 0.4]
        assert list(ci.k)         == [111, 222, 333, 11, 22, 33]


def test_send_uses_pose_at_transform_target(arm, daemon):
    """A Pose obtained via Pose @ Transform composes correctly on the wire."""
    base = Pose.from_xyz_quat([0.5, 0.0, 0.4], [0, 0, 0, 1])
    delta = Transform.translation([0.1, 0.0, 0.0])
    arm.send(base @ delta)
    with daemon.recv_command() as cmd:
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos) == [0.6, 0.0, 0.4]


# ============================================================================
# set_stiffness()
# ============================================================================

def test_set_stiffness_seeds_caches_no_send(arm, daemon):
    arm.set_stiffness(K=[100, 100, 800, 30, 30, 30],
                      D=[ 20,  20,  56, 11, 11, 11])
    # Cart cache + admittance inner-impedance K both updated.
    assert arm.robot._cart_cache["K"] == [100, 100, 800, 30, 30, 30]
    assert arm.robot._cart_cache["D"] == [ 20,  20,  56, 11, 11, 11]
    assert arm.robot._adm_cache["K"]  == [100, 100, 800, 30, 30, 30]
    # Nothing went on the wire.
    with daemon.recv_command(timeout=0.1) as cmd:
        assert cmd is None


def test_set_stiffness_K_only_leaves_D_alone(arm):
    d_before = list(arm.robot._cart_cache["D"])
    arm.set_stiffness(K=[1, 2, 3, 4, 5, 6])
    assert arm.robot._cart_cache["D"] == d_before


# ============================================================================
# move_to()
# ============================================================================

def test_move_to_emits_moveto_then_blocks(arm, daemon):
    target = Pose.from_xyz_quat([0.5, 0.0, 0.4], [0, 0, math.sin(0.1), math.cos(0.1)])
    # Publish state in the background so observe() at the end works.
    daemon.publish_until_received(
        arm.robot,
        pos=(0.5, 0.0, 0.4),
        quat_xyzw=(0, 0, math.sin(0.1), math.cos(0.1)),
    )

    t0 = time.monotonic()
    obs = arm.move_to(target, duration=0.1)    # short duration for fast tests
    elapsed = time.monotonic() - t0

    assert elapsed >= 0.1, f"move_to should block ≥ duration, got {elapsed:.3f}s"
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "moveTo"
        mt = cmd.config.moveTo
        assert list(mt.targetPos) == [0.5, 0.0, 0.4]
        assert mt.runTime == pytest.approx(0.1)
    # Returned observation is the post-move snapshot.
    assert obs.pose.approx_equal(target, pos_tol=1e-9, rot_tol=1e-9)


# ============================================================================
# hold()
# ============================================================================

def test_hold_locks_at_current_pose(arm, daemon):
    daemon.publish_until_received(
        arm.robot,
        pos=(0.42, 0.11, 0.33),
        quat_xyzw=(0, 0, 0, 1),
    )
    obs = arm.hold()
    assert np.allclose(obs.pose.pos, [0.42, 0.11, 0.33])
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "cartesianImpedance"
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos) == [0.42, 0.11, 0.33]


# ============================================================================
# relax()
# ============================================================================

def test_relax_sends_idle(arm, daemon):
    arm.relax()
    with daemon.recv_command() as cmd:
        assert cmd is not None
        assert cmd.config.which() == "idle"
        assert cmd.termination is False


# ============================================================================
# use_profile()
# ============================================================================

def test_use_profile_swaps_cartesian_yaml(daemon, tmp_path, monkeypatch):
    """use_profile reads configs/cartesian_impedance.<name>.yaml."""
    d = tmp_path / "configs"
    d.mkdir()
    (d / "cartesian_impedance.snug.yaml").write_text("""
K: [555, 555, 555, 25, 25, 25]
D: [50, 50, 50, 12, 12, 12]
q_null: [0,0,0,0,0,0,0]
K_null: 75.0
filter_alpha: 0.1
target_wrench: [0,0,0,0,0,0]
max_delta: [0,0,0,0,0,0]
use_friction: false
""")
    monkeypatch.setenv("FR3_CONFIG_DIR", str(d))

    a = Arm("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    a.connect()
    try:
        a.use_profile("snug")
        a.send(Pose.from_xyz_quat([0.5, 0, 0.4], [0, 0, 0, 1]))
        with daemon.recv_command() as cmd:
            ci = cmd.config.cartesianImpedance
            assert list(ci.k) == [555, 555, 555, 25, 25, 25]
            assert ci.kNull   == pytest.approx(75.0)
    finally:
        a.close()


def test_use_profile_with_explicit_controller_swaps_hybrid_yaml(
    daemon, tmp_path, monkeypatch
):
    """Arm.use_profile(name, controller='hybrid') routes to the hybrid
    profile, not the cartesian one."""
    from pathlib import Path
    from fr3_stack import Arm
    import yaml as _yaml

    cfg_dir = tmp_path / "configs"
    cfg_dir.mkdir()
    pkg_hybrid = (
        Path(__file__).resolve().parents[1]
        / "fr3_stack" / "configs" / "hybrid.yaml"
    )
    base = _yaml.safe_load(pkg_hybrid.read_text())
    base["impedance"]["K"] = [9, 9, 9, 9, 9, 9]
    (cfg_dir / "hybrid.tight.yaml").write_text(_yaml.safe_dump(base))

    monkeypatch.setenv("FR3_CONFIG_DIR", str(cfg_dir))
    with Arm("localhost") as arm:
        arm.use_profile("tight", controller="hybrid")
        assert arm.robot._hybrid_cache["K"] == [9, 9, 9, 9, 9, 9]
