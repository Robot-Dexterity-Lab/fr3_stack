"""Tests for ``fr3_stack.Robot`` against a fake daemon.

Streaming-wire tests only. Pose-friendly / sticky / blocking ergonomics live
on :class:`fr3_stack.Arm` and are covered in ``test_arm.py``.

The shared ``FakeDaemon`` and ``daemon`` / ``client`` fixtures live in
:mod:`tests.conftest`.
"""
from __future__ import annotations

import time

import pytest

from fr3_stack import Robot


# ============================================================================
# Send: idle / termination
# ============================================================================

def test_send_idle(client, daemon):
    client.send_idle()
    with daemon.recv_command() as cmd:
        assert cmd is not None
        assert cmd.config.which() == "idle"
        assert cmd.termination is False


def test_terminate_sets_flag(client, daemon):
    client.terminate()
    with daemon.recv_command() as cmd:
        assert cmd is not None
        assert cmd.termination is True
        assert cmd.config.which() == "idle"


# ============================================================================
# Send: cartesian impedance
# ============================================================================

def test_send_cartesian_impedance_full(client, daemon):
    client.send_cartesian_impedance(
        target_pos       = [0.5, 0.0, 0.4],
        target_quat_xyzw = [0.0, 0.0, 0.0, 1.0],
        K                = [100, 100, 800, 30, 30, 30],
        D                = [ 20,  20,  56, 11, 11, 11],
        K_null           = 5.0,
        filter_alpha     = 0.1,
    )
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "cartesianImpedance"
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos)      == [0.5, 0.0, 0.4]
        assert list(ci.targetQuatXyzw) == [0.0, 0.0, 0.0, 1.0]
        assert list(ci.k)              == [100, 100, 800, 30, 30, 30]
        assert list(ci.d)              == [ 20,  20,  56, 11, 11, 11]
        assert ci.kNull       == pytest.approx(5.0)
        assert ci.filterAlpha == pytest.approx(0.1)


def test_cartesian_impedance_uses_defaults_when_omitted(client, daemon):
    client.send_cartesian_impedance(
        target_pos       = [0.5, 0.0, 0.4],
        target_quat_xyzw = [0.0, 0.0, 0.0, 1.0],
    )
    with daemon.recv_command() as cmd:
        ci = cmd.config.cartesianImpedance
        # Defaults from cartesian_impedance.yaml.
        assert list(ci.k)         == [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
        assert ci.kNull           == pytest.approx(100.0)
        assert ci.filterAlpha     == pytest.approx(0.05)


def test_cartesian_impedance_cache_persists_across_calls(client, daemon):
    # First call: stash a custom K and filter_alpha in the cache.
    client.send_cartesian_impedance(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        K=[100, 100, 800, 30, 30, 30],
        filter_alpha=0.1,
    )
    with daemon.recv_command():
        pass

    # Second call: only the target moves, K/D/filter should match the cache.
    client.send_cartesian_impedance(
        target_pos=[0.6, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        ci = cmd.config.cartesianImpedance
        assert list(ci.targetPos) == [0.6, 0.0, 0.4]
        assert list(ci.k)         == [100, 100, 800, 30, 30, 30]
        assert ci.filterAlpha     == pytest.approx(0.1)


# ============================================================================
# Per-controller profiles (YAML-driven defaults)
# ============================================================================

def _write_profile(dir_, name: str, content: str) -> None:
    """Drop a YAML file in a temp configs/ dir for FR3_CONFIG_DIR."""
    (dir_ / f"{name}.yaml").write_text(content)


@pytest.fixture
def profile_dir(tmp_path, monkeypatch):
    """Empty configs/ dir wired up via FR3_CONFIG_DIR. Tests can drop
    <name>.yaml or <name>.<profile>.yaml inside and Robot() will read them
    in preference to the bundled defaults."""
    d = tmp_path / "configs"
    d.mkdir()
    monkeypatch.setenv("FR3_CONFIG_DIR", str(d))
    return d


def test_send_cartesian_impedance_uses_profile_yaml(profile_dir, daemon):
    """send_cartesian_impedance(profile='stiff') reads the matching yaml."""
    _write_profile(profile_dir, "cartesian_impedance.stiff", """
K: [800, 800, 800, 50, 50, 50]
D: [56, 56, 56, 14, 14, 14]
q_null: [0,0,0,0,0,0,0]
K_null: 200.0
filter_alpha: 0.2
target_wrench: [0,0,0,0,0,0]
max_delta: [0,0,0,0,0,0]
use_friction: false
""")
    r = Robot("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    r.connect()
    try:
        r.send_cartesian_impedance(
            target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
            profile="stiff",
        )
        with daemon.recv_command() as cmd:
            ci = cmd.config.cartesianImpedance
            assert list(ci.k)     == [800, 800, 800, 50, 50, 50]
            assert ci.kNull       == pytest.approx(200.0)
            assert ci.filterAlpha == pytest.approx(0.2)
        # Profile sticks: subsequent calls without profile= keep stiff defaults.
        r.send_cartesian_impedance(
            target_pos=[0.6, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        )
        with daemon.recv_command() as cmd:
            assert list(cmd.config.cartesianImpedance.k) == [800, 800, 800, 50, 50, 50]
        assert r.get_profile("cartesian_impedance") == "stiff"
    finally:
        r.close()


def test_set_profile_resets_cache_overrides(profile_dir, daemon):
    """After set_profile, prior cache overrides are forgotten."""
    _write_profile(profile_dir, "cartesian_impedance.stiff", """
K: [800, 800, 800, 50, 50, 50]
D: [56, 56, 56, 14, 14, 14]
q_null: [0,0,0,0,0,0,0]
K_null: 200.0
filter_alpha: 0.2
target_wrench: [0,0,0,0,0,0]
max_delta: [0,0,0,0,0,0]
use_friction: false
""")
    r = Robot("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    r.connect()
    try:
        # Cache has K=base from bundled yaml. Override to weird values.
        r.send_cartesian_impedance(
            target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
            K=[1, 2, 3, 4, 5, 6],
        )
        with daemon.recv_command():
            pass

        # Switching profile must drop the [1,2,3,4,5,6] override.
        r.set_profile("cartesian_impedance", "stiff")
        r.send_cartesian_impedance(
            target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        )
        with daemon.recv_command() as cmd:
            assert list(cmd.config.cartesianImpedance.k) == [800, 800, 800, 50, 50, 50]
    finally:
        r.close()


def test_unknown_profile_raises(profile_dir, daemon):
    r = Robot("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    r.connect()
    try:
        with pytest.raises(FileNotFoundError, match="no config file"):
            r.send_cartesian_impedance(
                target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
                profile="does_not_exist",
            )
    finally:
        r.close()


# ============================================================================
# Send: joint impedance
# ============================================================================

def test_send_joint_impedance(client, daemon):
    home = [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.7]
    client.send_joint_impedance(q_target=home)

    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "jointImpedance"
        ji = cmd.config.jointImpedance
        assert list(ji.qTarget) == home


# ============================================================================
# Send: admittance
# ============================================================================

def test_send_admittance_yaml_defaults_include_wrench_filter(client, daemon):
    """admittance.yaml's wrench_filter_alpha must reach the wire (default 1.0 =
    pass-through, matching pixi/yifan-hou)."""
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_admittance(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        ad = cmd.config.admittance
        assert ad.wrenchFilterAlpha == pytest.approx(1.0)


def test_send_admittance_wrench_filter_override(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_admittance(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        wrench_filter_alpha=0.5,
    )
    with daemon.recv_command() as cmd:
        assert cmd.config.admittance.wrenchFilterAlpha == pytest.approx(0.5)


# ============================================================================
# Send: hybrid
# ============================================================================

def test_send_hybrid_uses_yaml_defaults(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "hybrid"
        h = cmd.config.hybrid
        # Defaults from hybrid.yaml — check a few load-bearing fields land.
        assert len(list(h.tr)) == 36
        assert h.nAf == 0
        assert h.maxSpringForce  > 0.0
        assert h.maxSpringTorque > 0.0


def test_send_hybrid_per_tick_kwargs(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    import numpy as np
    Tr = np.eye(6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        n_af=2,
        Tr=Tr,
        target_wrench_Tr=[5, 0, 0, 0, 0, 0],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 2
        assert list(h.targetWrenchTr) == [5, 0, 0, 0, 0, 0]


def test_send_hybrid_tr_identity_string(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        Tr="identity",
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert list(h.tr)[:7] == [1, 0, 0, 0, 0, 0, 0]


def test_send_hybrid_rejects_bad_n_af(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    with pytest.raises(ValueError, match="n_af"):
        client.send_hybrid(
            target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
            n_af=7,
        )


def test_send_hybrid_tuning_via_profile_yaml(profile_dir, daemon):
    _write_profile(profile_dir, "hybrid.touchy", """
n_af: 1
Tr: identity
target_wrench: [10, 0, 0, 0, 0, 0]
admittance:
  M: [5, 5, 5, 0.5, 0.5, 0.5]
  K: [100, 100, 100, 10, 10, 10]
  D: [50, 50, 50, 5, 5, 5]
force_pid:
  P_trans: 0.05
  I_trans: 0.0
  D_trans: 0.0
  P_rot:   0.0
  I_rot:   0.0
  D_rot:   0.0
  I_limit: [10, 10, 10, 1, 1, 1]
stiction: [0, 0, 0, 0, 0, 0]
max_spring_force:  20.0
max_spring_torque: 5.0
impedance:
  K: [3000, 3000, 3000, 50, 50, 50]
  D: [110, 110, 110, 14, 14, 14]
q_null: [0,0,0,0,0,0,0]
K_null: 100.0
filter_alpha: 0.05
use_friction: true
""")
    r = Robot("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    r.connect()
    try:
        # Prime FT-sensor publication so the sensor gate passes.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not r.state.valid:
            daemon.publish_state(wrench_ft=(0,)*6)
            time.sleep(0.02)
        r.send_hybrid(
            target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
            profile="touchy",
        )
        with daemon.recv_command() as cmd:
            h = cmd.config.hybrid
            assert h.nAf == 1
            assert h.maxSpringForce  == pytest.approx(20.0)
            assert h.maxSpringTorque == pytest.approx(5.0)
    finally:
        r.close()


def test_send_hybrid_cache_persists(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        n_af=3,
        target_wrench_Tr=[1, 2, 3, 0, 0, 0],
    )
    with daemon.recv_command():
        pass

    # Second call without the n_af / target_wrench kwargs — cache should kick in.
    client.send_hybrid(
        target_pos=[0.6, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 3
        assert list(h.targetWrenchTr) == [1, 2, 3, 0, 0, 0]


def test_send_hybrid_yaml_default_wrench_deadband_disabled(client, daemon):
    """hybrid.yaml ships wrench_deadband all-zero — wire round-trips as length 6 of zeros (disabled)."""
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert list(h.wrenchDeadband) == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_send_hybrid_wrench_deadband_override(client, daemon):
    """Per-call kwarg overrides yaml default and lands on the wire."""
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        wrench_deadband=[0.05, 0.05, 0.05, 0.005, 0.005, 0.005],
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.wrenchDeadband) == [0.05, 0.05, 0.05, 0.005, 0.005, 0.005]


def test_send_hybrid_wrench_deadband_cached_across_calls(client, daemon):
    """First call sets wrench_deadband; second call without the kwarg uses cached value."""
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        wrench_deadband=[0.05, 0.05, 0.05, 0.005, 0.005, 0.005],
    )
    with daemon.recv_command():
        pass  # drain first command
    # Second call without the kwarg.
    client.send_hybrid(
        target_pos=[0.5, 0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.wrenchDeadband) == [0.05, 0.05, 0.05, 0.005, 0.005, 0.005]


# ============================================================================
# Validation
# ============================================================================

def test_wrong_length_position_rejected(client):
    with pytest.raises(ValueError, match="length"):
        client.send_cartesian_impedance(
            target_pos=[0.5, 0.0],   # length 2
            target_quat_xyzw=[0, 0, 0, 1],
        )


def test_wrong_length_quat_rejected(client):
    with pytest.raises(ValueError, match="length"):
        client.send_cartesian_impedance(
            target_pos=[0.5, 0, 0.4],
            target_quat_xyzw=[0, 0, 1],   # length 3
        )


def test_send_without_connect_raises():
    r = Robot("127.0.0.1", cmd_port=1, state_port=2)
    with pytest.raises(RuntimeError, match="not connected"):
        r.send_idle()


# ============================================================================
# State subscription
# ============================================================================

def test_state_initially_invalid(client):
    s = client.state
    assert s.valid is False


def test_state_updates_on_publish(client, daemon):
    seen = daemon.publish_until_received(
        client,
        controller="cartesian_impedance",
        pos=(0.1, 0.2, 0.3),
        quat_xyzw=(0, 0, 0, 1),
        timestamp=1234.5,
        running=True,
        last_error="",
    )
    assert seen, "client never received a state update"
    s = client.state
    assert s.controller == "cartesian_impedance"
    assert list(s.pos)  == [0.1, 0.2, 0.3]
    assert s.timestamp  == pytest.approx(1234.5)
    assert s.running    is True


def test_wait_for_state_returns_when_published(client, daemon):
    import threading
    pub_started = threading.Event()

    def publisher():
        pub_started.set()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            daemon.publish_state(controller="idle", pos=(1, 2, 3))
            time.sleep(0.02)

    t = threading.Thread(target=publisher, daemon=True)
    t.start()
    pub_started.wait()

    s = client.wait_for_state(timeout=2.0)
    assert s.valid is True
    assert list(s.pos) == [1.0, 2.0, 3.0]


def test_wait_for_state_times_out(client):
    with pytest.raises(TimeoutError):
        client.wait_for_state(timeout=0.2)


# ============================================================================
# Context manager
# ============================================================================

def test_context_manager_opens_and_closes(daemon):
    with Robot("127.0.0.1", cmd_port=daemon.cmd_port,
               state_port=daemon.state_port) as r:
        r.send_idle()
        with daemon.recv_command() as cmd:
            assert cmd is not None
    # After __exit__, sockets are released; another send raises.
    with pytest.raises(RuntimeError, match="not connected"):
        r.send_idle()


def test_require_ft_sensor_raises_with_op_name(client, daemon):
    """The helper's RuntimeError mentions the calling method name so the
    user can tell which API refused."""
    daemon.publish_until_received(client)
    with pytest.raises(RuntimeError, match="my_test_op:"):
        client._require_ft_sensor("my_test_op", timeout=0.1)
