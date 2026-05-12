"""Mock tests for the frankapy-parity hybrid force-position helpers.

Spec: docs/superpowers/specs/2026-05-10-hybrid-force-position-mock-tests-design.md

These exercise the Python client surface only — the C++ controller math
and real-machine behavior are explicitly out of scope. Mock-green is
necessary but not sufficient; run the §6 hardware checklist in the spec
before declaring the feature stable.
"""
from __future__ import annotations

import time

import numpy as np
import pytest

from fr3_stack.wire import SCHEMA


# ============================================================================
# send_hybrid_force_position — one-shot per-tick command
# ============================================================================

def test_send_hybrid_force_position_defaults_to_pure_position(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        target_force=[0.0]*6,
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 0
        assert list(h.targetWrenchTr) == [0.0]*6
        assert len(list(h.tr)) == 36


def test_send_hybrid_force_position_z_force_encodes(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        target_force=[0.0, 0.0, -5.0, 0.0, 0.0, 0.0],
        S=[1, 1, 0, 1, 1, 1],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 1
        # First Tr row is the unit vector along the single force axis (Z).
        tr_row0 = list(h.tr)[:6]
        assert tr_row0 == pytest.approx([0, 0, 1, 0, 0, 0])
        assert list(h.targetWrenchTr)[0] == pytest.approx(-5.0)


def test_send_hybrid_force_position_kps_persist_in_cache(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    K = [800.0, 800.0, 800.0, 40.0, 40.0, 40.0]

    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        target_force=[0.0]*6,
        position_kps_cart=K,
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.k) == pytest.approx(K)

    # Second call without K — cache should still have the previous value.
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        target_force=[0.0]*6,
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.k) == pytest.approx(K)


def test_send_hybrid_force_position_force_thresholds_semantics(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)

    # 1) Set thresholds.
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0]*6,
        force_thresholds=[30.0]*6,
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.forceThresholds) == pytest.approx([30.0]*6)

    # 2) Pass None → cache reused.
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0]*6,
    )
    with daemon.recv_command() as cmd:
        assert list(cmd.config.hybrid.forceThresholds) == pytest.approx([30.0]*6)

    # 3) Pass [] → cleared.
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0]*6,
        force_thresholds=[],
    )
    with daemon.recv_command() as cmd:
        assert len(list(cmd.config.hybrid.forceThresholds)) == 0


def test_send_hybrid_force_position_ft_gate_raises_without_wrench(client, daemon):
    # Daemon publishes state with NO wrench_ft (default kwarg is `()`).
    daemon.publish_until_received(client)  # wrench_ft defaults to ()
    with pytest.raises(RuntimeError, match="FT sensor"):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
            target_force=[0.0]*6,
            require_ft_sensor=True,
            ft_sensor_timeout=0.2,
        )


def test_send_hybrid_force_position_ft_gate_passes_with_zero_wrench(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    # Should not raise.
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4], target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0]*6,
        require_ft_sensor=True,
    )
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "hybrid"


# ============================================================================
# run_hybrid_force_position — blocking streaming loop
# ============================================================================

def test_run_hybrid_force_position_rejects_both_targets(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    with pytest.raises(ValueError, match="at most one"):
        client.run_hybrid_force_position(
            duration=0.05,
            target_poses=[([0.5, 0, 0.4], [0, 0, 0, 1])],
            target_fn=lambda t: ([0.5, 0, 0.4], [0, 0, 0, 1]),
        )


@pytest.mark.parametrize("duration,dt", [
    (0.0,  0.01),
    (-1.0, 0.01),
    (0.05, 0.0),
    (0.05, -0.01),
])
def test_run_hybrid_force_position_rejects_bad_duration_or_dt(
    client, daemon, duration, dt
):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    with pytest.raises(ValueError):
        client.run_hybrid_force_position(duration=duration, dt=dt)


def test_run_hybrid_force_position_holds_current_pose(
    client_streaming, daemon_streaming
):
    p0 = (0.4, 0.1, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02, require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            assert list(cmd.config.hybrid.targetPos) == pytest.approx(list(p0))


def test_run_hybrid_force_position_target_fn_per_tick(
    client_streaming, daemon_streaming
):
    p0 = np.array([0.4, 0.1, 0.5])
    q0 = [0.0, 0.0, 0.0, 1.0]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,)*6,
    )

    def fn(t):
        return (p0 + np.array([0.01 * t, 0.0, 0.0])).tolist(), q0

    with daemon_streaming.publish_loop(
        pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.08, dt=0.02, target_fn=fn, require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    xs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            xs.append(list(cmd.config.hybrid.targetPos)[0])
    assert len(xs) >= 2
    # Monotonic non-decreasing (tolerant to scheduler ties).
    for a, b in zip(xs, xs[1:]):
        assert b >= a - 1e-9


def test_run_hybrid_force_position_rejects_short_target_poses(client, daemon):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    # Need ceil(0.06/0.02)=3 entries; supply 1.
    with pytest.raises(ValueError, match="target_poses"):
        client.run_hybrid_force_position(
            duration=0.06, dt=0.02,
            target_poses=[([0.5, 0, 0.4], [0, 0, 0, 1])],
        )


def test_run_hybrid_force_position_ft_gate_runs_only_once(
    client_streaming, daemon_streaming
):
    # Seed once with wrench_ft, then publish_loop without it.
    daemon_streaming.publish_until_received(
        client_streaming,
        pos=(0.4, 0.0, 0.5), quat_xyzw=(0,0,0,1), wrench_ft=(0,)*6,
    )
    with daemon_streaming.publish_loop(
        pos=(0.4, 0.0, 0.5), quat_xyzw=(0,0,0,1),
        # NOTE: no wrench_ft. If FT-gate ran per tick this would raise.
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02, require_ft_sensor=True,
        )
    # Got here without exception → gate only fired up-front.
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2


# ============================================================================
# apply_effector_forces_along_axis — single-axis trapezoidal push
# ============================================================================

@pytest.mark.parametrize("kwargs,match", [
    (dict(run_duration=0.0,  acc_duration=0.01, max_translation=0.05,
          forces=[0,0,-1]),  "run_duration"),
    (dict(run_duration=0.04, acc_duration=0.03, max_translation=0.05,
          forces=[0,0,-1]),  "acc_duration"),
    (dict(run_duration=0.04, acc_duration=-0.01, max_translation=0.05,
          forces=[0,0,-1]),  "acc_duration"),
    (dict(run_duration=0.04, acc_duration=0.01, max_translation=0.0,
          forces=[0,0,-1]),  "max_translation"),
    (dict(run_duration=0.04, acc_duration=0.01, max_translation=0.05,
          forces=[0,0,0]),   "magnitude"),
])
def test_apply_effector_forces_validation(client, daemon, kwargs, match):
    daemon.publish_until_received(client, wrench_ft=(0,)*6)
    with pytest.raises(ValueError, match=match):
        client.apply_effector_forces_along_axis(**kwargs)


def test_apply_effector_forces_axis_encodes_into_tr(
    client_streaming, daemon_streaming
):
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[0.0, 0.0, -5.0], dt=0.01, require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 1
        # First Tr row is the unit force axis (-Z).
        assert list(h.tr)[:3] == pytest.approx([0.0, 0.0, -1.0])


def test_apply_effector_forces_trapezoid_ramp_shape(
    client_streaming, daemon_streaming
):
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    )
    mag = 5.0
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.06, acc_duration=0.02, max_translation=0.05,
            forces=[0.0, 0.0, -mag], dt=0.01, require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    fzs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            fzs.append(list(cmd.config.hybrid.targetWrenchTr)[0])
    # Drop trailing zero command(s) — the explicit hand-off at the end.
    while fzs and abs(fzs[-1]) < 1e-9:
        fzs.pop()

    assert len(fzs) >= 4, f"expected ≥4 ramp samples, got {fzs}"
    # First tick: deep in ramp-up.
    assert abs(fzs[0]) < 0.3 * mag, f"ramp-up start too high: {fzs[0]}"
    # Some tick hits the plateau.
    assert any(abs(v) > 0.9 * mag for v in fzs), f"never reached peak: {fzs}"
    # Last loop tick: in ramp-down (not plateau).
    assert abs(fzs[-1]) < 0.7 * mag, f"ramp-down end too high: {fzs[-1]}"


def test_apply_effector_forces_final_zero_command(
    client_streaming, daemon_streaming
):
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[0.0, 0.0, -5.0], dt=0.01, require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    with SCHEMA.Command.from_bytes(payloads[-1]) as cmd:
        h = cmd.config.hybrid
        assert list(h.targetWrenchTr) == pytest.approx([0.0]*6)
        assert list(h.targetPos) == pytest.approx(list(p0))


def test_apply_effector_forces_drift_abort_raises(
    client_streaming, daemon_streaming
):
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    # Seed at p0.
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,)*6,
    )
    # After 20 ms, jump pos far enough to trip max_translation=0.05.
    def pos_fn(t):
        return (0.6, 0.0, 0.5) if t > 0.02 else p0

    with daemon_streaming.publish_loop(
        period=0.005, pos_fn=pos_fn, quat_xyzw=q0, wrench_ft=(0,)*6,
    ):
        with pytest.raises(RuntimeError, match="max_translation"):
            client_streaming.apply_effector_forces_along_axis(
                run_duration=0.20, acc_duration=0.05, max_translation=0.05,
                forces=[0.0, 0.0, -5.0], dt=0.01, require_ft_sensor=False,
            )


def test_apply_effector_forces_ft_gate_raises_without_wrench(client, daemon):
    daemon.publish_until_received(client)  # no wrench_ft
    with pytest.raises(RuntimeError, match="FT sensor"):
        client.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[0.0, 0.0, -5.0], dt=0.01, require_ft_sensor=True,
        )
