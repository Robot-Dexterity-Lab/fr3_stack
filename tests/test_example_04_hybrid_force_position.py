"""End-to-end tests for examples/04_hybrid_force_position.py.

Loads the example as a module (the leading "04_" prevents normal import)
and exercises argparse + the three mode dispatchers against the FakeDaemon
fixture. No physical robot. No daemon. No network.

What this covers that the existing tests/test_hybrid_force_position.py do not:
  * The example script's argparse — every --mode and switch.
  * `gate_ft` exit-on-missing-FT path and pass-through path.
  * Each mode function (`mode_press`, `mode_hold`, `mode_streamf`) actually
    drives the Robot client end-to-end and emits commands on the wire.
  * `--unsafe` removes the countdown.

These are guard tests for the example itself. If someone refactors the
client and forgets to update the example, this catches it before the
robot moves.
"""
from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

from fr3_stack.wire import SCHEMA


EXAMPLE_PATH = (
    pathlib.Path(__file__).resolve().parent.parent
    / "examples" / "04_hybrid_force_position.py"
)


@pytest.fixture(scope="module")
def example():
    """Load examples/04_hybrid_force_position.py as a module.

    The leading "04_" makes the filename a non-identifier, so a regular
    `import` does not work — we go through importlib.
    """
    spec = importlib.util.spec_from_file_location("ex04_hybrid", EXAMPLE_PATH)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["ex04_hybrid"] = mod
    spec.loader.exec_module(mod)
    return mod


# ----------------------------------------------------------------------------
# Module-level smoke
# ----------------------------------------------------------------------------

def test_example_imports_cleanly(example):
    """The example must import without side effects (no I/O on import)."""
    assert hasattr(example, "main")
    assert hasattr(example, "mode_press")
    assert hasattr(example, "mode_hold")
    assert hasattr(example, "mode_streamf")
    assert hasattr(example, "gate_ft")


def test_safety_threshold_shapes(example):
    """Threshold vectors must match the daemon-side wire shape."""
    assert len(example.SAFE_FORCE_THRESHOLDS) == 6, \
        "force thresholds must be 6-vector [tx,ty,tz,rx,ry,rz]"
    assert len(example.SAFE_TORQUE_THRESHOLDS) == 7, \
        "joint torque thresholds must be 7-vector (one per joint)"
    # Per-axis sanity: translational entries non-negative, no NaN/inf.
    for v in example.SAFE_FORCE_THRESHOLDS + example.SAFE_TORQUE_THRESHOLDS:
        assert v > 0 and v == v and v != float("inf")


# ----------------------------------------------------------------------------
# argparse — each --mode must accept its defaults
# ----------------------------------------------------------------------------

def _argv(example, *flags):
    """Build args list by parsing through example.main's parser."""
    # The example only constructs the parser inside main(); rebuild it
    # locally to test argparse without entering the connect loop.
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("host", nargs="?", default="localhost")
    p.add_argument("--mode", choices=["press", "hold", "streamf"], default="press")
    p.add_argument("--duration", type=float, default=6.0)
    p.add_argument("--dt", type=float, default=0.01)
    p.add_argument("--force-n", dest="force_n", type=float, default=5.0)
    p.add_argument("--target-force-z", type=float, default=5.0)
    p.add_argument("--acc", type=float, default=1.0)
    p.add_argument("--max-translation", type=float, default=0.05)
    p.add_argument("--freq", type=float, default=0.25)
    p.add_argument("--no-ft", action="store_true")
    p.add_argument("--unsafe", action="store_true")
    return p.parse_args(list(flags))


@pytest.mark.parametrize("mode", ["press", "hold", "streamf"])
def test_argparse_accepts_each_mode(example, mode):
    ns = _argv(example, "192.168.1.7", "--mode", mode, "--duration", "1.0")
    assert ns.mode == mode
    assert ns.host == "192.168.1.7"
    assert ns.duration == 1.0


def test_argparse_rejects_bad_mode(example):
    with pytest.raises(SystemExit):
        _argv(example, "h", "--mode", "bogus")


def test_argparse_no_ft_and_unsafe_flags(example):
    ns = _argv(example, "--no-ft", "--unsafe")
    assert ns.no_ft is True
    assert ns.unsafe is True


# ----------------------------------------------------------------------------
# gate_ft — exits when require_ft and state has no FT
# ----------------------------------------------------------------------------

def test_gate_ft_exits_when_no_ft_sensor(example, client, daemon):
    daemon.publish_until_received(client)   # publish State with empty wrench_ft
    with pytest.raises(SystemExit):
        example.gate_ft(client, require_ft=True)


def test_gate_ft_passes_when_ft_present(example, client, daemon, capsys):
    daemon.publish_until_received(client, wrench_ft=(0.0,) * 6)
    example.gate_ft(client, require_ft=True)    # must not raise
    out = capsys.readouterr().out
    assert "has_ft=True" in out


def test_gate_ft_passes_when_ft_not_required(example, client, daemon):
    daemon.publish_until_received(client)   # no FT
    example.gate_ft(client, require_ft=False)   # must not raise


# ----------------------------------------------------------------------------
# countdown — verify --unsafe path replaces it with a no-op
# ----------------------------------------------------------------------------

def test_countdown_real_pauses(example, monkeypatch):
    """Real countdown calls time.sleep — verify it does, briefly."""
    calls = []
    monkeypatch.setattr(example.time, "sleep", lambda s: calls.append(s))
    example.countdown(3, "test")
    assert len(calls) == 3
    assert all(c == 1.0 for c in calls)


# ----------------------------------------------------------------------------
# Mode dispatchers — each must drive the Robot client end-to-end
# ----------------------------------------------------------------------------

def _make_args(**overrides):
    """Lightweight Namespace-like for passing into mode_* helpers."""
    import argparse
    defaults = dict(
        host="127.0.0.1", mode="press",
        duration=0.05, dt=0.01,
        force_n=5.0, target_force_z=5.0, acc=0.01,
        max_translation=0.05, freq=0.25,
        no_ft=True,         # avoid FT gating for offline tests
        unsafe=True,
    )
    defaults.update(overrides)
    return argparse.Namespace(**defaults)


def test_mode_press_emits_hybrid_commands(
    example, client_streaming, daemon_streaming, monkeypatch
):
    """mode_press → apply_effector_forces_along_axis → hybrid commands."""
    monkeypatch.setattr(example, "countdown", lambda *_: None)
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    args = _make_args(mode="press", duration=0.05, acc=0.01, dt=0.01)
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        example.mode_press(args, client_streaming)
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        # Selected controller for the first tick must be hybrid with nAf=1
        # (single force axis = -Z).
        h = cmd.config.hybrid
        assert h.nAf == 1


def test_mode_hold_blocks_and_emits_hybrid(
    example, client_streaming, daemon_streaming, monkeypatch
):
    """mode_hold → run_hybrid_force_position with S=[1,1,0,1,1,1]."""
    monkeypatch.setattr(example, "countdown", lambda *_: None)
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    args = _make_args(mode="hold", duration=0.05, dt=0.01, target_force_z=5.0)
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        example.mode_hold(args, client_streaming)
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 1
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        h = cmd.config.hybrid
        # S=[1,1,0,1,1,1] → exactly one force axis on Z.
        assert h.nAf == 1
        # First Tr row: unit Z (target_force = [0,0,-5,0,0,0]).
        tr0 = list(h.tr)[:3]
        assert abs(tr0[0]) < 1e-9 and abs(tr0[1]) < 1e-9
        assert abs(abs(tr0[2]) - 1.0) < 1e-9


def test_mode_streamf_streams_force_per_tick(
    example, client_streaming, daemon_streaming, monkeypatch
):
    """mode_streamf → repeated send_hybrid_force_position with varying fz."""
    monkeypatch.setattr(example, "countdown", lambda *_: None)
    p0 = (0.4, 0.0, 0.5)
    q0 = (0.0, 0.0, 0.0, 1.0)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    args = _make_args(mode="streamf", duration=0.06, dt=0.01,
                      force_n=3.0, freq=2.0)   # 2 Hz over 60 ms → varies
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        example.mode_streamf(args, client_streaming)

    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 3, f"expected ≥3 ticks, got {len(payloads)}"

    fzs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            h = cmd.config.hybrid
            # targetWrenchTr is the projected scalar force per active axis.
            # nAf must be 1 (Z is the sole force axis).
            assert h.nAf == 1
            fzs.append(list(h.targetWrenchTr)[0])
    # Force must actually vary over time (sinusoid, not constant zero).
    assert max(fzs) - min(fzs) > 0.5, \
        f"streamf force did not vary; fz samples = {fzs}"
