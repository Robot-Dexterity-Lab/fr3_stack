"""Cap'n Proto schema roundtrip tests.

These don't talk to a daemon — they verify that the schema as parsed by
pycapnp on the workstation matches what the C++ side will see (the C++ side
uses capnp_generate_cpp on the same .capnp file). If a field name or
ordinal drifts between sides, these break first.
"""
from __future__ import annotations

import pytest

from fr3_stack.robot import _SCHEMA


def test_schema_has_expected_types():
    assert hasattr(_SCHEMA, "Command")
    assert hasattr(_SCHEMA, "State")
    assert hasattr(_SCHEMA, "CartesianImpedanceCmd")
    assert hasattr(_SCHEMA, "JointImpedanceCmd")


def test_command_idle_roundtrip():
    cmd = _SCHEMA.Command.new_message()
    cmd.termination = False
    idle = cmd.config.init("idle")
    idle.dRate = [0.0] * 7
    idle.useFriction = False
    payload = cmd.to_bytes()

    with _SCHEMA.Command.from_bytes(payload) as parsed:
        assert parsed.termination is False
        assert parsed.config.which() == "idle"
        assert list(parsed.config.idle.dRate) == [0.0] * 7
        assert parsed.config.idle.useFriction is False


def test_command_cartesian_impedance_roundtrip():
    cmd = _SCHEMA.Command.new_message()
    cart = cmd.config.init("cartesianImpedance")
    cart.targetPos      = [0.5, 0.0, 0.4]
    cart.targetQuatXyzw = [0.0, 0.0, 0.0, 1.0]
    cart.k              = [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]
    cart.d              = [ 28.0,  28.0,  28.0,  9.0,  9.0,  9.0]
    cart.qNull          = [0.0] * 7
    cart.kNull          = 10.0
    cart.filterAlpha    = 0.05
    payload = cmd.to_bytes()

    with _SCHEMA.Command.from_bytes(payload) as parsed:
        assert parsed.config.which() == "cartesianImpedance"
        ci = parsed.config.cartesianImpedance
        assert list(ci.targetPos)      == [0.5, 0.0, 0.4]
        assert list(ci.targetQuatXyzw) == [0.0, 0.0, 0.0, 1.0]
        assert list(ci.k)[:3]          == [200.0, 200.0, 200.0]
        assert ci.kNull       == pytest.approx(10.0)
        assert ci.filterAlpha == pytest.approx(0.05)


def test_command_joint_impedance_roundtrip():
    cmd = _SCHEMA.Command.new_message()
    ji = cmd.config.init("jointImpedance")
    ji.qTarget     = [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.7]
    ji.kJoint      = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0]
    ji.dJoint      = [ 50.0,  50.0,  50.0,  50.0,  30.0,  25.0, 15.0]
    ji.filterAlpha = 0.05
    payload = cmd.to_bytes()

    with _SCHEMA.Command.from_bytes(payload) as parsed:
        assert parsed.config.which() == "jointImpedance"
        ji_p = parsed.config.jointImpedance
        assert list(ji_p.qTarget) == [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.7]
        assert list(ji_p.kJoint)[-1] == pytest.approx(50.0)


def test_state_roundtrip():
    st = _SCHEMA.State.new_message()
    st.controller = "cartesian_impedance"
    st.pos        = [0.5, 0.1, 0.4]
    st.quatXyzw   = [0.0, 0.0, 0.0, 1.0]
    st.q          = [0.0] * 7
    st.dq         = [0.0] * 7
    st.wrenchExt  = [0.0] * 6
    st.timestamp  = 12.345
    st.running    = True
    st.lastError  = ""
    payload = st.to_bytes()

    with _SCHEMA.State.from_bytes(payload) as parsed:
        assert parsed.controller == "cartesian_impedance"
        assert list(parsed.pos)  == [0.5, 0.1, 0.4]
        assert parsed.timestamp  == pytest.approx(12.345)
        assert parsed.running is True
        assert parsed.lastError == ""


def test_state_carries_error_field():
    st = _SCHEMA.State.new_message()
    st.controller = "idle"
    st.lastError  = "communication_constraints_violation"
    st.running    = False
    payload = st.to_bytes()

    with _SCHEMA.State.from_bytes(payload) as parsed:
        assert parsed.lastError == "communication_constraints_violation"
        assert parsed.running is False


def test_command_hybrid_wrench_deadband_roundtrip():
    """HybridCmd.wrenchDeadband round-trips length 0 (disabled) and length 6."""
    # Length 0 (default / disabled).
    cmd = _SCHEMA.Command.new_message()
    cmd.config.init("hybrid")
    payload = cmd.to_bytes()
    with _SCHEMA.Command.from_bytes(payload) as parsed:
        assert list(parsed.config.hybrid.wrenchDeadband) == []

    # Length 6 (per-axis).
    cmd = _SCHEMA.Command.new_message()
    h = cmd.config.init("hybrid")
    h.wrenchDeadband = [0.05, 0.05, 0.05, 0.005, 0.005, 0.005]
    payload = cmd.to_bytes()
    with _SCHEMA.Command.from_bytes(payload) as parsed:
        assert list(parsed.config.hybrid.wrenchDeadband) == [
            0.05, 0.05, 0.05, 0.005, 0.005, 0.005
        ]
