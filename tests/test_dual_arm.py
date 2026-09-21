"""Coordination tests; no physical robot connections."""
import time
from contextlib import ExitStack

import numpy as np
import pytest

from fr3_stack import (ArmSnapshot, CoordinationError, CoordinationState,
                       DualArmCoordinator, Pose, Robot, RobotArmEndpoint)
from tests.conftest import FakeDaemon


class Clock:
    def __init__(self): self.now = 10.0
    def __call__(self): return self.now
    def sleep(self, dt): self.now += dt


class Endpoint:
    def __init__(self, clock):
        self.clock = clock
        self.age = 0.0
        self.error = ""
        self.sent = []
        self.stops = 0
        self.fail_send = False
        self.fail_stop = False

    def snapshot(self):
        return ArmSnapshot(Pose.identity(), self.clock() - self.age, error=self.error)

    def send(self, target):
        if self.fail_send: raise RuntimeError("transport failure")
        self.sent.append(target)

    def stop(self):
        self.stops += 1
        if self.fail_stop: raise RuntimeError("stop transport failure")


@pytest.fixture
def rig():
    clock = Clock()
    left, right = Endpoint(clock), Endpoint(clock)
    c = DualArmCoordinator(left, right, clock=clock, sleep=clock.sleep)
    return c, left, right, clock


def test_pair_validation_precedes_either_send(rig):
    c, left, right, _ = rig
    c.arm()
    with pytest.raises(CoordinationError):
        c.send(Pose.identity(), Pose([np.nan, 0, 0], [0, 0, 0, 1]))
    assert not left.sent and not right.sent
    assert left.stops == right.stops == 1
    assert c.state == CoordinationState.FAULT


def test_partial_send_failure_stops_both_and_latches(rig):
    c, left, right, _ = rig
    c.arm()
    right.fail_send = True
    left.fail_stop = True
    with pytest.raises(CoordinationError, match="transport failure"):
        c.send(Pose.identity(), Pose.identity())
    assert len(left.sent) == 1 and not right.sent
    assert left.stops == right.stops == 1
    assert "left" in c.stop_errors
    assert c.sequence == 0
    with pytest.raises(CoordinationError, match="arm"):
        c.send(Pose.identity(), Pose.identity())
    right.fail_send = left.fail_stop = False
    c.arm()
    assert c.send(Pose.identity(), Pose.identity()) == 1


@pytest.mark.parametrize("age,error", [(0.2, ""), (0, "reflex"), (-0.1, "")])
def test_unhealthy_peer_prevents_dispatch(rig, age, error):
    c, left, right, _ = rig
    c.arm()
    right.age, right.error = age, error
    with pytest.raises(CoordinationError):
        c.send(Pose.identity(), Pose.identity())
    assert not left.sent and not right.sent
    assert left.stops == right.stops == 1


def test_state_reception_skew(rig):
    c, left, right, _ = rig
    right.age = 0.08
    with pytest.raises(CoordinationError, match="skew"):
        c.arm()


def test_run_uses_one_timebase_and_stops(rig):
    c, left, right, clock = rig
    c.arm()
    times = []
    def trajectory(t):
        times.append(t)
        return Pose([t, 0, 0], [0, 0, 0, 1]), Pose([-t, 0, 0], [0, 0, 0, 1])
    c.run(trajectory, duration=0.25, rate_hz=10)
    assert times == pytest.approx([0, 0.1, 0.2])
    assert [p.pos[0] for p in left.sent] == pytest.approx(times)
    assert [p.pos[0] for p in right.sent] == pytest.approx([-t for t in times])
    assert left.stops == right.stops == 1
    assert c.state == CoordinationState.DISARMED


@pytest.mark.parametrize("error", [RuntimeError("callback"), KeyboardInterrupt()])
def test_callback_failure_and_interrupt_stop_both(rig, error):
    c, left, right, _ = rig
    c.arm()
    def trajectory(t): raise error
    with pytest.raises(type(error)):
        c.run(trajectory, duration=1)
    assert c.state == CoordinationState.FAULT
    assert left.stops == right.stops == 1


def test_overrun_skips_missed_ticks(rig):
    c, left, right, clock = rig
    c.arm()
    times = []
    def trajectory(t):
        times.append(t)
        clock.now += 0.25
        return Pose.identity(), Pose.identity()
    c.run(trajectory, duration=0.65, rate_hz=10)
    assert times == pytest.approx([0, 0.3, 0.6])


def test_two_real_wire_endpoints_and_disconnect():
    with ExitStack() as stack:
        daemons = [FakeDaemon(), FakeDaemon()]
        for d in daemons: stack.callback(d.close)
        robots = [stack.enter_context(Robot("127.0.0.1", cmd_port=d.cmd_port,
                    state_port=d.state_port, command_send_timeout_ms=100)) for d in daemons]
        for d in daemons:
            stack.enter_context(d.publish_loop(pos=(0.4, 0, 0.5)))
        for r in robots: r.wait_for_state()
        stopped = []
        endpoints = [RobotArmEndpoint(r, stop_action=lambda r: stopped.append(r)) for r in robots]
        c = DualArmCoordinator(*endpoints, max_state_age=1, max_state_skew=1)
        c.arm()
        assert c.send(Pose([0.3, 0, 0.5], [0, 0, 0, 1]),
                      Pose([0.6, 0, 0.5], [0, 0, 0, 1])) == 1
        for d, x in zip(daemons, [0.3, 0.6]):
            with d.recv_command() as cmd:
                assert cmd is not None
                assert list(cmd.config.cartesianImpedance.targetPos) == [x, 0, 0.5]
        assert robots[0].state.received_at <= time.monotonic()
        robots[0].close()
        with pytest.raises(CoordinationError, match="no state"):
            c.observe()
        assert stopped == robots


def test_robot_send_timeout_without_peer():
    # Bind an ephemeral endpoint, close it, and connect without any daemon.
    daemon = FakeDaemon()
    port = daemon.cmd_port
    daemon.close()
    import zmq
    with Robot("127.0.0.1", cmd_port=port, command_send_timeout_ms=20) as robot:
        with pytest.raises(zmq.Again):
            robot.send_idle()


def test_adapter_rejects_unbounded_transport():
    with pytest.raises(ValueError, match="finite"):
        RobotArmEndpoint(Robot("127.0.0.1"), stop_action=lambda r: None)


def test_callback_finishing_after_duration_does_not_send(rig):
    c, left, right, clock = rig
    c.arm()
    def slow(t):
        clock.now += 2
        return Pose.identity(), Pose.identity()
    c.run(slow, duration=1)
    assert not left.sent and not right.sent
    assert left.stops == right.stops == 1


def test_explicit_stop_reports_failure_and_still_stops_peer(rig):
    c, left, right, _ = rig
    c.arm()
    left.fail_stop = True
    with pytest.raises(CoordinationError, match="stop failed"):
        c.stop()
    assert left.stops == right.stops == 1
    assert c.state == CoordinationState.FAULT


def test_direct_dispatch_interruption_stops_both(rig):
    c, left, right, _ = rig
    c.arm()
    def interrupt(target): raise KeyboardInterrupt()
    right.send = interrupt
    with pytest.raises(KeyboardInterrupt):
        c.send(Pose.identity(), Pose.identity())
    assert left.stops == right.stops == 1
    assert c.state == CoordinationState.FAULT
