"""Trajectory, failure lifecycle, and real localhost wire tests; no hardware."""
from __future__ import annotations

import csv
from dataclasses import replace
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from fr3_stack import Robot
from fr3_stack.state import State
from fr3_stack.wire import SCHEMA


SCRIPT = Path(__file__).resolve().parents[1] / 'examples' / 'joint_sysid.py'
spec = importlib.util.spec_from_file_location('joint_sysid_script', SCRIPT)
js = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = js
spec.loader.exec_module(js)
GAINS = dict(K_joint=[200.] * 7, D_joint=[20.] * 7, filter_alpha=.05, use_friction=False)


@pytest.mark.parametrize('mode', ['sine', 'chirp', 'multisine'])
def test_derivatives_and_smooth_segment_boundaries(mode):
    plan = js.Plan(mode=mode, joints=(1, 3, 7))
    trajectory = js.Trajectory(plan)
    # Check independent finite differences of each lower-order derivative.
    t = np.array([2.7, 6.2, 10.1, 20.8])
    h = 1e-5
    exact = trajectory.evaluate(t)
    left, right = trajectory.evaluate(t - h), trajectory.evaluate(t + h)
    for order in range(1, 4):
        np.testing.assert_allclose((right[order - 1] - left[order - 1]) / (2 * h), exact[order], atol=1e-8, rtol=1e-5)
    for segment in range(plan.segments):
        start = plan.hold + segment * (plan.seconds + plan.gap)
        for value in trajectory.evaluate([start, start + plan.seconds]):
            np.testing.assert_allclose(value, 0., atol=1e-12)
    for value in trajectory.evaluate([-1., plan.total + 1]):
        assert not value.any()


def test_sequential_joints_do_not_move_together():
    trajectory = js.Trajectory(js.Plan(mode='chirp', joints=(2, 6), seconds=20))
    q = trajectory.evaluate([12., 23., 34.])[0]
    assert np.flatnonzero(q[0]).tolist() == [1]
    assert np.flatnonzero(q[1]).tolist() == []
    assert np.flatnonzero(q[2]).tolist() == [5]


def test_multisine_peak_is_total_amplitude_and_seed_is_reproducible():
    p = js.Plan(mode='multisine', joints=(1, 2, 3, 4, 5, 6, 7))
    t = np.linspace(0, p.total, 10000)
    a = js.Trajectory(p).evaluate(t)[0]
    b = js.Trajectory(p).evaluate(t)[0]
    c = js.Trajectory(replace(p, seed=7)).evaluate(t)[0]
    np.testing.assert_array_equal(a, b)
    assert not np.array_equal(a, c)
    assert np.max(np.abs(a)) <= np.deg2rad(p.amplitude_deg)
    assert not np.array_equal(a[:, 0], a[:, 1])


@pytest.mark.parametrize('change', [dict(f0=float('nan')), dict(amplitude_deg=float('inf')),
    dict(joints=(1, 1)), dict(joints=(0,)), dict(joints=()), dict(ramp=11),
    dict(f0=0), dict(f1=.01), dict(tones=()), dict(seconds=-1), dict(seed=-1),
    dict(rate=0), dict(max_jerk=0), dict(seconds=601), dict(mode='chirp', f1=10)])
def test_bad_plan_is_rejected(change):
    with pytest.raises(ValueError):
        js.Trajectory(replace(js.Plan(), **change))


def test_reference_limits_include_ramps_and_inactive_joints():
    with pytest.raises(ValueError, match='repulsion'):
        js.Trajectory(js.Plan()).check([0, -.6, 0, 0, 0, 1.8, .7])
    with pytest.raises(ValueError, match='jerk'):
        js.Trajectory(js.Plan(seconds=20, ramp=.1, max_acceleration=10)).check(js.PREVIEW_Q)
    with pytest.raises(ValueError, match='velocity'):
        js.Trajectory(js.Plan(amplitude_deg=20)).check(js.PREVIEW_Q)
    with pytest.raises(ValueError, match='acceleration'):
        js.Trajectory(js.Plan(max_acceleration=.00001)).check(js.PREVIEW_Q)
    peaks = js.Trajectory(js.Plan()).check(js.PREVIEW_Q)
    assert peaks['velocity_rad_s'][0] > 0
    assert peaks['velocity_rad_s'][1:] == [0.] * 6


def test_preview_never_imports_robot_and_does_not_overwrite(tmp_path):
    output = tmp_path / 'preview'
    code = '''
import runpy, sys
class RejectRobot:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('fr3_stack', 'zmq', 'capnp')):
            raise RuntimeError('preview tried to import hardware transport')
sys.meta_path.insert(0, RejectRobot())
sys.argv = [sys.argv[1], '--host', 'must-not-connect.invalid', '--output', sys.argv[2]]
runpy.run_path(sys.argv[0], run_name='__main__')
'''
    result = subprocess.run([sys.executable, '-c', code, str(SCRIPT), str(output)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['status'] == 'preview'
    assert metadata['sysid_ready'] is False
    assert '<svg' in (output / 'preview.svg').read_text()
    with (output / 'reference.csv').open() as f:
        rows = list(csv.DictReader(f))
    assert float(rows[0]['q_ref4']) == js.PREVIEW_Q[3]
    before = (output / 'metadata.json').read_bytes()
    assert js.main(['--output', str(output)]) == 1
    assert (output / 'metadata.json').read_bytes() == before


class Clock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class StubRobot:
    def __init__(self, clock, fail=None):
        self.clock = clock
        self.fail = fail
        self.sent = []
        self.terminated = False
        self.changes = {}

    @property
    def state(self):
        fields = dict(valid=True, running=True, controller='joint_impedance',
                      q=js.PREVIEW_Q.copy(), dq=np.zeros(7), received_at=self.clock(), timestamp=1.)
        fields.update(self.changes)
        return State(**fields)

    def wait_for_state(self, timeout):
        return self.state

    def send_joint_impedance(self, target, **gains):
        self.sent.append((target.copy(), gains))
        assert len(self.sent) < 100, 'test loop stopped advancing time'
        if self.fail:
            raise self.fail

    def terminate(self):
        self.terminated = True


def tiny_plan():
    return js.Trajectory(js.Plan(mode='sine', amplitude_deg=.001, seconds=.12, ramp=.05, hold=.02, rate=100))


def test_stream_uses_actual_time_and_preserves_gains():
    clock = Clock()
    robot = StubRobot(clock)
    metadata = {}
    buffer = io.StringIO()
    js.stream(robot, tiny_plan(), js.PREVIEW_Q, GAINS, csv.writer(buffer), metadata, clock=clock, sleep=clock.sleep)
    np.testing.assert_array_equal(robot.sent[0][0], js.PREVIEW_Q)
    np.testing.assert_array_equal(robot.sent[-1][0], js.PREVIEW_Q)
    assert all(g == GAINS for _, g in robot.sent)
    assert metadata['commands_sent'] == len(robot.sent)
    rows = list(csv.reader(io.StringIO(buffer.getvalue())))
    assert all(float(b[1]) > float(a[1]) for a, b in zip(rows, rows[1:]))


@pytest.mark.parametrize('change,match', [
    (dict(received_at=0.), 'stale'), (dict(valid=False), 'valid'),
    (dict(running=False), 'healthy'), (dict(last_error='reflex'), 'healthy'),
    (dict(controller='idle'), 'joint_impedance'),
    (dict(dq=np.ones(7)), 'speed'), (dict(q=np.full(7, float('nan'))), 'finite'),
    (dict(timestamp=float('nan')), 'timestamp')])
def test_bad_feedback_cannot_send_motion(change, match):
    clock = Clock()
    robot = StubRobot(clock)
    robot.changes.update(change)
    with pytest.raises((RuntimeError, ValueError), match=match):
        js.stream(robot, tiny_plan(), js.PREVIEW_Q, GAINS, csv.writer(io.StringIO()), {}, clock=clock, sleep=clock.sleep)
    assert not robot.sent


def test_tracking_error_aborts_before_send():
    clock = Clock()
    robot = StubRobot(clock)
    robot.changes['q'] = js.PREVIEW_Q + .06
    with pytest.raises(RuntimeError, match='tracking'):
        js.stream(robot, tiny_plan(), js.PREVIEW_Q, GAINS, csv.writer(io.StringIO()), {}, clock=clock, sleep=clock.sleep)
    assert not robot.sent


def test_pause_does_not_trigger_catchup_commands():
    clock = Clock()
    robot = StubRobot(clock)
    def stalled_sleep(seconds):
        clock.sleep(seconds + .2)
    with pytest.raises(RuntimeError, match='deadline'):
        js.stream(robot, tiny_plan(), js.PREVIEW_Q, GAINS, csv.writer(io.StringIO()), {}, clock=clock, sleep=stalled_sleep)
    assert not robot.sent


@pytest.mark.parametrize('failure', [RuntimeError('send failed'), KeyboardInterrupt()])
def test_motion_failure_requests_termination_and_saves_partial_record(tmp_path, failure):
    robot = StubRobot(js.time.monotonic, failure)
    metadata = {'commands_sent': 0}
    args = SimpleNamespace(stale_after=.2, tracking_error=.05, max_lateness=.05, finish='hold')
    with pytest.raises(type(failure)):
        js.run_live(robot, tiny_plan(), GAINS, tmp_path, metadata, args)
    assert robot.terminated
    saved = json.loads((tmp_path / 'metadata.json').read_text())
    assert saved['status'] == 'aborted'
    assert saved['termination'] == 'requested_unconfirmed'
    assert saved['motion_command_attempted']


def test_preflight_failure_does_not_change_robot(tmp_path):
    robot = StubRobot(js.time.monotonic)
    robot.changes['controller'] = 'cartesian_impedance'
    args = SimpleNamespace(stale_after=.2, tracking_error=.05, max_lateness=.05, finish='hold')
    with pytest.raises(RuntimeError, match='joint_impedance'):
        js.run_live(robot, tiny_plan(), GAINS, tmp_path, {}, args)
    assert not robot.sent and not robot.terminated
    assert json.loads((tmp_path / 'metadata.json').read_text())['status'] == 'aborted'


def test_real_wire_commands_and_recording_contract(daemon_streaming, tmp_path):
    metadata = {'commands_sent': 0}
    args = SimpleNamespace(stale_after=.2, tracking_error=.05, max_lateness=.1, finish='hold')
    with Robot('127.0.0.1', daemon_streaming.cmd_port, daemon_streaming.state_port,
               command_send_timeout_ms=20) as robot:
        with daemon_streaming.publish_loop(controller='joint_impedance', q=js.PREVIEW_Q.tolist(), timestamp=123.):
            js.run_live(robot, tiny_plan(), GAINS, tmp_path, metadata, args)
    messages = daemon_streaming.drain_commands()
    assert messages
    for payload in messages:
        with SCHEMA.Command.from_bytes(payload) as command:
            assert not command.termination
            assert command.config.which() == 'jointImpedance'
            cfg = command.config.jointImpedance
            np.testing.assert_array_equal(list(cfg.kJoint), GAINS['K_joint'])
            np.testing.assert_array_equal(list(cfg.dJoint), GAINS['D_joint'])
            assert cfg.filterAlpha == GAINS['filter_alpha']
            assert not cfg.useFriction
            assert abs(cfg.qTarget[0] - js.PREVIEW_Q[0]) <= np.deg2rad(.001)
            np.testing.assert_array_equal(list(cfg.qTarget)[1:], js.PREVIEW_Q[1:])
    with (tmp_path / 'commands_and_state.csv').open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == metadata['commands_sent']
    assert metadata['status'] == 'completed'
    assert all(float(row['state_daemon_timestamp']) == 123. for row in rows)
    np.testing.assert_allclose([float(rows[-1][f'q_target{i}']) for i in range(1, 8)], js.PREVIEW_Q)


def test_log_failure_during_motion_requests_termination(tmp_path, monkeypatch):
    real_writer = js.csv.writer
    class FullDiskWriter:
        def __init__(self):
            self.count = 0
        def writerow(self, row):
            self.count += 1
            if self.count > 1:
                raise OSError('disk full')
    def writer(file, *args, **kwargs):
        if file.name.endswith('commands_and_state.csv'):
            return FullDiskWriter()
        return real_writer(file, *args, **kwargs)
    monkeypatch.setattr(js.csv, 'writer', writer)
    robot = StubRobot(js.time.monotonic)
    args = SimpleNamespace(stale_after=.2, tracking_error=.05, max_lateness=.05, finish='hold')
    with pytest.raises(OSError, match='disk full'):
        js.run_live(robot, tiny_plan(), GAINS, tmp_path, {}, args)
    assert robot.terminated
    saved = json.loads((tmp_path / 'metadata.json').read_text())
    assert saved['status'] == 'aborted'
    assert saved['commands_sent'] == 1


def test_termination_failure_is_not_reported_as_stopped():
    class Disconnected:
        def terminate(self):
            raise RuntimeError('disconnected')
    metadata = {}
    js.request_termination(Disconnected(), metadata)
    assert metadata['termination'] == 'request_failed: disconnected'


def test_failed_normal_termination_is_not_success(tmp_path):
    class DisconnectedAtEnd(StubRobot):
        def terminate(self):
            raise RuntimeError('disconnected')
    robot = DisconnectedAtEnd(js.time.monotonic)
    args = SimpleNamespace(stale_after=.2, tracking_error=.05, max_lateness=.1, finish='terminate')
    with pytest.raises(RuntimeError, match='termination request failed'):
        js.run_live(robot, tiny_plan(), GAINS, tmp_path, {}, args)
    saved = json.loads((tmp_path / 'metadata.json').read_text())
    assert saved['status'] == 'aborted'
    assert saved['termination'] == 'request_failed: disconnected'
