"""Recording client validation and real Python -> Capnp -> C++ REP integration.

Set FR3_RECORDING_TEST_SERVER to the hardware-free CMake/manual harness binary.
No test connects to a robot or opens its FCI interface.
"""
from __future__ import annotations

import csv
import errno
import json
import os
import queue
import selectors
import subprocess
import threading
import time

import pytest
import zmq

from fr3_stack import Robot, RecordingError, RecordingTimeout
from fr3_stack.wire import SCHEMA


@pytest.fixture
def recording_server(tmp_path):
    binary = os.environ.get("FR3_RECORDING_TEST_SERVER")
    if not binary:
        pytest.skip("set FR3_RECORDING_TEST_SERVER for C++ wire integration")
    with (tmp_path / "service.log").open("w") as log:
        process = subprocess.Popen([binary], stdout=subprocess.PIPE, stderr=log, text=True)
        try:
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ)
                assert selector.select(5), "C++ recording harness did not start"
            endpoint = process.stdout.readline().strip()
            assert endpoint.startswith("tcp://127.0.0.1:"), endpoint
            yield int(endpoint.rsplit(":", 1)[1])
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            process.stdout.close()


def rows(path):
    with path.open() as file:
        return list(csv.DictReader(file))


def test_recording_start_stop_restart_without_motion_socket(recording_server, tmp_path):
    robot = Robot("127.0.0.1", recording_port=recording_server)
    assert not robot.recording_status().active
    first = robot.start_recording(tmp_path / "first.csv")
    assert first.active and first.ok and not first.complete
    assert first.recording_id == robot.recording_id
    time.sleep(0.025)
    stopped = robot.stop_recording()
    assert stopped.complete and stopped.finished and not stopped.active
    data = rows(tmp_path / "first.csv")
    assert stopped.written == len(data) > 0
    assert [int(row["seq"]) for row in data] == list(range(len(data)))
    assert float(data[0]["t_s"]) == 0
    assert len(data[0]) == 80
    assert robot.stop_recording() == stopped
    time.sleep(0.015)
    assert rows(tmp_path / "first.csv") == data

    second = robot.start_recording(tmp_path / "second.csv")
    assert second.recording_id != first.recording_id
    time.sleep(0.015)
    final = robot.stop_recording()
    assert final.complete and final.written > 0
    data2 = rows(tmp_path / "second.csv")
    assert data2[0]["seq"] == "0" and float(data2[0]["t_s"]) == 0
    assert float(data2[0]["q0"]) > float(data[-1]["q0"])
    assert robot._cmd_sock is None and robot._state_sock is None


def test_recording_does_not_send_or_change_motion_commands(recording_server, daemon):
    with Robot("127.0.0.1", daemon.cmd_port, daemon.state_port,
               recording_port=recording_server) as robot:
        q = [0.0, -0.5, 0.0, -1.8, 0.0, 1.8, 0.7]
        robot.send_joint_impedance(q, K_joint=[123.0] * 7)
        with daemon.recv_command() as command:
            assert command.config.which() == "jointImpedance"
        # Use harness-generated frames; the motion fake receives no recording commands.
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            robot.start_recording(directory + "/data.csv")
            robot.stop_recording()
        with daemon.recv_command(timeout=0.03) as command:
            assert command is None
        robot.send_joint_impedance(q)
        with daemon.recv_command() as command:
            assert list(command.config.jointImpedance.kJoint) == [123.0] * 7


def test_recording_refuses_busy_stale_stop_and_old_retries(recording_server, tmp_path):
    robot = Robot("127.0.0.1", recording_port=recording_server)
    one = robot.start_recording(tmp_path / "one.csv", recording_id="one")
    assert robot.start_recording(tmp_path / "one.csv", recording_id="one").active
    with pytest.raises(RecordingError, match="already active"):
        robot.start_recording(tmp_path / "busy.csv", recording_id="busy")
    assert robot.recording_id == "one"
    assert robot.stop_recording().complete
    two = robot.start_recording(tmp_path / "two.csv", recording_id="two")
    with pytest.raises(RecordingError, match="mismatch"):
        robot.stop_recording(one.recording_id)
    assert robot.recording_status().recording_id == two.recording_id
    assert robot.recording_status().active
    assert robot.stop_recording().complete
    with pytest.raises(RecordingError, match="already used"):
        robot.start_recording(tmp_path / "one.csv", recording_id="one")
    assert not (tmp_path / "busy.csv").exists()


def test_existing_file_and_missing_parent_fail_without_truncation(recording_server, tmp_path):
    robot = Robot("127.0.0.1", recording_port=recording_server)
    existing = tmp_path / "existing.csv"
    existing.write_text("preserve me\n")
    with pytest.raises(RecordingError) as error:
        robot.start_recording(existing)
    assert error.value.status.error_code == errno.EEXIST
    assert not error.value.status.active
    assert existing.read_text() == "preserve me\n"
    with pytest.raises(RecordingError) as error:
        robot.start_recording(tmp_path / "missing" / "data.csv")
    assert error.value.status.error_code == errno.ENOENT
    assert robot.start_recording(tmp_path / "recovered.csv").active
    assert robot.stop_recording().complete


def test_separate_client_can_query_and_stop_by_id(recording_server, tmp_path):
    owner = Robot("127.0.0.1", recording_port=recording_server)
    first = owner.start_recording(tmp_path / "run.csv")
    owner.close()  # closing a Python client is not a recording stop
    other = Robot("127.0.0.1", recording_port=recording_server)
    assert other.recording_status().active
    with pytest.raises(ValueError, match="recording_id"):
        other.stop_recording()
    assert other.stop_recording(first.recording_id).complete


def test_malformed_request_does_not_kill_service(recording_server):
    with zmq.Context.instance().socket(zmq.REQ) as socket:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, 1000)
        socket.connect(f"tcp://127.0.0.1:{recording_server}")
        for payload in (b"invalid", b"\xff" * 16):
            socket.send(payload)
            with SCHEMA.RecordingReply.from_bytes(socket.recv()) as reply:
                assert not reply.accepted and reply.error
        request = SCHEMA.RecordingRequest.new_message(action="status")
        socket.send_multipart([request.to_bytes(), b"extra"])
        with SCHEMA.RecordingReply.from_bytes(socket.recv()) as reply:
            assert not reply.accepted and "single frame" in reply.error
    assert not Robot("127.0.0.1", recording_port=recording_server).recording_status().active


@pytest.mark.parametrize("path", ["relative.csv", "", "/bad\0path", "/" + "x" * 4096])
def test_recording_path_validation_before_network(path):
    robot = Robot("127.0.0.1")
    with pytest.raises(ValueError):
        robot.start_recording(path)
    assert robot.recording_id is None and robot._cmd_sock is None


@pytest.mark.parametrize("value", ["", "bad id", "x" * 129, 5])
def test_recording_id_validation_before_network(value):
    with pytest.raises(ValueError):
        Robot("127.0.0.1").start_recording("/tmp/test.csv", recording_id=value)


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
def test_recording_timeout_validation_before_network(timeout):
    robot = Robot("127.0.0.1")
    with pytest.raises(ValueError):
        robot.start_recording("/tmp/test.csv", timeout=timeout)
    assert robot.recording_id is None


def test_timeout_is_unconfirmed_and_next_request_uses_fresh_socket():
    endpoint = queue.Queue()
    errors = []
    received = threading.Event()
    release_reply = threading.Event()
    client_finished = threading.Event()

    def peer():
        try:
            with zmq.Context.instance().socket(zmq.REP) as socket:
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.RCVTIMEO, 2000)
                socket.bind("tcp://127.0.0.1:*")
                endpoint.put(socket.getsockopt(zmq.LAST_ENDPOINT).decode())
                with SCHEMA.RecordingRequest.from_bytes(socket.recv()) as request:
                    run_id = str(request.recordingId)
                    assert str(request.action) == "start"
                received.set()
                # Apply the request but hold acknowledgement until the client
                # has timed out. No timing-dependent sleep or dropped final reply.
                assert release_reply.wait(3)
                for i in range(2):
                    if i:
                        with SCHEMA.RecordingRequest.from_bytes(socket.recv()) as request:
                            assert str(request.action) == "status"
                    reply = SCHEMA.RecordingReply.new_message()
                    reply.accepted = True
                    reply.status.recordingId = run_id
                    reply.status.active = reply.status.ok = True
                    socket.send(reply.to_bytes())
                assert client_finished.wait(3)
        except BaseException as exc:
            errors.append(exc)

    thread = threading.Thread(target=peer)
    thread.start()
    try:
        port = int(endpoint.get(timeout=2).rsplit(":", 1)[1])
        robot = Robot("127.0.0.1", recording_port=port)
        with pytest.raises(RecordingTimeout) as error:
            robot.start_recording("/tmp/unconfirmed.csv", timeout=0.2)
        assert received.wait(1), errors
        assert error.value.recording_id == robot.recording_id
        release_reply.set()
        assert robot.recording_status(timeout=2).recording_id == robot.recording_id
    finally:
        release_reply.set()
        client_finished.set()
        thread.join(timeout=3)
    assert not thread.is_alive() and not errors


def test_standalone_cli_start_status_stop(recording_server, tmp_path, capsys):
    from fr3_stack.recording import main

    args = ["--host", "127.0.0.1", "--port", str(recording_server)]
    path = str(tmp_path / "cli.csv")
    assert main(args + ["start", path, "--recording-id", "cli-run"]) == 0
    start = json.loads(capsys.readouterr().out)
    assert start["active"] and start["recording_id"] == "cli-run"
    # Each CLI call creates/closes its client; the recording stays on the daemon.
    assert main(args + ["status"]) == 0
    assert json.loads(capsys.readouterr().out)["active"]
    assert main(args + ["stop", "stale-id"]) == 2
    refused = json.loads(capsys.readouterr().err)
    assert refused["status"]["active"]
    assert main(args + ["stop", "cli-run"]) == 0
    stopped = json.loads(capsys.readouterr().out)
    assert stopped["complete"] and not stopped["active"]
    assert main(args + ["start", path]) == 2
    assert json.loads(capsys.readouterr().err)["status"]["error_code"] == errno.EEXIST


def test_standalone_cli_timeout_reports_unconfirmed_id(monkeypatch, capsys):
    from fr3_stack.recording import main

    def timeout(*args, **kwargs):
        raise RecordingTimeout("retry-this-id")

    monkeypatch.setattr(Robot, "start_recording", timeout)
    assert main(["--host", "127.0.0.1", "start", "/tmp/test.csv"]) == 3
    error = json.loads(capsys.readouterr().err)
    assert error["outcome"] == "unconfirmed"
    assert error["recording_id"] == "retry-this-id"


@pytest.mark.parametrize("port", [0, 65536, True, "5557"])
def test_recording_port_validation(port):
    with pytest.raises(ValueError):
        Robot("127.0.0.1", recording_port=port)
