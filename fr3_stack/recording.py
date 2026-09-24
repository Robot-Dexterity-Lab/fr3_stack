"""Acknowledged recording control, independent of the motion-command socket."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import zmq

from .wire import SCHEMA


@dataclass(frozen=True)
class RecordingStatus:
    recording_id: str
    path: str
    active: bool
    ok: bool
    finished: bool
    complete: bool
    written: int
    discarded: int
    dropped: int
    error: str
    error_code: int

    @classmethod
    def from_capnp(cls, value) -> "RecordingStatus":
        return cls(str(value.recordingId), str(value.path), bool(value.active),
                   bool(value.ok), bool(value.finished), bool(value.complete),
                   int(value.written), int(value.discarded), int(value.dropped),
                   str(value.error), int(value.errorCode))


class RecordingError(RuntimeError):
    def __init__(self, message: str, status: RecordingStatus):
        super().__init__(message)
        self.status = status


class RecordingTimeout(TimeoutError):
    """Outcome may be applied remotely; inspect status before continuing."""
    def __init__(self, recording_id: str):
        super().__init__(
            "Recording request timed out; outcome is unconfirmed. Query recording_status() "
            "and retry with the same recording_id; do not assume recording started/stopped."
        )
        self.recording_id = recording_id


def validate_timeout(timeout: float) -> None:
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")


def request_recording(address: str, action: str, recording_id: str = "",
                      path: str = "", timeout: float = 5.0) -> RecordingStatus:
    validate_timeout(timeout)
    request = SCHEMA.RecordingRequest.new_message()
    request.action, request.recordingId, request.path = action, recording_id, path
    deadline = time.monotonic() + timeout

    def remaining_ms() -> int:
        return max(0, math.ceil((deadline - time.monotonic()) * 1000))

    # A new REQ socket per operation avoids an EFSM-stuck socket after a timeout.
    # Never share the conflated PUSH motion socket or touch controller caches.
    with zmq.Context.instance().socket(zmq.REQ) as socket:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.IMMEDIATE, 1)
        socket.setsockopt(zmq.MAXMSGSIZE, 65536)
        socket.connect(address)
        try:
            if not socket.poll(remaining_ms(), zmq.POLLOUT):
                raise RecordingTimeout(recording_id)
            socket.send(request.to_bytes(), flags=zmq.DONTWAIT)
            if not socket.poll(remaining_ms(), zmq.POLLIN):
                raise RecordingTimeout(recording_id)
            payload = socket.recv(flags=zmq.DONTWAIT)
        except zmq.Again as exc:
            raise RecordingTimeout(recording_id) from exc
    with SCHEMA.RecordingReply.from_bytes(payload) as reply:
        status = RecordingStatus.from_capnp(reply.status)
        if not reply.accepted:
            raise RecordingError(str(reply.error), status)
        return status


def main(argv=None) -> int:
    """Standalone recording control; requires no motion command connection."""
    import argparse
    import json
    import sys
    from dataclasses import asdict
    from .robot import Robot

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="NUC address, not robot IP")
    parser.add_argument("--port", type=int, default=5557)
    parser.add_argument("--timeout", type=float, default=5.0)
    commands = parser.add_subparsers(dest="action", required=True)
    start = commands.add_parser("start", help="create a new CSV and enable capture")
    start.add_argument("path", help="absolute NUC/container path")
    start.add_argument("--recording-id", help="reuse this ID when retrying an unconfirmed start")
    stop = commands.add_parser("stop", help="stop the matching run, drain, flush and close")
    stop.add_argument("recording_id")
    commands.add_parser("status", help="inspect active or most recent recording")
    args = parser.parse_args(argv)
    robot = None
    try:
        robot = Robot(args.host, recording_port=args.port)
        if args.action == "start":
            state = robot.start_recording(args.path, recording_id=args.recording_id, timeout=args.timeout)
        elif args.action == "stop":
            state = robot.stop_recording(args.recording_id, timeout=args.timeout)
        else:
            state = robot.recording_status(timeout=args.timeout)
        print(json.dumps(asdict(state)))
        return 2 if state.error or (args.action == "stop" and not state.complete) else 0
    except RecordingTimeout as exc:
        print(json.dumps({"error": str(exc), "recording_id": exc.recording_id,
                          "outcome": "unconfirmed"}), file=sys.stderr)
        return 3
    except RecordingError as exc:
        print(json.dumps({"error": str(exc), "status": asdict(exc.status)}), file=sys.stderr)
        return 2
    except ValueError as exc:
        print(json.dumps({"error": str(exc)}), file=sys.stderr)
        return 2
    finally:
        if robot is not None:
            robot.close()  # deliberately does not stop the daemon's recording
