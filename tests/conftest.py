"""Shared test fixtures: a fake daemon binding the real ZMQ + Cap'n Proto wire.

Tests for both ``Robot`` (low-level) and ``Arm`` (facade) talk to this fake.
The client can't tell the difference between FakeDaemon and the real fr3-stack
binary — same socket types, same conflate options, same Cap'n Proto schema.

Each test gets a fresh daemon on ephemeral ports, so they're parallel-safe.
"""
from __future__ import annotations

import contextlib
import threading
import time

import pytest
import zmq

from fr3_stack import Robot
from fr3_stack.wire import SCHEMA


def _port_of(sock: zmq.Socket) -> int:
    endpoint = sock.getsockopt(zmq.LAST_ENDPOINT).decode()
    return int(endpoint.rsplit(":", 1)[1])


class FakeDaemon:
    """Stand-in for fr3-stack. PULL-binds for commands, PUB-binds for state."""

    def __init__(self, conflate_cmd: bool = True) -> None:
        self.ctx = zmq.Context.instance()

        self.cmd_sock = self.ctx.socket(zmq.PULL)
        if conflate_cmd:
            self.cmd_sock.setsockopt(zmq.CONFLATE, 1)
        self.cmd_sock.bind("tcp://127.0.0.1:0")
        self.cmd_port = _port_of(self.cmd_sock)

        self.state_sock = self.ctx.socket(zmq.PUB)
        self.state_sock.bind("tcp://127.0.0.1:0")
        self.state_port = _port_of(self.state_sock)

    # ---- inbound (commands from client) ---------------------------------

    @contextlib.contextmanager
    def recv_command(self, timeout: float = 1.0):
        """Yield the next Command message (or None on timeout) inside a
        with-block. pycapnp 2.x ties the parsed message's lifetime to a
        context manager, so callers must use ``with daemon.recv_command()``.
        """
        self.cmd_sock.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
        try:
            payload = self.cmd_sock.recv()
        except zmq.Again:
            yield None
            return
        with SCHEMA.Command.from_bytes(payload) as parsed:
            yield parsed

    def drain_commands(self, settle: float = 0.02, max_wait: float = 1.0):
        """Receive every queued command. For non-CONFLATE sockets only.

        Reads with a short timeout in a loop; returns when no command has
        arrived for ``settle`` seconds or ``max_wait`` total has elapsed.
        Each yielded message is wrapped in its own ``with`` block by the
        caller — this method returns *bytes* so we don't fight pycapnp's
        lifetime model.

        NOTE: ``settle`` is a quiescent-period detector — stop any
        background publisher before calling, otherwise each new message
        resets the zmq.Again timer and the loop only exits at ``max_wait``.

        Returns:
            list[bytes] — raw payloads in arrival order. Decode with
            ``SCHEMA.Command.from_bytes(payload)``.
        """
        out: list[bytes] = []
        deadline = time.monotonic() + max_wait
        self.cmd_sock.setsockopt(zmq.RCVTIMEO, int(settle * 1000))
        try:
            while time.monotonic() < deadline:
                try:
                    out.append(self.cmd_sock.recv())
                except zmq.Again:
                    break
        finally:
            self.cmd_sock.setsockopt(zmq.RCVTIMEO, -1)
        return out

    # ---- outbound (state to client) -------------------------------------

    def publish_state(
        self,
        *,
        controller: str = "idle",
        pos = (0.0, 0.0, 0.0),
        quat_xyzw = (0.0, 0.0, 0.0, 1.0),
        q = (0.0,) * 7,
        dq = (0.0,) * 7,
        wrench_ext = (0.0,) * 6,
        wrench_ft = (),
        timestamp: float = 0.0,
        running: bool = True,
        last_error: str = "",
    ) -> None:
        st = SCHEMA.State.new_message()
        st.controller = controller
        st.pos        = list(pos)
        st.quatXyzw   = list(quat_xyzw)
        st.q          = list(q)
        st.dq         = list(dq)
        st.wrenchExt  = list(wrench_ext)
        st.wrenchFt   = list(wrench_ft)
        st.timestamp  = timestamp
        st.running    = running
        st.lastError  = last_error
        self.state_sock.send(st.to_bytes())

    def publish_until_received(self, client: Robot, timeout: float = 2.0,
                               **state_kwargs) -> bool:
        """Spam state until the client's SUB has connected and seen one.

        ZMQ PUB drops messages sent before SUB attaches (the 'slow joiner'
        problem), so test code can't rely on a single publish_state landing.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.publish_state(**state_kwargs)
            time.sleep(0.02)
            if client.state.valid:
                return True
        return False

    @contextlib.contextmanager
    def publish_loop(self, period: float = 0.005, pos_fn=None, **state_kwargs):
        """Run a background publisher thread for the duration of the block.

        ``pos_fn(t_since_start) -> tuple[float,float,float]`` overrides ``pos``
        when given; it runs on the publisher thread, so avoid sharing mutable
        state with the test thread without a lock. ``state_kwargs`` are
        forwarded to ``publish_state``. Teardown stops the thread (1s join
        timeout).
        """
        stop = threading.Event()
        t_start = time.monotonic()

        def _run():
            while not stop.is_set():
                kw = dict(state_kwargs)
                if pos_fn is not None:
                    kw["pos"] = pos_fn(time.monotonic() - t_start)
                self.publish_state(**kw)
                time.sleep(period)

        th = threading.Thread(target=_run, daemon=True)
        th.start()
        try:
            yield
        finally:
            stop.set()
            th.join(timeout=1.0)

    def close(self) -> None:
        self.cmd_sock.close(linger=0)
        self.state_sock.close(linger=0)


@pytest.fixture
def daemon():
    d = FakeDaemon()
    yield d
    d.close()


@pytest.fixture
def client(daemon):
    """A connected ``Robot`` pointed at the test ``FakeDaemon``."""
    r = Robot("127.0.0.1", cmd_port=daemon.cmd_port, state_port=daemon.state_port)
    r.connect()
    yield r
    r.close()


@pytest.fixture
def daemon_streaming():
    """Like ``daemon`` but cmd PULL is non-CONFLATE, so streaming tests
    can capture every command in a sequence."""
    d = FakeDaemon(conflate_cmd=False)
    yield d
    d.close()


@pytest.fixture
def client_streaming(daemon_streaming):
    r = Robot(
        "127.0.0.1",
        cmd_port=daemon_streaming.cmd_port,
        state_port=daemon_streaming.state_port,
    )
    r.connect()
    yield r
    r.close()
