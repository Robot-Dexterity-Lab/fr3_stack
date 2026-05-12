"""Process-isolated interpolation controller (UMI-style API).

Bridges sparse policy outputs to dense Cartesian-impedance commands in a
separate process so inference spikes can't starve the control loop. Exposes
``servoL`` / ``schedule_waypoint`` / ``get_state``.
"""
from __future__ import annotations

import enum
import multiprocessing as mp
import time
from typing import Sequence, Union

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp

from .geometry import Pose
from .robot import Robot

ArrayLike = Union[np.ndarray, Sequence[float]]


class _Cmd(enum.IntEnum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2


# =============================================================================
# Pose trajectory interpolator
# =============================================================================

class PoseTrajectoryInterpolator:
    """Time-parameterized SE(3) trajectory: linear on pos, SLERP on quat.
    Clamps to ``[times[0], times[-1]]`` — no extrapolation."""

    def __init__(
        self,
        times:      ArrayLike,
        positions:  ArrayLike,
        quats_xyzw: ArrayLike,
    ):
        t = np.asarray(times,      dtype=float)
        p = np.asarray(positions,  dtype=float).reshape(-1, 3)
        q = np.asarray(quats_xyzw, dtype=float).reshape(-1, 4)
        if not (len(t) == len(p) == len(q) >= 1):
            raise ValueError(
                f"times/positions/quats length mismatch: {len(t)}/{len(p)}/{len(q)}"
            )
        if len(t) > 1 and not np.all(np.diff(t) > 0):
            raise ValueError("times must be strictly increasing")
        # Normalize quats; flip signs so SLERP doesn't take the long way around.
        q = q / np.linalg.norm(q, axis=1, keepdims=True)
        for i in range(1, len(q)):
            if np.dot(q[i], q[i - 1]) < 0:
                q[i] = -q[i]
        self._t = t
        self._p = p
        self._q = q
        self._refresh()

    @classmethod
    def from_pose(cls, t: float, pose: Pose) -> "PoseTrajectoryInterpolator":
        return cls([t], [pose.pos], [pose.quat])

    def _refresh(self) -> None:
        if len(self._t) >= 2:
            self._p_interp = interp1d(
                self._t, self._p, axis=0,
                bounds_error=False,
                fill_value=(self._p[0], self._p[-1]),
            )
            self._slerp = Slerp(self._t, Rotation.from_quat(self._q))
        else:
            self._p_interp = None
            self._slerp = None

    def __call__(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """``(pos, quat_xyzw)`` at time ``t``; clamps to range."""
        if len(self._t) == 1:
            return self._p[0].copy(), self._q[0].copy()
        tc = max(self._t[0], min(self._t[-1], float(t)))
        return self._p_interp(tc), self._slerp(tc).as_quat()

    @property
    def times(self) -> np.ndarray:
        return self._t.copy()

    @property
    def last_time(self) -> float:
        return float(self._t[-1])

    def drive_to_waypoint(
        self,
        pose:      Pose,
        time:      float,
        curr_time: float,
    ) -> "PoseTrajectoryInterpolator":
        """Replace future plan with a linear segment to ``pose`` by ``time``.
        Past samples (``t < curr_time``) are preserved for idempotency."""
        if time <= curr_time:
            raise ValueError(f"target time {time} must be > curr_time {curr_time}")
        curr_pos, curr_quat = self(curr_time)
        mask = self._t < curr_time
        new_t = np.concatenate([self._t[mask], [curr_time, time]])
        new_p = np.vstack([self._p[mask], curr_pos[None, :], np.asarray(pose.pos)[None, :]])
        new_q = np.vstack([self._q[mask], curr_quat[None, :], np.asarray(pose.quat)[None, :]])
        return PoseTrajectoryInterpolator(new_t, new_p, new_q)

    def schedule_waypoint(
        self,
        pose:               Pose,
        time:               float,
        curr_time:          float,
        last_waypoint_time: float,
    ) -> "PoseTrajectoryInterpolator":
        """Insert a waypoint at ``time``. Later waypoints are dropped (newer
        schedule wins). ``time <= curr_time`` falls back to ``drive_to_waypoint``."""
        if time <= curr_time:
            return self.drive_to_waypoint(
                pose, max(time, curr_time + 1e-3), curr_time
            )
        curr_pos, curr_quat = self(curr_time)
        # Keep strictly before curr_time AND time (anything later supersedes).
        mask = (self._t < curr_time) & (self._t < time)
        new_t = np.concatenate([self._t[mask], [curr_time, time]])
        new_p = np.vstack([self._p[mask], curr_pos[None, :], np.asarray(pose.pos)[None, :]])
        new_q = np.vstack([self._q[mask], curr_quat[None, :], np.asarray(pose.quat)[None, :]])
        order = np.argsort(new_t)
        new_t, new_p, new_q = new_t[order], new_p[order], new_q[order]
        # Dedup: curr_time may collide with an existing waypoint.
        keep = np.concatenate([[True], np.diff(new_t) > 1e-6])
        return PoseTrajectoryInterpolator(new_t[keep], new_p[keep], new_q[keep])


# =============================================================================
# Process-isolated controller
# =============================================================================

class InterpolationController(mp.Process):
    """Subprocess worker that owns a :class:`Robot` and streams Cartesian
    impedance commands at ``frequency`` Hz, evaluating an interpolator updated
    by caller-side ``servoL`` / ``schedule_waypoint``."""

    def __init__(
        self,
        host:                 str,
        *,
        cmd_port:             int = 5555,
        state_port:           int = 5556,
        frequency:            float = 200.0,
        stiffness:            ArrayLike = (200.0, 200.0, 200.0, 20.0, 20.0, 20.0),
        joints_init:          ArrayLike | None = None,
        joints_init_duration: float = 4.0,
        state_buffer_size:    int = 256,
        launch_timeout:       float = 5.0,
        verbose:              bool = False,
    ):
        """``joints_init`` (7-vec) triggers a joint-impedance hold for
        ``joints_init_duration`` s before switching to Cartesian."""
        super().__init__(name="fr3-stack-interpolation-controller", daemon=True)

        self._host       = host
        self._cmd_port   = cmd_port
        self._state_port = state_port
        if frequency <= 0:
            raise ValueError(f"frequency must be > 0, got {frequency}")
        self._dt = 1.0 / float(frequency)

        self._stiffness = np.asarray(stiffness, dtype=float).reshape(-1)
        if self._stiffness.shape != (6,):
            raise ValueError(f"stiffness must be length-6, got shape {self._stiffness.shape}")

        if joints_init is not None:
            jq = np.asarray(joints_init, dtype=float).reshape(-1)
            if jq.shape != (7,):
                raise ValueError(f"joints_init must be length-7, got shape {jq.shape}")
            self._joints_init = jq
        else:
            self._joints_init = None
        self._joints_init_duration = float(joints_init_duration)
        self._launch_timeout = float(launch_timeout)
        self._verbose = verbose

        # Bounded queue protects worker from runaway producer.
        self._cmd_queue: mp.Queue = mp.Queue(maxsize=128)
        self._mgr = mp.Manager()
        self._state_buf = self._mgr.list()
        self._state_buf_max = int(state_buffer_size)
        self._state_lock = self._mgr.Lock()
        self._ready_event = mp.Event()

    # ---- caller-side API --------------------------------------------------

    @property
    def is_ready(self) -> bool:
        return self._ready_event.is_set()

    def start(self, wait: bool = True) -> None:
        super().start()
        if wait:
            self._ready_event.wait(self._launch_timeout)
            if not self._ready_event.is_set():
                raise TimeoutError(
                    f"InterpolationController did not become ready within "
                    f"{self._launch_timeout}s — check daemon connectivity"
                )

    def stop(self, wait: bool = True) -> None:
        try:
            self._cmd_queue.put_nowait({"cmd": int(_Cmd.STOP)})
        except Exception:
            pass
        if wait:
            self.join(timeout=2.0)

    def __enter__(self) -> "InterpolationController":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def servoL(self, pose: Pose, duration: float = 0.1) -> None:
        """Drive to ``pose`` over ``duration`` s. ``duration < 1/frequency`` raises."""
        if duration < self._dt:
            raise ValueError(
                f"duration {duration} must be ≥ 1/freq = {self._dt:.4f}s"
            )
        self._cmd_queue.put({
            "cmd":      int(_Cmd.SERVOL),
            "pos":      np.asarray(pose.pos,  dtype=float).copy(),
            "quat":     np.asarray(pose.quat, dtype=float).copy(),
            "duration": float(duration),
        })

    def schedule_waypoint(self, pose: Pose, target_time: float) -> None:
        """Schedule ``pose`` at wall-clock ``target_time``. Stale waypoints are dropped."""
        self._cmd_queue.put({
            "cmd":         int(_Cmd.SCHEDULE_WAYPOINT),
            "pos":         np.asarray(pose.pos,  dtype=float).copy(),
            "quat":        np.asarray(pose.quat, dtype=float).copy(),
            "target_time": float(target_time),
        })

    def get_state(self, k: int | None = None):
        """Latest state dict (``k=None``) or last ``k`` (oldest first).
        Returns {} / [] before the worker has produced anything."""
        with self._state_lock:
            if k is None:
                return dict(self._state_buf[-1]) if self._state_buf else {}
            return [dict(s) for s in list(self._state_buf)[-k:]]

    # ---- subprocess -------------------------------------------------------

    def run(self) -> None:
        robot = Robot(self._host, cmd_port=self._cmd_port, state_port=self._state_port)
        robot.connect()
        try:
            robot._cart_cache["K"] = self._stiffness.tolist()
            robot._adm_cache["K"]  = self._stiffness.tolist()

            if self._joints_init is not None:
                robot.send_joint_impedance(q_target=self._joints_init.tolist())
                time.sleep(self._joints_init_duration)

            # Anchor the interpolator at the first daemon state.
            s = robot.wait_for_state(timeout=5.0)
            curr_pos = s.pos.copy()
            curr_quat = s.quat_xyzw.copy()

            t0 = time.monotonic()
            interp = PoseTrajectoryInterpolator(
                [t0], [curr_pos], [curr_quat],
            )
            last_waypoint_time = t0
            iter_idx = 0
            keep_running = True

            while keep_running:
                t_now = time.monotonic()

                pos, quat = interp(t_now)
                robot.send_cartesian_impedance(
                    target_pos       = pos.tolist(),
                    target_quat_xyzw = quat.tolist(),
                )

                latest = robot.state
                state_dict = {
                    "pos":        latest.pos.copy(),
                    "quat_xyzw":  latest.quat_xyzw.copy(),
                    "q":          latest.q.copy(),
                    "dq":         latest.dq.copy(),
                    "wrench_ext": latest.wrench_ext.copy(),
                    "wrench_ft":  None if latest.wrench_ft is None else latest.wrench_ft.copy(),
                    "timestamp":  latest.timestamp,
                    "received":   time.time(),
                }
                with self._state_lock:
                    self._state_buf.append(state_dict)
                    while len(self._state_buf) > self._state_buf_max:
                        self._state_buf.pop(0)

                # One command per cycle keeps the loop period predictable.
                try:
                    cmd = self._cmd_queue.get_nowait()
                except Exception:
                    cmd = None
                if cmd is not None:
                    code = cmd["cmd"]
                    if code == int(_Cmd.STOP):
                        keep_running = False
                    elif code == int(_Cmd.SERVOL):
                        target_pose = Pose(cmd["pos"], cmd["quat"])
                        target_time = (t_now + self._dt) + cmd["duration"]
                        interp = interp.drive_to_waypoint(
                            target_pose,
                            time      = target_time,
                            curr_time = t_now + self._dt,
                        )
                        last_waypoint_time = target_time
                    elif code == int(_Cmd.SCHEDULE_WAYPOINT):
                        target_pose = Pose(cmd["pos"], cmd["quat"])
                        # Caller passes wall-clock; the loop runs on monotonic.
                        target_time = time.monotonic() - time.time() + cmd["target_time"]
                        interp = interp.schedule_waypoint(
                            target_pose,
                            time               = target_time,
                            curr_time          = t_now + self._dt,
                            last_waypoint_time = last_waypoint_time,
                        )
                        last_waypoint_time = target_time

                if iter_idx == 0:
                    self._ready_event.set()
                iter_idx += 1

                t_next = t0 + iter_idx * self._dt
                sleep_for = t_next - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            try:
                robot.send_idle()
            except Exception:
                pass
            robot.close()
            self._ready_event.set()  # unblock caller's start() if we died early
            if self._verbose:
                print(f"[InterpolationController] disconnected from {self._host}")
