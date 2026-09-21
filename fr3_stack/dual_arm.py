"""Transport-independent, best-effort paired control. Not hardware synchronized."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable, Protocol

import numpy as np

from .geometry import Pose


@dataclass(frozen=True)
class ArmSnapshot:
    pose: Pose
    received_at: float  # local monotonic seconds, never the remote robot clock
    running: bool = True
    error: str = ""


class ArmEndpoint(Protocol):
    """Calls must be bounded; implementations own transport and stop policy."""

    def snapshot(self) -> ArmSnapshot: ...
    def send(self, target: Pose) -> None: ...
    def stop(self) -> None: ...


class CoordinationState(str, Enum):
    DISARMED = "disarmed"
    READY = "ready"
    RUNNING = "running"
    FAULT = "fault"


class CoordinationError(RuntimeError):
    pass


def _pose_copy(pose: Pose) -> Pose:
    p = np.asarray(pose.pos, dtype=float).reshape(3).copy()
    q = np.asarray(pose.quat, dtype=float).reshape(4).copy()
    norm = float(np.linalg.norm(q))
    if not np.isfinite(p).all() or not np.isfinite(q).all() or not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("target must have finite position and a nonzero finite quaternion")
    return Pose(p, q / norm)


class DualArmCoordinator:
    """Single-owner orchestration over two independently implemented endpoints.

    Targets use each arm's own base frame. No implicit calibration, socket access,
    ownership lease, background watchdog, or atomic paired execution is provided.
    Faults are latched until explicit arm(); stop is attempted on BOTH endpoints.
    """

    def __init__(self, left: ArmEndpoint, right: ArmEndpoint, *,
                 max_state_age: float = 0.1, max_state_skew: float = 0.05,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        if left is right:
            raise ValueError("left and right must be distinct endpoints")
        for name, value in (("max_state_age", max_state_age), ("max_state_skew", max_state_skew)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.left, self.right = left, right
        self.max_state_age, self.max_state_skew = max_state_age, max_state_skew
        self._clock, self._sleep = clock, sleep
        self.state = CoordinationState.DISARMED
        self.fault_reason = ""
        self.stop_errors: dict[str, str] = {}
        self.sequence = 0

    def _snapshots(self) -> tuple[ArmSnapshot, ArmSnapshot]:
        pair = self.left.snapshot(), self.right.snapshot()
        now = self._clock()
        for name, snapshot in zip(("left", "right"), pair):
            age = now - snapshot.received_at
            if not math.isfinite(age) or age < 0 or age > self.max_state_age:
                raise CoordinationError(f"{name}: stale or invalid state reception time")
            if not snapshot.running or snapshot.error:
                raise CoordinationError(f"{name}: {snapshot.error or 'daemon not running'}")
            _pose_copy(snapshot.pose)
        if abs(pair[0].received_at - pair[1].received_at) > self.max_state_skew:
            raise CoordinationError("state reception skew exceeds limit")
        return pair

    def _stop_both(self) -> None:
        self.stop_errors = {}
        for name, endpoint in (("left", self.left), ("right", self.right)):
            try:
                endpoint.stop()
            except Exception as exc:
                self.stop_errors[name] = str(exc)

    def _fault(self, reason: str) -> None:
        self.state = CoordinationState.FAULT
        self.fault_reason = reason
        self._stop_both()

    def arm(self) -> tuple[ArmSnapshot, ArmSnapshot]:
        """Explicit readiness check/recovery. Does not send a motion command."""
        if self.state == CoordinationState.RUNNING:
            raise CoordinationError("stop before re-arming")
        try:
            pair = self._snapshots()
        except Exception as exc:
            self._fault(str(exc))
            raise CoordinationError(str(exc)) from exc
        self.state = CoordinationState.READY
        self.fault_reason = ""
        self.stop_errors = {}
        return pair

    def observe(self) -> tuple[ArmSnapshot, ArmSnapshot]:
        if self.state not in (CoordinationState.READY, CoordinationState.RUNNING):
            raise CoordinationError("arm() is required")
        try:
            return self._snapshots()
        except Exception as exc:
            self._fault(str(exc))
            raise CoordinationError(str(exc)) from exc

    def send(self, left_target: Pose, right_target: Pose) -> int:
        """Validate the entire pair, then send sequentially; no atomicity promise."""
        if self.state not in (CoordinationState.READY, CoordinationState.RUNNING):
            raise CoordinationError("arm() is required")
        try:
            left, right = _pose_copy(left_target), _pose_copy(right_target)
            self._snapshots()
            self.left.send(left)
            self.right.send(right)
        except (KeyboardInterrupt, SystemExit) as exc:
            self._fault(f"paired dispatch interrupted: {exc}")
            raise
        except Exception as exc:
            self._fault(str(exc))
            raise CoordinationError(str(exc)) from exc
        self.sequence += 1
        self.state = CoordinationState.RUNNING
        return self.sequence

    def stop(self) -> None:
        """Attempt both application-defined stop policies, even if one fails."""
        self._stop_both()
        if self.stop_errors:
            self.state = CoordinationState.FAULT
            self.fault_reason = f"stop failed: {self.stop_errors}"
            raise CoordinationError(self.fault_reason)
        if self.state != CoordinationState.FAULT:
            self.state = CoordinationState.DISARMED

    def run(self, target_fn: Callable[[float], tuple[Pose, Pose]], *,
            duration: float, rate_hz: float = 30.0) -> None:
        """Run one common timebase; overruns skip ticks instead of burst replay.

        The caller must arm first. Always attempts paired stop on normal exit,
        callback failure, or interruption. Callbacks must return promptly.
        """
        if not math.isfinite(duration) or duration <= 0 or not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("duration and rate_hz must be finite and positive")
        if self.state != CoordinationState.READY:
            raise CoordinationError("arm() is required before run()")
        start, period = self._clock(), 1.0 / rate_hz
        tick = 0
        try:
            while True:
                now = self._clock()
                if now - start >= duration:
                    break
                self.observe()
                targets = target_fn(now - start)
                if self._clock() - start >= duration:
                    break
                self.send(*targets)
                tick = max(tick + 1, math.floor((self._clock() - start) / period) + 1)
                deadline = min(start + tick * period, start + duration)
                self._sleep(max(0.0, deadline - self._clock()))
        except BaseException as exc:
            if self.state != CoordinationState.FAULT:
                self._fault(f"control loop interrupted: {exc}")
            raise
        else:
            self.stop()
