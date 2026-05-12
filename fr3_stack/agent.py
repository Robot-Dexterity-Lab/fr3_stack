"""``reset / observe / step`` runtime over :class:`Arm` for policy rollouts."""
from __future__ import annotations

import time
from typing import Sequence, Union

import numpy as np

from .client import Arm
from .geometry import Pose
from .state import Observation

ArrayLike = Union[np.ndarray, Sequence[float]]


class RobotAgent:
    """``reset / observe / step`` facade. Drop through to ``agent.arm`` /
    ``agent.robot`` for anything outside the rollout loop."""

    def __init__(
        self,
        host: str,
        *,
        home_pose:         Pose,
        control_hz:        float = 30.0,
        default_stiffness: ArrayLike | None = None,
        reset_duration:    float = 2.0,
        **arm_kwargs,
    ):
        """``default_stiffness`` is re-applied on every ``reset()`` so a
        change inside the loop doesn't leak across episodes."""
        if control_hz <= 0:
            raise ValueError(f"control_hz must be > 0, got {control_hz}")
        if reset_duration <= 0:
            raise ValueError(f"reset_duration must be > 0, got {reset_duration}")

        self._arm = Arm(host, **arm_kwargs)
        self._home = home_pose
        self._dt   = 1.0 / float(control_hz)
        self._default_K = (
            np.asarray(default_stiffness, dtype=float).copy()
            if default_stiffness is not None else None
        )
        self._reset_duration = float(reset_duration)
        self._t_next: float = 0.0   # next-tick deadline; set by reset()

    # ---- escape hatches -------------------------------------------------

    @property
    def arm(self) -> Arm:
        return self._arm

    @property
    def robot(self):
        return self._arm.robot

    # ---- lifecycle ------------------------------------------------------

    def connect(self) -> None: self._arm.connect()
    def close(self)   -> None: self._arm.close()

    def __enter__(self) -> "RobotAgent":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- core API --------------------------------------------------------

    def reset(self) -> Observation:
        """Min-jerk to ``home_pose``, re-apply ``default_stiffness``, reset
        the rate-limit clock. Returns the post-home observation."""
        if self._default_K is not None:
            self._arm.set_stiffness(K=self._default_K)
        obs = self._arm.move_to(self._home, duration=self._reset_duration)
        self._t_next = time.monotonic() + self._dt
        return obs

    def observe(self) -> Observation:
        return self._arm.observe()

    def step(
        self,
        target: Pose,
        *,
        stiffness: ArrayLike | None = None,
    ) -> Observation:
        """Send one target, rate-limit to ``control_hz``, return next obs.

        If a tick overruns 1/hz, no sleep — the loop catches up flat-out
        until ``_t_next`` is back in the future.
        """
        self._arm.send(target, stiffness=stiffness)

        sleep_for = self._t_next - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        self._t_next += self._dt

        return self._arm.observe()
