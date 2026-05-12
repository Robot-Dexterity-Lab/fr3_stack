"""``State`` (raw daemon publish) and ``Observation`` (policy-loop snapshot).

``State.wrench_ft`` maps the daemon's empty-list-for-absent convention to
``Optional[ndarray]`` so callers can branch on ``has_ft_sensor``.
"""
from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .geometry import Pose


class ControllerType(str, enum.Enum):
    IDLE                = "idle"
    CARTESIAN_IMPEDANCE = "cartesian_impedance"
    JOINT_IMPEDANCE     = "joint_impedance"
    ADMITTANCE          = "admittance"
    HYBRID              = "hybrid"
    # Mirrors the strings the daemon publishes; not enforced as exhaustive.


@dataclass
class State:
    controller:  str        = "idle"
    pos:         np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat_xyzw:   np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    q:           np.ndarray = field(default_factory=lambda: np.zeros(7))
    dq:          np.ndarray = field(default_factory=lambda: np.zeros(7))
    wrench_ext:  np.ndarray = field(default_factory=lambda: np.zeros(6))
    # FT-sensor wrench in base frame (R_O_EE applied on the daemon side), or
    # None if no sensor backend / no frame yet. When ``ft_compensated`` is
    # True, ``wrench_ft`` is gravity+bias-compensated and ``wrench_ft_raw``
    # carries the uncompensated stream; otherwise the two are equal.
    wrench_ft:       Optional[np.ndarray] = None
    wrench_ft_raw:   Optional[np.ndarray] = None
    ft_compensated:  bool                 = False
    timestamp:   float      = 0.0
    running:     bool       = False
    last_error:  str        = ""
    valid:       bool       = False

    @property
    def has_ft_sensor(self) -> bool:
        return self.wrench_ft is not None

    @property
    def pose(self) -> Pose:
        return Pose(self.pos.copy(), self.quat_xyzw.copy())

    def copy(self) -> "State":
        """Deep copy. Needed before mutating any field — without it, callers
        race the SUB thread and clobber the daemon-side cache."""
        return State(
            controller     = self.controller,
            pos            = self.pos.copy(),
            quat_xyzw      = self.quat_xyzw.copy(),
            q              = self.q.copy(),
            dq             = self.dq.copy(),
            wrench_ext     = self.wrench_ext.copy(),
            wrench_ft      = self.wrench_ft.copy()     if self.wrench_ft     is not None else None,
            wrench_ft_raw  = self.wrench_ft_raw.copy() if self.wrench_ft_raw is not None else None,
            ft_compensated = self.ft_compensated,
            timestamp      = self.timestamp,
            running        = self.running,
            last_error     = self.last_error,
            valid          = self.valid,
        )

    @classmethod
    def from_capnp(cls, msg) -> "State":
        ft_list     = list(msg.wrenchFt)
        # getattr for back-compat with pre-bump daemons during rolling upgrade.
        ft_raw_list     = list(getattr(msg, "wrenchFtRaw", []) or [])
        ft_compensated  = bool(getattr(msg, "ftCompensated", False))
        return cls(
            controller     = str(msg.controller),
            pos            = np.asarray(list(msg.pos)),
            quat_xyzw      = np.asarray(list(msg.quatXyzw)),
            q              = np.asarray(list(msg.q)),
            dq             = np.asarray(list(msg.dq)),
            wrench_ext     = np.asarray(list(msg.wrenchExt)),
            wrench_ft      = np.asarray(ft_list) if len(ft_list) == 6 else None,
            wrench_ft_raw  = np.asarray(ft_raw_list) if len(ft_raw_list) == 6 else None,
            ft_compensated = ft_compensated,
            timestamp      = float(msg.timestamp),
            running        = bool(msg.running),
            last_error     = str(msg.lastError),
            valid          = True,
        )


@dataclass(frozen=True)
class Observation:
    """Snapshot returned by ``Arm.observe()``. Arrays are independent copies."""
    pose:      Pose
    q:         np.ndarray
    dq:        np.ndarray
    wrench:    np.ndarray      # base frame; FT if has_ft else libfranka estimate
    has_ft:    bool
    timestamp: float
