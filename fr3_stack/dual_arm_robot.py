"""Robot transport adapter; coordination logic lives in dual_arm.py."""
from __future__ import annotations

from typing import Callable

from .dual_arm import ArmSnapshot, CoordinationError
from .geometry import Pose
from .robot import Robot


class RobotArmEndpoint:
    """Wrap an already connected Robot with an explicit task-specific stop policy.

    Use a finite Robot command_send_timeout_ms. A send is transport acceptance,
    not a daemon acknowledgement. stop_action must be bounded and is not a
    hardware safety stop. Client lifecycle remains owned by the caller.
    """

    def __init__(self, robot: Robot, *, stop_action: Callable[[Robot], None]):
        if robot.command_send_timeout_ms is None:
            raise ValueError("dual-arm transport requires a finite command_send_timeout_ms")
        if not callable(stop_action):
            raise TypeError("stop_action must be callable")
        self.robot = robot
        self._stop_action = stop_action

    def snapshot(self) -> ArmSnapshot:
        s = self.robot.state
        if not s.valid or s.received_at is None:
            raise CoordinationError("no state received from daemon")
        return ArmSnapshot(s.pose, s.received_at, s.running, s.last_error)

    def send(self, target: Pose) -> None:
        self.robot.send_cartesian_impedance(target.pos, target.quat)

    def stop(self) -> None:
        self._stop_action(self.robot)
