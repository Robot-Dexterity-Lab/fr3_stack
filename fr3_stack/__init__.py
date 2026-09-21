"""Python client for the fr3-stack daemon."""
from .dual_arm import ArmEndpoint, ArmSnapshot, CoordinationError, CoordinationState, DualArmCoordinator
from .dual_arm_robot import RobotArmEndpoint
from .agent import RobotAgent
from .client import Arm
from .geometry import Pose, Transform
from .interpolation import InterpolationController, PoseTrajectoryInterpolator
from .measure import Recorder
from .robot import Robot
from .state import ControllerType, Observation, State

__all__ = [
    "Arm",
    "ArmEndpoint",
    "ArmSnapshot",
    "CoordinationError",
    "CoordinationState",
    "DualArmCoordinator",
    "RobotArmEndpoint",
    "ControllerType",
    "InterpolationController",
    "Observation",
    "Pose",
    "PoseTrajectoryInterpolator",
    "Recorder",
    "Robot",
    "RobotAgent",
    "State",
    "Transform",
]
__version__ = "0.1.0"
