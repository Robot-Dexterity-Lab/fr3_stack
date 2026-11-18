"""Optional SSIK conversion from an FR3 tool pose to a bounded joint target.

This module does not send commands. A policy can call :meth:`Fr3PoseIK.solve`
and pass the returned target to ``Robot.send_joint_impedance`` after its own
state and workspace checks.
"""

from __future__ import annotations

import numpy as np


def _pose(value, name: str) -> np.ndarray:
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1.0, atol=1e-5)):
        raise ValueError(f"{name} must be a finite 4x4 rigid transform")
    return pose


class Fr3PoseIK:
    """Solve a tool pose with SSIK's FR3 flange solver and a fixed tool offset.

    ``base_T_tool`` is the desired tool pose in ``fr3_link0``. The prebuilt
    solver targets ``fr3_link8``. ``flange_T_tool`` maps that flange to the
    selected tool frame; supply the measured transform for a mounted hand or
    plate. A target outside the nearby in-limits branch returns ``None``.
    """

    def __init__(self, flange_T_tool=None, *, max_joint_step_rad: float = 0.075,
                 solver=None):
        if solver is None:
            try:
                from ssik.prebuilt.franka import fr3_ik as solver
            except ImportError as exc:
                raise ImportError("Install fr3-stack[ik] to use Fr3PoseIK") from exc
        if (getattr(solver, "DOF", None) != 7
                or getattr(solver, "BASE_LINK", None) != "fr3_link0"
                or getattr(solver, "EE_LINK", None) != "fr3_link8"):
            raise ValueError("SSIK solver must target FR3 fr3_link0 to fr3_link8")
        if not np.isfinite(max_joint_step_rad) or max_joint_step_rad <= 0:
            raise ValueError("max_joint_step_rad must be positive and finite")
        self.solver = solver
        self.max_joint_step_rad = float(max_joint_step_rad)
        self.flange_T_tool = _pose(
            np.eye(4) if flange_T_tool is None else flange_T_tool,
            "flange_T_tool",
        ).copy()
        self.tool_T_flange = np.linalg.inv(self.flange_T_tool)

    def solve(self, base_T_tool, q_seed) -> np.ndarray | None:
        target = _pose(base_T_tool, "base_T_tool") @ self.tool_T_flange
        seed = np.asarray(q_seed, dtype=float)
        if seed.shape != (7,) or not np.isfinite(seed).all():
            raise ValueError("q_seed must be seven finite joint angles")
        solutions = self.solver.solve(
            target, q_seed=seed, max_solutions=1,
            seed_tolerance=self.max_joint_step_rad, respect_limits=True,
        )
        if not solutions:
            return None
        q = np.asarray(solutions[0].q, dtype=float)
        if (q.shape != (7,) or not np.isfinite(q).all()
                or np.max(np.abs(q - seed)) > self.max_joint_step_rad + 1e-9):
            raise RuntimeError("SSIK returned an invalid or discontinuous FR3 target")
        return q.copy()
