"""Pose to joint conversion with the optional SSIK adapter."""

from types import SimpleNamespace

import numpy as np
import pytest

from fr3_stack.ssik_adapter import Fr3PoseIK


class FakeFr3Solver:
    DOF = 7
    BASE_LINK = "fr3_link0"
    EE_LINK = "fr3_link8"

    def __init__(self, result):
        self.result = result
        self.target = None
        self.kwargs = None

    def solve(self, target, **kwargs):
        self.target = target
        self.kwargs = kwargs
        return [] if self.result is None else [SimpleNamespace(q=self.result)]


def test_tool_offset_and_seed_bound():
    flange_T_tool = np.eye(4)
    flange_T_tool[2, 3] = 0.1
    base_T_tool = np.eye(4)
    base_T_tool[0, 3] = 0.5
    seed = np.zeros(7)
    result = np.full(7, 0.02)
    solver = FakeFr3Solver(result)
    ik = Fr3PoseIK(flange_T_tool, solver=solver, max_joint_step_rad=0.075)

    q = ik.solve(base_T_tool, seed)

    np.testing.assert_allclose(q, result)
    np.testing.assert_allclose(solver.target[:3, 3], [0.5, 0, -0.1])
    np.testing.assert_allclose(solver.kwargs["q_seed"], seed)
    assert solver.kwargs["max_solutions"] == 1
    assert solver.kwargs["seed_tolerance"] == 0.075
    assert solver.kwargs["respect_limits"] is True
    q[0] = 1.0
    assert result[0] == 0.02


def test_missing_or_far_solution_is_not_commanded():
    seed = np.zeros(7)
    assert Fr3PoseIK(solver=FakeFr3Solver(None)).solve(np.eye(4), seed) is None
    with pytest.raises(RuntimeError, match="discontinuous"):
        Fr3PoseIK(solver=FakeFr3Solver(np.full(7, 0.2))).solve(np.eye(4), seed)


def test_invalid_pose_is_rejected():
    ik = Fr3PoseIK(solver=FakeFr3Solver(np.zeros(7)))
    bad = np.eye(4)
    bad[0, 0] = -1.0
    with pytest.raises(ValueError, match="rigid transform"):
        ik.solve(bad, np.zeros(7))


def test_prebuilt_fr3_round_trip_when_ssik_is_installed():
    pytest.importorskip("ssik")
    from ssik.prebuilt.franka import fr3_ik

    q = np.array([0.0, -0.5775085, 0.0, -1.7515253, 0.0, 1.1740168, 0.0])
    result = Fr3PoseIK().solve(fr3_ik.fk(q), q)
    assert result is not None
    np.testing.assert_allclose(result, q, atol=1e-5)
