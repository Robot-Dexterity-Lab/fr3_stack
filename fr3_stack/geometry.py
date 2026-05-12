"""SE(3) types: absolute ``Pose`` and relative ``Transform``.

Storage is ``pos`` (3) + ``quat`` xyzw (4), matching the wire schema.
Composition (runtime-checked): ``Pose @ Transform → Pose``,
``Transform @ Transform → Transform``. ``Pose @ Pose`` is undefined — use
``inv_compose`` for the relative transform.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Union

import numpy as np
from scipy.spatial.transform import Rotation as _R

ArrayLike = Union[np.ndarray, Sequence[float]]


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n == 0.0:
        raise ValueError("zero quaternion is not a valid rotation")
    return q / n


# ---- Pose -----------------------------------------------------------------

@dataclass(frozen=True)
class Pose:
    """Absolute SE(3) pose. Position [m] + unit quaternion [x,y,z,w]."""
    pos:  np.ndarray
    quat: np.ndarray   # xyzw

    def __post_init__(self):
        # frozen=True blocks setattr — use object.__setattr__.
        object.__setattr__(self, "pos",  np.asarray(self.pos,  dtype=float).reshape(3))
        object.__setattr__(self, "quat", np.asarray(self.quat, dtype=float).reshape(4))

    @classmethod
    def identity(cls) -> "Pose":
        return cls(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def from_xyz_quat(cls, pos: ArrayLike, quat_xyzw: ArrayLike) -> "Pose":
        return cls(np.asarray(pos, dtype=float), np.asarray(quat_xyzw, dtype=float))

    @classmethod
    def from_matrix(cls, T: np.ndarray) -> "Pose":
        T = np.asarray(T, dtype=float).reshape(4, 4)
        return cls(T[:3, 3].copy(), _R.from_matrix(T[:3, :3]).as_quat())

    @property
    def R(self) -> np.ndarray:
        return _R.from_quat(self.quat).as_matrix()

    @property
    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3]  = self.pos
        return T

    def translated(self, dxyz: ArrayLike) -> "Pose":
        d = np.asarray(dxyz, dtype=float).reshape(3)
        return Pose(self.pos + d, self.quat.copy())

    def approx_equal(
        self, other: "Pose", *, pos_tol: float = 1e-6, rot_tol: float = 1e-6
    ) -> bool:
        if np.linalg.norm(self.pos - other.pos) > pos_tol:
            return False
        # Quaternion double-cover: q and -q are the same rotation.
        dot = abs(float(np.dot(self.quat, other.quat)))
        return (1.0 - min(dot, 1.0)) < (rot_tol ** 2) / 2.0

    def __matmul__(self, t: "Transform") -> "Pose":
        if isinstance(t, Pose):
            raise TypeError(
                "Pose @ Pose is undefined — use `self.inv_compose(other)` to get "
                "the relative Transform from self to other."
            )
        if not isinstance(t, Transform):
            return NotImplemented   # type: ignore[return-value]
        r_self = _R.from_quat(self.quat)
        new_pos  = self.pos + r_self.apply(t.pos)
        new_quat = _normalize_quat((r_self * _R.from_quat(t.quat)).as_quat())
        return Pose(new_pos, new_quat)

    def inv_compose(self, other: "Pose") -> "Transform":
        """T such that ``self @ T == other``."""
        r_inv = _R.from_quat(self.quat).inv()
        d_pos  = r_inv.apply(other.pos - self.pos)
        d_quat = _normalize_quat((r_inv * _R.from_quat(other.quat)).as_quat())
        return Transform(d_pos, d_quat)


# ---- Transform ------------------------------------------------------------

@dataclass(frozen=True)
class Transform:
    """Relative SE(3) transform. No anchored frame on its own."""
    pos:  np.ndarray
    quat: np.ndarray   # xyzw

    def __post_init__(self):
        object.__setattr__(self, "pos",  np.asarray(self.pos,  dtype=float).reshape(3))
        object.__setattr__(self, "quat", np.asarray(self.quat, dtype=float).reshape(4))

    @classmethod
    def identity(cls) -> "Transform":
        return cls(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def translation(cls, xyz: ArrayLike) -> "Transform":
        return cls(np.asarray(xyz, dtype=float), np.array([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def rotation(cls, quat_xyzw: ArrayLike) -> "Transform":
        return cls(np.zeros(3), _normalize_quat(np.asarray(quat_xyzw, dtype=float)))

    @classmethod
    def from_axis_angle(
        cls,
        axis: ArrayLike,
        angle_rad: float,
        *,
        pos: ArrayLike | None = None,
    ) -> "Transform":
        a = np.asarray(axis, dtype=float).reshape(3)
        a = a / np.linalg.norm(a)
        rotvec = a * float(angle_rad)
        q = _R.from_rotvec(rotvec).as_quat()
        p = np.zeros(3) if pos is None else np.asarray(pos, dtype=float).reshape(3)
        return cls(p, q)

    @classmethod
    def from_matrix(cls, T: np.ndarray) -> "Transform":
        T = np.asarray(T, dtype=float).reshape(4, 4)
        return cls(T[:3, 3].copy(), _R.from_matrix(T[:3, :3]).as_quat())

    @property
    def R(self) -> np.ndarray:
        return _R.from_quat(self.quat).as_matrix()

    @property
    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.R
        T[:3, 3]  = self.pos
        return T

    def inverse(self) -> "Transform":
        r_inv = _R.from_quat(self.quat).inv()
        return Transform(-r_inv.apply(self.pos), r_inv.as_quat())

    def __matmul__(self, other: "Transform") -> "Transform":
        if isinstance(other, Pose):
            raise TypeError(
                "Transform @ Pose is undefined — left operand must be Pose for "
                "absolute composition."
            )
        if not isinstance(other, Transform):
            return NotImplemented   # type: ignore[return-value]
        r_self = _R.from_quat(self.quat)
        new_pos  = self.pos + r_self.apply(other.pos)
        new_quat = _normalize_quat((r_self * _R.from_quat(other.quat)).as_quat())
        return Transform(new_pos, new_quat)
