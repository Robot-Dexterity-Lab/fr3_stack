"""Frankapy-style selection / force-axis decomposition helpers.

frankapy's run_dynamic_force_position takes a length-6 selection vector
S in [0, 1] where 1 = position-controlled, 0 = force-controlled, plus a
length-6 target_force in world frame. fr3_stack's wire instead carries
a 6×6 axis-decomposition matrix Tr (first n_af rows are
force-controlled) and a target_wrench_Tr expressed in Tr basis.

HFVC is binary per axis — float S values are thresholded at 0.5 with a
one-shot warning.
"""
from __future__ import annotations

from typing import Any

import numpy as np


_S_BLEND_WARN_LOGGED = False


def selection_to_tr_n_af(S: Any) -> tuple[list[float], int]:
    """frankapy selection → (Tr_flat36, n_af). ``S[i]>=0.5`` ⇒ position.

    Force-controlled axes occupy the first ``n_af`` rows of Tr; the result is
    a permutation matrix (Tr⁻¹ = Trᵀ; the controller's singularity check is
    trivially satisfied). Axes 0..2 are translation, 3..5 rotation.
    """
    global _S_BLEND_WARN_LOGGED
    s = np.asarray(S, dtype=float).reshape(-1)
    if s.size != 6:
        raise ValueError(f"S must be length 6, got {s.size}")
    if np.any((s > 1e-12) & (s < 1.0 - 1e-12)) and not _S_BLEND_WARN_LOGGED:
        # Warn once: HFVC can't blend, so float S silently becomes a 0/1 step.
        import logging as _logging
        _logging.getLogger(__name__).warning(
            "selection vector S contains values in (0,1) (%s) — HFVC is "
            "binary per axis; thresholding at 0.5 (>=0.5 → position, <0.5 "
            "→ force). Pass 0 or 1 explicitly to silence this warning.",
            s.tolist(),
        )
        _S_BLEND_WARN_LOGGED = True
    is_pos = s >= 0.5
    force_axes = np.where(~is_pos)[0]
    pos_axes   = np.where( is_pos)[0]
    Tr = np.zeros((6, 6), dtype=float)
    row = 0
    for ax in force_axes:
        Tr[row, ax] = 1.0
        row += 1
    for ax in pos_axes:
        Tr[row, ax] = 1.0
        row += 1
    return Tr.flatten().tolist(), int(force_axes.size)


def target_force_world_to_tr_space(S: Any,
                                    target_force_world: Any) -> list[float]:
    """Pack world-frame target_force into target_wrench_Tr (matches the Tr
    layout from ``selection_to_tr_n_af``; trailing position-axis slots = 0)."""
    s = np.asarray(S, dtype=float).reshape(-1)
    if s.size != 6:
        raise ValueError(f"S must be length 6, got {s.size}")
    f = np.asarray(target_force_world, dtype=float).reshape(-1)
    if f.size != 6:
        raise ValueError(f"target_force_world must be length 6, got {f.size}")
    out = [0.0] * 6
    is_pos = s >= 0.5
    force_axes = np.where(~is_pos)[0]
    for slot, ax in enumerate(force_axes):
        out[slot] = float(f[ax])
    return out


def force_axis_to_tr(forces_xyz: Any) -> tuple[list[float], int, list[float], float]:
    """3-vec force → (Tr_flat36, n_af=1, target_wrench_Tr, ||forces||).

    Builds an orthonormal 3-basis with row 0 = f/||f||; translational block
    only — rotation stays identity (position-controlled).
    """
    f = np.asarray(forces_xyz, dtype=float).reshape(-1)
    if f.size != 3:
        raise ValueError(f"forces_xyz must be length 3, got {f.size}")
    mag = float(np.linalg.norm(f))
    if mag < 1e-12:
        raise ValueError("forces_xyz has zero magnitude — no axis to control")
    u = f / mag
    # Gram-Schmidt complement: pick the world axis least aligned with u, then
    # orthonormalize. Avoids the standard "pick x then project" pitfall when
    # u ≈ x_hat.
    seed_idx = int(np.argmin(np.abs(u)))
    seed = np.zeros(3); seed[seed_idx] = 1.0
    v = seed - u * float(u @ seed)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    Tr = np.zeros((6, 6), dtype=float)
    Tr[0, :3] = u
    Tr[1, :3] = v
    Tr[2, :3] = w
    Tr[3, 3] = Tr[4, 4] = Tr[5, 5] = 1.0
    target_wrench_Tr = [mag, 0.0, 0.0, 0.0, 0.0, 0.0]
    return Tr.flatten().tolist(), 1, target_wrench_Tr, mag
