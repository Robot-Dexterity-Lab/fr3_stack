"""Cap'n Proto schema load + list normalizers."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import capnp
import numpy as np


def _locate_schema() -> Path:
    """Find proto/fr3.capnp.

    Search:
      1. bundled directly inside wire/ (some install layouts)
      2. wheel-install layout: fr3_stack/fr3.capnp (one up from wire/)
      3. editable / dev: ../proto/fr3.capnp at any ancestor
    """
    here = Path(__file__).parent
    bundled = here / "fr3.capnp"
    if bundled.exists():
        return bundled
    parent_pkg = here.parent / "fr3.capnp"
    if parent_pkg.exists():
        return parent_pkg
    for ancestor in here.parents:
        cand = ancestor / "proto" / "fr3.capnp"
        if cand.exists():
            return cand
    raise FileNotFoundError(
        f"fr3.capnp not found next to {here}, at {parent_pkg}, "
        f"nor at any ../proto/fr3.capnp"
    )


SCHEMA = capnp.load(str(_locate_schema()))


def aslist(x: Any, n: int) -> list[float]:
    """Coerce x to a length-n list of floats; raise if it doesn't fit."""
    a = np.asarray(x, dtype=float).reshape(-1)
    if a.size != n:
        raise ValueError(f"expected length {n}, got {a.size}")
    return a.tolist()


def aslist_tr(x: Any) -> list[float]:
    """Normalize a Tr matrix to a row-major length-36 list. Accepts:
      * the literal "identity"
      * a 6×6 nested list / 2D numpy array
      * a flat length-36 sequence
    """
    if isinstance(x, str):
        if x == "identity":
            return np.eye(6).flatten().tolist()
        raise ValueError(f"unknown Tr literal: {x!r}")
    arr = np.asarray(x, dtype=float)
    if arr.shape == (6, 6):
        return arr.flatten().tolist()
    if arr.shape == (36,) or arr.ndim == 1 and arr.size == 36:
        return arr.reshape(36).tolist()
    raise ValueError(f"Tr must be 6×6 or length-36, got shape {arr.shape}")
