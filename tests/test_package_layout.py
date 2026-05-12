"""Migration-time package layout tests.

Verifies the rebrand from `fr3` → `fr3-stack` didn't break:
  * top-level imports (Robot / Pose / State / ControllerType)
  * the schema-locator walk-up that supports both editable + wheel installs
  * bundled configs survive packaging
  * the wire schema parses into the structs the daemon expects

These tests are migration-specific. They catch regressions if someone moves
files around again (e.g. drops `proto/` back inside the Python package, or
breaks src layout in pyproject.toml).
"""
from __future__ import annotations

import pytest

import fr3_stack
from fr3_stack import ControllerType, Pose, Robot, State
from fr3_stack.config import load_controller_config
from fr3_stack.robot import _SCHEMA, _locate_schema


# ---- top-level surface ------------------------------------------------------

def test_top_level_exports_complete():
    """`from fr3_stack import Robot` etc. works."""
    expected = {"Robot", "Pose", "State", "ControllerType"}
    assert expected.issubset(set(fr3_stack.__all__))


def test_top_level_classes_resolved():
    """Re-exports point at the actual classes, not strings."""
    assert isinstance(Robot, type)
    assert isinstance(Pose, type)
    assert isinstance(State, type)
    assert isinstance(ControllerType, type)


def test_module_has_version():
    assert hasattr(fr3_stack, "__version__")
    assert isinstance(fr3_stack.__version__, str)


# ---- schema location --------------------------------------------------------

def test_locate_schema_returns_existing_file():
    p = _locate_schema()
    assert p.exists(), f"schema path returned but file missing: {p}"
    assert p.is_file()
    assert p.name == "fr3.capnp"


def test_locate_schema_in_known_layout():
    """Schema lives under either `proto/` (editable) or alongside the
    package (`fr3_stack/`, wheel install via force-include). Anything else
    means a layout regression."""
    p = _locate_schema()
    parent_name = p.parent.name
    assert parent_name in {"proto", "fr3_stack"}, (
        f"schema parent dir {parent_name!r} not in expected set "
        f"(full path: {p})"
    )


def test_schema_loaded_with_required_types():
    """Smoke: pycapnp parsed the schema and exposed the structs the
    daemon side relies on. Missing means schema loaded but is broken."""
    required = (
        "Command", "State",
        "CartesianImpedanceCmd", "JointImpedanceCmd",
        "AdmittanceCmd", "HybridCmd", "MoveToCmd",
    )
    missing = [n for n in required if not hasattr(_SCHEMA, n)]
    assert not missing, f"schema missing types: {missing}"


# ---- bundled config payload -------------------------------------------------

@pytest.mark.parametrize("controller", [
    "cartesian_impedance",
    "joint_impedance",
    "admittance",
    "hybrid",
])
def test_controller_configs_load(controller):
    """Per-controller YAML configs are findable. Catches regressions where
    the configs/ subdir gets dropped from the wheel or moved to a path
    load_controller_config doesn't search."""
    cfg = load_controller_config(controller)
    assert isinstance(cfg, dict)
    assert cfg, f"{controller} yaml parsed empty"


def test_unknown_controller_rejected():
    with pytest.raises(Exception):
        load_controller_config("not_a_real_controller")
