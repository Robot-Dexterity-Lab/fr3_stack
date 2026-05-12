"""Connection config + per-controller YAML defaults.

Connection search: $FR3_CONFIG, ./fr3.yaml, ~/.config/fr3/config.yaml.
Controller YAML search: $FR3_CONFIG_DIR/, ./configs/, <package>/configs/.
Env vars FR3_DESK_USER/PASS/NUC_HOST/ROBOT_IP override file values.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

# Early validation so a typo gives a clear error instead of "file not found".
KNOWN_CONTROLLERS = frozenset({
    "idle",
    "cartesian_impedance",
    "joint_impedance",
    "admittance",
    "hybrid",
})


def _candidate_paths() -> list[Path]:
    paths: list[Path] = []
    if env := os.environ.get("FR3_CONFIG"):
        paths.append(Path(env).expanduser())
    paths.append(Path.cwd() / "fr3.yaml")
    paths.append(Path.home() / ".config" / "fr3" / "config.yaml")
    return paths


def load_config() -> dict[str, Any]:
    """Merged config dict. Missing files are skipped; env vars alone work."""
    cfg: dict[str, Any] = {"desk": {}}

    for path in _candidate_paths():
        if path.is_file():
            try:
                import yaml
            except ImportError as e:
                raise ImportError(
                    f"found {path} but PyYAML is not installed; "
                    f"`pip install pyyaml` or remove the file"
                ) from e
            with path.open() as f:
                loaded = yaml.safe_load(f) or {}
            for k, v in loaded.items():
                if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                    cfg[k] = {**cfg[k], **v}
                else:
                    cfg[k] = v
            cfg["_loaded_from"] = str(path)
            break

    # Env vars override file values.
    if v := os.environ.get("FR3_NUC_HOST"):  cfg["nuc_host"] = v
    if v := os.environ.get("FR3_ROBOT_IP"):  cfg["robot_ip"] = v
    if v := os.environ.get("FR3_DESK_USER"): cfg.setdefault("desk", {})["user"] = v
    if v := os.environ.get("FR3_DESK_PASS"): cfg.setdefault("desk", {})["password"] = v

    return cfg


def desk_credentials(cfg: dict | None = None) -> tuple[str, str]:
    """Return (user, password) or raise."""
    cfg = cfg or load_config()
    d = cfg.get("desk") or {}
    user, pwd = d.get("user"), d.get("password")
    if not user or not pwd:
        raise RuntimeError(
            "no Desk credentials. Set them in fr3.yaml under 'desk:' or "
            "export FR3_DESK_USER / FR3_DESK_PASS."
        )
    return user, pwd


# ---- Per-controller config -------------------------------------------------

def _controller_search_dirs() -> list[Path]:
    dirs: list[Path] = []
    if env := os.environ.get("FR3_CONFIG_DIR"):
        dirs.append(Path(env).expanduser())
    dirs.append(Path.cwd() / "configs")
    dirs.append(Path(__file__).parent / "configs")
    return dirs


def load_controller_config(
    name: str,
    profile: str | None = None,
) -> dict[str, Any]:
    """Load configs/<name>[.<profile>].yaml.

    Profile files are self-contained — no merging with the base when a
    profile is named.
    """
    if name not in KNOWN_CONTROLLERS:
        raise ValueError(
            f"unknown controller {name!r}; expected one of "
            f"{sorted(KNOWN_CONTROLLERS)}"
        )

    try:
        import yaml
    except ImportError as e:
        raise ImportError(
            "PyYAML is required for per-controller config loading; "
            "`pip install pyyaml`"
        ) from e

    fname = f"{name}.{profile}.yaml" if profile else f"{name}.yaml"
    searched: list[str] = []
    for d in _controller_search_dirs():
        path = d / fname
        searched.append(str(path))
        if path.is_file():
            with path.open() as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                raise ValueError(
                    f"{path}: expected a mapping at top level, got "
                    f"{type(data).__name__}"
                )
            data["_loaded_from"] = str(path)
            return data

    raise FileNotFoundError(
        f"no config file {fname!r} found. Searched:\n  "
        + "\n  ".join(searched)
        + (f"\nDoes profile {profile!r} exist for {name!r}?" if profile else "")
    )
