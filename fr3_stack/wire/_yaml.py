"""YAML→cache flatteners and the controller-flatteners registry.

Per-controller YAML lives in configs/<name>.yaml. These functions turn
the parsed dict into a flat dict matching ``send_<controller>`` kwargs
on Robot.

q_null=[0]*7 is a sentinel — the daemon snapshots the activation pose
and uses that as the nullspace anchor instead of literally pulling
joints to 0 (which would be a singular config on FR3).
"""
from __future__ import annotations

from ._schema import aslist, aslist_tr


def flat_idle(y: dict) -> dict:
    """idle.yaml → flat dict matching send_idle kwargs."""
    return {
        "d_rate":       aslist(y["d_rate"], 7),
        "use_friction": bool(y["use_friction"]),
    }


def flat_cart(y: dict) -> dict:
    """cartesian_impedance.yaml → flat dict matching send_cartesian_impedance kwargs."""
    return {
        "K":                     aslist(y["K"], 6),
        "D":                     aslist(y["D"], 6),
        "q_null":                aslist(y["q_null"], 7),
        "K_null":                float(y["K_null"]),
        # Nullspace damping. 0 = auto = 2·√K_null. Optional; older YAMLs
        # without this key keep the auto behavior.
        "D_null":                float(y.get("D_null", 0.0)),
        # Per-joint nullspace τ clip. 0 = no clip. Optional.
        "max_tau_null":          float(y.get("max_tau_null", 0.0)),
        "filter_alpha":          float(y["filter_alpha"]),
        "target_wrench":         aslist(y["target_wrench"], 6),
        "max_delta":             aslist(y["max_delta"], 6),
        "use_friction":          bool(y["use_friction"]),
        # Smoothing toggles default true to preserve historical behavior
        # for older YAMLs that don't list them.
        "linear_interp":         bool(y.get("linear_interp", True)),
        "ema":                   bool(y.get("ema",           True)),
    }


def flat_joint(y: dict) -> dict:
    return {
        "K_joint":      aslist(y["K_joint"], 7),
        "D_joint":      aslist(y["D_joint"], 7),
        "filter_alpha": float(y["filter_alpha"]),
        "use_friction": bool(y["use_friction"]),
    }


def flat_adm(y: dict) -> dict:
    return {
        "M_adm":              aslist(y["admittance"]["M"], 6),
        "K_adm":              aslist(y["admittance"]["K"], 6),
        "D_adm":              aslist(y["admittance"]["D"], 6),
        "K":                  aslist(y["impedance"]["K"], 6),
        "D":                  aslist(y["impedance"]["D"], 6),
        "q_null":             aslist(y["q_null"], 7),
        "K_null":              float(y["K_null"]),
        "D_null":             float(y.get("D_null", 0.0)),
        "max_tau_null":       float(y.get("max_tau_null", 0.0)),
        "filter_alpha":        float(y["filter_alpha"]),
        # Default 1.0 = pass-through (no smoothing) so the controller's F_ext
        # matches what `fr3-ft-publish` shows tick-for-tick. Drop to 0.02-0.1
        # if admittance accel becomes jittery on a noisy sensor.
        "wrench_filter_alpha": float(y.get("wrench_filter_alpha", 1.0)),
        # CRISP-style LP filters on dq and output torque. Default 1.0 keeps
        # the legacy fr3_stack behavior (no smoothing) for old YAMLs that
        # don't list them; admittance.yaml ships 0.5 / 0.2 (CRISP defaults).
        "dq_filter_alpha":            float(y.get("dq_filter_alpha", 1.0)),
        "output_torque_filter_alpha": float(y.get("output_torque_filter_alpha", 1.0)),
        # Pixi-style smoothing chain (default off = legacy fr3_stack behavior).
        # Pixi ships max_delta_tau=0.5, error_clip=[0.1,0.1,0.1,0.5,0.5,0.5].
        "max_delta_tau":              float(y.get("max_delta_tau", 0.0)),
        "error_clip":                 aslist(y["error_clip"], 6)
                                        if y.get("error_clip") else [],
        "use_friction":        bool(y["use_friction"]),
    }


def flat_hybrid(y: dict) -> dict:
    """hybrid.yaml → flat dict matching send_hybrid kwargs."""
    pid = y["force_pid"]
    return {
        "n_af":             int(y["n_af"]),
        "Tr":               aslist_tr(y["Tr"]),
        "target_wrench_Tr": aslist(y["target_wrench"], 6),

        "M_adm":            aslist(y["admittance"]["M"], 6),
        "K_adm":            aslist(y["admittance"]["K"], 6),
        "D_adm":            aslist(y["admittance"]["D"], 6),

        "P_trans":          float(pid["P_trans"]),
        "I_trans":          float(pid["I_trans"]),
        "D_trans":          float(pid["D_trans"]),
        "P_rot":            float(pid["P_rot"]),
        "I_rot":            float(pid["I_rot"]),
        "D_rot":            float(pid["D_rot"]),
        "I_limit":          aslist(pid["I_limit"], 6),

        "stiction":         aslist(y["stiction"], 6),
        "max_spring_force":  float(y["max_spring_force"]),
        "max_spring_torque": float(y["max_spring_torque"]),

        "K":                aslist(y["impedance"]["K"], 6),
        "D":                aslist(y["impedance"]["D"], 6),
        "q_null":           aslist(y["q_null"], 7),
        "K_null":           float(y["K_null"]),
        "D_null":           float(y.get("D_null", 0.0)),
        "max_tau_null":     float(y.get("max_tau_null", 0.0)),
        "filter_alpha":         float(y["filter_alpha"]),
        # F_ext EMA. 1.0 = pass-through. See hybrid.yaml for the rationale.
        "wrench_filter_alpha":  float(y.get("wrench_filter_alpha", 1.0)),
        # Pixi-style smoothing chain. Defaults below = legacy (no smoothing,
        # no rate cap, no error clip). hybrid.yaml ships pixi defaults.
        "dq_filter_alpha":            float(y.get("dq_filter_alpha", 1.0)),
        "output_torque_filter_alpha": float(y.get("output_torque_filter_alpha", 1.0)),
        "max_delta_tau":              float(y.get("max_delta_tau", 0.0)),
        "error_clip":                 aslist(y["error_clip"], 6)
                                        if y.get("error_clip") else [],
        # Per-axis soft deadband on F_ext (after EMA). Empty list = disabled.
        "wrench_deadband":            aslist(y["wrench_deadband"], 6)
                                        if y.get("wrench_deadband") else [],
        "use_friction":         bool(y["use_friction"]),
        # LERP/SLERP bridge to 1 kHz. Default true matches cart and the schema.
        "linear_interp":        bool(y.get("linear_interp", True)),
        # EMA on inner_v before outer D — kills LERP boundary buzz. Schema
        # default 0.1; pass-through is 1.0.
        "inner_v_filter_alpha": float(y.get("inner_v_filter_alpha", 0.1)),

        # Soft contact-trip thresholds (frankapy parity). Default empty list
        # ⇒ no per-call cap (daemon's startup setCollisionBehavior applies).
        # Per-axis zero within a non-empty list ⇒ that axis is unbounded.
        "force_thresholds":  aslist(y["force_thresholds"], 6)
                                if y.get("force_thresholds")  else [],
        "torque_thresholds": aslist(y["torque_thresholds"], 7)
                                if y.get("torque_thresholds") else [],
    }


# Map controller name → (yaml flattener, cache attr name on Robot).
# Drives Robot.set_profile() and the lazy-reload path in send_X(profile=...).
CONTROLLER_FLATTENERS: dict[str, tuple] = {
    "idle":                (flat_idle,   "_idle_cache"),
    "cartesian_impedance": (flat_cart,   "_cart_cache"),
    "joint_impedance":     (flat_joint,  "_joint_cache"),
    "admittance":          (flat_adm,    "_adm_cache"),
    "hybrid":              (flat_hybrid, "_hybrid_cache"),
}
