"""Streaming Robot client — ZMQ + Cap'n Proto to the NUC daemon.

Wire conventions: meters in base frame, xyzw quaternions, 6-vec
stiffness/damping [tx,ty,tz,rx,ry,rz]. The wire is stateless; per-controller
caches in this class make partial-update kwargs work.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Sequence, Union

import numpy as np
import zmq

from .config import load_controller_config
from .state import State
from .wire import (
    CONTROLLER_FLATTENERS,
    SCHEMA,
    _locate_schema,  # noqa: F401  (re-exported for back-compat)
    aslist,
    aslist_tr,
    flat_adm,
    flat_cart,
    flat_hybrid,
    flat_idle,
    flat_joint,
    force_axis_to_tr,
    selection_to_tr_n_af,
    target_force_world_to_tr_space,
)

logger = logging.getLogger(__name__)

ArrayLike = Union[np.ndarray, Sequence[float]]

# Back-compat alias for external scripts that imported the pre-split name.
_SCHEMA = SCHEMA


class Robot:
    """Streaming client for the NUC-side fr3-stack daemon."""

    def __init__(
        self,
        host: str,
        cmd_port: int = 5555,
        state_port: int = 5556,
        *,
        profiles: Optional[dict[str, str]] = None,
    ):
        """``profiles`` selects per-controller defaults at init, e.g.
        ``{"cartesian_impedance": "stiff"}``."""
        self._cmd_addr   = f"tcp://{host}:{cmd_port}"
        self._state_addr = f"tcp://{host}:{state_port}"
        self._ctx: Optional[zmq.Context] = None
        self._cmd_sock: Optional[zmq.Socket] = None
        self._state_sock: Optional[zmq.Socket] = None

        self._state = State()
        self._state_lock = threading.Lock()
        self._sub_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

        # SUB-loop parse errors throttle to 1/s; a stuck protocol mismatch
        # would otherwise flood at the daemon publish rate.
        self._sub_err_last_t: float = 0.0
        self._sub_err_suppressed: int = 0

        # Eager-load YAML so a missing/invalid config fails at construction
        # rather than as a KeyError deep in send_X.
        prof = profiles or {}
        self._profiles: dict[str, Optional[str]] = {
            "idle":                prof.get("idle"),
            "cartesian_impedance": prof.get("cartesian_impedance"),
            "joint_impedance":     prof.get("joint_impedance"),
            "admittance":          prof.get("admittance"),
            "hybrid":              prof.get("hybrid"),
        }
        self._idle_cache   = flat_idle  (load_controller_config("idle",                self._profiles["idle"]))
        self._cart_cache   = flat_cart  (load_controller_config("cartesian_impedance", self._profiles["cartesian_impedance"]))
        self._joint_cache  = flat_joint (load_controller_config("joint_impedance",     self._profiles["joint_impedance"]))
        self._adm_cache    = flat_adm   (load_controller_config("admittance",          self._profiles["admittance"]))
        self._hybrid_cache = flat_hybrid(load_controller_config("hybrid",              self._profiles["hybrid"]))

    # ---- lifecycle --------------------------------------------------------

    def connect(self) -> None:
        self._ctx = zmq.Context.instance()

        self._cmd_sock = self._ctx.socket(zmq.PUSH)
        self._cmd_sock.setsockopt(zmq.CONFLATE, 1)
        self._cmd_sock.setsockopt(zmq.SNDHWM,   1)
        self._cmd_sock.connect(self._cmd_addr)

        self._state_sock = self._ctx.socket(zmq.SUB)
        self._state_sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._state_sock.setsockopt(zmq.CONFLATE,  1)
        self._state_sock.setsockopt(zmq.RCVHWM,    1)
        self._state_sock.connect(self._state_addr)

        self._stop.clear()
        self._sub_thread = threading.Thread(target=self._sub_loop, daemon=True)
        self._sub_thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._sub_thread is not None and self._sub_thread.is_alive():
            self._sub_thread.join(timeout=1.0)
        for s in (self._cmd_sock, self._state_sock):
            if s is not None:
                s.close(linger=0)
        self._cmd_sock = self._state_sock = None

    def __enter__(self):  self.connect(); return self
    def __exit__(self, *exc): self.close()

    # ---- per-controller config / profile ----------------------------------

    def set_profile(self, controller: str, profile: str | None) -> None:
        """Reload from configs/<controller>[.<profile>].yaml; resets the cache.

        ``profile=None`` reverts to the base file. Any prior in-cache overrides
        from earlier ``send_X(K=...)`` calls are lost.
        """
        if controller not in CONTROLLER_FLATTENERS:
            raise ValueError(
                f"unknown controller {controller!r}; expected one of "
                f"{sorted(CONTROLLER_FLATTENERS)}"
            )
        flattener, attr = CONTROLLER_FLATTENERS[controller]
        y = load_controller_config(controller, profile)
        setattr(self, attr, flattener(y))
        self._profiles[controller] = profile

    def get_profile(self, controller: str) -> str | None:
        """Return the active profile for ``controller`` (None = base)."""
        if controller not in self._profiles:
            raise ValueError(f"unknown controller {controller!r}")
        return self._profiles[controller]

    # ---- streaming setters ------------------------------------------------

    def send_idle(
        self,
        *,
        profile:      str | None = None,
        d_rate:       np.ndarray | list[float] | None = None,
        use_friction: bool | None = None,
        termination:  bool = False,
    ) -> None:
        """Gravity-comp + per-joint damping (hand-guidable). See configs/idle.yaml.

        ``termination=True`` tells the daemon to exit its RT loop.
        """
        if profile is not None:
            self.set_profile("idle", profile)
        c = self._idle_cache
        if d_rate is not None:       c["d_rate"]       = aslist(d_rate, 7)
        if use_friction is not None: c["use_friction"] = bool(use_friction)

        cmd = SCHEMA.Command.new_message()
        cmd.termination = termination
        idle = cmd.config.init("idle")
        idle.dRate       = c["d_rate"]
        idle.useFriction = c["use_friction"]
        self._send(cmd)

    def send_cartesian_impedance(
        self,
        target_pos:            np.ndarray | list[float],
        target_quat_xyzw:      np.ndarray | list[float],
        *,
        profile:               str | None = None,
        K:                     np.ndarray | list[float] | None = None,
        D:                     np.ndarray | list[float] | None = None,
        q_null:                np.ndarray | list[float] | None = None,
        K_null:                float | None = None,
        D_null:                float | None = None,
        max_tau_null:          float | None = None,
        filter_alpha:          float | None = None,
        target_wrench:         np.ndarray | list[float] | None = None,
        max_delta:             np.ndarray | list[float] | None = None,
        use_friction:          bool | None = None,
        linear_interp:         bool | None = None,
        ema:                   bool | None = None,
    ) -> None:
        if profile is not None:
            self.set_profile("cartesian_impedance", profile)
        c = self._cart_cache
        if K is not None:                     c["K"]                     = aslist(K, 6)
        if D is not None:                     c["D"]                     = aslist(D, 6)
        if q_null is not None:                c["q_null"]                = aslist(q_null, 7)
        if K_null is not None:                c["K_null"]                = float(K_null)
        if D_null is not None:                c["D_null"]                = float(D_null)
        if max_tau_null is not None:          c["max_tau_null"]          = float(max_tau_null)
        if filter_alpha is not None:          c["filter_alpha"]          = float(filter_alpha)
        if target_wrench is not None:         c["target_wrench"]         = aslist(target_wrench, 6)
        if max_delta is not None:             c["max_delta"]             = aslist(max_delta, 6)
        if use_friction is not None:          c["use_friction"]          = bool(use_friction)
        if linear_interp is not None:         c["linear_interp"]         = bool(linear_interp)
        if ema is not None:                   c["ema"]                   = bool(ema)

        cmd = SCHEMA.Command.new_message()
        cmd.termination = False
        cart = cmd.config.init("cartesianImpedance")
        cart.targetPos           = aslist(target_pos, 3)
        cart.targetQuatXyzw      = aslist(target_quat_xyzw, 4)
        cart.k                   = c["K"]
        cart.d                   = c["D"]
        cart.qNull               = c["q_null"]
        cart.kNull               = c["K_null"]
        cart.dNull               = c.get("D_null",       0.0)
        cart.maxTauNull          = c.get("max_tau_null", 0.0)
        cart.filterAlpha         = c["filter_alpha"]
        cart.targetWrench        = c["target_wrench"]
        cart.maxDelta            = c["max_delta"]
        cart.useFriction         = c["use_friction"]
        cart.linearInterp        = c["linear_interp"]
        cart.ema                 = c["ema"]
        self._send(cmd)

    def send_move_to(
        self,
        target_pos:       np.ndarray | list[float],
        target_quat_xyzw: np.ndarray | list[float],
        run_time:         float,
        *,
        K:                np.ndarray | list[float] | None = None,
        D:                np.ndarray | list[float] | None = None,
        q_null:           np.ndarray | list[float] | None = None,
        K_null:           float | None = None,
        D_null:           float | None = None,
        max_tau_null:     float | None = None,
    ) -> None:
        """Min-jerk move to a Cartesian goal over ``run_time`` seconds.

        For resets / setup moves only — policy code should stream
        ``send_cartesian_impedance`` directly. Per-call gain overrides do NOT
        mutate the cart cache. Size ``run_time`` against ``|v|≈1.875·Δp/T``
        and ``|a|≈5.77·Δp/T²``; 1 s for ≤30 cm is usually safe.
        """
        if not (run_time > 0.0):
            raise ValueError(f"run_time must be > 0, got {run_time}")

        c = self._cart_cache
        k_eff           = aslist(K,      6) if K      is not None else c["K"]
        d_eff           = aslist(D,      6) if D      is not None else c["D"]
        q_null_eff      = aslist(q_null, 7) if q_null is not None else c["q_null"]
        K_null_eff      = float(K_null)        if K_null      is not None else c["K_null"]
        D_null_eff      = float(D_null)        if D_null      is not None else c.get("D_null",       0.0)
        max_tau_null_eff = float(max_tau_null) if max_tau_null is not None else c.get("max_tau_null", 0.0)

        cmd = SCHEMA.Command.new_message()
        cmd.termination = False
        mt = cmd.config.init("moveTo")
        mt.targetPos      = aslist(target_pos, 3)
        mt.targetQuatXyzw = aslist(target_quat_xyzw, 4)
        mt.runTime        = float(run_time)
        mt.k              = k_eff
        mt.d              = d_eff
        mt.qNull          = q_null_eff
        mt.kNull          = K_null_eff
        mt.dNull          = D_null_eff
        mt.maxTauNull     = max_tau_null_eff
        self._send(cmd)

    def send_joint_impedance(
        self,
        q_target:     np.ndarray | list[float],
        *,
        profile:      str | None = None,
        K_joint:      np.ndarray | list[float] | None = None,
        D_joint:      np.ndarray | list[float] | None = None,
        filter_alpha: float | None = None,
        use_friction: bool | None = None,
    ) -> None:
        if profile is not None:
            self.set_profile("joint_impedance", profile)
        c = self._joint_cache
        if K_joint is not None:      c["K_joint"]      = aslist(K_joint, 7)
        if D_joint is not None:      c["D_joint"]      = aslist(D_joint, 7)
        if filter_alpha is not None: c["filter_alpha"] = float(filter_alpha)
        if use_friction is not None: c["use_friction"] = bool(use_friction)

        cmd = SCHEMA.Command.new_message()
        cmd.termination = False
        ji = cmd.config.init("jointImpedance")
        ji.qTarget     = aslist(q_target, 7)
        ji.kJoint      = c["K_joint"]
        ji.dJoint      = c["D_joint"]
        ji.filterAlpha = c["filter_alpha"]
        ji.useFriction = c["use_friction"]
        self._send(cmd)

    def send_admittance(
        self,
        target_pos:          np.ndarray | list[float],
        target_quat_xyzw:    np.ndarray | list[float],
        *,
        profile:             str | None = None,
        M_adm:               np.ndarray | list[float] | None = None,
        K_adm:               np.ndarray | list[float] | None = None,
        D_adm:               np.ndarray | list[float] | None = None,
        K:                   np.ndarray | list[float] | None = None,
        D:                   np.ndarray | list[float] | None = None,
        q_null:              np.ndarray | list[float] | None = None,
        K_null:                     float | None = None,
        D_null:                     float | None = None,
        max_tau_null:               float | None = None,
        filter_alpha:               float | None = None,
        wrench_filter_alpha:        float | None = None,
        dq_filter_alpha:            float | None = None,
        output_torque_filter_alpha: float | None = None,
        max_delta_tau:              float | None = None,
        error_clip:                 np.ndarray | list[float] | None = None,
        use_friction:               bool | None = None,
        require_ft_sensor:   bool = True,
        ft_sensor_timeout:   float = 2.0,
    ) -> None:
        """Cartesian admittance: virtual M-K-D at the EE driven by F_ext.

        Refuses to start unless a calibrated FT sensor is publishing; pass
        ``require_ft_sensor=False`` to fall back to libfranka's O_F_ext_hat_K
        (debug only — ~3-5 N noise, biased by load).
        """
        if require_ft_sensor:
            self._require_ft_sensor("send_admittance", ft_sensor_timeout)

        if profile is not None:
            self.set_profile("admittance", profile)
        c = self._adm_cache
        if M_adm is not None:        c["M_adm"]        = aslist(M_adm, 6)
        if K_adm is not None:        c["K_adm"]        = aslist(K_adm, 6)
        if D_adm is not None:        c["D_adm"]        = aslist(D_adm, 6)
        if K is not None:            c["K"]            = aslist(K, 6)
        if D is not None:            c["D"]            = aslist(D, 6)
        if q_null is not None:       c["q_null"]       = aslist(q_null, 7)
        if K_null is not None:              c["K_null"]              = float(K_null)
        if D_null is not None:              c["D_null"]              = float(D_null)
        if max_tau_null is not None:        c["max_tau_null"]        = float(max_tau_null)
        if filter_alpha is not None:               c["filter_alpha"]               = float(filter_alpha)
        if wrench_filter_alpha is not None:        c["wrench_filter_alpha"]        = float(wrench_filter_alpha)
        if dq_filter_alpha is not None:            c["dq_filter_alpha"]            = float(dq_filter_alpha)
        if output_torque_filter_alpha is not None: c["output_torque_filter_alpha"] = float(output_torque_filter_alpha)
        if max_delta_tau is not None:              c["max_delta_tau"]              = float(max_delta_tau)
        if error_clip is not None:
            c["error_clip"] = [] if len(error_clip) == 0 else aslist(error_clip, 6)
        if use_friction is not None:               c["use_friction"]               = bool(use_friction)

        cmd = SCHEMA.Command.new_message()
        cmd.termination = False
        ad = cmd.config.init("admittance")
        ad.targetPos         = aslist(target_pos, 3)
        ad.targetQuatXyzw    = aslist(target_quat_xyzw, 4)
        ad.mAdm              = c["M_adm"]
        ad.kAdm              = c["K_adm"]
        ad.dAdm              = c["D_adm"]
        ad.k                 = c["K"]
        ad.d                 = c["D"]
        ad.qNull             = c["q_null"]
        ad.kNull             = c["K_null"]
        ad.dNull             = c.get("D_null",       0.0)
        ad.maxTauNull        = c.get("max_tau_null", 0.0)
        ad.filterAlpha             = c["filter_alpha"]
        ad.wrenchFilterAlpha       = c["wrench_filter_alpha"]
        ad.dqFilterAlpha           = c["dq_filter_alpha"]
        ad.outputTorqueFilterAlpha = c["output_torque_filter_alpha"]
        ad.maxDeltaTau             = c.get("max_delta_tau", 0.0)
        ad.errorClip               = c.get("error_clip",    [])
        ad.useFriction             = c["use_friction"]
        self._send(cmd)

    def send_hybrid(
        self,
        target_pos:        np.ndarray | list[float],
        target_quat_xyzw:  np.ndarray | list[float],
        *,
        profile:           str | None = None,
        target_wrench_Tr:  np.ndarray | list[float] | None = None,
        n_af:              int | None = None,
        Tr:                np.ndarray | list[float] | str | None = None,
        force_thresholds:  np.ndarray | list[float] | None = None,
        torque_thresholds: np.ndarray | list[float] | None = None,
        wrench_deadband:   np.ndarray | list[float] | None = None,
        linear_interp:     bool | None = None,
        inner_v_filter_alpha: float | None = None,
        require_ft_sensor: bool = True,
        ft_sensor_timeout: float = 2.0,
    ) -> None:
        """Hybrid force-position control. Most gains live in hybrid.yaml.

        First ``n_af`` rows of ``Tr`` are force-controlled (PID tracks
        ``target_wrench_Tr``); remaining axes track pose rigidly. ``n_af=0``
        collapses to pure admittance. ``Tr`` accepts ``"identity"``, a 6×6,
        or a flat length-36. ``force_thresholds`` / ``torque_thresholds``
        (N / Nm) are soft contact-trip caps; an empty list keeps the daemon's
        startup ``setCollisionBehavior`` defaults, per-axis 0 means uncapped.
        Same FT-sensor gate as ``send_admittance``.
        """
        if require_ft_sensor:
            self._require_ft_sensor("send_hybrid", ft_sensor_timeout)

        if profile is not None:
            self.set_profile("hybrid", profile)
        c = self._hybrid_cache
        if n_af is not None:             c["n_af"]             = int(n_af)
        if Tr is not None:               c["Tr"]               = aslist_tr(Tr)
        if target_wrench_Tr is not None: c["target_wrench_Tr"] = aslist(target_wrench_Tr, 6)
        if force_thresholds is not None:
            c["force_thresholds"]  = [] if len(force_thresholds) == 0 \
                                        else aslist(force_thresholds, 6)
        if torque_thresholds is not None:
            c["torque_thresholds"] = [] if len(torque_thresholds) == 0 \
                                        else aslist(torque_thresholds, 7)
        if wrench_deadband is not None:
            c["wrench_deadband"] = [] if len(wrench_deadband) == 0 \
                                      else aslist(wrench_deadband, 6)
        if linear_interp is not None: c["linear_interp"] = bool(linear_interp)
        if inner_v_filter_alpha is not None:
            c["inner_v_filter_alpha"] = float(inner_v_filter_alpha)

        if not 0 <= c["n_af"] <= 6:
            raise ValueError(f"n_af must be in [0,6], got {c['n_af']}")

        cmd = SCHEMA.Command.new_message()
        cmd.termination = False
        h = cmd.config.init("hybrid")
        h.targetPos       = aslist(target_pos, 3)
        h.targetQuatXyzw  = aslist(target_quat_xyzw, 4)
        h.nAf             = c["n_af"]
        h.tr              = c["Tr"]
        h.targetWrenchTr  = c["target_wrench_Tr"]
        h.mAdm            = c["M_adm"]
        h.kAdm            = c["K_adm"]
        h.dAdm            = c["D_adm"]
        h.pidPTrans       = c["P_trans"]
        h.pidITrans       = c["I_trans"]
        h.pidDTrans       = c["D_trans"]
        h.pidPRot         = c["P_rot"]
        h.pidIRot         = c["I_rot"]
        h.pidDRot         = c["D_rot"]
        h.pidILimit       = c["I_limit"]
        h.stiction        = c["stiction"]
        h.maxSpringForce  = c["max_spring_force"]
        h.maxSpringTorque = c["max_spring_torque"]
        h.k               = c["K"]
        h.d               = c["D"]
        h.qNull           = c["q_null"]
        h.kNull           = c["K_null"]
        h.dNull           = c.get("D_null",       0.0)
        h.maxTauNull      = c.get("max_tau_null", 0.0)
        h.filterAlpha       = c["filter_alpha"]
        h.wrenchFilterAlpha = c["wrench_filter_alpha"]
        h.dqFilterAlpha            = c.get("dq_filter_alpha",            1.0)
        h.outputTorqueFilterAlpha  = c.get("output_torque_filter_alpha", 1.0)
        h.maxDeltaTau              = c.get("max_delta_tau",              0.0)
        h.errorClip                = c.get("error_clip",                 [])
        h.useFriction       = c["use_friction"]
        h.forceThresholds  = c["force_thresholds"]
        h.torqueThresholds = c["torque_thresholds"]
        h.wrenchDeadband   = c.get("wrench_deadband", [])
        h.linearInterp     = c.get("linear_interp", True)
        h.innerVFilterAlpha = c.get("inner_v_filter_alpha", 0.1)
        self._send(cmd)

    # ---- frankapy-style hybrid force-position helpers ---------------------

    def send_hybrid_force_position(
        self,
        target_pos:        np.ndarray | list[float],
        target_quat_xyzw:  np.ndarray | list[float],
        target_force:      np.ndarray | list[float],
        *,
        S:                 np.ndarray | list[float] | None = None,
        position_kps_cart: np.ndarray | list[float] | None = None,
        force_thresholds:  np.ndarray | list[float] | None = None,
        torque_thresholds: np.ndarray | list[float] | None = None,
        linear_interp:     bool | None = None,
        inner_v_filter_alpha: float | None = None,
        profile:           str | None = None,
        require_ft_sensor: bool = True,
        ft_sensor_timeout: float = 2.0,
    ) -> None:
        """frankapy-style hybrid command: takes ``S`` + base-frame ``target_force``.

        ``S[i] >= 0.5`` ⇒ position-controlled, ``< 0.5`` ⇒ force-controlled.
        ``target_force`` is base-frame; entries on position axes are ignored.
        ``position_kps_cart`` mutates the hybrid cache (sticky).
        """
        if profile is not None:
            self.set_profile("hybrid", profile)

        S_eff = [1.0] * 6 if S is None else S
        Tr_flat, n_af = selection_to_tr_n_af(S_eff)
        twr_tr = target_force_world_to_tr_space(S_eff, target_force)

        c = self._hybrid_cache
        if position_kps_cart is not None:
            c["K"] = aslist(position_kps_cart, 6)

        self.send_hybrid(
            target_pos=target_pos,
            target_quat_xyzw=target_quat_xyzw,
            Tr=Tr_flat,
            n_af=n_af,
            target_wrench_Tr=twr_tr,
            force_thresholds=force_thresholds,
            torque_thresholds=torque_thresholds,
            linear_interp=linear_interp,
            inner_v_filter_alpha=inner_v_filter_alpha,
            require_ft_sensor=require_ft_sensor,
            ft_sensor_timeout=ft_sensor_timeout,
        )

    def run_hybrid_force_position(
        self,
        duration:           float,
        *,
        target_poses:       Sequence | None = None,
        target_force:       np.ndarray | list[float] | None = None,
        S:                  np.ndarray | list[float] | None = None,
        dt:                 float = 0.01,
        position_kps_cart:  np.ndarray | list[float] | None = None,
        force_thresholds:   np.ndarray | list[float] | None = None,
        torque_thresholds:  np.ndarray | list[float] | None = None,
        linear_interp:      bool | None = None,
        inner_v_filter_alpha: float | None = None,
        target_fn=None,
        profile:            str | None = None,
        require_ft_sensor:  bool = True,
    ) -> None:
        """Stream hybrid commands at 1/dt Hz for ``duration`` seconds (blocking).

        Pass at most one of ``target_poses`` (N≥ceil(duration/dt) tuples or
        Pose objects) or ``target_fn(t) -> (pos, quat_xyzw)``. Neither ⇒ hold
        current pose. Does NOT auto-relax on return.
        """
        if target_poses is not None and target_fn is not None:
            raise ValueError("pass at most one of target_poses / target_fn")
        if not (duration > 0.0):
            raise ValueError(f"duration must be > 0, got {duration}")
        if not (dt > 0.0):
            raise ValueError(f"dt must be > 0, got {dt}")

        S_eff = [1.0] * 6 if S is None else S
        force_eff = [0.0] * 6 if target_force is None else target_force

        # FT-sensor gate once up-front; per-tick sends bypass via require_ft_sensor=False.
        if require_ft_sensor:
            self._require_ft_sensor("run_hybrid_force_position", 2.0)

        if target_poses is None and target_fn is None:
            obs = self.wait_for_state()
            p_hold = obs.pos.copy()
            q_hold = obs.quat_xyzw.copy()

            def _hold_fn(_t: float):
                return p_hold, q_hold
            target_fn = _hold_fn

        if target_poses is not None:
            n_steps = int(np.ceil(duration / dt))
            poses = list(target_poses)
            if len(poses) < n_steps:
                raise ValueError(
                    f"target_poses has {len(poses)} entries, need at least "
                    f"{n_steps} for duration={duration}s at dt={dt}s"
                )

            def _seq_fn(t: float):
                idx = min(int(t / dt), len(poses) - 1)
                p = poses[idx]
                if hasattr(p, "pos") and hasattr(p, "quat"):
                    return p.pos, p.quat
                return p[0], p[1]
            target_fn = _seq_fn

        # First tick installs S / position_kps_cart / thresholds into the cache;
        # later ticks reuse it (avoid re-flattening Tr each iteration).
        t_start = time.monotonic()
        next_t  = t_start
        first   = True
        while True:
            now = time.monotonic()
            t = now - t_start
            if t >= duration:
                break
            pos, quat = target_fn(t)
            if first:
                self.send_hybrid_force_position(
                    target_pos=pos,
                    target_quat_xyzw=quat,
                    target_force=force_eff,
                    S=S_eff,
                    position_kps_cart=position_kps_cart,
                    force_thresholds=force_thresholds,
                    torque_thresholds=torque_thresholds,
                    linear_interp=linear_interp,
                    inner_v_filter_alpha=inner_v_filter_alpha,
                    profile=profile,
                    require_ft_sensor=False,
                )
                first = False
            else:
                self.send_hybrid_force_position(
                    target_pos=pos,
                    target_quat_xyzw=quat,
                    target_force=force_eff,
                    S=S_eff,
                    require_ft_sensor=False,
                )
            next_t += dt
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

    def apply_effector_forces_along_axis(
        self,
        run_duration:       float,
        acc_duration:       float,
        max_translation:    float,
        forces:             np.ndarray | list[float],
        *,
        dt:                 float = 0.01,
        force_thresholds:   np.ndarray | list[float] | None = None,
        torque_thresholds:  np.ndarray | list[float] | None = None,
        position_kps_cart:  np.ndarray | list[float] | None = None,
        profile:            str | None = None,
        require_ft_sensor:  bool = True,
    ) -> None:
        """Apply a trapezoidally-ramped force along ``forces / ||forces||``.

        Pose is held on the other 5 axes. Ramps up over ``acc_duration``,
        holds, ramps back to 0. ``max_translation`` (m) aborts early if EE
        drifts further from its start in any direction.
        """
        if not (run_duration > 0.0):
            raise ValueError(f"run_duration must be > 0, got {run_duration}")
        if not (acc_duration < 0.5 * run_duration):
            raise ValueError(
                f"acc_duration ({acc_duration}) must be < 0.5 * run_duration "
                f"({0.5 * run_duration})"
            )
        if acc_duration < 0.0:
            raise ValueError(f"acc_duration must be >= 0, got {acc_duration}")
        if not (max_translation > 0.0):
            raise ValueError(
                f"max_translation must be > 0, got {max_translation}"
            )

        Tr_flat, n_af, _twr_tr_unit, mag = force_axis_to_tr(forces)
        if mag == 0.0:
            raise ValueError("forces has zero magnitude")

        if profile is not None:
            self.set_profile("hybrid", profile)
        if position_kps_cart is not None:
            self._hybrid_cache["K"] = aslist(position_kps_cart, 6)

        if require_ft_sensor:
            self._require_ft_sensor("apply_effector_forces_along_axis", 2.0)

        obs = self.wait_for_state()
        p0 = obs.pos.copy()
        q0 = obs.quat_xyzw.copy()

        def _ramped_force(t: float) -> float:
            if acc_duration <= 0.0:
                return mag if 0.0 <= t < run_duration else 0.0
            t_tail = run_duration - acc_duration
            if t < acc_duration:
                return mag * (t / acc_duration)
            if t > t_tail:
                return mag * max(0.0, (run_duration - t) / acc_duration)
            return mag

        t_start = time.monotonic()
        next_t  = t_start
        first   = True
        aborted = False
        while True:
            now = time.monotonic()
            t = now - t_start
            if t >= run_duration:
                break

            cur = self.state
            if cur.valid:
                drift = float(np.linalg.norm(cur.pos - p0))
                if drift > max_translation:
                    aborted = True
                    break

            target_wrench_Tr = [_ramped_force(t), 0.0, 0.0, 0.0, 0.0, 0.0]
            if first:
                self.send_hybrid(
                    target_pos=p0,
                    target_quat_xyzw=q0,
                    Tr=Tr_flat,
                    n_af=n_af,
                    target_wrench_Tr=target_wrench_Tr,
                    force_thresholds=force_thresholds,
                    torque_thresholds=torque_thresholds,
                    require_ft_sensor=False,
                )
                first = False
            else:
                self.send_hybrid(
                    target_pos=p0,
                    target_quat_xyzw=q0,
                    target_wrench_Tr=target_wrench_Tr,
                    require_ft_sensor=False,
                )
            next_t += dt
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.monotonic()

        # Final zero-force command for a clean hand-off if the caller
        # leaves the daemon in hybrid mode.
        self.send_hybrid(
            target_pos=p0,
            target_quat_xyzw=q0,
            target_wrench_Tr=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            require_ft_sensor=False,
        )
        if aborted:
            raise RuntimeError(
                "apply_effector_forces_along_axis: max_translation "
                f"({max_translation:.3f} m) exceeded — call aborted"
            )

    def terminate(self) -> None:
        """Tell the daemon to stop the RT loop and exit."""
        self.send_idle(termination=True)

    # ---- read -------------------------------------------------------------

    @property
    def state(self) -> State:
        with self._state_lock:
            return self._state.copy()

    def wait_for_state(self, timeout: float = 5.0) -> State:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.state
            if s.valid:
                return s
            time.sleep(0.01)
        raise TimeoutError("no state from daemon")

    def _require_ft_sensor(self, op: str, timeout: float = 2.0) -> None:
        """Block until State.has_ft_sensor or raise RuntimeError.

        ``op`` names the calling method in the error message.
        """
        s = self.wait_for_state(timeout=max(timeout, 0.5))
        deadline = time.monotonic() + timeout
        while not s.has_ft_sensor and time.monotonic() < deadline:
            time.sleep(0.05)
            s = self.state
        if not s.has_ft_sensor:
            raise RuntimeError(
                f"{op}: no FT sensor wrench available — State.wrench_ft is "
                "None. Either start fr3-stack with `--ft-sensor-kind <kind> "
                "--ft-sensor-config <str>` (env: FR3_FT_SENSOR_KIND, "
                "FR3_FT_SENSOR_CONFIG), or pass require_ft_sensor=False to "
                "fall back to libfranka's joint-torque estimate (debug only)."
            )

    # ---- internal ---------------------------------------------------------

    def _send(self, cmd) -> None:
        if self._cmd_sock is None:
            raise RuntimeError("not connected; call connect() or use context manager")
        self._cmd_sock.send(cmd.to_bytes())

    def _sub_loop(self) -> None:
        assert self._state_sock is not None
        poller = zmq.Poller()
        poller.register(self._state_sock, zmq.POLLIN)
        while not self._stop.is_set():
            socks = dict(poller.poll(timeout=200))
            if self._state_sock in socks:
                try:
                    payload = self._state_sock.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    continue
                try:
                    with SCHEMA.State.from_bytes(payload) as msg:
                        new = State.from_capnp(msg)
                except Exception:                       # noqa: BLE001
                    now = time.monotonic()
                    if now - self._sub_err_last_t >= 1.0:
                        suppressed = self._sub_err_suppressed
                        self._sub_err_last_t = now
                        self._sub_err_suppressed = 0
                        if suppressed:
                            logger.warning(
                                "failed to parse State message "
                                "(%d more suppressed in last 1s)",
                                suppressed,
                                exc_info=True,
                            )
                        else:
                            logger.warning(
                                "failed to parse State message",
                                exc_info=True,
                            )
                    else:
                        self._sub_err_suppressed += 1
                    continue
                with self._state_lock:
                    self._state = new
