# Wire protocol for the fr3-stack daemon.
#
# Single source of truth: this file is loaded by both the Python client
# (via pycapnp at runtime) and the NUC C++ daemon (via capnp_generate_cpp
# at build time). Editing it in one place propagates to both sides.
#
# Vector ordering conventions (mirrored on both sides):
#   pos       : [x, y, z]
#   quatXyzw  : [x, y, z, w]            (scipy.spatial.transform.Rotation order)
#   k / d     : [tx, ty, tz, rx, ry, rz] (translation first, then rotation)
#   q / qTarget : 7-vector, joint order j1..j7
#
# Each Command is a *complete* config — clients should send full struct each
# time. The Python client caches the last-sent values to make incremental
# updates ergonomic, so partial sends never hit the wire.

@0xb84d2c8e7f3a915a;

# Idle / hand-guidable mode (gravity-comp + per-joint inertia-aware damping +
# optional friction compensation). All fields tunable from Python so user can
# fine-tune the hand-guiding feel without rebuilding the daemon.
#
# Damping math: τ_damp[i] = -dRate[i] · (M(q)·dq)[i]
# Time constant per joint i is 1/dRate[i]. Joints with high inertia (J2, J4)
# usually want a higher dRate to feel similarly damped to lighter joints.
#
# Friction comp uses the Cognetti / FrankaEmikaPandaDynModel sigmoid with
# baked-in FR3 fp1/fp2/fp3 — when on, the natural mechanical friction in
# each joint is actively cancelled (matches franka_ros2/CRISP gravity_comp).
struct IdleCmd {
    dRate           @0 :List(Float64);   # length 7, per-joint damping rate [1/s]
    useFriction     @1 :Bool;            # cancel joint friction (Cognetti FR3 fit)
}

struct CartesianImpedanceCmd {
    targetPos       @0 :List(Float64);   # length 3
    targetQuatXyzw  @1 :List(Float64);   # length 4
    k               @2 :List(Float64);   # length 6
    d               @3 :List(Float64);   # length 6
    qNull           @4 :List(Float64);   # length 7
    kNull           @5 :Float64;
    filterAlpha     @6 :Float64;
    # @7 was useOperationalSpace; OSC removed 2026-05-09 — see
    # docs/controllers.md "Why we use J^T, not real OSC" for the rationale.
    # Capnp requires ordinals to be contiguous, so the slot stays as a typed
    # placeholder (Bool, default false). Both client and daemon ignore it.
    unusedOperationalSpace @7 :Bool;
    targetWrench    @8 :List(Float64);   # length 6, F_target in base frame; τ += J^T · F
    maxDelta        @9 :List(Float64);   # length 6, |error| clipped per axis (0 = no clip)
    useFriction     @10 :Bool;           # add Stribeck friction comp
    # --- target smoothing toggles ---
    # LERP between received cmds (deoxys/UMI-style). Bridges low-rate client
    # cmds → 1 kHz daemon target. Disable for raw-step input (debug, or when
    # client is already producing a 1 kHz dense stream).
    linearInterp    @11 :Bool = true;
    # First-order EMA on the (possibly LERP'd) target before PD error.
    # τ ≈ (1−α)/α · 1 ms ≈ 19 ms at α=0.05. Disable when LERP is enough or
    # when the extra 19 ms phase lag matters.
    ema             @12 :Bool = true;
    # Nullspace damping. 0 (default) ⇒ auto-derive as 2·√kNull (critical for
    # unit mass). Set non-zero to decouple from kNull — useful when the
    # elbow "rings" with the auto value or when you want a soft anchor but
    # stiff stopper.
    dNull           @13 :Float64;
    # Per-joint clip on the nullspace τ contribution before it's summed into
    # the final command. 0 (default) ⇒ no clip. Typical guard: 10-20 Nm.
    # Prevents the nullspace term from saturating joint torques near
    # singularities or for large q_null offsets.
    maxTauNull      @14 :Float64;
}

struct JointImpedanceCmd {
    qTarget         @0 :List(Float64);   # length 7
    kJoint          @1 :List(Float64);   # length 7
    dJoint          @2 :List(Float64);   # length 7
    filterAlpha     @3 :Float64;
    useFriction     @4 :Bool;
}

# Cartesian admittance: virtual mass-spring-damper at the EE driven by F_ext
# (libfranka's O_F_ext_hat_K — same vector we publish as wrenchExt). The inner
# integrator produces a moving "compliant target" that is then tracked by the
# regular cartesian impedance loop.
struct AdmittanceCmd {
    # Outer (admittance) loop — virtual dynamics in task space
    targetPos       @0 :List(Float64);   # length 3
    targetQuatXyzw  @1 :List(Float64);   # length 4
    mAdm            @2 :List(Float64);   # length 6, virtual mass
    kAdm            @3 :List(Float64);   # length 6, virtual spring (toward target)
    dAdm            @4 :List(Float64);   # length 6, virtual damper
    # Inner (impedance) loop — same as CartesianImpedanceCmd
    k               @5 :List(Float64);   # length 6
    d               @6 :List(Float64);   # length 6
    qNull           @7 :List(Float64);   # length 7
    kNull           @8 :Float64;
    filterAlpha     @9 :Float64;
    useFriction     @10 :Bool;
    # EMA low-pass on F_ext before the outer loop. At 1 kHz tick, 0.02 ≈ 3 Hz
    # cutoff (Bota-ROS-example default), 1.0 = pass-through. Tame admittance
    # jitter caused by M⁻¹ amplifying HF wrench noise.
    wrenchFilterAlpha @11 :Float64;
    # EMA low-pass on raw dq before it enters the inner-loop D term, the
    # nullspace D term, and friction compensation. CRISP equivalent:
    # `filter.dq` (default 0.5). 1.0 = pass-through (fr3_stack legacy
    # behavior, snappier but noisier). Drop to 0.3-0.5 to soften the D
    # response and match a "polished" admittance feel on FR3.
    dqFilterAlpha @12 :Float64;
    # EMA low-pass on the final commanded τ before it leaves the controller.
    # CRISP equivalent: `filter.output_torque` (default 0.2). 1.0 = pass-
    # through. 0.1-0.3 = visibly smoother joint motion at the cost of small
    # control-loop phase lag — almost always worth it on real hardware.
    outputTorqueFilterAlpha @13 :Float64;
    # Nullspace damping. 0 ⇒ auto = 2·√kNull. See CartesianImpedanceCmd.dNull.
    dNull                   @14 :Float64;
    # Per-joint nullspace τ clip. 0 ⇒ no clip. See CartesianImpedanceCmd.maxTauNull.
    maxTauNull              @15 :Float64;
    # Per-tick |Δτ| cap (Nm). Pixi-style torque-rate saturation: each tick the
    # controller clips |τ[i] − τ_prev[i]| ≤ maxDeltaTau before the output EMA.
    # 0 ⇒ disabled (legacy behavior). Pixi default 0.5 → 500 Nm/s at 1 kHz.
    maxDeltaTau             @16 :Float64;
    # Per-axis pose-error clip applied before K·e in the outer impedance loop.
    # Length 0 ⇒ disabled. Length 6 ⇒ per-axis cap [|ex|,|ey|,|ez|,|er_x|,|er_y|,|er_z|]
    # in (m, m, m, rad, rad, rad). Pixi default [0.1, 0.1, 0.1, 0.5, 0.5, 0.5].
    errorClip               @17 :List(Float64);
}

# Hybrid force-position controller. See nuc/main.cpp HybridController for
# the algorithm (yifan-hou HFVC inner loop + cartesian impedance outer).
#
# Conventions:
#   * Tr is a 6×6 axis-decomposition matrix sent ROW-MAJOR as 36 floats
#     (so Tr[0..6]=row0, Tr[6..12]=row1, …). The first nAf rows of Tr are
#     force-controlled axes; the remaining are velocity-controlled.
#   * targetWrenchTr is the wrench command in Tr-space (only the first nAf
#     entries are active; the rest are ignored).
#   * Sign: targetWrenchTr is the wrench the ROBOT should apply to the
#     environment. PID error inside the daemon is computed as
#     (targetWrenchTr − Tr·(−F_ext_world)).
#   * NEVER set targetWrench non-zero on cartesianImpedance in parallel —
#     hybrid already drives the inner FT-PID loop.
struct HybridCmd {
    targetPos       @0  :List(Float64);   # length 3
    targetQuatXyzw  @1  :List(Float64);   # length 4

    # Force-velocity decomposition.
    nAf             @2  :UInt8;           # 0..6
    tr              @3  :List(Float64);   # length 36, row-major 6×6
    targetWrenchTr  @4  :List(Float64);   # length 6, in Tr-space

    # Inner admittance dynamics (M·a = wrench − D·v + K·err).
    mAdm            @5  :List(Float64);   # length 6, virtual mass
    kAdm            @6  :List(Float64);   # length 6, virtual stiffness
    dAdm            @7  :List(Float64);   # length 6, virtual damping

    # Force-tracking PID gains (scalar per trans/rot block).
    pidPTrans       @8  :Float64;
    pidITrans       @9  :Float64;
    pidDTrans       @10 :Float64;
    pidPRot         @11 :Float64;
    pidIRot         @12 :Float64;
    pidDRot         @13 :Float64;
    pidILimit       @14 :List(Float64);   # length 6, anti-windup clamp

    # Per-axis stiction dead-band (Tr-space). Use stiction OR PID, not both.
    stiction        @15 :List(Float64);   # length 6

    # Spring force/torque magnitude clip. 0 = no clip on that block.
    maxSpringForce  @16 :Float64;         # [N]
    maxSpringTorque @17 :Float64;         # [Nm]

    # Outer cartesian impedance (tracks inner_SE3 → τ).
    k               @18 :List(Float64);   # length 6
    d               @19 :List(Float64);   # length 6
    qNull           @20 :List(Float64);   # length 7
    kNull           @21 :Float64;
    filterAlpha     @22 :Float64;
    useFriction     @23 :Bool;

    # Soft contact-trip thresholds (frankapy parity). Per-call override on
    # top of the daemon's startup setCollisionBehavior. libfranka does NOT
    # allow re-running setCollisionBehavior while robot.control() is live,
    # so the daemon enforces these by monitoring O_F_ext_hat_K and
    # tau_ext_hat_filtered itself: on |F_i| > forceThresholds[i] or
    # |τ_j| > torqueThresholds[j] it switches active controller to
    # gravity-comp, resets, and reports the trip via state.lastError.
    #
    # Length 0 OR all-zero  ⇒  no per-call cap (startup defaults apply).
    # Per-axis zero entry within a non-empty list  ⇒  that axis is unbounded.
    # Units: Newtons (forceThresholds[0..3]) and Newton-metres (forceThresholds[3..6],
    # torqueThresholds[0..7]).
    forceThresholds  @24 :List(Float64);   # length 0 or 6 ([fx,fy,fz,tx,ty,tz])
    torqueThresholds @25 :List(Float64);   # length 0 or 7 (per-joint τ_ext)

    # Nullspace damping. 0 ⇒ auto = 2·√kNull. See CartesianImpedanceCmd.dNull.
    dNull            @26 :Float64;
    # Per-joint nullspace τ clip. 0 ⇒ no clip. See CartesianImpedanceCmd.maxTauNull.
    maxTauNull       @27 :Float64;

    # F_ext EMA: F_filt = α·F_meas + (1−α)·F_filt. Mirror of admittance's
    # field. 1.0 = pass-through (matches pixi/yifan-hou). Schema default 1.0
    # so old clients that omit the field don't freeze the filter at 0.
    wrenchFilterAlpha @28 :Float64 = 1.0;
    # EMA on raw dq before outer D-term + nullspace damping + friction comp.
    # 1.0 = pass-through (legacy). Pixi default 0.5 (~stronger smoothing).
    dqFilterAlpha           @29 :Float64 = 1.0;
    # EMA on final commanded τ before it leaves the controller.
    # 1.0 = pass-through (legacy). Pixi default 0.5.
    outputTorqueFilterAlpha @30 :Float64 = 1.0;
    # Per-tick |Δτ| cap (Nm). See AdmittanceCmd.maxDeltaTau.
    maxDeltaTau             @31 :Float64;
    # Per-axis outer pose-error clip. See AdmittanceCmd.errorClip.
    errorClip               @32 :List(Float64);
    # Per-axis soft deadband on F_ext (base frame), applied AFTER wrenchFilterAlpha
    # EMA and BEFORE the admittance integrator / force PID. Soft = shrinkage:
    # y_i = sign(F_i)·max(|F_i| − eps_i, 0). Length 0 ⇒ disabled (default).
    # Length 6 ⇒ per-axis ε in N (translation 0..2) and Nm (rotation 3..5).
    # Use ~2-3× the residual reported by fr3-ft-calibrate (tens of mN / few mNm).
    # Independent of `stiction` — this one acts on F_ext itself and helps both
    # the n_af=0 admittance path and the n_af>0 PID path.
    wrenchDeadband          @33 :List(Float64);
    # LERP/SLERP the received pose target up to the 1 kHz daemon tick. Same
    # bridge used on the cartesianImpedance path (see CartesianImpedanceCmd
    # .linearInterp). Default true so older clients that don't set the field
    # automatically benefit from smoothing — particularly visible on circle
    # / sweep trajectories streamed at 100-200 Hz, where the raw step train
    # otherwise excites the outer impedance at the client rate. Disable for
    # raw-step input (debug) or when the client is already streaming dense
    # 1 kHz targets.
    linearInterp            @34 :Bool = true;
    # EMA on inner_v_ before it enters the outer D·(v − inner_v) damping
    # term. inner_v_ tracks the LERP'd target at 1-tick lag, which means it
    # inherits LERP's segment-boundary velocity discontinuities (~100 Hz
    # energy from client-rate jitter). Without this LP, those discontinuities
    # propagate through D into joint τ as an audible 100 Hz buzz. 0.1 ≈
    # 9.5 ms LP at 1 kHz tick — smooths across one segment boundary, keeps
    # the legitimate tracking velocity bandwidth (~1 Hz for typical
    # streaming tasks). Schema default 0.1 so unset clients get smoothing.
    # Set 1.0 to disable (pre-LP behavior).
    innerVFilterAlpha       @35 :Float64 = 0.1;
}

# Time-parameterized go-to-pose. Daemon spins up a min-jerk generator anchored
# at the live pose and feeds its output as the cartesian-impedance target on
# every RT tick until run_time elapses; afterwards the controller holds at the
# goal with the supplied impedance gains.
#
# Use this for resets / setup moves, NOT for closed-loop policy execution
# (policies should send cartesianImpedance directly — they're already smooth).
#
# runTime must be sized by the caller to keep peak velocity / acceleration in
# bounds: |v|_peak ≈ 1.875·Δp/T, |a|_peak ≈ 5.77·Δp/T². 1 s for ≤30 cm is
# usually safe; round up when in doubt.
struct MoveToCmd {
    targetPos       @0 :List(Float64);   # length 3
    targetQuatXyzw  @1 :List(Float64);   # length 4
    runTime         @2 :Float64;         # seconds, > 0
    # Impedance config used while the trajectory plays AND after it finishes.
    # filterAlpha is forced to 1.0 internally (the trajectory is already
    # smooth — additional LP filtering would only add lag).
    k               @3 :List(Float64);   # length 6
    d               @4 :List(Float64);   # length 6
    qNull           @5 :List(Float64);   # length 7
    kNull           @6 :Float64;
    # Nullspace damping. 0 ⇒ auto = 2·√kNull. See CartesianImpedanceCmd.dNull.
    dNull           @7 :Float64;
    # Per-joint nullspace τ clip. 0 ⇒ no clip. See CartesianImpedanceCmd.maxTauNull.
    maxTauNull      @8 :Float64;
}

struct Command {
    termination     @0 :Bool;
    config :union {
        idle               @1 :IdleCmd;
        cartesianImpedance @2 :CartesianImpedanceCmd;
        jointImpedance     @3 :JointImpedanceCmd;
        admittance         @4 :AdmittanceCmd;
        hybrid             @5 :HybridCmd;
        moveTo             @6 :MoveToCmd;
    }
}

struct State {
    controller      @0 :Text;
    pos             @1 :List(Float64);   # length 3
    quatXyzw        @2 :List(Float64);   # length 4
    q               @3 :List(Float64);   # length 7
    dq              @4 :List(Float64);   # length 7
    wrenchExt       @5 :List(Float64);   # length 6, libfranka O_F_ext_hat_K (base frame)
    timestamp       @6 :Float64;
    running         @7 :Bool;
    lastError       @8 :Text;
    # External-force/torque from the calibrated FT sensor, rotated to base via
    # R_O_EE. Length 6 when --ft-sensor-kind is set on the daemon AND the
    # backend's worker thread has published at least one frame; empty list
    # otherwise. Python client maps empty → State.wrench_ft = None so
    # consumers can detect "no real sensor available" with State.has_ft_sensor.
    #
    # If the daemon loaded ~/.config/fr3-stack/ft_calibration.yaml at startup,
    # this stream is payload-gravity + bias compensated (ftCompensated = true).
    # Otherwise it's the raw sensor reading rotated to base. The raw signal is
    # always available separately as wrenchFtRaw — useful for diagnostics or
    # for plotting before/after the calibration's effect.
    wrenchFt        @9 :List(Float64);
    # Always raw (only the sensor's startup tare). Length 6 iff a sensor is
    # attached and has published. When ftCompensated = false, this carries
    # exactly the same values as wrenchFt.
    wrenchFtRaw     @10 :List(Float64);
    # True when the daemon is subtracting payload gravity + bias from the
    # downstream wrench (controllers AND wrenchFt). Implies a calibration
    # YAML was successfully loaded at boot.
    ftCompensated   @11 :Bool;
}
