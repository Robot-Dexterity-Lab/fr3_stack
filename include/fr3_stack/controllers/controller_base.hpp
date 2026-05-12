// Controller abstract base + Cfg structs + ControllerType enum.
//
// Each controller in fr3_stack/controllers/<name>_controller.hpp inherits
// Controller and overrides reset()/compute(). Cfg structs are plain data:
// the dispatcher fills them from a parsed Cap'n Proto Command and hands
// them to the controller via set_cfg().
//
// Wire compatibility: ControllerType enum names were renamed for clarity
// (Idle → GravityCompensation, Hybrid → HybridForceMotion). The Cap'n
// Proto schema (proto/fr3.capnp) keeps the original wire field names
// (`idle`, `hybrid`) so existing Python clients are unaffected. The
// dispatcher in main.cpp does the wire-string ↔ enum mapping.

#pragma once

#include <Eigen/Dense>
#include <franka/model.h>
#include <franka/robot_state.h>

#include <fr3_stack/utils/controllers_common.hpp>

#include <array>
#include <string>

enum class ControllerType {
    GravityCompensation,    // wire: "idle"
    CartesianImpedance,
    JointImpedance,
    CartesianAdmittance,    // wire: "admittance"
    HybridForceMotion       // wire: "hybrid"
};

struct CartesianImpedanceCfg {
    Eigen::Affine3d target{Eigen::Affine3d::Identity()};
    Vector6d K  = (Vector6d() << 200, 200, 200, 20, 20, 20).finished();
    Vector6d D  = (Vector6d() <<  28,  28,  28,  9,  9,  9).finished();
    // q_null = all-zero is treated by the controller as "use snapshot of q
    // at activation time" instead of literally pulling joints to 0 (which
    // is a bad pose for FR3). Set explicitly to override.
    Vector7d q_null{Vector7d::Zero()};
    double   K_null{100.0};
    // Nullspace damping. 0 = auto-derive as 2·√K_null (critical for unit mass).
    // Set non-zero to decouple from K_null — useful when the elbow rings with
    // the auto value, or when you want a soft anchor but a stiff stopper.
    double   D_null{0.0};
    // Per-joint clip on the nullspace τ contribution. 0 = no clip. Typical
    // guard: 10-20 Nm. Prevents nullspace saturation near singularities or
    // under large q_null offsets.
    double   max_tau_null{0.0};
    double   filter_alpha{0.05};
    Vector6d target_wrench{Vector6d::Zero()};   // F_target in base frame
    Vector6d max_delta{Vector6d::Zero()};       // 0 = no clip per axis
    bool     use_friction{false};
    // Target-smoothing toggles. linear_interp gates the LERP/SLERP that
    // bridges low-rate client cmds to the 1 kHz daemon tick (handled in
    // main.cpp dispatch). ema gates the first-order LP filter inside this
    // controller's compute(). Both default true to preserve historical
    // behavior; turn ema off when linear_interp alone is enough (the LP
    // adds ~19 ms phase lag with no benefit when interp is on and target
    // rate < ~5 Hz).
    bool     linear_interp{true};
    bool     ema{true};
};

struct JointImpedanceCfg {
    Vector7d q_target{Vector7d::Zero()};
    Vector7d K = (Vector7d() << 600, 600, 600, 600, 250, 150, 50).finished();
    Vector7d D = (Vector7d() <<  50,  50,  50,  50,  30,  25, 15).finished();
    double   filter_alpha{0.05};
    bool     use_friction{false};
};

struct CartesianAdmittanceCfg {
    Eigen::Affine3d target{Eigen::Affine3d::Identity()};
    // Outer (admittance) loop
    Vector6d M_adm = (Vector6d() <<  5,  5,  5,  0.5, 0.5, 0.5).finished();
    Vector6d K_adm = (Vector6d() << 200,200,200,  20,  20,  20).finished();
    Vector6d D_adm = (Vector6d() <<  60, 60, 60,   8,   8,   8).finished();
    // Inner (impedance) loop
    Vector6d K     = (Vector6d() << 200,200,200,  20,  20,  20).finished();
    Vector6d D     = (Vector6d() <<  28, 28, 28,   9,   9,   9).finished();
    Vector7d q_null{Vector7d::Zero()};
    double   K_null{100.0};
    // Nullspace damping. 0 = auto = 2·√K_null. See CartesianImpedanceCfg.
    double   D_null{0.0};
    // Per-joint clip on nullspace τ. 0 = no clip. See CartesianImpedanceCfg.
    double   max_tau_null{0.0};
    double   filter_alpha{0.05};
    // EMA low-pass on F_ext before the outer loop. F_filt[k+1] =
    // wrench_filter_alpha · F_meas + (1 − wrench_filter_alpha) · F_filt[k].
    // At 1 kHz tick: 0.02 ≈ 3 Hz cutoff (Bota ROS example default), 1.0 =
    // pass-through. F_ext is multiplied by M⁻¹ in the outer loop, so any
    // HF noise gets amplified into jittery EE accel — keep this low
    // (0.01-0.05).
    double   wrench_filter_alpha{0.02};
    // EMA on raw dq before D-term, nullspace damping, and friction comp.
    // 1.0 = pass-through (default keeps existing tests deterministic).
    // Production default: 0.5 (set in admittance.yaml).
    double   dq_filter_alpha{1.0};
    // EMA on final τ before returning to libfranka. 1.0 = pass-through.
    // Production default: 0.2 (set in admittance.yaml).
    double   output_torque_filter_alpha{1.0};
    // Per-tick |Δτ| cap (Nm). 0 = disabled. Pixi-style torque-rate sat
    // applied before the output τ EMA. Pixi default 0.5.
    double   max_delta_tau{0.0};
    // Per-axis outer pose-error clip applied before K·e (m / rad).
    // All-zero = disabled. Pixi default [0.1,0.1,0.1,0.5,0.5,0.5].
    Vector6d error_clip{Vector6d::Zero()};
    bool     use_friction{false};
};

// Layered: HFVC inner produces a "compliant target" SE(3) trajectory,
// outer Cartesian impedance tracks it. n_af = 0 → pure admittance,
// n_af = 6 → pure force control. See yifan-hou/force_control and
// Hou & Mason ICRA 2019.
//
// On force-controlled axes the contact force comes from K·drift (set K
// low for soft, high for stiff). target_wrench_Tr is the *closed-loop*
// setpoint for the FT-PID — NOT a feedforward; never add J^T·target_wrench
// at the outer layer (that double-applies the command and bypasses spring
// clip / FT loop / safety limits).
struct HybridForceMotionCfg {
    Eigen::Affine3d target{Eigen::Affine3d::Identity()};
    // Force command in Tr-space (first n_af entries are the active axes).
    Vector6d target_wrench_Tr{Vector6d::Zero()};

    // 6×6 force-velocity decomposition. First n_af rows of Tr are
    // force-controlled, remaining 6−n_af are velocity-controlled.
    // Identity = world-frame axis-aligned hybrid (force-z, velocity-xy etc).
    Eigen::Matrix<double, 6, 6> Tr{Eigen::Matrix<double, 6, 6>::Identity()};
    int n_af{0};   // 0 = pure admittance, 6 = pure force control

    // Inner admittance dynamics (virtual M-K-D).
    Vector6d M_adm = (Vector6d() <<   5,   5,   5,  0.5, 0.5, 0.5).finished();
    Vector6d K_adm = (Vector6d() << 200, 200, 200,   20,  20,  20).finished();
    Vector6d D_adm = (Vector6d() <<  60,  60,  60,    8,   8,   8).finished();

    // Force-tracking PID (scalar per trans/rot block, like yifan-hou & CRISP).
    double P_trans{0.0}, I_trans{0.0}, D_trans{0.0};
    double P_rot{0.0},   I_rot{0.0},   D_rot{0.0};
    Vector6d I_limit  = (Vector6d() << 10, 10, 10, 5, 5, 5).finished();
    Vector6d stiction = Vector6d::Zero();   // per-axis dead-band in Tr-space
    double   max_spring_force{50.0};        // clip K_adm·err magnitude (N), 0=off
    double   max_spring_torque{10.0};       // clip K_adm·err magnitude (Nm), 0=off

    // Velocity-axis tracking caps (P0 #5 fix). Without these a small pose
    // error becomes a huge commanded velocity (err / 1 ms) — only kDeltaTauMax
    // bounds the resulting τ, which is too far down the chain.
    double max_inner_v{0.5};      // [m/s]
    double max_inner_w{1.5};      // [rad/s]

    // F_ext EMA (P0 #6 fix). libfranka O_F_ext_hat_K has 3-5 N noise floor;
    // raw values feed straight into PID·P and integrate into PID·I.
    double wrench_filter_alpha{0.02};

    // Per-axis soft deadband on F_ext (base frame, after the EMA). Zero =
    // disabled per axis; all-zero (default) = feature off. Suppresses DC
    // drift from FT calibration residuals before F_ext reaches the
    // admittance integrator (M⁻¹·F_ext) and the force-tracking PID. Use
    // values ~2-3× the residual reported by fr3-ft-calibrate (typically
    // tens of mN translation, a few mNm rotation). Independent of
    // `stiction` — that one acts on the PID tracking error in Tr-space
    // and only helps the n_af>0 path. This one helps both paths.
    Vector6d wrench_deadband{Vector6d::Zero()};

    // Outer impedance (tracks inner_SE3).
    Vector6d K = (Vector6d() << 200, 200, 200, 20, 20, 20).finished();
    Vector6d D = (Vector6d() <<  28,  28,  28,  9,  9,  9).finished();
    Vector7d q_null{Vector7d::Zero()};
    double   K_null{100.0};
    // Nullspace damping. 0 = auto = 2·√K_null. See CartesianImpedanceCfg.
    double   D_null{0.0};
    // Per-joint clip on nullspace τ. 0 = no clip. See CartesianImpedanceCfg.
    double   max_tau_null{0.0};
    double   filter_alpha{0.05};
    // EMA on raw dq before outer D-term + nullspace damping + friction comp.
    // 1.0 = pass-through. Pixi default 0.5.
    double   dq_filter_alpha{1.0};
    // EMA on final τ before returning to libfranka. 1.0 = pass-through.
    // Pixi default 0.5.
    double   output_torque_filter_alpha{1.0};
    // Per-tick |Δτ| cap (Nm). 0 = disabled. Pixi default 0.5.
    double   max_delta_tau{0.0};
    // Per-axis outer pose-error clip applied before K·e.
    // All-zero = disabled. Pixi default [0.1,0.1,0.1,0.5,0.5,0.5].
    Vector6d error_clip{Vector6d::Zero()};
    bool     use_friction{false};
    // Bridge low-rate client targets up to the 1 kHz daemon tick via LERP/SLERP
    // (same path as CartesianImpedanceCfg.linear_interp; handled in main.cpp
    // dispatch). On at default so circle / sweep trajectories streamed at
    // 100-200 Hz don't excite the outer impedance at the client rate.
    bool     linear_interp{true};
    // EMA on inner_v_ before it enters the outer D·(v − inner_v) damping.
    // Without this LP, LERP's segment-boundary velocity discontinuities
    // (= 100 Hz energy under client jitter) propagate through D to joint τ
    // as audible buzz. 0.1 ≈ 9.5 ms LP at 1 kHz tick. 1.0 = pass-through.
    double   inner_v_filter_alpha{0.1};

    // Soft contact-trip thresholds (per-call override on top of the daemon's
    // startup setCollisionBehavior). libfranka can't re-arm setCollisionBehavior
    // while control() is running, so the dispatcher monitors O_F_ext_hat_K /
    // tau_ext_hat_filtered itself and switches to gravity-comp on a trip.
    // Per-axis zero ⇒ that axis is unbounded; all-zero ⇒ feature disabled.
    // Read by main.cpp's RT callback when this is the active controller; the
    // controller's compute() ignores them.
    Vector6d force_thresholds{Vector6d::Zero()};   // |F_ext[i]| cap (N, Nm)
    Vector7d torque_thresholds{Vector7d::Zero()};  // |τ_ext[j]| cap (Nm)
};

class Controller {
 public:
    virtual ~Controller() = default;
    virtual ControllerType type() const = 0;
    virtual std::string name() const = 0;
    virtual void reset(const franka::RobotState& s) = 0;
    virtual std::array<double, 7> compute(const franka::RobotState&,
                                          const franka::Model&) = 0;
};
