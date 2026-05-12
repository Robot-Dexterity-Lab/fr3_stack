// =============================================================================
// Cartesian impedance controller for Franka Research 3.
//
// Maps an SE(3) pose error to a joint-torque command via the standard
// impedance law (Hogan 1985) projected through J^T, plus a
// damped-pseudoinverse nullspace projector that biases the kinematically
// redundant DOF toward a configurable rest pose without fighting the task.
//
//   Cartesian wrench   :  F_imp  = -K · e  -  D · ẋ                        (1)
//   Task torque        :  τ_task = J^T · F_imp                              (2)
//   Nullspace bias     :  τ_null = N · (K_null·(q_d − q) − 2·√K_null · q̇)  (3)
//                         N      = I − J^T · J^T#
//   Total command      :  τ      = τ_task + τ_null + c(q,q̇)
//                                 + J^T · F_target + τ_jl  [+ τ_fric]      (4)
//
// Pose error (world frame, SO(3) logarithm form):
//
//   e_p =  p − p_d
//   e_θ = -R · log3( R⁻¹ · R_d )       (rotvec axis·angle, in radians)
//
// Gravity compensation g(q) is added internally by FCI; do NOT add it
// here or it will be double-applied.
//
// We deliberately do NOT use Khatib's operational-space inertia weighting
// (τ = J^T · Λ · F_imp). On FR3 it silently halves rotational K_eff because
// Λ_rot ≈ 0.04 kg·m² is tiny, dropping commanded torque below wrist
// stiction. See docs/controllers.md "Why we use J^T, not real OSC" for
// the full rationale and a comparison with deoxys' split-Λ implementation.
//
// References:
//   Hogan,  "Impedance Control: An Approach to Manipulation",
//           ASME J. Dynamic Systems, Measurement, and Control, 1985.
// =============================================================================

#pragma once

#include <Eigen/Dense>

#include <fr3_stack/controllers/controller_base.hpp>

class CartesianImpedanceController : public Controller {
 public:
    // Replace the active configuration (gains, target, options) wholesale.
    // Filter state and the nullspace snapshot are NOT touched; call
    // reset() first if a clean activation transient is required.
    void set_cfg(const CartesianImpedanceCfg& cfg) { cfg_ = cfg; }

    // Lightweight per-tick target override. Used by motion generators that
    // stream an SE(3) setpoint at high rate without re-sending the full
    // gain configuration. All other cfg fields and the LP smoother state
    // are preserved, so a generator-driven move shares the impedance
    // configuration installed by the most recent set_cfg().
    void set_target(const Eigen::Affine3d& T) { cfg_.target = T; }

    // Whether the dispatcher should feed cfg_.target through the streaming
    // LERP interpolator before set_target(). Read from main.cpp's RT path.
    bool linear_interp_enabled() const { return cfg_.linear_interp; }

    ControllerType        type() const override;
    std::string           name() const override;
    void                  reset(const franka::RobotState& state) override;
    std::array<double, 7> compute(const franka::RobotState& state,
                                  const franka::Model&      model) override;

 private:
    // Configuration installed by the dispatcher. Updated atomically (under
    // a try-lock in the RT path) at the start of each compute().
    CartesianImpedanceCfg cfg_;

    // First-order low-pass state on the SE(3) target. Decouples the 1 kHz
    // inner loop from the slower (≈100 Hz) command stream and absorbs
    // step changes from upstream planners.
    Eigen::Vector3d    smoothed_t_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond smoothed_q_{Eigen::Quaterniond::Identity()};

    // Snapshot of q at activation. Used as the nullspace anchor whenever
    // cfg_.q_null is identically zero (the wire default), since pulling
    // FR3 toward q = 0 is mechanically a poor pose. Set cfg_.q_null
    // explicitly to override.
    Vector7d q_null_snapshot_{Vector7d::Zero()};
};
