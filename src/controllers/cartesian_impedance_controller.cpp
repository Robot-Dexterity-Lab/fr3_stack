// CartesianImpedanceController — implementation. See header for the
// equations and references; this file is the RT-safe realization.

#include <fr3_stack/controllers/cartesian_impedance_controller.hpp>
#include <fr3_stack/utils/controllers_common.hpp>
#include <fr3_stack/utils/fiters.hpp>

#include <algorithm>
#include <cmath>

namespace {

// Damped-pseudoinverse regularization for J^T# in the nullspace projector.
// J·J^T + λ²·I keeps the linear solve well-conditioned through manipulability
// dips. λ = 5e-2 corresponds to ≈ 5 cm equivalent damping in operational
// space — large enough to survive near-singular configurations, small enough
// that the nullspace damping term remains effective in the bulk of the
// workspace.
constexpr double kPinvDamping = 5.0e-2;

// Critical damping ratio for the nullspace stabilizer (D_null = 2·√K_null).
constexpr double kNullCriticalDamping = 2.0;

}  // namespace

// -----------------------------------------------------------------------------
// Identity / metadata
// -----------------------------------------------------------------------------
ControllerType CartesianImpedanceController::type() const {
    return ControllerType::CartesianImpedance;
}

std::string CartesianImpedanceController::name() const {
    return "cartesian_impedance";
}

// -----------------------------------------------------------------------------
// reset
// -----------------------------------------------------------------------------
// Initialize the LP smoother to the current end-effector pose (so the first
// tick produces zero spring force) and snapshot q for the nullspace anchor.
void CartesianImpedanceController::reset(const franka::RobotState& s) {
    const Eigen::Affine3d T(Eigen::Matrix4d::Map(s.O_T_EE.data()));

    smoothed_t_ = T.translation();
    smoothed_q_ = Eigen::Quaterniond(T.linear());

    // Nullspace anchor defaults to "hold this q" rather than literal zero;
    // see the cfg_.q_null comment in the header.
    q_null_snapshot_ = Eigen::Map<const Vector7d>(s.q.data());
}

// -----------------------------------------------------------------------------
// compute
// -----------------------------------------------------------------------------
// Single 1 kHz tick. Returns the joint torque command excluding gravity
// (FCI applies g(q) internally).
std::array<double, 7> CartesianImpedanceController::compute(
    const franka::RobotState& s, const franka::Model& model) {

    // -------------------------------------------------------------------------
    // 1. Read robot state (zero-copy views into libfranka's arrays).
    // -------------------------------------------------------------------------
    Eigen::Map<const Vector7d> q (s.q.data());
    Eigen::Map<const Vector7d> dq(s.dq.data());

    const Eigen::Affine3d    T(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    const Eigen::Vector3d    p = T.translation();
    const Eigen::Quaterniond R(T.linear());

    // -------------------------------------------------------------------------
    // 2. Smooth the SE(3) target with a first-order low-pass (when enabled).
    //    Translation: standard EMA. Orientation: SLERP to filter_alpha.
    //    Hides upstream step rate from the inner loop. With cfg_.ema = false
    //    the smoother is bypassed entirely — the controller tracks the raw
    //    cfg_.target (which the dispatcher may have already LERP'd).
    // -------------------------------------------------------------------------
    const Eigen::Quaterniond q_d_target(cfg_.target.linear());
    if (cfg_.ema) {
        smoothed_t_ = (1.0 - cfg_.filter_alpha) * smoothed_t_
                    +        cfg_.filter_alpha  * cfg_.target.translation();
        smoothed_q_ = smoothed_q_.slerp(cfg_.filter_alpha, q_d_target);
    } else {
        smoothed_t_ = cfg_.target.translation();
        smoothed_q_ = q_d_target;
    }

    // -------------------------------------------------------------------------
    // 3. Cartesian pose error (world frame).
    //    e_p = p − p_d.
    //    e_θ = -R · log3(R⁻¹ · R_d) — true SO(3) logarithm, returns
    //    rotvec axis·angle in radians. CRISP-parity. The earlier
    //    quaternion-vec form (sin(θ/2)·n) gave HALF the magnitude of
    //    log3 for any given angular error, silently halving the user's
    //    effective rotational stiffness; log3 keeps K_rot honest. Eigen's
    //    AngleAxisd inside log3() also handles the short-way-around
    //    selection automatically (angle ∈ [0, π]), making the explicit
    //    quaternion hemisphere flip unnecessary here.
    // -------------------------------------------------------------------------
    Vector6d e;
    e.head<3>() = p - smoothed_t_;

    const Eigen::Matrix3d dR_mat =
        T.linear().transpose() * smoothed_q_.toRotationMatrix();
    e.tail<3>() = -T.linear() * log3(dR_mat);

    // Bound the spring action under a stepped target (pre-LP-filter spike).
    // Per-axis: max_delta[i] == 0 disables clipping on axis i.
    e = clip_error(e, cfg_.max_delta);

    // -------------------------------------------------------------------------
    // 4. Geometric Jacobian and end-effector twist.
    // -------------------------------------------------------------------------
    const std::array<double, 42> J_arr =
        model.zeroJacobian(franka::Frame::kEndEffector, s);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(J_arr.data());

    const Vector6d v = J * dq;

    // -------------------------------------------------------------------------
    // 5. Impedance wrench (Eq. 1) and projection to joint torque (Eq. 2).
    //    We deliberately do NOT use Khatib's operational-space inertia
    //    weighting (τ = J^T · Λ · F). On FR3 it silently halves rotational
    //    K_eff because Λ_rot ≈ 0.04 kg·m² is tiny, dropping commanded
    //    torque below wrist stiction. See docs/controllers.md
    //    "Why we use J^T, not real OSC" for the full rationale.
    // -------------------------------------------------------------------------
    const Vector6d F_imp =
        cfg_.K.asDiagonal() * (-e) - cfg_.D.asDiagonal() * v;
    const Vector7d tau_task = J.transpose() * F_imp;

    // -------------------------------------------------------------------------
    // 6. Nullspace bias (Eq. 4).
    //    Damped pseudoinverse:  J^T# = (J·J^T + λ²·I)⁻¹·J
    //    Nullspace projector :  N    = I − J^T · J^T#
    //    Drive the redundant DOF toward q_null_eff with a critically damped
    //    proportional law; multiplying by N guarantees this term cannot
    //    perturb the task wrench applied at the end-effector.
    // -------------------------------------------------------------------------
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += kPinvDamping * kPinvDamping;

    const Eigen::Matrix<double, 6, 7> JT_pinv = JJt.ldlt().solve(J);
    const Eigen::Matrix<double, 7, 7> N =
        Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * JT_pinv;

    const Vector7d q_null_eff =
        cfg_.q_null.isZero() ? q_null_snapshot_ : cfg_.q_null;

    const double d_null_eff =
        (cfg_.D_null > 0.0) ? cfg_.D_null
                            : kNullCriticalDamping * std::sqrt(cfg_.K_null);
    Vector7d tau_null = N * (cfg_.K_null * (q_null_eff - q) - d_null_eff * dq);
    if (cfg_.max_tau_null > 0.0) {
        for (int i = 0; i < 7; ++i) {
            tau_null[i] = std::clamp(tau_null[i],
                                     -cfg_.max_tau_null,
                                      cfg_.max_tau_null);
        }
    }

    // -------------------------------------------------------------------------
    // 7. Feedforward task wrench, Coriolis, joint-limit barrier, friction.
    // -------------------------------------------------------------------------
    const Vector7d tau_wrench = J.transpose() * cfg_.target_wrench;

    const std::array<double, 7> c_arr = model.coriolis(s);
    Eigen::Map<const Vector7d> c(c_arr.data());

    Vector7d tau = tau_task + tau_null + c + tau_wrench
                 + joint_limit_repulsion(q);

    if (cfg_.use_friction) tau += friction_compensation(dq);

    // -------------------------------------------------------------------------
    // 8. Return as a libfranka-compatible std::array.
    // -------------------------------------------------------------------------
    std::array<double, 7> out{};
    Eigen::Map<Vector7d>(out.data()) = tau;
    return out;
}
