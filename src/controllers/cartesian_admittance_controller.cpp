#include <fr3_stack/controllers/cartesian_admittance_controller.hpp>
#include <fr3_stack/sensors/wrench_frame.hpp>
#include <fr3_stack/utils/controllers_common.hpp>
#include <fr3_stack/utils/log.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

ControllerType CartesianAdmittanceController::type() const {
    return ControllerType::CartesianAdmittance;
}

std::string CartesianAdmittanceController::name() const {
    return "cartesian_admittance";
}

void CartesianAdmittanceController::reset(const franka::RobotState& s) {
    Eigen::Affine3d T(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    inner_t_         = T.translation();
    inner_q_         = Eigen::Quaterniond(T.linear());
    inner_v_         = Vector6d::Zero();
    smoothed_t_      = inner_t_;
    smoothed_q_      = inner_q_;
    q_null_snapshot_ = Eigen::Map<const Vector7d>(s.q.data());
    F_ext_cache_     = Vector6d::Zero();
    F_ext_filt_      = Vector6d::Zero();
    // Seed dq_filt_ at zero (the arm is stationary at activation), and
    // tau_filt_ at zero so the first tick starts smoothly from no command.
    dq_filt_         = Vector7d::Zero();
    tau_filt_        = Vector7d::Zero();

    if (wrench_src_) {
        std::cerr << log_pfx() << "admittance: using FT sensor '"
                  << wrench_src_->kind()
                  << "' as F_ext (mount='" << wrench_src_->mount_frame_name()
                  << "', rotated sensor→base via R_base_sensor).\n";
    } else {
        std::cerr << log_pfx() << "WARNING: admittance is using libfranka's "
                     "O_F_ext_hat_K (joint-torque-derived estimate) as "
                     "the external wrench input — no F/T sensor attached. "
                     "Sensitivity is limited (~3-5 N noise floor) and the "
                     "estimate biases with the load. For precision force "
                     "control pass --ft-sensor-kind <kind> "
                     "--ft-sensor-config <path>.\n";
    }
}

std::array<double, 7> CartesianAdmittanceController::compute(
    const franka::RobotState& s, const franka::Model& model) {
    Eigen::Map<const Vector7d> q     (s.q.data());
    Eigen::Map<const Vector7d> dq_raw(s.dq.data());

    // Optional EMA low-pass on dq before it enters any damping term.
    // Softens the D-term response at the cost of a few ms of phase lag.
    // α=1.0 = pass-through (legacy fr3_stack behavior, snappier but noisier).
    const double a_dq = std::clamp(cfg_.dq_filter_alpha, 0.0, 1.0);
    dq_filt_ = a_dq * dq_raw + (1.0 - a_dq) * dq_filt_;
    const Vector7d& dq = dq_filt_;

    Eigen::Affine3d T_ee(Eigen::Matrix4d::Map(s.O_T_EE.data()));

    // External wrench source: real FT sensor when available (sensor frame
    // → base via R_base_sensor, which pulls the mount frame from the sensor
    // itself — bota declares "flange", ATI will declare its own — so this
    // controller never hardcodes a libfranka frame). See
    // sensors/wrench_frame.hpp. F_ext_cache_ holds the last good value so a
    // try_lock contention or a not-yet-ready sensor stream just reuses the
    // previous tick (no zero-spike).
    if (wrench_src_) {
        Vector6d F_sensor;
        if (wrench_src_->read(F_sensor)) {
            const Eigen::Matrix3d R = R_base_sensor(*wrench_src_, s, model);
            F_ext_cache_.head<3>() = R * F_sensor.head<3>();
            F_ext_cache_.tail<3>() = R * F_sensor.tail<3>();
        }
    } else {
        F_ext_cache_ = Eigen::Map<const Vector6d>(s.O_F_ext_hat_K.data());
    }

    // EMA low-pass on F_ext before the outer loop. The outer loop applies
    // M⁻¹ to F_ext, so HF noise gets amplified into accel jitter — even a
    // calibrated FT sensor needs this for stable admittance. α=0 disables
    // (filter freezes), α=1 is pass-through.
    const double a_w = std::clamp(cfg_.wrench_filter_alpha, 0.0, 1.0);
    F_ext_filt_ = a_w * F_ext_cache_ + (1.0 - a_w) * F_ext_filt_;
    const Vector6d& F_ext = F_ext_filt_;

    // Smooth the user's commanded target (LP, like cart_imp).
    smoothed_t_ = (1 - cfg_.filter_alpha) * smoothed_t_
                + cfg_.filter_alpha * cfg_.target.translation();
    Eigen::Quaterniond q_d_target(cfg_.target.linear());
    smoothed_q_ = smoothed_q_.slerp(cfg_.filter_alpha, q_d_target);

    // Outer admittance loop:
    //   M·a + D·v + K·(inner − target) = F_ext
    // → a = M⁻¹·(F_ext − D·v + K·(target − inner))
    Vector6d adm_err;
    adm_err.head<3>() = smoothed_t_ - inner_t_;
    Eigen::Quaterniond inner_aligned = inner_q_;
    if (smoothed_q_.coeffs().dot(inner_aligned.coeffs()) < 0.0)
        inner_aligned.coeffs() = -inner_aligned.coeffs();
    Eigen::Quaterniond dR = smoothed_q_ * inner_aligned.inverse();
    adm_err.tail<3>() = 2.0 * dR.vec();   // small-angle log map

    Vector6d adm_force = F_ext
                       - cfg_.D_adm.asDiagonal() * inner_v_
                       + cfg_.K_adm.asDiagonal() * adm_err;
    Vector6d a = cfg_.M_adm.cwiseInverse().asDiagonal() * adm_force;

    // Semi-implicit Euler integration at 1 ms.
    constexpr double kDt = 0.001;
    inner_v_ += a * kDt;
    inner_t_ += inner_v_.head<3>() * kDt;
    Eigen::Vector3d w = inner_v_.tail<3>();
    double wn = w.norm();
    if (wn > 1e-9) {
        Eigen::Quaterniond dq_rot(Eigen::AngleAxisd(wn * kDt, w / wn));
        inner_q_ = (dq_rot * inner_q_).normalized();
    }

    // Inner impedance loop: same math as CartesianImpedanceController,
    // but the target is inner_t_/inner_q_ (the compliant trajectory).
    Eigen::Vector3d    p = T_ee.translation();
    Eigen::Quaterniond R(T_ee.linear());

    Vector6d e;
    e.head<3>() = p - inner_t_;
    Eigen::Quaterniond R_aligned = R;
    if (inner_q_.coeffs().dot(R_aligned.coeffs()) < 0.0)
        R_aligned.coeffs() = -R_aligned.coeffs();
    Eigen::Quaterniond dRe = R_aligned.inverse() * inner_q_;
    e.tail<3>() = -T_ee.linear() * dRe.vec();

    // Pixi-style outer pose-error clip: caps |K·e| at K·error_clip so an
    // inner_SE3_ flung far by the outer admittance can't slam the joints.
    // Per-axis zero = no clip on that axis.
    for (int i = 0; i < 6; ++i) {
        if (cfg_.error_clip[i] > 0.0)
            e[i] = std::clamp(e[i], -cfg_.error_clip[i], cfg_.error_clip[i]);
    }

    std::array<double, 42> J_arr =
        model.zeroJacobian(franka::Frame::kEndEffector, s);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(J_arr.data());
    Vector6d v = J * dq;

    // Damp the *error* velocity, not the absolute EE velocity. The inner
    // impedance tracks inner_t/inner_q which moves at inner_v_, so damping
    // must reference (v_actual − v_inner). Using v_actual alone leaves
    // outer D fighting inner_v whenever F_ext is driving the admittance,
    // costing phase margin and producing a "phantom mass" feel. inner_v_
    // is world-frame, same as J·dq.
    Vector6d F_imp = cfg_.K.asDiagonal() * (-e)
                   - cfg_.D.asDiagonal() * (v - inner_v_);
    Vector7d tau_task = J.transpose() * F_imp;

    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += 0.05 * 0.05;
    Eigen::Matrix<double, 6, 7> JT_pinv = JJt.ldlt().solve(J);
    Eigen::Matrix<double, 7, 7> N =
        Eigen::Matrix<double, 7, 7>::Identity() - J.transpose() * JT_pinv;
    const Vector7d q_null_eff =
        cfg_.q_null.isZero() ? q_null_snapshot_ : cfg_.q_null;
    const double d_null_eff =
        (cfg_.D_null > 0.0) ? cfg_.D_null : 2.0 * std::sqrt(cfg_.K_null);
    Vector7d tau_null = N * (cfg_.K_null * (q_null_eff - q) - d_null_eff * dq);
    if (cfg_.max_tau_null > 0.0) {
        for (int i = 0; i < 7; ++i) {
            tau_null[i] = std::clamp(tau_null[i],
                                     -cfg_.max_tau_null,
                                      cfg_.max_tau_null);
        }
    }

    std::array<double, 7> c_arr = model.coriolis(s);
    Eigen::Map<const Vector7d> c(c_arr.data());
    Vector7d tau = tau_task + tau_null + c + joint_limit_repulsion(q);
    if (cfg_.use_friction) tau += friction_compensation(dq);

    // Pixi-style torque-rate saturation: clip per-tick |τ − τ_filt_| ≤
    // max_delta_tau (Nm). At 1 kHz, max_delta_tau=0.5 → 500 Nm/s slew cap.
    // Bounds impulsive K·e kicks before they reach the joints. 0 = disabled.
    if (cfg_.max_delta_tau > 0.0) {
        const double cap = cfg_.max_delta_tau;
        for (int i = 0; i < 7; ++i) {
            const double dtau = std::clamp(tau[i] - tau_filt_[i], -cap, cap);
            tau[i] = tau_filt_[i] + dtau;
        }
    }

    // Optional EMA low-pass on the final commanded τ. Smooths visible joint
    // motion at the cost of a few ms of phase lag. α=1.0 = pass-through.
    const double a_t = std::clamp(cfg_.output_torque_filter_alpha, 0.0, 1.0);
    tau_filt_ = a_t * tau + (1.0 - a_t) * tau_filt_;

    std::array<double, 7> out{};
    for (int i = 0; i < 7; ++i) out[i] = tau_filt_[i];
    return out;
}
