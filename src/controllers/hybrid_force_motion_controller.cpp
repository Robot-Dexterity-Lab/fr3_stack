#include <fr3_stack/controllers/hybrid_force_motion_controller.hpp>
#include <fr3_stack/sensors/wrench_frame.hpp>
#include <fr3_stack/utils/controllers_common.hpp>
#include <fr3_stack/utils/log.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

ControllerType HybridForceMotionController::type() const {
    return ControllerType::HybridForceMotion;
}

std::string HybridForceMotionController::name() const {
    return "hybrid_force_motion";
}

void HybridForceMotionController::reset(const franka::RobotState& s) {
    Eigen::Affine3d T(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    inner_t_         = T.translation();
    inner_q_         = Eigen::Quaterniond(T.linear());
    inner_v_         = Vector6d::Zero();
    smoothed_t_      = inner_t_;
    smoothed_q_      = inner_q_;
    q_null_snapshot_ = Eigen::Map<const Vector7d>(s.q.data());
    F_ext_cache_     = Vector6d::Zero();
    F_ext_filt_      = Vector6d::Zero();
    F_ext_init_      = false;
    // Arm is stationary at activation; start dq filter and τ history at zero
    // so the first ticks ramp up cleanly through the EMA + rate-sat chain.
    dq_filt_         = Vector7d::Zero();
    tau_filt_        = Vector7d::Zero();
    inner_v_filt_    = Vector6d::Zero();

    wrench_pid_I_.setZero();
    wrench_err_prev_.setZero();

    // Force Tr/Sf/Sv recompute on first compute().
    prev_n_af_ = -1;
    prev_Tr_.setZero();

    if (wrench_src_) {
        std::cerr << log_pfx() << "hybrid: using FT sensor '"
                  << wrench_src_->kind()
                  << "' as F_ext (mount='" << wrench_src_->mount_frame_name()
                  << "', rotated sensor→base via R_base_sensor).\n";
    } else {
        std::cerr << log_pfx() << "WARNING: hybrid is using libfranka's "
                     "O_F_ext_hat_K (~3-5 N noise floor) as F_ext — "
                     "no FT sensor attached. Pass --ft-sensor-kind for "
                     "precision force control.\n";
    }
    std::cerr << log_pfx() << "hybrid: n_af=" << cfg_.n_af << "\n";
}

std::array<double, 7> HybridForceMotionController::compute(
    const franka::RobotState& s, const franka::Model& model) {
    Eigen::Map<const Vector7d> q     (s.q.data());
    Eigen::Map<const Vector7d> dq_raw(s.dq.data());

    // EMA on raw dq before it enters the outer D-term, nullspace damping, and
    // friction comp. 1.0 = pass-through (legacy); pixi default 0.5 softens the
    // D response at the cost of a few ms phase lag.
    const double a_dq = std::clamp(cfg_.dq_filter_alpha, 0.0, 1.0);
    dq_filt_ = a_dq * dq_raw + (1.0 - a_dq) * dq_filt_;
    const Vector7d& dq = dq_filt_;

    Eigen::Affine3d T_ee(Eigen::Matrix4d::Map(s.O_T_EE.data()));

    if (wrench_src_) {
        Vector6d F_sensor;
        if (wrench_src_->read(F_sensor)) {
            // R_base_sensor pulls the mount frame from the sensor itself
            // (bota → "flange", ATI → whatever its config says), so this
            // controller never has to know where the wrench source sits on
            // the robot. See sensors/wrench_frame.hpp.
            const Eigen::Matrix3d R = R_base_sensor(*wrench_src_, s, model);
            F_ext_cache_.head<3>() = R * F_sensor.head<3>();
            F_ext_cache_.tail<3>() = R * F_sensor.tail<3>();
        }
    } else {
        F_ext_cache_ = Eigen::Map<const Vector6d>(s.O_F_ext_hat_K.data());
    }

    // P0 #6 fix: EMA F_ext before feeding PID + spring/damping mix.
    if (!F_ext_init_) { F_ext_filt_ = F_ext_cache_; F_ext_init_ = true; }
    const double a_w = std::clamp(cfg_.wrench_filter_alpha, 0.0, 1.0);
    F_ext_filt_ = a_w * F_ext_cache_ + (1.0 - a_w) * F_ext_filt_;
    // Per-axis soft deadband (after EMA, before any downstream use). Zero
    // eps ⇒ pass-through; non-zero ⇒ shrinks sub-eps drift to zero so it
    // never leaks into M⁻¹·F_ext or the PID integrator.
    const Vector6d F_ext = soft_deadband(F_ext_filt_, cfg_.wrench_deadband);

    // LP-smooth user target.
    smoothed_t_ = (1 - cfg_.filter_alpha) * smoothed_t_
                + cfg_.filter_alpha * cfg_.target.translation();
    Eigen::Quaterniond q_d_target(cfg_.target.linear());
    smoothed_q_ = smoothed_q_.slerp(cfg_.filter_alpha, q_d_target);

    // Recompute Sf/Sv/Tr_inv if Tr or n_af changed; reset PID state and
    // project inner_v_ through Sf (P0 #4 fix).
    const int n_af = std::clamp(cfg_.n_af, 0, 6);
    const bool changed =
        (n_af != prev_n_af_) || ((cfg_.Tr - prev_Tr_).norm() > 1e-12);
    if (changed) {
        Sf_.setZero();  Sv_.setZero();
        for (int i = 0; i < 6; ++i) {
            if (i < n_af) Sf_(i, i) = 1.0; else Sv_(i, i) = 1.0;
        }
        if (std::abs(cfg_.Tr.determinant()) < 1e-6) {
            static LogThrottle singular_tr_throttle;
            singular_tr_throttle.maybe_log(
                std::cerr, "hybrid: singular Tr (det≈0), using I");
            Tr_     = Eigen::Matrix<double, 6, 6>::Identity();
            Tr_inv_ = Tr_;
        } else {
            Tr_     = cfg_.Tr;
            Tr_inv_ = Tr_.inverse();
        }
        // Velocity-axis residual must not bleed into newly-force-controlled
        // axes after a switch. yifan-hou achieves the same end via
        // SE3_TrefTadj cleanup; we use the simpler selection projection.
        inner_v_ = Tr_inv_ * (Sf_ * (Tr_ * inner_v_));
        wrench_pid_I_.setZero();
        wrench_err_prev_.setZero();
        prev_n_af_ = n_af;
        prev_Tr_   = cfg_.Tr;
    }

    Vector6d adm_err;
    adm_err.head<3>() = smoothed_t_ - inner_t_;
    Eigen::Quaterniond inner_aligned = inner_q_;
    if (smoothed_q_.coeffs().dot(inner_aligned.coeffs()) < 0.0)
        inner_aligned.coeffs() = -inner_aligned.coeffs();
    Eigen::Quaterniond dR = smoothed_q_ * inner_aligned.inverse();
    adm_err.tail<3>() = 2.0 * dR.vec();

    constexpr double kDt = 0.001;

    if (n_af == 0) {
        // Pure admittance: M·a = F_ext − D·v + K·err
        Vector6d adm_force = F_ext
                           - cfg_.D_adm.asDiagonal() * inner_v_
                           + cfg_.K_adm.asDiagonal() * adm_err;
        Vector6d a = cfg_.M_adm.cwiseInverse().asDiagonal() * adm_force;
        inner_v_ += a * kDt;
    } else {
        // HFVC: spring + force PID + damping on force axes; rigid tracking
        // (clamped) on velocity axes.
        Vector6d wrench_spring = cfg_.K_adm.asDiagonal() * adm_err;
        if (cfg_.max_spring_force > 0) {
            double n = wrench_spring.head<3>().norm();
            if (n > cfg_.max_spring_force)
                wrench_spring.head<3>() *= cfg_.max_spring_force / n;
        }
        if (cfg_.max_spring_torque > 0) {
            double n = wrench_spring.tail<3>().norm();
            if (n > cfg_.max_spring_torque)
                wrench_spring.tail<3>() *= cfg_.max_spring_torque / n;
        }

        Vector6d wrench_cmd_world = Tr_inv_ * cfg_.target_wrench_Tr;
        Vector6d wrench_err = wrench_cmd_world + F_ext;

        wrench_pid_I_ += wrench_err;
        wrench_pid_I_ = wrench_pid_I_
                            .cwiseMax(-cfg_.I_limit)
                            .cwiseMin( cfg_.I_limit);

        Vector6d wrench_PID;
        wrench_PID.head<3>() =
            cfg_.P_trans * wrench_err.head<3>() +
            cfg_.I_trans * wrench_pid_I_.head<3>() +
            cfg_.D_trans * (wrench_err.head<3>() -
                            wrench_err_prev_.head<3>());
        wrench_PID.tail<3>() =
            cfg_.P_rot * wrench_err.tail<3>() +
            cfg_.I_rot * wrench_pid_I_.tail<3>() +
            cfg_.D_rot * (wrench_err.tail<3>() -
                          wrench_err_prev_.tail<3>());
        wrench_err_prev_ = wrench_err;

        Vector6d wrench_spring_Tr = Tr_ * wrench_spring;
        Vector6d wrench_err_Tr    = Tr_ * wrench_err;
        Vector6d wrench_PID_Tr    = Tr_ * wrench_PID;

        for (int i = 0; i < 6; ++i) {
            if (std::abs(wrench_err_Tr(i)) < cfg_.stiction(i))
                wrench_err_Tr(i) = 0.0;
        }

        Vector6d wrench_damp_Tr =
            -Tr_ * (cfg_.D_adm.asDiagonal() * inner_v_);

        Vector6d wrench_all_Tr = Sf_ * (wrench_spring_Tr + wrench_err_Tr +
                                        wrench_PID_Tr   + wrench_damp_Tr);

        Eigen::Matrix<double, 6, 6> M_Tr =
            Tr_ * cfg_.M_adm.asDiagonal() * Tr_inv_;
        Vector6d accel_Tr = M_Tr.fullPivLu().solve(wrench_all_Tr);

        Vector6d motion_Tr = Tr_ * inner_v_;
        motion_Tr += accel_Tr * kDt;
        motion_Tr = Sf_ * motion_Tr;

        // P0 #5 fix: clamp velocity-axis tracking velocity.
        Vector6d v_track_Tr = Tr_ * (adm_err / kDt);
        {
            Eigen::Vector3d v_lin = v_track_Tr.head<3>();
            Eigen::Vector3d v_ang = v_track_Tr.tail<3>();
            const double n_lin = v_lin.norm(), n_ang = v_ang.norm();
            if (cfg_.max_inner_v > 0 && n_lin > cfg_.max_inner_v)
                v_lin *= cfg_.max_inner_v / n_lin;
            if (cfg_.max_inner_w > 0 && n_ang > cfg_.max_inner_w)
                v_ang *= cfg_.max_inner_w / n_ang;
            v_track_Tr.head<3>() = v_lin;
            v_track_Tr.tail<3>() = v_ang;
        }
        motion_Tr += Sv_ * v_track_Tr;

        inner_v_ = Tr_inv_ * motion_Tr;
    }

    // Integrate inner pose (semi-implicit Euler, 1 ms).
    inner_t_ += inner_v_.head<3>() * kDt;
    Eigen::Vector3d w = inner_v_.tail<3>();
    const double wn = w.norm();
    if (wn > 1e-9) {
        Eigen::Quaterniond dq_rot(Eigen::AngleAxisd(wn * kDt, w / wn));
        inner_q_ = (dq_rot * inner_q_).normalized();
    }

    // Outer Cartesian-impedance loop (tracks inner pose).
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
    // inner_SE3_ flung far by the inner admittance can't slam the joints.
    // Per-axis zero = no clip on that axis.
    for (int i = 0; i < 6; ++i) {
        if (cfg_.error_clip[i] > 0.0)
            e[i] = std::clamp(e[i], -cfg_.error_clip[i], cfg_.error_clip[i]);
    }

    std::array<double, 42> J_arr =
        model.zeroJacobian(franka::Frame::kEndEffector, s);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(J_arr.data());
    Vector6d v = J * dq;

    // Damp the *error* velocity, not the absolute EE velocity. The outer
    // target is inner_t/inner_q which moves at inner_v_, so the correct
    // damping reference is (v_actual − v_inner). Using v_actual alone makes
    // outer D fight the legitimate tracking motion whenever inner is moving
    // (e.g. under F_ext drive) — introduces a phase lag that the
    // under-damped outer (ωₙ≈24 rad/s, ζ≈0.45) cannot absorb.
    //
    // inner_v_ is recomputed each tick from (smoothed_t − inner_t)/kDt for
    // velocity axes — a 1-tick lag P-tracker with no smoothing — which
    // means it inherits the LERP'd target's velocity discontinuities at
    // segment boundaries (~100 Hz under client jitter). Feeding that raw
    // into D produces an audible 100 Hz buzz. LP the inner_v reference
    // for the damping term to decouple it; the position integration above
    // still uses the raw inner_v_ so tracking accuracy is unchanged.
    // inner_v_ is in world frame, same as J·dq, so subtraction is direct.
    const double a_iv = std::clamp(cfg_.inner_v_filter_alpha, 0.0, 1.0);
    inner_v_filt_ = a_iv * inner_v_ + (1.0 - a_iv) * inner_v_filt_;
    Vector6d F_imp = cfg_.K.asDiagonal() * (-e)
                   - cfg_.D.asDiagonal() * (v - inner_v_filt_);
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

    if (!tau.allFinite()) {
        static LogThrottle non_finite_throttle;
        non_finite_throttle.maybe_log(
            std::cerr, "hybrid: non-finite τ, sending zero");
        tau.setZero();
    }

    // Pixi-style torque-rate saturation: clip per-tick |τ − τ_filt_| ≤
    // max_delta_tau (Nm). At 1 kHz, max_delta_tau=0.5 → 500 Nm/s slew cap.
    // Bounds impulsive K·e kicks before they reach the joints.
    if (cfg_.max_delta_tau > 0.0) {
        const double cap = cfg_.max_delta_tau;
        for (int i = 0; i < 7; ++i) {
            const double dtau = std::clamp(tau[i] - tau_filt_[i], -cap, cap);
            tau[i] = tau_filt_[i] + dtau;
        }
    }

    // Output τ EMA. α=1 ⇒ pass-through; pixi default 0.5.
    const double a_t = std::clamp(cfg_.output_torque_filter_alpha, 0.0, 1.0);
    tau_filt_ = a_t * tau + (1.0 - a_t) * tau_filt_;

    std::array<double, 7> out{};
    for (int i = 0; i < 7; ++i) out[i] = tau_filt_[i];
    return out;
}
