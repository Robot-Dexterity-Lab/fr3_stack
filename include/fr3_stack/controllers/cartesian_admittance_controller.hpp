// Cartesian admittance: a virtual mass-spring-damper at the EE driven by
// F_ext produces a moving "compliant target", which an inner Cartesian
// impedance loop tracks. The user feels a soft push on contact (force →
// motion); the inner loop closes on the *commanded* compliant pose, not
// the user's raw target, so the arm responds smoothly without fighting
// the spring.
//
//   M·a + D·v + K·(inner − target) = F_ext            (outer)
//   τ = J^T·(K·(actual − inner) − D·(J·dq)) + c       (inner, impedance)
//
// F_ext source: WrenchSource (e.g. Bota wrist FT, sensor frame → base via
// R_O_EE) when attached, else libfranka's O_F_ext_hat_K estimate
// (~3-5 N noise floor). LP-filtered before the outer loop because M⁻¹
// amplifies any HF noise into accel jitter.
//
// Wire string remains "admittance" for client compat.

#pragma once

#include <Eigen/Dense>

#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/sensors/bota/wrench_source.hpp>

class CartesianAdmittanceController : public Controller {
 public:
    void set_cfg(const CartesianAdmittanceCfg& c) { cfg_ = c; }
    void set_wrench_source(WrenchSource* src) { wrench_src_ = src; }

    ControllerType        type() const override;
    std::string           name() const override;
    void                  reset(const franka::RobotState& s) override;
    std::array<double, 7> compute(const franka::RobotState& s,
                                  const franka::Model& model) override;

    // Test hooks — exposed for tests/cpp/test_controller_math.cpp to verify
    // F_ext filter / inner integration state. Not used by the daemon path.
    const Vector6d&            F_ext_filt()  const { return F_ext_filt_; }
    const Vector6d&            F_ext_cache() const { return F_ext_cache_; }
    const Eigen::Vector3d&     inner_t()     const { return inner_t_; }
    const Eigen::Quaterniond&  inner_q()     const { return inner_q_; }
    const Vector6d&            inner_v()     const { return inner_v_; }
    const Vector7d&            dq_filt()     const { return dq_filt_; }
    const Vector7d&            tau_filt()    const { return tau_filt_; }

 private:
    CartesianAdmittanceCfg cfg_;
    WrenchSource*          wrench_src_{nullptr};       // not owned
    Vector6d               F_ext_cache_{Vector6d::Zero()};   // pre-filter
    Vector6d               F_ext_filt_{Vector6d::Zero()};    // post-LP
    Eigen::Vector3d        inner_t_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond     inner_q_{Eigen::Quaterniond::Identity()};
    Vector6d               inner_v_{Vector6d::Zero()};
    Eigen::Vector3d        smoothed_t_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond     smoothed_q_{Eigen::Quaterniond::Identity()};
    Vector7d               q_null_snapshot_{Vector7d::Zero()};
    Vector7d               dq_filt_{Vector7d::Zero()};        // post-LP joint vel
    Vector7d               tau_filt_{Vector7d::Zero()};       // post-LP output τ
};
