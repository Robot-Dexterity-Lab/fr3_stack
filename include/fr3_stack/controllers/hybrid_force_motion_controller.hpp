// Hybrid force/motion controller for FR3 — yifan-hou HFVC inner +
// Cartesian impedance outer. n_af = 0 collapses to pure admittance,
// n_af = 6 to pure force control; in between, the first n_af rows of
// Tr name force-controlled axes and the rest are velocity-controlled.
//
// Refs:
//   Hou & Mason 2019 ICRA — Robust Execution of Contact-Rich Motion Plans
//                           by Hybrid Force-Velocity Control
//   yifan-hou/force_control (others/force_control/) — reference impl
//   CRISP cartesian_hybrid_controller.cpp (ROS2 wrap of same algorithm)
//
// Layout:
//   inner: HFVC integrates inner_t_/inner_q_ (the moving compliant
//          target). Force axes (Sf): spring + force PID + damping +
//          stiction → Newton's law in Tr-space → integrated velocity.
//          Velocity axes (Sv): rigid tracking, velocity from pose error
//          directly (clamped by max_inner_v / max_inner_w).
//   outer: J^T·(K·(inner − actual) − D·(J·dq)) tracks inner pose to τ.
//
// Sign convention: F_ext is the EXTERNAL wrench on the robot. Robot's
// reaction is −F_ext. PID error = (target_wrench − robot_action) =
//   (target_wrench − (−F_ext)) = target_wrench + F_ext.
//
// NB: We deliberately DO NOT add J^T·target_wrench at the outer layer.
// The force command drives the inner FT-PID loop; outer K·drift produces
// the contact force. Adding J^T·F_d here would double-count and bypass
// the spring clip / FT loop / safety limits.
//
// Wire string remains "hybrid" for client compat. Renamed in C++ from
// HybridController to match hybrid_force_motion_controller.hpp filename
// and disambiguate from "hybrid X/Y" in other domains.
//
// Resolved P0s from docs/controller_review.md (vs old HybridController):
//   #4 Tr/n_af switch: inner_v_ now projected through Sf so velocity-axis
//      residual can't bleed into newly-force-controlled axes.
//   #5 Velocity-axis tracking: max_inner_v / max_inner_w clamps prevent
//      a 1mm err / 1ms = 1m/s commanded velocity at the inner level.
//   #6 F_ext now LP-filtered (wrench_filter_alpha), matching admittance,
//      before being fed to the FT-PID and the spring/damping mix.

#pragma once

#include <Eigen/Dense>

#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/sensors/bota/wrench_source.hpp>

class HybridForceMotionController : public Controller {
 public:
    void set_cfg(const HybridForceMotionCfg& c) { cfg_ = c; }
    void set_wrench_source(WrenchSource* src) { wrench_src_ = src; }

    // Per-tick pose-target override used by the dispatcher's streaming LERP
    // (see main.cpp). Other cfg fields and inner/outer filter state are
    // untouched so the bridged target shares the gain/dynamics config
    // installed by the most recent set_cfg().
    void set_target(const Eigen::Affine3d& T) { cfg_.target = T; }

    // Whether the dispatcher should feed cfg_.target through the streaming
    // LERP interpolator before set_target(). Mirrors CartesianImpedance.
    bool linear_interp_enabled() const { return cfg_.linear_interp; }

    ControllerType        type() const override;
    std::string           name() const override;
    void                  reset(const franka::RobotState& s) override;
    std::array<double, 7> compute(const franka::RobotState& s,
                                  const franka::Model& model) override;

 private:
    HybridForceMotionCfg cfg_;
    WrenchSource*        wrench_src_{nullptr};       // not owned
    Vector6d             F_ext_cache_{Vector6d::Zero()};   // pre-filter
    Vector6d             F_ext_filt_{Vector6d::Zero()};    // post-LP, fed to inner
    bool                 F_ext_init_{false};
    Eigen::Vector3d      inner_t_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond   inner_q_{Eigen::Quaterniond::Identity()};
    Vector6d             inner_v_{Vector6d::Zero()};
    Eigen::Vector3d      smoothed_t_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond   smoothed_q_{Eigen::Quaterniond::Identity()};
    Vector7d             q_null_snapshot_{Vector7d::Zero()};

    // HFVC state
    Eigen::Matrix<double, 6, 6> Tr_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> Tr_inv_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> Sf_{Eigen::Matrix<double, 6, 6>::Zero()};
    Eigen::Matrix<double, 6, 6> Sv_{Eigen::Matrix<double, 6, 6>::Identity()};
    Vector6d                    wrench_pid_I_{Vector6d::Zero()};
    Vector6d                    wrench_err_prev_{Vector6d::Zero()};
    int                         prev_n_af_{-1};
    Eigen::Matrix<double, 6, 6> prev_Tr_{Eigen::Matrix<double, 6, 6>::Zero()};

    // Pixi-style smoothing chain.
    Vector7d dq_filt_{Vector7d::Zero()};   // post-EMA joint velocity
    Vector7d tau_filt_{Vector7d::Zero()};  // post-rate-sat + post-EMA τ
    // EMA of inner_v_ for the outer D damping reference. Decouples LERP
    // segment-boundary velocity discontinuities from joint τ.
    Vector6d inner_v_filt_{Vector6d::Zero()};
};
