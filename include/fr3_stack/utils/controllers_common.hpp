// Constants, typedefs, and small helpers shared by all FR3 controllers.
// Production-tuned for Franka Research 3 — bakes in FR3 hard limits and
// the joint-limit / friction defaults that have been validated on the
// real arm. For controller-agnostic templated versions of the same math
// (different params per robot, no FR3 constants), see
// fr3_stack/utils/joint_limits.hpp and fr3_stack/utils/friction_model.hpp.
//
// All functions here are RT-safe: fixed-size Eigen, no heap allocation,
// no syscalls. Suitable for direct use inside a 1 kHz libfranka callback.

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

// libfranka rejects step torque jumps as discontinuities; 1 Nm/ms matches
// franka_example_controllers and gives the impedance loops headroom.
constexpr double kDeltaTauMax = 1.0;  // [Nm] per 1 ms

// FR3 hard joint limits (rad). From Franka Research 3 documentation.
constexpr std::array<double, 7> kFr3QMax = {
     2.7437,  1.7837,  2.9007, -0.1518,  2.8065,  4.5169,  3.0159};
constexpr std::array<double, 7> kFr3QMin = {
    -2.7437, -1.7837, -2.9007, -3.0421, -2.8065,  0.5445, -3.0159};

// Soft barrier near each joint limit: starts pushing back inside the last
// 10% of the range, with up to ±10 Nm at the limit itself. Mirrors CRISP's
// get_joint_limit_torque but with FR3-specific constants baked in.
constexpr double kJointLimitMargin = 0.1;
constexpr double kJointLimitMaxTau = 10.0;

// Linear ramp from 0 (at margin edge) to ±kJointLimitMaxTau (at the limit),
// then saturates. Sums into τ before torque-rate limiting; does not fight
// Coriolis / FCI gravity comp.
inline Vector7d joint_limit_repulsion(const Vector7d& q) {
    Vector7d tau = Vector7d::Zero();
    for (int i = 0; i < 7; ++i) {
        const double range  = kFr3QMax[i] - kFr3QMin[i];
        const double margin = kJointLimitMargin * range;
        const double upper  = kFr3QMax[i] - margin;
        const double lower  = kFr3QMin[i] + margin;
        if (q[i] > upper)
            tau[i] = -kJointLimitMaxTau * (q[i] - upper) / margin;
        else if (q[i] < lower)
            tau[i] =  kJointLimitMaxTau * (lower - q[i]) / margin;
    }
    return tau.cwiseMax(-kJointLimitMaxTau).cwiseMin(kJointLimitMaxTau);
}

// Stribeck-like friction comp: τ_fric_robot = -[fp1·tanh(fp2·dq) + fp3·dq]
// (opposes motion). To cancel it we ADD the positive of that to the
// commanded torque. Defaults are conservative for FR3 — tune by editing.
// (Different model from CRISP's sigmoid `get_friction`; both ship in the
// repo, choose per use case.)
inline Vector7d friction_compensation(const Vector7d& dq) {
    static const Vector7d fp1 =
        (Vector7d() << 0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.2).finished();   // Coulomb [Nm]
    static const Vector7d fp2 = Vector7d::Constant(50.0);                // tanh sharpness
    static const Vector7d fp3 = Vector7d::Constant(0.05);                // viscous [Nm·s/rad]
    Vector7d tau;
    for (int i = 0; i < 7; ++i)
        tau[i] = fp1[i] * std::tanh(fp2[i] * dq[i]) + fp3[i] * dq[i];
    return tau;
}

// Per-axis symmetric clip. max_delta[i] == 0 leaves axis i unclipped.
// Used by Cartesian impedance to bound the spring action under a stepped
// pose target (pre-LP-filter spike).
inline Vector6d clip_error(const Vector6d& e, const Vector6d& max_delta) {
    Vector6d out = e;
    for (int i = 0; i < 6; ++i) {
        if (max_delta[i] > 0.0)
            out[i] = std::max(-max_delta[i], std::min(max_delta[i], e[i]));
    }
    return out;
}

// Per-axis soft deadband (shrinkage / soft thresholding):
//
//     y_i = sign(x_i) · max(|x_i| − eps_i, 0)
//
// Used to suppress sub-noise-floor signals on F_ext after the EMA, so DC
// drift from FT calibration residuals does not leak into the admittance
// integrator (M⁻¹·F_ext) or the force-tracking PID. Soft (not hard) so the
// boundary stays continuous: hard cutoff "|x|<eps ⇒ 0" creates a
// discontinuity that can chatter when a real contact force grazes the
// threshold.
//
// eps_i = 0 ⇒ that axis passes through unchanged (disabled per-axis).
// All-zero eps ⇒ no-op for the whole vector.
inline Vector6d soft_deadband(const Vector6d& x, const Vector6d& eps) {
    Vector6d y;
    for (int i = 0; i < 6; ++i) {
        const double e = std::max(eps(i), 0.0);
        const double a = std::abs(x(i));
        y(i) = (a > e) ? std::copysign(a - e, x(i)) : 0.0;
    }
    return y;
}
