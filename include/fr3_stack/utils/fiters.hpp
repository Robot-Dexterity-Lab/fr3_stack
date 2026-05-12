// Lightweight RT-safe filters / Lie-algebra helpers used by the standalone
// controllers. Header-only, no allocations, no heap. Filename mirrors CRISP's
// `crisp_controllers/utils/fiters.hpp` (their typo, kept for grep-ability).

#pragma once

#include <Eigen/Dense>

// First-order EMA: smoothed[k+1] = (1 − α)·smoothed[k] + α·target.
// α ∈ [0, 1]; α = 0 freezes the filter, α = 1 passes through.
inline double ema(double smoothed, double target, double alpha) {
    return (1.0 - alpha) * smoothed + alpha * target;
}

inline Eigen::Vector3d ema(const Eigen::Vector3d& smoothed,
                           const Eigen::Vector3d& target,
                           double alpha) {
    return (1.0 - alpha) * smoothed + alpha * target;
}

// Slerp toward `target` by α, with a hemisphere flip so the path on the
// 4-sphere is always the short one. Without the flip, a target on the
// opposite hemisphere causes a 360° spin instead of a small correction.
inline Eigen::Quaterniond slerp_quat(const Eigen::Quaterniond& smoothed,
                                     const Eigen::Quaterniond& target,
                                     double alpha) {
    Eigen::Quaterniond t = target;
    if (smoothed.coeffs().dot(t.coeffs()) < 0.0) t.coeffs() = -t.coeffs();
    return smoothed.slerp(alpha, t);
}

// SO(3) logarithm: returns axis · angle ∈ ℝ³, with angle ∈ [0, π].
// Equivalent to `pinocchio::log3(R)`. Eigen's AngleAxisd extracts the
// (axis, angle) pair from R; the product is the so(3) Lie-algebra element.
// Stable across the full rotation range — use this instead of the
// quaternion-vec small-angle approximation for pose errors.
inline Eigen::Vector3d log3(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}
