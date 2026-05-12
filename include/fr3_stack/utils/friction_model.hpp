// Sigmoid-based static-friction model for FR3, identified by the
// FrankaEmikaPandaDynModel project (Cognetti et al.). Same shape CRISP
// uses, refactored to a fixed-size template so RT callers get no heap
// allocation. Returns a torque to ADD to τ_d to compensate friction.
//
// Form (per-joint):
//   f(dq) = fp1 · σ(−fp2·(dq + fp3))  −  fp1 · σ(−fp2·fp3)
// where σ(x) = 1 / (1 + e^-x). The constant offset σ(−fp2·fp3) makes
// f(0) = 0 so we don't inject torque at rest.
//
// Source / param fits: https://github.com/marcocognetti/FrankaEmikaPandaDynModel
// CRISP defaults (FR3): fp1, fp2, fp3 are 7-vectors per joint — see
// others/crisp_controllers/src/torque_feedback_controller.yaml.

#pragma once

#include <Eigen/Dense>
#include <cmath>

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>
get_friction(const Eigen::MatrixBase<Derived>& dq,
             const Eigen::MatrixBase<Derived>& fp1,
             const Eigen::MatrixBase<Derived>& fp2,
             const Eigen::MatrixBase<Derived>& fp3) {
    using Scalar = typename Derived::Scalar;
    using V = Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, 1>;

    V tau(dq.size());
    for (Eigen::Index i = 0; i < dq.size(); ++i) {
        const Scalar e1 = std::exp(-fp2(i) * (dq(i) + fp3(i)));
        const Scalar e2 = std::exp(-fp2(i) * fp3(i));
        tau(i) = fp1(i) / (Scalar(1) + e1) - fp1(i) / (Scalar(1) + e2);
    }
    return tau;
}
