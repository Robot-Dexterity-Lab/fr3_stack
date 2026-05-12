// Joint-limit repulsion torque: linear ramp inside a `safe_range` band near
// each joint limit, off elsewhere. Summed into τ on every controller so the
// arm pushes itself away from its hard stops before the FCI reflex trips.
//
// Math borrowed from CRISP (learnsyslab/crisp_controllers); refactored to
// be template-on-Eigen-Derived so fixed-size callers (Vector7d) get no
// heap allocation and the per-tick output type matches the input.
//   https://github.com/learnsyslab/crisp_controllers/blob/3c92f5dd7f764eff0551815b9bea36eb2d5f41f7/include/crisp_controllers/utils/joint_limits.hpp

#pragma once

#include <Eigen/Dense>
#include <algorithm>

// Per-joint repulsion torque, length N (one entry per joint).
//
//   d_low   = q − q_min          (≥ 0 when inside the legal range)
//   d_up    = q_max − q
//   ramp_low = clamp((safe_range − d_low) / safe_range, 0, 1)   ┐ 0 at the band
//   ramp_up  = clamp((safe_range − d_up ) / safe_range, 0, 1)   ┘ edge, 1 at limit
//   τ        = +max_torque · ramp_low − max_torque · ramp_up
//
// Sign: positive τ pushes the joint angle UP (away from q_min); negative τ
// pushes it DOWN (away from q_max). The clamp on the ramps means a joint
// already past the limit still produces exactly max_torque, never more.
//
// Defaults (safe_range = 0.3 rad, max_torque = 5 Nm) are conservative for
// FR3 — bump max_torque if a stiffer outer controller can overpower it.
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1>
get_joint_limit_torque(
    const Eigen::MatrixBase<Derived>& joint_positions,
    const Eigen::MatrixBase<Derived>& lower_limits,
    const Eigen::MatrixBase<Derived>& upper_limits,
    double safe_range = 0.3,  // [rad] band width on each side
    double max_torque = 5.0   // [Nm]  saturation magnitude at the limit
) {
    using Scalar = typename Derived::Scalar;
    using V = Eigen::Matrix<Scalar, Derived::RowsAtCompileTime, 1>;

    V torques = V::Zero(joint_positions.size());
    for (Eigen::Index i = 0; i < joint_positions.size(); ++i) {
        const Scalar d_low = joint_positions(i) - lower_limits(i);
        const Scalar d_up  = upper_limits(i) - joint_positions(i);
        if (d_low < safe_range) {
            const Scalar r = std::clamp<Scalar>(
                (safe_range - d_low) / safe_range, Scalar(0), Scalar(1));
            torques(i) += max_torque * r;
        }
        if (d_up < safe_range) {
            const Scalar r = std::clamp<Scalar>(
                (safe_range - d_up) / safe_range, Scalar(0), Scalar(1));
            torques(i) -= max_torque * r;
        }
    }
    return torques;
}
