// Mock franka::Model. The controllers call:
//   - zeroJacobian(Frame, RobotState&)  → 6×7 column-major
//   - bodyJacobian(Frame, RobotState&)  → 6×7 column-major (used by some
//     standalone teaching variants)
//   - mass(RobotState&)                  → 7×7 column-major
//   - coriolis(RobotState&)              → 7-vector
//
// The mock lets the test set these directly, then the controllers see
// whatever the test prepared.

#pragma once

#include <array>

#include <franka/robot_state.h>

namespace franka {

// Mirrors libfranka's frame names so wrench_frame.hpp's string→enum lookup
// compiles in the test harness. The mock only needs the enum to exist; the
// FT sensor pathway isn't exercised by the math-mock tests.
enum class Frame {
    kJoint1, kJoint2, kJoint3, kJoint4, kJoint5, kJoint6, kJoint7,
    kFlange, kEndEffector, kStiffness
};

struct Model {
    std::array<double, 42> J_zero{};   // for zeroJacobian
    std::array<double, 42> J_body{};   // for bodyJacobian
    std::array<double, 49> M_inertia{};
    std::array<double, 7>  c_vec{};
    // Mock pose() returns identity (real libfranka returns 4x4 col-major).
    // Tests don't drive a wrench source so this is never inspected.
    std::array<double, 16> T_pose{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1};

    std::array<double, 42> zeroJacobian(Frame, const RobotState&) const { return J_zero; }
    std::array<double, 42> bodyJacobian(Frame, const RobotState&) const { return J_body; }
    std::array<double, 49> mass        (const RobotState&)         const { return M_inertia; }
    std::array<double, 7>  coriolis    (const RobotState&)         const { return c_vec; }
    std::array<double, 16> pose        (Frame, const RobotState&)  const { return T_pose; }
};

}  // namespace franka
