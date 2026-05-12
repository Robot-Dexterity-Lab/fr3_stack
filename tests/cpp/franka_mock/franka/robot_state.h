// Minimal libfranka mock — just enough surface to compile the
// controllers under test. Real libfranka isn't available on macOS dev
// machines (it's Linux + RT only); this mock fills in for unit tests.
//
// Fields here mirror franka::RobotState's layout. The controllers only
// touch q / dq / O_T_EE / O_F_ext_hat_K / tau_J_d / time, so that's
// what we expose. If a controller starts using a new field, add it
// here with the same type as the real lib.

#pragma once

#include <array>

namespace franka {

struct RobotState {
    std::array<double, 7>  q{};
    std::array<double, 7>  dq{};
    std::array<double, 16> O_T_EE{};        // column-major SE(3)
    std::array<double, 6>  O_F_ext_hat_K{};
    std::array<double, 7>  tau_J_d{};
    struct Time {
        double t{0.0};
        double toSec() const { return t; }
    } time{};
};

}  // namespace franka
