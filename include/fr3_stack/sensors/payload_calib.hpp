// Payload calibration for FT sensors — mass, center-of-mass, and constant
// bias estimated by the offline tool `fr3-ft-calibrate`. Used by
// CompensatedWrenchSource to subtract gravity + bias before downstream
// consumers (admittance / hybrid controllers, state publisher) see the
// wrench.
//
// File format mirrors what fr3_stack/sensors/bota/_common.py:save_yaml writes:
//
//     mass:           0.350
//     center_of_mass: [0.001, -0.002, 0.025]
//     force_bias:     [0.10, -0.20, 0.05]
//     torque_bias:    [0.001, -0.002, 0.0001]
//     # rotation from libfranka's EE frame to the bota sensor frame.
//     # default identity. R_z(+π/4) on rigs where Desk has the standard
//     # hand frame configured (libfranka's O_T_EE then carries -45° about z
//     # vs the actual flange/sensor orientation). See
//     # docs/postmortems/ft_calibration_2026-05-10.md.
//     rpy_ee_sensor:  [0.0, 0.0, 0.7853981633974483]
//     # optional residuals — purely informational, daemon prints them at boot
//     mean_force_residual_N:    0.12
//     mean_torque_residual_Nm:  0.003

#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>

struct PayloadCalib {
    double mass = 0.0;                         // kg
    Eigen::Vector3d com = Eigen::Vector3d::Zero();   // m, sensor frame
    Eigen::Vector3d f_bias = Eigen::Vector3d::Zero();// N
    Eigen::Vector3d t_bias = Eigen::Vector3d::Zero();// Nm
    // Constant rotation from libfranka's EE frame to the bota sensor frame.
    // Defaults to identity for back-compat with old yamls. Apply consistently
    // wherever sensor wrench ↔ base rotates: R_O_sensor = R_O_EE · R_ee_sensor.
    Eigen::Matrix3d R_ee_sensor = Eigen::Matrix3d::Identity();
    // Optional fit-quality numbers from the solver. -1 ⇒ not present in YAML.
    double residual_force_N   = -1.0;
    double residual_torque_Nm = -1.0;
};

// Load a calibration YAML. Returns std::nullopt if the file does not exist
// (treated as "no calibration, run uncompensated"). Throws std::runtime_error
// for parse / schema errors so a malformed file fails loud rather than
// silently disabling compensation.
std::optional<PayloadCalib> load_payload_calib(const std::string& path);

// Default lookup order (first existing match wins; "" if none found):
//   1. $FR3_FT_CALIB explicit file path
//   2. $FR3_FT_CALIB_DIR/ft_calibration.yaml
//   3. /opt/fr3-stack/calib/ft_calibration.yaml  (docker-compose mount target)
//
// docker-compose.yml bind-mounts the host's
// `<repo>/fr3_stack/sensors/bota/config/` to /opt/fr3-stack/calib/, so the
// daemon and the Python tooling agree on a single source of truth. Out-of-
// Docker C++ runs should pass --ft-calib explicitly.
std::string default_calib_path();
