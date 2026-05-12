// Helper that resolves a WrenchSource's mount-frame metadata to the live
// rotation R_base_sensor at the current robot state.
//
// Lives in a separate header so the WrenchSource base interface
// (sensors/bota/wrench_source.hpp) stays libfranka-free — tools like
// fr3-ft (src/bin/ft_dump.cpp) explicitly build without libfranka.
//
// Usage in a controller's compute(const RobotState& s, const Model& model):
//
//     Vector6d F_sensor;
//     if (wrench_src_->read(F_sensor)) {
//         const Eigen::Matrix3d R = R_base_sensor(*wrench_src_, s, model);
//         F_world.head<3>() = R * F_sensor.head<3>();
//         F_world.tail<3>() = R * F_sensor.tail<3>();
//     }

#pragma once

#include <fr3_stack/sensors/bota/wrench_source.hpp>

#include <franka/model.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>
#include <stdexcept>
#include <string_view>

inline franka::Frame parse_franka_frame(std::string_view name) {
    if (name == "flange")        return franka::Frame::kFlange;
    if (name == "end_effector")  return franka::Frame::kEndEffector;
    if (name == "stiffness")     return franka::Frame::kStiffness;
    if (name == "joint1")        return franka::Frame::kJoint1;
    if (name == "joint2")        return franka::Frame::kJoint2;
    if (name == "joint3")        return franka::Frame::kJoint3;
    if (name == "joint4")        return franka::Frame::kJoint4;
    if (name == "joint5")        return franka::Frame::kJoint5;
    if (name == "joint6")        return franka::Frame::kJoint6;
    if (name == "joint7")        return franka::Frame::kJoint7;
    throw std::runtime_error(
        std::string("parse_franka_frame: unknown frame name '") +
        std::string(name) +
        "' — allowed: flange, end_effector, stiffness, joint1..joint7");
}

// Returns the rotation from the sensor body frame to the base frame at the
// current robot state. Caller multiplies this by the sensor-frame wrench to
// get the base-frame wrench (separately for the force and torque halves —
// no moment-arm correction; we transport about the sensor's origin).
inline Eigen::Matrix3d R_base_sensor(const WrenchSource& src,
                                     const franka::RobotState& s,
                                     const franka::Model& model) {
    const franka::Frame f = parse_franka_frame(src.mount_frame_name());
    const std::array<double, 16> p = model.pose(f, s);
    const Eigen::Matrix3d R_O_mount =
        Eigen::Matrix4d::Map(p.data()).block<3, 3>(0, 0);
    return R_O_mount * src.R_mount_sensor();
}
