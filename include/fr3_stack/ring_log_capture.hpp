#pragma once

#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/ring_log.hpp>

namespace fr3_stack {

// Same fixed-size capture path used by the daemon and math-mock tests.
// Call on the control thread AFTER dispatch, compute(), and final slew limiting.
// State is the input to that computation; tau_cmd is its outgoing command,
// before downstream libfranka/robot processing (not measured total torque).
inline RingLogFrame capture_ring_log_frame(
    const franka::RobotState& s, const Controller& active,
    const std::array<double, 7>& tau_out, std::uint64_t seq,
    double callback_end_s, double control_period_s) {
    RingLogFrame fr{};
    fr.seq = seq;
    fr.t_s = callback_end_s;
    fr.robot_time_s = s.time.toSec();
    fr.control_period_s = control_period_s;
    for (int i = 0; i < 7; ++i) {
        fr.q[i] = s.q[i];
        fr.dq[i] = s.dq[i];
        fr.tau_J[i] = s.tau_J[i];
        fr.tau_cmd[i] = tau_out[i];
    }
    const Eigen::Affine3d T_ee(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    const Eigen::Quaterniond q_ee(T_ee.rotation());
    for (int i = 0; i < 3; ++i) fr.ee_pos[i] = T_ee.translation()[i];
    fr.ee_quat_xyzw[0] = q_ee.x();
    fr.ee_quat_xyzw[1] = q_ee.y();
    fr.ee_quat_xyzw[2] = q_ee.z();
    fr.ee_quat_xyzw[3] = q_ee.w();

    // Legacy Cartesian columns remain pre-controller-EMA targets.
    Eigen::Affine3d T_tgt;
    if (active.pose_target(T_tgt)) {
        const Eigen::Quaterniond q_tgt(T_tgt.rotation());
        for (int i = 0; i < 3; ++i) fr.target_pos[i] = T_tgt.translation()[i];
        fr.target_quat_xyzw[0] = q_tgt.x();
        fr.target_quat_xyzw[1] = q_tgt.y();
        fr.target_quat_xyzw[2] = q_tgt.z();
        fr.target_quat_xyzw[3] = q_tgt.w();
    }
    JointImpedanceLogState joint;
    if (active.joint_log_state(joint)) {
        fr.joint_target_valid = 1;
        for (int i = 0; i < 7; ++i) {
            fr.q_target[i] = joint.target[i];
            fr.q_target_filtered[i] = joint.filtered_target[i];
            fr.K_joint[i] = joint.K[i];
            fr.D_joint[i] = joint.D[i];
        }
        fr.filter_alpha = joint.filter_alpha;
        fr.joint_use_friction = joint.use_friction ? 1 : 0;
        fr.joint_reset_count = joint.reset_count;
    }
    fr.controller = static_cast<std::uint32_t>(active.type());
    return fr;
}

}  // namespace fr3_stack
