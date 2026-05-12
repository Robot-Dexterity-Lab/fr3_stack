#include <fr3_stack/controllers/joint_impedance_controller.hpp>
#include <fr3_stack/utils/controllers_common.hpp>

ControllerType JointImpedanceController::type() const {
    return ControllerType::JointImpedance;
}

std::string JointImpedanceController::name() const {
    return "joint_impedance";
}

void JointImpedanceController::reset(const franka::RobotState& s) {
    smoothed_q_ = Eigen::Map<const Vector7d>(s.q.data());
}

std::array<double, 7> JointImpedanceController::compute(
    const franka::RobotState& s, const franka::Model& model) {
    Eigen::Map<const Vector7d> q (s.q.data());
    Eigen::Map<const Vector7d> dq(s.dq.data());
    smoothed_q_ = (1 - cfg_.filter_alpha) * smoothed_q_
                + cfg_.filter_alpha * cfg_.q_target;
    Vector7d tau = cfg_.K.asDiagonal() * (smoothed_q_ - q)
                 - cfg_.D.asDiagonal() * dq;
    std::array<double, 7> c_arr = model.coriolis(s);
    Eigen::Map<const Vector7d> c(c_arr.data());
    tau += c + joint_limit_repulsion(q);
    if (cfg_.use_friction) tau += friction_compensation(dq);
    std::array<double, 7> out{};
    for (int i = 0; i < 7; ++i) out[i] = tau[i];
    return out;
}
