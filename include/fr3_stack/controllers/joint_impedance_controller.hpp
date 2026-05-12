// Joint-space PD with optional friction comp. Defaults are hand-tuned per
// joint (NOT 2·√K critical damping — that assumes unit inertia, which is
// false on a real arm). Same K/D values as franka_example_controllers'
// joint_impedance_example_controller.

#pragma once

#include <fr3_stack/controllers/controller_base.hpp>

class JointImpedanceController : public Controller {
 public:
    void set_cfg(const JointImpedanceCfg& c) { cfg_ = c; }

    ControllerType        type() const override;
    std::string           name() const override;
    void                  reset(const franka::RobotState& s) override;
    std::array<double, 7> compute(const franka::RobotState& s,
                                  const franka::Model& model) override;

 private:
    JointImpedanceCfg cfg_;
    Vector7d          smoothed_q_{Vector7d::Zero()};
};
