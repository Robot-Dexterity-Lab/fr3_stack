// Hand-guidable mode for kinesthetic teaching (was IdleController, then
// pure zero-torque). FCI adds gravity internally, so the arm hangs
// balanced; on top of that we add three optional layers:
//
//   1. Inertia-aware per-joint damping: τ_damp[i] = -d_rate[i] · (M(q)·dq)[i]
//      Mass-weighting gives each joint a velocity-decay time constant of
//      1 / d_rate[i] regardless of pose. d_rate is per-joint so heavy
//      proximal joints (J2, J4) can be damped harder than wrists without
//      making the whole arm feel sticky — same idea as franka_ros2's
//      per-joint d_gains in joint_impedance_example_controller.yaml.
//
//   2. Friction compensation (use_friction): Cognetti / FrankaEmikaPandaDynModel
//      sigmoid model that ADDS torque in the direction of motion to cancel
//      each joint's natural mechanical friction. This is what makes pixi /
//      crisp / franka_ros2 gravity_comp feel butter-smooth — without it J2
//      and J4 feel notably heavier than wrists because they have ~2-4× the
//      Coulomb friction of J6/J7 (see fp1 in friction_model.hpp).
//
//   3. Soft velocity wall:  τ_wall = -sign(dq)·tau_wall_max · f
//      f ramps 0→1 as |dq[i]| goes from `v_warn_frac · dq_max` to
//      `v_clip_frac · dq_max` (FR3 hard joint velocity limit). Software
//      backstop that keeps `dq` clear of FCI's `joint_velocity_violation`
//      reflex even when the operator pushes hard or releases mid-swing.
//
// Why all this: pure zero-torque on libfranka + FR3 trips
// `joint_velocity_violation` within seconds of any hand-guiding push,
// because nothing dissipates the kinetic energy the operator injects.
// franka_ros2 also commands zero torque but its hardware interface adds
// its own LP filter that `libfranka::robot.control(callback)` does not.
// We get the same softness via friction comp + just enough damping.
//
// Wire string remains "idle" for client compatibility — see
// proto/fr3.capnp and the dispatcher in main.cpp.

#pragma once

#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/utils/controllers_common.hpp>

// Per-joint damping rate [1/s]. Time constant per joint i is 1 / d_rate[i].
// Default biases higher on J2 (and a little on J4) — these are the heavy
// proximal joints that benefit from extra damping for a stable feel; the
// wrists are kept very loose so they don't fight your wrist while teaching.
//   J2/J4 ≈ 1.0 → ~1 s decay
//   others ≈ 0.3 → ~3.3 s decay (basically free-floating)
inline Vector7d default_idle_d_rate() {
    return (Vector7d() << 0.3, 1.0, 0.3, 1.0, 0.3, 0.3, 0.3).finished();
}

struct IdleCfg {
    Vector7d d_rate{default_idle_d_rate()};
    bool     use_friction{true};
};

class GravityCompensationController : public Controller {
 public:
    ControllerType        type() const override;
    std::string           name() const override;
    void                  reset(const franka::RobotState& s) override;
    std::array<double, 7> compute(const franka::RobotState& s,
                                  const franka::Model& model) override;

    void set_cfg(const IdleCfg& c) { cfg_ = c; }

    // Soft velocity wall — ratio of FR3 hard joint velocity limits.
    //   |dq| < v_warn_frac · dq_max  →  no wall torque (just damping).
    //   v_warn_frac → v_clip_frac    →  wall torque ramps 0 → tau_wall_max.
    //   |dq| > v_clip_frac · dq_max  →  saturated at ±tau_wall_max.
    // These are compile-time tunables (rare to need from Python). Lowered
    // v_warn_frac to 0.35 so the wall picks up slack when d_rate is small.
    double v_warn_frac {0.35};
    double v_clip_frac {0.80};
    double tau_wall_max{80.0};   // [Nm]

 private:
    IdleCfg cfg_{};
};
