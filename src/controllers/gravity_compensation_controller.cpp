#include <fr3_stack/controllers/gravity_compensation_controller.hpp>
#include <fr3_stack/utils/friction_model.hpp>

#include <algorithm>
#include <cmath>

ControllerType GravityCompensationController::type() const {
    return ControllerType::GravityCompensation;
}

std::string GravityCompensationController::name() const {
    return "gravity_compensation";
}

void GravityCompensationController::reset(const franka::RobotState&) {}

// FR3 hard joint velocity limits [rad/s]. From Franka Research 3 docs.
// libfranka's joint_velocity_violation reflex trips when |dq[i]| reaches
// (or briefly overshoots) these — the soft wall below is positioned to
// keep us away from them.
static constexpr std::array<double, 7> kFr3DqMax = {
    2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};

// Cognetti / FrankaEmikaPandaDynModel sigmoid friction params for FR3.
// Same numbers crisp_controllers/cartesian_controller.yaml ships with —
// see https://github.com/marcocognetti/FrankaEmikaPandaDynModel.
//   τ_fric(dq) = fp1 · σ(−fp2·(dq+fp3))  −  fp1 · σ(−fp2·fp3)
// Subtracting the second term zeroes the model at dq=0 so we don't inject
// torque at rest. The static Vector7d's are constructed once at first
// call (thread-safe in C++11+, free at steady state — no heap on the RT path).
static const Vector7d& kFrictionFp1() {
    static const Vector7d v =
        (Vector7d() << 0.54615, 0.87224, 0.64068, 1.2794,
                       0.83904, 0.30301, 0.56489).finished();
    return v;
}
static const Vector7d& kFrictionFp2() {
    static const Vector7d v =
        (Vector7d() << 5.1181, 9.0657, 10.136, 5.5903,
                       8.3469, 17.133, 10.336).finished();
    return v;
}
static const Vector7d& kFrictionFp3() {
    static const Vector7d v =
        (Vector7d() << 0.039533,  0.025882, -0.04607,  0.036194,
                       0.026226, -0.021047,  0.0035526).finished();
    return v;
}

std::array<double, 7> GravityCompensationController::compute(
    const franka::RobotState& s, const franka::Model& model) {
    Eigen::Map<const Vector7d> dq(s.dq.data());

    // 1. Inertia-aware per-joint damping.
    //    τ_damp[i] = -d_rate[i] · (M · dq)[i]
    //    Each joint decays with time constant 1 / d_rate[i] regardless of
    //    pose. The default favors heavier proximal joints (J2, J4) so the
    //    arm doesn't feel "saggy" there while wrists stay loose.
    std::array<double, 49> M_arr = model.mass(s);
    Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(M_arr.data());
    Vector7d Mdq = M * dq;
    Vector7d tau = -cfg_.d_rate.cwiseProduct(Mdq);

    // 2. Friction compensation (optional, default on). Sigmoid model from
    //    Cognetti et al. — the SAME formula and params CRISP / pixi use,
    //    so this gives the franka_ros2 hand-guiding feel: J2/J4 stop
    //    feeling notably heavier than wrists once their natural Coulomb
    //    friction is cancelled.
    if (cfg_.use_friction) {
        // get_friction is templated on the input type; passing the Map and
        // a Vector7d together makes template deduction conflict, so
        // materialize dq into a Vector7d first.
        const Vector7d dq_v = dq;
        tau += get_friction(dq_v, kFrictionFp1(), kFrictionFp2(), kFrictionFp3());
    }

    // 3. Soft velocity wall — keeps |dq| safely under FR3's hard limits.
    for (int i = 0; i < 7; ++i) {
        const double v       = std::abs(dq[i]);
        const double dq_max  = kFr3DqMax[i];
        const double v_warn  = v_warn_frac * dq_max;
        const double v_clip  = v_clip_frac * dq_max;
        if (v > v_warn) {
            const double frac =
                std::min(1.0, (v - v_warn) / std::max(v_clip - v_warn, 1e-9));
            const double sgn = dq[i] >= 0.0 ? 1.0 : -1.0;
            tau[i] -= sgn * tau_wall_max * frac;
        }
    }

    std::array<double, 7> out;
    for (int i = 0; i < 7; ++i) out[i] = tau[i];
    return out;
}
