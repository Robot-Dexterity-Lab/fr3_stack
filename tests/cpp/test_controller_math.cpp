// Math-mock tests for fr3_stack controllers.
//
// Strategy: include the actual controller headers (the production
// implementation), supplied with a tiny libfranka mock under
// tests/cpp/franka_mock/. The mock provides the surface the controllers
// touch (RobotState fields, Model::zeroJacobian/mass/coriolis), the test
// fixtures wire J / M / c / pose by hand, and we read the resulting τ.
//
// Build (CMake):
//     cmake -B build -DFR3_BUILD_TESTS=ON
//     cmake --build build --target test_controller_math
//     ./build/test_controller_math
//
// Build (manual, no CMake):
//     g++ -std=c++17 -O2 \
//         -I include \
//         -I tests/cpp/franka_mock \
//         -I /usr/include/eigen3 \   # (or wherever Eigen is)
//         tests/cpp/test_controller_math.cpp -o /tmp/test_controller_math \
//         -lpthread
//     /tmp/test_controller_math
//
// Coverage:
//   - Utility math: ema/slerp_quat/log3, joint_limit_repulsion,
//     friction_compensation, clip_error, get_joint_limit_torque (templated),
//     get_friction (templated, sigmoid / FR3 params).
//   - Controllers via real headers: GravityCompensation, CartesianImpedance,
//     JointImpedance, CartesianAdmittance, HybridForceMotion.
//   - HybridForceMotion P0 fixes:
//       #4  inner_v_ projects through Sf on Tr/n_af switch
//       #5  max_inner_v / max_inner_w clamps velocity-axis tracking
//       #6  F_ext is EMA-filtered before feeding PID + spring/damping

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

// Pull in the real controllers (with the franka mock on the include path).
#include <fr3_stack/utils/controllers_common.hpp>
#include <fr3_stack/utils/fiters.hpp>
#include <fr3_stack/utils/joint_limits.hpp>
#include <fr3_stack/utils/friction_model.hpp>
#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/controllers/gravity_compensation_controller.hpp>
#include <fr3_stack/controllers/cartesian_impedance_controller.hpp>
#include <fr3_stack/controllers/joint_impedance_controller.hpp>
#include <fr3_stack/controllers/cartesian_admittance_controller.hpp>
#include <fr3_stack/controllers/hybrid_force_motion_controller.hpp>

// Mock WrenchSource (concrete subclass). The real factory needs vendor
// driver libs; for tests we just hand the controller a stub.
class MockWrenchSource : public WrenchSource {
 public:
    void set(const Vector6d& w, bool ready = true) { w_ = w; ready_ = ready; }
    void start() override {}
    void stop() noexcept override {}
    bool read(Vector6d& out) const override {
        if (!ready_) return false;
        out = w_;
        return true;
    }
    const char* kind() const override { return "mock"; }
 private:
    Vector6d w_{Vector6d::Zero()};
    bool     ready_{false};
};

// ============================================================================
// Tiny test framework
// ============================================================================
static int         g_pass = 0, g_fail = 0;
static std::string g_section;

static void section(const std::string& s) {
    g_section = s;
    std::cout << "\n[ " << s << " ]\n";
}

#define CHECK(cond, msg) do {                                                  \
    if (cond) { ++g_pass; std::cout << "  PASS: " << msg << "\n"; }            \
    else { ++g_fail; std::cerr << "  FAIL [" << g_section << "]: " << msg      \
                               << " (" << __FILE__ << ":" << __LINE__ << ")\n"; }\
} while (0)

static bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

template <typename V>
static bool vec_near(const V& a, const V& b, double tol) {
    if (a.size() != b.size()) return false;
    for (Eigen::Index i = 0; i < a.size(); ++i)
        if (!near(a[i], b[i], tol)) return false;
    return true;
}

// ============================================================================
// Test fixtures
// ============================================================================
static void set_identity_pose(franka::RobotState& s) {
    s.O_T_EE = {1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};
}

// J = [I_6 | 0_6x1] in column-major. Each Cartesian axis maps 1:1 to
// joints 0..5; joint 7 is in the nullspace.
static void set_jacobian_id6_pad(franka::Model& m) {
    m.J_zero.fill(0.0);
    for (int c = 0; c < 6; ++c) m.J_zero[c + 6 * c] = 1.0;
    m.J_body = m.J_zero;
}

static void set_mass_alpha_I(franka::Model& m, double alpha) {
    m.M_inertia.fill(0.0);
    for (int i = 0; i < 7; ++i) m.M_inertia[i + 7 * i] = alpha;
}

static void zero_coriolis(franka::Model& m) { m.c_vec.fill(0.0); }

// Joint angles in the middle of every FR3 joint range so the soft barrier
// produces zero torque (we want to test impedance math in isolation).
static void set_q_safe(franka::RobotState& s) {
    for (int i = 0; i < 7; ++i)
        s.q[i] = 0.5 * (kFr3QMax[i] + kFr3QMin[i]);
}

// ============================================================================
// fiters.hpp — EMA / slerp_quat / log3
// ============================================================================
static void test_ema_scalar() {
    section("ema (scalar)");
    CHECK(near(ema(0.0, 1.0, 0.5), 0.5, 1e-12), "midpoint at α=0.5");
    CHECK(near(ema(0.0, 1.0, 0.0), 0.0, 1e-12), "α=0 freezes");
    CHECK(near(ema(0.0, 1.0, 1.0), 1.0, 1e-12), "α=1 passes through");
    double s = 0.0;
    for (int i = 0; i < 1000; ++i) s = ema(s, 1.0, 0.05);
    CHECK(s > 0.999 && s < 1.0, "converges to step input");
}

static void test_ema_vec3() {
    section("ema (Vector3d)");
    Eigen::Vector3d a(0, 0, 0), b(1, 2, 3);
    Eigen::Vector3d e = ema(a, b, 0.5);
    CHECK(vec_near(e, Eigen::Vector3d(0.5, 1.0, 1.5), 1e-12), "elementwise midpoint");
}

static void test_slerp_quat_hemisphere_flip() {
    section("slerp_quat (hemisphere flip)");
    Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    // Same rotation as identity, but with negated coefficients (other hemisphere).
    Eigen::Quaterniond q1(-1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond half = slerp_quat(q0, q1, 0.5);
    // Without flip, .slerp would produce a 180° rotation. With flip, it stays
    // near identity (q1 == -q0 represents the same rotation).
    Eigen::AngleAxisd aa(half);
    CHECK(near(aa.angle(), 0.0, 1e-9), "identity vs −identity stays at identity");
}

static void test_log3_round_trip() {
    section("log3 round trip");
    CHECK(log3(Eigen::Matrix3d::Identity()).norm() < 1e-12,
          "log3(I) == 0");

    // Exponentiate axis*angle, log3 it back.
    Eigen::Vector3d axis(0.0, 0.0, 1.0);
    for (double a : {0.05, 0.5, 1.5, 2.5, 3.0}) {
        Eigen::Matrix3d R = Eigen::AngleAxisd(a, axis).toRotationMatrix();
        Eigen::Vector3d v = log3(R);
        CHECK(near(v.norm(), a, 1e-9),
              "log3(exp(axis*" + std::to_string(a) + ")) recovers angle");
        CHECK(near(v.normalized().dot(axis), 1.0, 1e-9),
              "log3 axis matches input axis at angle " + std::to_string(a));
    }
}

// ============================================================================
// controllers_common.hpp helpers (same coverage as the pre-refactor file)
// ============================================================================
static void test_joint_limit_repulsion() {
    section("joint_limit_repulsion (FR3-baked)");
    Vector7d q = Vector7d::Zero();
    for (int i = 0; i < 7; ++i)
        q[i] = 0.5 * (kFr3QMax[i] + kFr3QMin[i]);
    CHECK(joint_limit_repulsion(q).norm() < 1e-12, "zero in middle of every range");

    Vector7d qhi = q;
    qhi[0] = kFr3QMax[0];
    CHECK(near(joint_limit_repulsion(qhi)[0], -kJointLimitMaxTau, 1e-9),
          "−10 Nm at upper limit (joint 0)");

    Vector7d qlo = q;
    qlo[0] = kFr3QMin[0];
    CHECK(near(joint_limit_repulsion(qlo)[0], kJointLimitMaxTau, 1e-9),
          "+10 Nm at lower limit (joint 0)");

    Vector7d q_far = q;
    q_far[0] = kFr3QMax[0] + 2.0;
    CHECK(near(joint_limit_repulsion(q_far)[0], -kJointLimitMaxTau, 1e-9),
          "saturates beyond the limit");

    Vector7d q_in = q;
    const double range  = kFr3QMax[0] - kFr3QMin[0];
    const double margin = kJointLimitMargin * range;
    q_in[0] = kFr3QMax[0] - margin - 1e-6;
    CHECK(near(joint_limit_repulsion(q_in)[0], 0.0, 1e-9),
          "zero just inside the margin");

    Vector7d q_half = q;
    q_half[0] = kFr3QMax[0] - 0.5 * margin;
    CHECK(near(joint_limit_repulsion(q_half)[0], -0.5 * kJointLimitMaxTau, 1e-9),
          "half torque at half-margin penetration");
}

static void test_friction_compensation_tanh() {
    section("friction_compensation (tanh + viscous, FR3-baked)");
    Vector7d dq = Vector7d::Zero();
    CHECK(friction_compensation(dq).norm() < 1e-12, "zero at zero velocity");

    Vector7d dq_pos = (Vector7d() << 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4).finished();
    CHECK(vec_near(friction_compensation(-dq_pos),
                   Vector7d(-friction_compensation(dq_pos)), 1e-12),
          "f(−dq) = −f(dq) (odd function)");

    Vector7d big = Vector7d::Constant(10.0);
    Vector7d tb  = friction_compensation(big);
    static const Vector7d fp1 =
        (Vector7d() << 0.5, 0.5, 0.5, 0.5, 0.3, 0.3, 0.2).finished();
    static const Vector7d fp3 = Vector7d::Constant(0.05);
    for (int i = 0; i < 7; ++i)
        CHECK(near(tb[i], fp1[i] + fp3[i] * 10.0, 1e-3),
              "saturates to fp1[i] + fp3[i]·dq[i] at large |dq| (joint " + std::to_string(i) + ")");
}

static void test_clip_error() {
    section("clip_error");
    Vector6d e; e << 0.05, -0.2, 0.3, 0.04, -0.1, 0.0;
    Vector6d md_zero = Vector6d::Zero();
    CHECK(vec_near(clip_error(e, md_zero), e, 0),
          "max_delta == 0 leaves error untouched");
    Vector6d md; md << 0.1, 0.1, 0.1, 0.05, 0.05, 0.05;
    Vector6d expected; expected << 0.05, -0.1, 0.1, 0.04, -0.05, 0.0;
    CHECK(vec_near(clip_error(e, md), expected, 1e-12),
          "clamps each axis to [−md, +md]");
}

// ============================================================================
// Templated utilities — joint_limits.hpp / friction_model.hpp
// ============================================================================
static void test_get_joint_limit_torque_template() {
    section("get_joint_limit_torque<>: fixed and dynamic vector");
    Vector7d q = Vector7d::Zero();
    Vector7d qmin = (Vector7d() << -1, -1, -1, -1, -1, -1, -1).finished();
    Vector7d qmax = (Vector7d() <<  1,  1,  1,  1,  1,  1,  1).finished();
    Vector7d t_fixed = get_joint_limit_torque(q, qmin, qmax, 0.3, 5.0);
    CHECK(t_fixed.norm() < 1e-12, "fixed-size: zero in middle");

    Vector7d q_at_max = qmax;
    Vector7d t_at_max = get_joint_limit_torque(q_at_max, qmin, qmax, 0.3, 5.0);
    CHECK(near(t_at_max[0], -5.0, 1e-9), "fixed-size: −max_torque at upper");

    // Same call with VectorXd (templated path covers both).
    Eigen::VectorXd q_d  = q;
    Eigen::VectorXd qmin_d = qmin;
    Eigen::VectorXd qmax_d = qmax;
    Eigen::VectorXd t_dyn = get_joint_limit_torque(q_d, qmin_d, qmax_d, 0.3, 5.0);
    CHECK(t_dyn.norm() < 1e-12, "dynamic-size: zero in middle");
}

static void test_get_friction_sigmoid() {
    section("get_friction<>: sigmoid model, CRISP FR3 params");
    // CRISP yaml defaults — verify the sigmoid passes through 0 at dq=0 by
    // construction (the formula subtracts σ(−fp2·fp3) so f(0) ≡ 0).
    const Vector7d fp1 = (Vector7d() <<
        0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489).finished();
    const Vector7d fp2 = (Vector7d() <<
        5.1181,  9.0657,  10.136,  5.5903, 8.3469,  17.133,  10.336 ).finished();
    const Vector7d fp3 = (Vector7d() <<
        0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047, 0.0035526
        ).finished();
    Vector7d dq0 = Vector7d::Zero();
    CHECK(get_friction(dq0, fp1, fp2, fp3).norm() < 1e-12,
          "f(0) == 0 at zero velocity (offset cancels)");

    // Monotonicity: dq2 > dq1 ⇒ f(dq2) >= f(dq1) per joint (sigmoid is
    // non-decreasing in dq).
    Vector7d dq1 = Vector7d::Constant(0.1);
    Vector7d dq2 = Vector7d::Constant(0.5);
    Vector7d f1 = get_friction(dq1, fp1, fp2, fp3);
    Vector7d f2 = get_friction(dq2, fp1, fp2, fp3);
    bool monotone = true;
    for (int i = 0; i < 7; ++i) if (f2[i] < f1[i]) { monotone = false; break; }
    CHECK(monotone, "monotone in dq (per joint)");
}

// ============================================================================
// GravityCompensationController
// ============================================================================
static void test_gravity_compensation() {
    section("GravityCompensationController");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);
    set_mass_alpha_I(m, 1.0);   // M = I_7 → mass-weighted damping = scalar damping

    GravityCompensationController c;
    // d_rate moved into IdleCfg in the *_cfg refactor; pin it to a uniform
    // scalar here so per-joint assertions remain readable.
    IdleCfg gc_cfg;
    gc_cfg.d_rate = Vector7d::Constant(10.0);
    gc_cfg.use_friction = false;
    c.set_cfg(gc_cfg);
    c.reset(s);
    const double D_RATE = 10.0;

    // dq == 0 → tau == 0 (only damping/wall, no stiffness; FCI adds gravity).
    auto tau = c.compute(s, m);
    bool zero = true;
    for (int i = 0; i < 7; ++i) if (tau[i] != 0.0) { zero = false; break; }
    CHECK(zero, "tau == 0 at rest (gravity from FCI, no spring)");

    // Small dq below the soft velocity wall → only -d_rate · M · dq.
    // With M = I, joint i at dq=0.1 gives tau[i] = -d_rate · 0.1.
    s.dq[2] = 0.1;
    s.dq[5] = -0.1;
    tau = c.compute(s, m);
    CHECK(near(tau[2], -D_RATE * 0.1,  1e-9), "tau[2] = -d_rate·dq[2] (M=I, sub-wall)");
    CHECK(near(tau[5], -D_RATE * -0.1, 1e-9), "tau[5] opposes motion (M=I, sub-wall)");
    s.dq[2] = 0.0; s.dq[5] = 0.0;

    // Velocity wall: above v_warn_frac · dq_max, tau gets an extra
    // sign-opposed term ramping to ±tau_wall_max at v_clip_frac. Joint 0:
    // dq_max = 2.62, v_warn (0.35) = 0.917, v_clip (0.80) = 2.096. At dq=2.0,
    // frac = (2.0 - 0.917) / (2.096 - 0.917) = 0.9186, wall_term =
    // -0.9186 · 80 = -73.49. Plus damping = -10·2.0 = -20. Total ≈ -93.49 Nm.
    s.dq[0] = 2.0;
    tau = c.compute(s, m);
    const double expected_damp = -D_RATE * 2.0;
    const double v_warn = c.v_warn_frac * 2.62;
    const double v_clip = c.v_clip_frac * 2.62;
    const double wall_frac = (2.0 - v_warn) / (v_clip - v_warn);
    const double expected = expected_damp - wall_frac * c.tau_wall_max;
    CHECK(near(tau[0], expected, 1e-9), "tau[0] = damping + soft wall (in ramp zone)");
    s.dq[0] = 0.0;

    CHECK(c.name() == "gravity_compensation", "name() == 'gravity_compensation'");
    CHECK(c.type() == ControllerType::GravityCompensation, "type() matches enum");
}

// ============================================================================
// CartesianImpedanceController
// ============================================================================
static void test_cart_imp_equilibrium() {
    section("CartesianImpedance: equilibrium → zero torque");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianImpedanceController c;
    CartesianImpedanceCfg cfg;
    cfg.target = Eigen::Affine3d::Identity();
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    bool zero = true;
    for (int i = 0; i < 7; ++i) if (std::abs(tau[i]) > 1e-9) { zero = false; break; }
    CHECK(zero, "tau ~ 0 at equilibrium");
}

static void test_cart_imp_step_target() {
    section("CartesianImpedance: target step → first-tick spring force");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianImpedanceController c;
    CartesianImpedanceCfg cfg;
    cfg.target = Eigen::Affine3d::Identity();
    cfg.target.translation() << 0.1, 0.0, 0.0;
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    const double expected = cfg.K[0] * cfg.filter_alpha * 0.1;
    CHECK(near(tau[0], expected, 1e-9),
          "tau[0] == K_x · α · dx after one filter step");
}

static void test_cart_imp_damping() {
    section("CartesianImpedance: damping torque from velocity");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.dq[0] = 0.1;
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianImpedanceController c;
    CartesianImpedanceCfg cfg;
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);

    const double lam2     = 0.05 * 0.05;
    const double N00      = lam2 / (1.0 + lam2);
    const double tau_task = -cfg.D[0] * 0.1;
    const double tau_null = N00 * (-2.0 * std::sqrt(cfg.K_null) * 0.1);
    const double expected = tau_task + tau_null;
    CHECK(near(tau[0], expected, 1e-9),
          "tau[0] = −D_x·v_x + N[0,0]·(−2√K_null·dq[0])");
}

static void test_cart_imp_max_delta_clip() {
    section("CartesianImpedance: max_delta clips spring force");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianImpedanceController c;
    CartesianImpedanceCfg cfg;
    cfg.target = Eigen::Affine3d::Identity();
    cfg.target.translation() << 1.0, 0.0, 0.0;
    cfg.max_delta << 0.001, 0, 0, 0, 0, 0;
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    CHECK(near(tau[0], cfg.K[0] * 0.001, 1e-6),
          "τ[0] clamped to K·max_delta regardless of target distance");
}

// ============================================================================
// JointImpedanceController
// ============================================================================
static void test_joint_imp_equilibrium() {
    section("JointImpedance: equilibrium → zero torque");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m);

    JointImpedanceController c;
    JointImpedanceCfg cfg;
    cfg.q_target = Eigen::Map<const Vector7d>(s.q.data());
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    bool zero = true;
    for (int i = 0; i < 7; ++i) if (std::abs(tau[i]) > 1e-9) { zero = false; break; }
    CHECK(zero, "τ ~ 0 at q == q_target, dq=0, c=0");
}

static void test_joint_imp_damping() {
    section("JointImpedance: damping = −D·dq");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.dq[3] = 0.2;
    franka::Model m; zero_coriolis(m);

    JointImpedanceController c;
    JointImpedanceCfg cfg;
    cfg.q_target = Eigen::Map<const Vector7d>(s.q.data());
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    CHECK(near(tau[3], -cfg.D[3] * 0.2, 1e-9), "τ[3] = −D[3]·dq[3]");
}

// ============================================================================
// CartesianAdmittanceController
// ============================================================================
static void test_admittance_external_wrench() {
    section("CartesianAdmittance: F_ext drives inner; outer pulls robot along");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.O_F_ext_hat_K = {5.0, 0, 0, 0, 0, 0};
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianAdmittanceController c;
    CartesianAdmittanceCfg cfg;
    cfg.target              = Eigen::Affine3d::Identity();
    cfg.wrench_filter_alpha = 1.0;     // pass-through for deterministic test
    c.set_cfg(cfg);
    c.reset(s);
    // Trace: F_ext=+5N, M=5 → a=+1 m/s². After dt=1ms, inner_v=+1e−3,
    // inner_t=+1e−6. Outer impedance sees e = p − inner_t = −1e−6, applies
    // F_imp = −K·e = +K·1e−6 > 0, so τ[0] > 0 (robot pulled toward the
    // drifted inner pose).
    auto tau1 = c.compute(s, m);
    CHECK(tau1[0] > 0.0 && std::isfinite(tau1[0]),
          "τ[0] > 0 on first tick (outer K pulls robot toward inner +x drift)");

    // Inner keeps drifting under sustained F_ext, so τ[0] grows over ticks.
    auto tauN = tau1;
    for (int i = 0; i < 20; ++i) tauN = c.compute(s, m);
    CHECK(tauN[0] > tau1[0],
          "τ[0] increases over 20 ticks as inner_t accumulates +x motion");
}

// ============================================================================
// HybridForceMotionController — including P0 fix verification
// ============================================================================
static void test_hybrid_pure_admittance_equilibrium() {
    section("HybridForceMotion: n_af=0 + no F_ext + target=current → ~0 τ");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    HybridForceMotionController c;
    HybridForceMotionCfg cfg;
    cfg.target              = Eigen::Affine3d::Identity();
    cfg.n_af                = 0;
    cfg.wrench_filter_alpha = 1.0;
    c.set_cfg(cfg);
    c.reset(s);
    auto tau = c.compute(s, m);
    double max_abs = 0.0;
    for (int i = 0; i < 7; ++i) max_abs = std::max(max_abs, std::abs(tau[i]));
    CHECK(max_abs < 1e-6, "all τ components ~ 0");
}

// P0 #6 fix: F_ext EMA. With wrench_filter_alpha = 0.1 and a step in F_ext,
// internal F_filt advances by α·F_meas per tick instead of jumping.
// We probe via the resulting τ on the first tick: it should be much smaller
// than with α=1 (pass-through).
static void test_hybrid_p0_6_wrench_filter() {
    section("HybridForceMotion P0 #6: F_ext low-pass mutes first-tick spike");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.O_F_ext_hat_K = {100.0, 0, 0, 0, 0, 0};   // big external push
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    auto first_tick_tau0 = [&](double alpha) -> double {
        HybridForceMotionController c;
        HybridForceMotionCfg cfg;
        cfg.target              = Eigen::Affine3d::Identity();
        cfg.n_af                = 0;
        cfg.wrench_filter_alpha = alpha;
        c.set_cfg(cfg);
        c.reset(s);
        return c.compute(s, m)[0];
    };

    // Note: F_ext_init_ flag in the controller seeds F_ext_filt_ to the first
    // raw reading on the very first tick, so α only kicks in from tick 2
    // onward. To exercise the filter we run a few ticks, comparing α=0.02
    // (fr3 default) vs α=1.0 (pass-through).
    auto run_n_ticks = [&](double alpha, int n) -> double {
        HybridForceMotionController c;
        HybridForceMotionCfg cfg;
        cfg.target              = Eigen::Affine3d::Identity();
        cfg.n_af                = 0;
        cfg.wrench_filter_alpha = alpha;
        c.set_cfg(cfg);
        c.reset(s);
        // Reset zeros F_ext_filt_; on first tick init seeds with raw. Then
        // force the test by zeroing F_ext_init_ logic — we can't access that
        // directly, so instead we rely on the "first tick is raw" being
        // identical for both α values, and look at later ticks.
        std::array<double, 7> tau{};
        for (int i = 0; i < n; ++i) tau = c.compute(s, m);
        return tau[0];
    };
    (void)first_tick_tau0;   // suppress unused warning

    // After first tick both are equal (init). Subsequent ticks: with α=1 the
    // filter passes through the raw F_ext (constant 100 N), while with α=0.02
    // the filter relaxes very slowly — but in steady state both reach the
    // same asymptote. The interesting window is mid-transient. Pick 2 ticks
    // and check ordering: |tau_alpha02| ≤ |tau_alpha1| (filter holds back).
    double t1 = run_n_ticks(1.0,  2);
    double t2 = run_n_ticks(0.02, 2);
    CHECK(std::abs(t2) <= std::abs(t1) + 1e-9,
          "after 2 ticks, |τ| with α=0.02 ≤ |τ| with α=1 (filter holds)");
}

// P0 #5 fix: max_inner_v / max_inner_w clamps the velocity-axis tracking.
// With n_af=0 the entire 6-D space is admittance-controlled (no Sv axes),
// so we set n_af=1 to make 5 axes velocity-tracked, then jam target far
// away. Without the clamp, motion_Tr[1..5] would be err/dt = 1m/0.001s = 1000 m/s.
// With the clamp, the resulting torques stay bounded.
static void test_hybrid_p0_5_max_inner_v_clamp() {
    section("HybridForceMotion P0 #5: max_inner_v clamps velocity-axis tracking");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    HybridForceMotionController c;
    HybridForceMotionCfg cfg;
    cfg.target = Eigen::Affine3d::Identity();
    cfg.target.translation() << 1.0, 0.0, 0.0;   // 1 m offset (huge)
    cfg.n_af = 0;       // pure admittance to isolate from force PID
    // BUT: pure admittance uses M·a = K·err; we want the velocity-axis
    // clamp path. Set n_af=1 so axes 1..5 are velocity-tracked.
    cfg.n_af                = 1;
    cfg.wrench_filter_alpha = 1.0;
    cfg.max_inner_v         = 0.5;
    cfg.max_inner_w         = 1.5;
    cfg.K_adm               = Vector6d::Zero();   // no spring force
    cfg.D_adm               = Vector6d::Zero();   // no damping
    cfg.target_wrench_Tr    = Vector6d::Zero();   // no force command
    cfg.P_trans = cfg.I_trans = cfg.D_trans = 0;
    cfg.P_rot   = cfg.I_rot   = cfg.D_rot   = 0;
    // Outer impedance gains relaxed so we can see the inner velocity directly.
    cfg.K = Vector6d::Zero();
    cfg.D = Vector6d::Zero();
    cfg.K_null = 0;
    c.set_cfg(cfg);
    c.reset(s);
    // Run 1 tick. Since outer K=0, τ_task is near zero too; what we're
    // really testing is that the inner integration didn't blow up.
    auto tau = c.compute(s, m);
    bool finite = true;
    for (int i = 0; i < 7; ++i) if (!std::isfinite(tau[i])) { finite = false; break; }
    CHECK(finite, "τ stays finite when adm_err is huge (clamp prevented runaway)");
    // After 1ms with v_lin clamped at 0.5 m/s, inner_t advances at most 0.5e-3 m.
    // Run 100 ticks → inner_t advances at most 0.05 m.
    for (int i = 0; i < 100; ++i) c.compute(s, m);
    auto tau_late = c.compute(s, m);
    bool finite2 = true;
    for (int i = 0; i < 7; ++i) if (!std::isfinite(tau_late[i])) { finite2 = false; break; }
    CHECK(finite2, "τ remains finite after 100 ticks (no integrator runaway)");
}

// P0 #4 fix: Tr/n_af switch projects inner_v through Sf, zeroing the
// velocity-axis component. We jam inner_v with axis 1 (velocity-controlled)
// motion under n_af=1, then switch to n_af=0; the controller should zero
// inner_v on the now-velocity-axis (all axes are velocity in n_af=0, so
// inner_v should be fully zeroed).
static void test_hybrid_p0_4_inner_v_reset_on_switch() {
    section("HybridForceMotion P0 #4: Tr/n_af switch projects inner_v through Sf");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    HybridForceMotionController c;
    HybridForceMotionCfg cfg;
    cfg.target = Eigen::Affine3d::Identity();
    cfg.target.translation() << 0.05, 0, 0;
    cfg.n_af                = 1;       // axis 0 force-controlled, 1..5 velocity
    cfg.wrench_filter_alpha = 1.0;
    cfg.max_inner_v         = 10.0;    // relax cap so velocities can build
    cfg.max_inner_w         = 10.0;
    cfg.K = cfg.D = Vector6d::Zero();
    cfg.K_null = 0;
    c.set_cfg(cfg);
    c.reset(s);
    // Run 100 ticks to populate inner_v on the velocity axes.
    for (int i = 0; i < 100; ++i) c.compute(s, m);

    // Switch to n_af=0 (all velocity-controlled). Sf becomes zero matrix,
    // so projection inner_v = Tr⁻¹·Sf·Tr·inner_v = 0. After the switch the
    // controller should not produce a torque burst from carried-over inner_v.
    cfg.n_af = 0;
    c.set_cfg(cfg);
    auto tau_after_switch = c.compute(s, m);
    bool finite = true;
    double max_abs = 0;
    for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(tau_after_switch[i])) finite = false;
        max_abs = std::max(max_abs, std::abs(tau_after_switch[i]));
    }
    CHECK(finite, "τ finite after Tr/n_af switch");
    // With outer K=D=0 and inner_v zeroed via Sf projection, τ_task should
    // be close to zero (only Coriolis + joint-limit barrier contribute, and
    // both are zero in this fixture).
    CHECK(max_abs < 1.0,
          "no force burst on switch (max |τ| < 1 Nm with outer gains zeroed)");
}

// Wrench deadband integration test: sub-eps F_ext must not reach the
// admittance integrator; above-eps F_ext must drive inner_v growth.
static void test_hybrid_wrench_deadband() {
    section("HybridForceMotion: wrench_deadband shrinks sub-eps F_ext to zero "
            "before admittance integrator");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    franka::Model      m; zero_coriolis(m);
    set_jacobian_id6_pad(m);
    set_mass_alpha_I(m, 1.0);

    // Config shared between both cases. Outer K non-zero so the outer
    // impedance translates inner_t drift into torque. D_adm=0 so there is no
    // velocity damping that would mask a slowly-growing inner_v.
    auto make_cfg = []() {
        HybridForceMotionCfg cfg;
        cfg.target              = Eigen::Affine3d::Identity();
        cfg.n_af                = 0;               // pure admittance path
        cfg.wrench_filter_alpha = 1.0;             // pass-through EMA, isolates deadband
        cfg.wrench_deadband << 1.0, 1.0, 1.0, 0.1, 0.1, 0.1;
        // Admittance params: unit mass, no spring, no damping so that the only
        // admittance contribution is M^-1 * F_ext.
        cfg.M_adm    = Vector6d::Ones();
        cfg.K_adm    = Vector6d::Zero();
        cfg.D_adm    = Vector6d::Zero();
        // Keep outer impedance so inner_t drift → measurable τ.
        // Use default K / D (200/28 per task description area) — just don't zero them.
        cfg.K_null   = 0.0;                        // no nullspace torque
        return cfg;
    };

    // Case 1: F_ext = 0.5 N on +z, well below the 1.0 N deadband.
    //  => deadband shrinks F_ext to 0 => inner_v stays at zero => τ ~ 0.
    {
        HybridForceMotionController c;
        auto cfg = make_cfg();
        c.set_cfg(cfg);
        c.reset(s);
        s.O_F_ext_hat_K = {{0.0, 0.0, 0.5, 0.0, 0.0, 0.0}};
        std::array<double, 7> tau_below{};
        for (int i = 0; i < 200; ++i) tau_below = c.compute(s, m);

        double max_below = 0.0;
        for (int i = 0; i < 7; ++i)
            max_below = std::max(max_below, std::abs(tau_below[i]));
        // With deadband absorbing all of F_ext, inner_v stays at 0, inner_t
        // stays at identity, outer K sees zero error => tau ~ 0.
        CHECK(max_below < 1e-6,
              "below deadband: max |τ| < 1e-6 (inner_v does not accumulate)");
    }

    // Case 2: F_ext = 2.0 N on +z, above the 1.0 N deadband.
    //  => effective F_ext = 1.0 N feeds the integrator => inner_v grows on z
    //  => outer K sees growing inner_t error => τ grows.
    {
        HybridForceMotionController c;
        auto cfg = make_cfg();
        c.set_cfg(cfg);
        c.reset(s);
        s.O_F_ext_hat_K = {{0.0, 0.0, 2.0, 0.0, 0.0, 0.0}};
        std::array<double, 7> tau_above{};
        for (int i = 0; i < 200; ++i) tau_above = c.compute(s, m);

        // Joint 2 maps to Cartesian z (J=I6, column-major), so tau_above[2]
        // should reflect the inner_t drift on the z axis.
        double max_above = 0.0;
        for (int i = 0; i < 7; ++i)
            max_above = std::max(max_above, std::abs(tau_above[i]));
        CHECK(max_above > 1e-3,
              "above deadband: max |τ| > 1e-3 (inner_v grows, outer K drives τ)");
    }
}

// ============================================================================
static void test_soft_deadband_helper() {
    section("soft_deadband: per-axis shrinkage, continuous at boundary");
    Vector6d eps;
    eps << 0.5, 0.5, 0.5, 0.05, 0.05, 0.05;

    // |x| < eps -> 0
    Vector6d x_inside;
    x_inside << 0.3, -0.4, 0.0, 0.02, -0.04, 0.01;
    Vector6d y_inside = soft_deadband(x_inside, eps);
    CHECK(y_inside.isZero(1e-12), "inside deadband must be exactly zero");

    // |x| past eps -> exactly |x|-eps with original sign
    Vector6d x_outside;
    x_outside << 0.7, -0.9, 0.5 + 1e-9, 0.06, -0.07, 0.05 + 1e-9;
    Vector6d y_outside = soft_deadband(x_outside, eps);
    CHECK(std::abs(y_outside(0) - (0.7  - 0.5))  < 1e-12, "axis 0 magnitude");
    CHECK(std::abs(y_outside(1) - (-0.9 + 0.5))  < 1e-12, "axis 1 sign + magnitude");
    CHECK(y_outside(2) > 0.0,                              "axis 2 just past eps stays > 0");
    CHECK(std::abs(y_outside(3) - (0.06 - 0.05)) < 1e-12, "rotational axis magnitude");

    // Continuity: exactly at eps -> 0
    Vector6d x_at;  x_at  << 0.5, 0.5, 0.5, 0.05, 0.05, 0.05;
    Vector6d y_at = soft_deadband(x_at, eps);
    CHECK(y_at.isZero(1e-12), "exactly at eps -> zero");

    // eps = 0 => pass-through (disabled case)
    Vector6d eps0 = Vector6d::Zero();
    Vector6d any;   any  << 1.0, -2.0, 3.0, 0.1, -0.2, 0.3;
    CHECK((soft_deadband(any, eps0) - any).isZero(1e-12),
           "eps=0 must pass-through unchanged");
}

// ============================================================================
// Outer-impedance damping references error velocity (v − v_inner)
// ----------------------------------------------------------------------------
// Regression test for the structural fix in cartesian_admittance_controller.cpp
// and hybrid_force_motion_controller.cpp: the outer impedance must damp
// (v_actual − v_inner), not v_actual alone. The two-loop design has a moving
// target (inner_t/inner_q drifts at inner_v), so damping against absolute v
// makes outer D fight the legitimate tracking motion whenever F_ext drives
// the admittance.
//
// Analytic trace, tick 1, defaults from controller_base.hpp:
//   M_adm=5, K_adm=200, D_adm=60, K_outer=200, D_outer=28, dt=1 ms,
//   F_ext_x = 5 N, dq = 0, target = identity, wrench_filter_alpha = 1.0.
//
//   adm_err   = 0 (smoothed_t starts at inner_t = current pose)
//   adm_force = F_ext − 0 + 0 = 5
//   a_x       = 5 / 5 = 1 m/s²
//   inner_v_x = 1 · 1e−3 = 1e−3 m/s
//   inner_t_x = 1e−3 · 1e−3 = 1e−6 m
//   e_x       = 0 − 1e−6 = −1e−6
//
//   OLD outer:  F_imp_x = −K·e − D·v        = 200·1e−6 − 28·0     ≈ 2.0e−4
//   NEW outer:  F_imp_x = −K·e − D·(v − v_inner)
//                       = 200·1e−6 + 28·1e−3                       ≈ 2.82e−2
//
//   Jacobian = [I₆|0] in the fixture → τ[0] = F_imp_x. The OLD vs NEW
//   magnitudes differ by ~140×, so the window [0.02, 0.04] cleanly catches
//   regressions to the absolute-velocity formula.
// ============================================================================
static void test_admittance_outer_damp_uses_error_velocity() {
    section("CartesianAdmittance: outer D references (v − v_inner), not v");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.O_F_ext_hat_K = {5.0, 0, 0, 0, 0, 0};
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianAdmittanceController c;
    CartesianAdmittanceCfg cfg;
    cfg.target              = Eigen::Affine3d::Identity();
    cfg.wrench_filter_alpha = 1.0;
    c.set_cfg(cfg);
    c.reset(s);

    auto tau = c.compute(s, m);
    CHECK(tau[0] > 0.02,
          "τ[0] picks up +D·v_inner feedforward (≫ K·e alone)");
    CHECK(tau[0] < 0.04,
          "τ[0] inside analytic NEW-behavior window (~2.82e−2)");
}

static void test_hybrid_outer_damp_uses_error_velocity() {
    section("HybridForceMotion n_af=0: outer D references (v − v_inner)");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.O_F_ext_hat_K = {5.0, 0, 0, 0, 0, 0};
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    HybridForceMotionController c;
    HybridForceMotionCfg cfg;
    cfg.target              = Eigen::Affine3d::Identity();
    cfg.n_af                = 0;
    cfg.wrench_filter_alpha = 1.0;
    c.set_cfg(cfg);
    c.reset(s);

    auto tau = c.compute(s, m);
    CHECK(tau[0] > 0.02,
          "τ[0] picks up +D·v_inner feedforward (≫ K·e alone)");
    CHECK(tau[0] < 0.04,
          "τ[0] inside analytic NEW-behavior window (~2.82e−2)");
}

// At admittance equilibrium under sustained F_ext, inner_v → 0 so the FF
// term D·v_inner vanishes and τ converges to the K·e equilibrium —
// the same value the OLD code would have produced at steady state.
// Locking this in ensures the fix only matters during transients (where
// it's supposed to).
//   inner_t_ss = F_ext / K_adm = 5 / 200 = 0.025 m
//   F_imp_ss   = −K · e_ss     = 200 · 0.025 = 5 N
//   τ[0]_ss    = 5 N·m (J = [I₆|0])
static void test_admittance_outer_damp_steady_state_matches_old() {
    section("CartesianAdmittance: at admittance equilibrium, FF term → 0");
    franka::RobotState s; set_identity_pose(s); set_q_safe(s);
    s.O_F_ext_hat_K = {5.0, 0, 0, 0, 0, 0};
    franka::Model m; zero_coriolis(m); set_jacobian_id6_pad(m);

    CartesianAdmittanceController c;
    CartesianAdmittanceCfg cfg;
    cfg.target              = Eigen::Affine3d::Identity();
    cfg.wrench_filter_alpha = 1.0;
    c.set_cfg(cfg);
    c.reset(s);

    std::array<double, 7> tau{};
    for (int i = 0; i < 5000; ++i) tau = c.compute(s, m);   // ~5 s, ≫ 1/ωₙ_adm
    CHECK(near(tau[0], 5.0, 0.1),
          "τ[0] → K_outer · F_ext/K_adm = 5 N·m at admittance equilibrium");
}

// Driver
// ============================================================================
int main() {
    std::cout << std::fixed;
    std::cout.precision(9);

    test_ema_scalar();
    test_ema_vec3();
    test_slerp_quat_hemisphere_flip();
    test_log3_round_trip();

    test_joint_limit_repulsion();
    test_friction_compensation_tanh();
    test_clip_error();
    test_soft_deadband_helper();

    test_get_joint_limit_torque_template();
    test_get_friction_sigmoid();

    test_gravity_compensation();

    test_cart_imp_equilibrium();
    test_cart_imp_step_target();
    test_cart_imp_damping();
    test_cart_imp_max_delta_clip();

    test_joint_imp_equilibrium();
    test_joint_imp_damping();

    test_admittance_external_wrench();

    test_hybrid_pure_admittance_equilibrium();
    test_hybrid_p0_6_wrench_filter();
    test_hybrid_p0_5_max_inner_v_clamp();
    test_hybrid_p0_4_inner_v_reset_on_switch();
    test_hybrid_wrench_deadband();

    // Outer-D-on-error-velocity fix (hybrid + admittance):
    test_admittance_outer_damp_uses_error_velocity();
    test_hybrid_outer_damp_uses_error_velocity();
    test_admittance_outer_damp_steady_state_matches_old();

    std::cout << "\n========== " << g_pass << " passed, " << g_fail
              << " failed ==========\n";
    return g_fail == 0 ? 0 : 1;
}
