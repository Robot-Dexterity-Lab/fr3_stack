// fr3 — NUC-side daemon.
//
// Single binary that owns:
//   - the libfranka connection (1 kHz RT control loop)
//   - controller dispatch (gravity_compensation / cartesian_impedance /
//     joint_impedance / cartesian_admittance / hybrid_force_motion)
//   - a ZMQ command receiver  (PULL,  CONFLATE)
//   - a ZMQ state publisher   (PUB,   CONFLATE), ~200 Hz
//
// Wire protocol: Cap'n Proto over ZMQ (proto/fr3.capnp, shared with the
// Python client). Wire field names (`idle`, `admittance`, `hybrid`) are
// preserved for client compatibility even though the C++ enum / class
// names were tightened up — see the dispatcher below for the mapping.
//
// Controllers themselves live under include/fr3_stack/controllers/;
// shared helpers under include/fr3_stack/utils/. main.cpp is meant to
// stay thin: ZMQ + capnp + dispatch + the RT callback. If you find
// yourself adding control math here, push it into the controller header.
//
// Build:  see CMakeLists.txt (uses capnp_generate_cpp to compile the schema)
// Run:    sudo ./fr3 --robot 192.168.1.11

#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <zmq.hpp>

#include <capnp/message.h>
#include <capnp/serialize.h>

#include "fr3.capnp.h"   // generated from the .capnp schema
#include <fr3_stack/motion_generator.hpp>
#include <fr3_stack/sensors/bota/wrench_source.hpp>
#include <fr3_stack/sensors/compensated_wrench_source.hpp>
#include <fr3_stack/sensors/payload_calib.hpp>

#include <fr3_stack/utils/controllers_common.hpp>
#include <fr3_stack/utils/log.hpp>
#include <fr3_stack/utils/streaming_target_interpolator.hpp>
#include <fr3_stack/controllers/controller_base.hpp>
#include <fr3_stack/controllers/gravity_compensation_controller.hpp>
#include <fr3_stack/controllers/cartesian_impedance_controller.hpp>
#include <fr3_stack/controllers/joint_impedance_controller.hpp>
#include <fr3_stack/controllers/cartesian_admittance_controller.hpp>
#include <fr3_stack/controllers/hybrid_force_motion_controller.hpp>

#include <memory>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// Daemon-local constants, signal handler, and PendingCommand
// ============================================================================
//
// Controllers, Cfgs, ControllerType, the Vector6/7 typedefs, joint-limit
// repulsion, friction comp, error-clip, and log helpers all live in
// include/fr3_stack/{controllers,utils}/*.hpp and are pulled in above.
// What stays in main.cpp is what's specific to the daemon I/O loop:
// the SIGINT flag, ZMQ tick / state-publish periods, and PendingCommand
// (which carries a unique_ptr<TargetGenerator> from motion_generator.hpp
// — that's a daemon-level concern, not a controller concern).

constexpr int kStatePeriodMs    = 5;      // 200 Hz state publish
constexpr int kCmdRecvTimeoutMs = 50;

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop = true; }

struct PendingCommand {
    ControllerType         type{ControllerType::GravityCompensation};
    IdleCfg                idle;
    CartesianImpedanceCfg  cart;
    JointImpedanceCfg      joint;
    CartesianAdmittanceCfg adm;
    HybridForceMotionCfg   hybrid;
    // Optional. Set by parse_command() for moveTo only; the RT thread takes
    // ownership via std::move on activation, calls start() with the live
    // pose, and feeds step() into cart_ctrl.set_target() each tick.
    // nullptr for non-moveTo commands → RT thread drops any active generator.
    std::unique_ptr<TargetGenerator> generator;
};

// ============================================================================
// Cap'n Proto ↔ struct
// ============================================================================

namespace {

template <int N>
Eigen::Matrix<double, N, 1> read_vec(::capnp::List<double>::Reader r,
                                      const char* field) {
    if (r.size() != size_t(N))
        throw std::runtime_error(std::string(field) + ": expected length "
                                  + std::to_string(N) + ", got "
                                  + std::to_string(r.size()));
    Eigen::Matrix<double, N, 1> v;
    for (int i = 0; i < N; ++i) v[i] = r[i];
    return v;
}

template <int N, typename Builder>
void write_vec(Builder builder, const Eigen::Matrix<double, N, 1>& v) {
    auto list = builder.initPos(N);  // not used; see specific writers below
    for (int i = 0; i < N; ++i) list.set(i, v[i]);
}

PendingCommand parse_command(const void* data, size_t size_bytes) {
    // Cap'n Proto requires word-aligned (8-byte) input. ZMQ's msg.data() makes
    // no such guarantee — the buffer comes from the message allocator and can
    // start at any byte. Copy into an aligned vector before parsing. We're on
    // the cmd_thread (not RT), so the allocation is free.
    if (size_bytes % sizeof(::capnp::word) != 0)
        throw std::runtime_error("command not a multiple of capnp word size");
    const size_t n_words = size_bytes / sizeof(::capnp::word);
    std::vector<::capnp::word> aligned(n_words);
    std::memcpy(aligned.data(), data, size_bytes);
    kj::ArrayPtr<const ::capnp::word> words(aligned.data(), n_words);
    ::capnp::FlatArrayMessageReader reader(words);
    auto cmd = reader.getRoot<Command>();
    auto cfg = cmd.getConfig();

    PendingCommand out;
    if (cmd.getTermination()) g_stop = true;

    switch (cfg.which()) {
        case Command::Config::IDLE: {
            auto c = cfg.getIdle();
            out.type = ControllerType::GravityCompensation;
            out.idle.d_rate       = read_vec<7>(c.getDRate(), "dRate");
            out.idle.use_friction = c.getUseFriction();
            break;
        }
        case Command::Config::CARTESIAN_IMPEDANCE: {
            auto c = cfg.getCartesianImpedance();
            out.type = ControllerType::CartesianImpedance;
            auto pos  = read_vec<3>(c.getTargetPos(),      "targetPos");
            auto quat = read_vec<4>(c.getTargetQuatXyzw(), "targetQuatXyzw");
            Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]);  // wxyz from xyzw
            q.normalize();
            out.cart.target = Eigen::Affine3d::Identity();
            out.cart.target.linear()      = q.toRotationMatrix();
            out.cart.target.translation() = pos;
            out.cart.K                     = read_vec<6>(c.getK(),     "k");
            out.cart.D                     = read_vec<6>(c.getD(),     "d");
            out.cart.q_null                = read_vec<7>(c.getQNull(), "qNull");
            out.cart.K_null                = c.getKNull();
            out.cart.D_null                = c.getDNull();
            out.cart.max_tau_null          = c.getMaxTauNull();
            out.cart.filter_alpha          = c.getFilterAlpha();
            out.cart.target_wrench         = read_vec<6>(c.getTargetWrench(), "targetWrench");
            out.cart.max_delta             = read_vec<6>(c.getMaxDelta(),     "maxDelta");
            out.cart.use_friction          = c.getUseFriction();
            out.cart.linear_interp         = c.getLinearInterp();
            out.cart.ema                   = c.getEma();
            break;
        }
        case Command::Config::JOINT_IMPEDANCE: {
            auto c = cfg.getJointImpedance();
            out.type = ControllerType::JointImpedance;
            out.joint.q_target     = read_vec<7>(c.getQTarget(), "qTarget");
            out.joint.K            = read_vec<7>(c.getKJoint(),  "kJoint");
            out.joint.D            = read_vec<7>(c.getDJoint(),  "dJoint");
            out.joint.filter_alpha = c.getFilterAlpha();
            out.joint.use_friction = c.getUseFriction();
            break;
        }
        case Command::Config::ADMITTANCE: {
            auto c = cfg.getAdmittance();
            out.type = ControllerType::CartesianAdmittance;
            auto pos  = read_vec<3>(c.getTargetPos(),      "targetPos");
            auto quat = read_vec<4>(c.getTargetQuatXyzw(), "targetQuatXyzw");
            Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]);
            q.normalize();
            out.adm.target = Eigen::Affine3d::Identity();
            out.adm.target.linear()      = q.toRotationMatrix();
            out.adm.target.translation() = pos;
            out.adm.M_adm        = read_vec<6>(c.getMAdm(),  "mAdm");
            out.adm.K_adm        = read_vec<6>(c.getKAdm(),  "kAdm");
            out.adm.D_adm        = read_vec<6>(c.getDAdm(),  "dAdm");
            out.adm.K            = read_vec<6>(c.getK(),     "k");
            out.adm.D            = read_vec<6>(c.getD(),     "d");
            out.adm.q_null       = read_vec<7>(c.getQNull(), "qNull");
            out.adm.K_null       = c.getKNull();
            out.adm.D_null       = c.getDNull();
            out.adm.max_tau_null = c.getMaxTauNull();
            out.adm.filter_alpha               = c.getFilterAlpha();
            out.adm.wrench_filter_alpha        = c.getWrenchFilterAlpha();
            out.adm.dq_filter_alpha            = c.getDqFilterAlpha();
            out.adm.output_torque_filter_alpha = c.getOutputTorqueFilterAlpha();
            out.adm.max_delta_tau              = c.getMaxDeltaTau();
            // errorClip: length 0 keeps default (all-zero = disabled). Length 6
            // applies per-axis. Anything else is a wire format error.
            {
                auto ec = c.getErrorClip();
                if (ec.size() == 6) {
                    out.adm.error_clip = read_vec<6>(ec, "errorClip");
                } else if (ec.size() != 0) {
                    throw std::runtime_error(
                        "admittance.errorClip: expected length 0 or 6, got "
                        + std::to_string(ec.size()));
                }
            }
            out.adm.use_friction               = c.getUseFriction();
            break;
        }
        case Command::Config::MOVE_TO: {
            auto c = cfg.getMoveTo();
            // moveTo is a thin wrapper: select the cartesian-impedance
            // controller + spin up a min-jerk generator. The generator's
            // start pose is captured in the RT thread at activation time
            // (pose_now), not here, so the trajectory always begins from
            // the live pose even if there's a small ZMQ delivery delay.
            out.type = ControllerType::CartesianImpedance;
            auto pos  = read_vec<3>(c.getTargetPos(),      "targetPos");
            auto quat = read_vec<4>(c.getTargetQuatXyzw(), "targetQuatXyzw");
            Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]);
            q.normalize();
            const double run_time = c.getRunTime();
            if (!(run_time > 0.0))
                throw std::runtime_error("moveTo: runTime must be > 0");

            // Seed cart cfg with goal pose as a fallback (if generator
            // somehow doesn't run, controller still holds at goal). The
            // generator overrides target each tick while it's active.
            out.cart.target = Eigen::Affine3d::Identity();
            out.cart.target.linear()      = q.toRotationMatrix();
            out.cart.target.translation() = pos;
            out.cart.K            = read_vec<6>(c.getK(),     "k");
            out.cart.D            = read_vec<6>(c.getD(),     "d");
            out.cart.q_null       = read_vec<7>(c.getQNull(), "qNull");
            out.cart.K_null       = c.getKNull();
            out.cart.D_null       = c.getDNull();
            out.cart.max_tau_null = c.getMaxTauNull();
            // Min-jerk output is already C² smooth — extra LP filtering
            // would only add lag, so disable the smoother.
            out.cart.filter_alpha = 1.0;

            out.generator = std::make_unique<MinJerkGenerator>(pos, q, run_time);
            break;
        }
        case Command::Config::HYBRID: {
            auto c = cfg.getHybrid();
            out.type = ControllerType::HybridForceMotion;
            auto pos  = read_vec<3>(c.getTargetPos(),      "targetPos");
            auto quat = read_vec<4>(c.getTargetQuatXyzw(), "targetQuatXyzw");
            Eigen::Quaterniond q(quat[3], quat[0], quat[1], quat[2]);
            q.normalize();
            out.hybrid.target = Eigen::Affine3d::Identity();
            out.hybrid.target.linear()      = q.toRotationMatrix();
            out.hybrid.target.translation() = pos;

            // Force-velocity decomposition.
            out.hybrid.n_af = std::clamp(int(c.getNAf()), 0, 6);
            auto tr_list = c.getTr();
            if (tr_list.size() != 36)
                throw std::runtime_error(
                    "tr: expected 36 elements, got " + std::to_string(tr_list.size()));
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 6; ++j)
                    out.hybrid.Tr(i, j) = tr_list[i * 6 + j];
            out.hybrid.target_wrench_Tr =
                read_vec<6>(c.getTargetWrenchTr(), "targetWrenchTr");

            // Inner admittance dynamics.
            out.hybrid.M_adm = read_vec<6>(c.getMAdm(), "mAdm");
            out.hybrid.K_adm = read_vec<6>(c.getKAdm(), "kAdm");
            out.hybrid.D_adm = read_vec<6>(c.getDAdm(), "dAdm");

            // Force-tracking PID.
            out.hybrid.P_trans = c.getPidPTrans();
            out.hybrid.I_trans = c.getPidITrans();
            out.hybrid.D_trans = c.getPidDTrans();
            out.hybrid.P_rot   = c.getPidPRot();
            out.hybrid.I_rot   = c.getPidIRot();
            out.hybrid.D_rot   = c.getPidDRot();
            out.hybrid.I_limit  = read_vec<6>(c.getPidILimit(), "pidILimit");
            out.hybrid.stiction = read_vec<6>(c.getStiction(),  "stiction");
            out.hybrid.max_spring_force  = c.getMaxSpringForce();
            out.hybrid.max_spring_torque = c.getMaxSpringTorque();

            // Outer cartesian impedance.
            out.hybrid.K = read_vec<6>(c.getK(), "k");
            out.hybrid.D = read_vec<6>(c.getD(), "d");
            out.hybrid.q_null      = read_vec<7>(c.getQNull(), "qNull");
            out.hybrid.K_null      = c.getKNull();
            out.hybrid.D_null      = c.getDNull();
            out.hybrid.max_tau_null = c.getMaxTauNull();
            out.hybrid.filter_alpha = c.getFilterAlpha();
            // F_ext EMA. Default 1.0 in schema → α=1 (pass-through) if the
            // sender omits the field, so the controller never freezes on
            // F_ext_filt initial value.
            out.hybrid.wrench_filter_alpha = c.getWrenchFilterAlpha();
            out.hybrid.dq_filter_alpha            = c.getDqFilterAlpha();
            out.hybrid.output_torque_filter_alpha = c.getOutputTorqueFilterAlpha();
            out.hybrid.max_delta_tau              = c.getMaxDeltaTau();
            // errorClip: length 0 keeps default (all-zero = disabled). Length 6
            // applies per-axis. Anything else is a wire format error.
            {
                auto ec = c.getErrorClip();
                if (ec.size() == 6) {
                    out.hybrid.error_clip = read_vec<6>(ec, "errorClip");
                } else if (ec.size() != 0) {
                    throw std::runtime_error(
                        "hybrid.errorClip: expected length 0 or 6, got "
                        + std::to_string(ec.size()));
                }
            }
            // wrenchDeadband: length 0 keeps default (all-zero = disabled).
            // Length 6 applies per-axis (N for trans, Nm for rot). Mirrors
            // errorClip's wire convention.
            {
                auto wd = c.getWrenchDeadband();
                if (wd.size() == 6) {
                    out.hybrid.wrench_deadband = read_vec<6>(wd, "wrenchDeadband");
                } else if (wd.size() != 0) {
                    throw std::runtime_error(
                        "hybrid.wrenchDeadband: expected length 0 or 6, got "
                        + std::to_string(wd.size()));
                }
            }
            out.hybrid.use_friction = c.getUseFriction();
            out.hybrid.linear_interp = c.getLinearInterp();
            out.hybrid.inner_v_filter_alpha = c.getInnerVFilterAlpha();

            // Soft contact-trip thresholds. Length 0 keeps the default (all
            // zeros = disabled). When non-empty it must match length 6/7;
            // an explicit length-6 of zeros also disables — same as omitting.
            auto ft = c.getForceThresholds();
            if (ft.size() == 6) {
                out.hybrid.force_thresholds = read_vec<6>(ft, "forceThresholds");
            } else if (ft.size() != 0) {
                throw std::runtime_error(
                    "forceThresholds: expected length 0 or 6, got "
                    + std::to_string(ft.size()));
            }
            auto tt = c.getTorqueThresholds();
            if (tt.size() == 7) {
                out.hybrid.torque_thresholds = read_vec<7>(tt, "torqueThresholds");
            } else if (tt.size() != 0) {
                throw std::runtime_error(
                    "torqueThresholds: expected length 0 or 7, got "
                    + std::to_string(tt.size()));
            }
            break;
        }
    }
    return out;
}

template <typename ListBuilder, typename Container>
void fill_list(ListBuilder list, const Container& c) {
    for (size_t i = 0; i < c.size(); ++i) list.set(i, c[i]);
}

kj::Array<::capnp::word> serialize_state(const franka::RobotState& s,
                                          const std::string& ctrl_name,
                                          bool running,
                                          const std::string& last_error,
                                          const Vector6d* wrench_ft_base,
                                          const Vector6d* wrench_ft_raw_base,
                                          bool            ft_compensated) {
    ::capnp::MallocMessageBuilder mb;
    auto st = mb.initRoot<State>();

    Eigen::Affine3d T(Eigen::Matrix4d::Map(s.O_T_EE.data()));
    Eigen::Quaterniond R(T.linear());
    Eigen::Vector3d p = T.translation();

    st.setController(ctrl_name);
    {
        auto l = st.initPos(3);
        l.set(0, p.x()); l.set(1, p.y()); l.set(2, p.z());
    }
    {
        auto l = st.initQuatXyzw(4);
        l.set(0, R.x()); l.set(1, R.y()); l.set(2, R.z()); l.set(3, R.w());
    }
    fill_list(st.initQ (7),  s.q);
    fill_list(st.initDq(7),  s.dq);
    fill_list(st.initWrenchExt(6), s.O_F_ext_hat_K);

    if (wrench_ft_base) {
        auto l = st.initWrenchFt(6);
        for (int i = 0; i < 6; ++i) l.set(i, (*wrench_ft_base)[i]);
    }
    if (wrench_ft_raw_base) {
        auto l = st.initWrenchFtRaw(6);
        for (int i = 0; i < 6; ++i) l.set(i, (*wrench_ft_raw_base)[i]);
    }
    // No FT source / no fresh frame yet → leave wrench lists as default empty.
    // Python client maps empty → State.wrench_ft / wrench_ft_raw = None.
    st.setFtCompensated(ft_compensated);

    st.setTimestamp(s.time.toSec());
    st.setRunning(running);
    st.setLastError(last_error);

    return ::capnp::messageToFlatArray(mb);
}

std::array<double, 7> rate_limit(const std::array<double, 7>& prev,
                                  const std::array<double, 7>& target) {
    std::array<double, 7> out{};
    for (int i = 0; i < 7; ++i) {
        double d = std::max(std::min(target[i] - prev[i], kDeltaTauMax), -kDeltaTauMax);
        out[i] = prev[i] + d;
    }
    return out;
}

}  // namespace

// ============================================================================
// Daemon
// ============================================================================

struct Args {
    std::string    robot_ip;
    int            cmd_port{5555};
    int            state_port{5556};
    ControllerType initial_controller{ControllerType::GravityCompensation};
    // Optional FT sensor source. `kind` selects the backend (currently:
    // "bota"); `config` is the vendor-specific config string passed straight
    // to make_wrench_source(). For Bota that's the full path to the driver
    // JSON, e.g. "/path/to/bota_driver_cpp/driver_config/ethercat.json".
    // Sensor wrench is raw (no payload compensator, no internal tare) — the
    // offline calibration's f_bias absorbs the sensor's electrical zero, so
    // the saved bias stays valid across daemon restarts at any pose.
    std::string    ft_sensor_kind;
    std::string    ft_sensor_config;
    // Explicit path to the payload calibration YAML. Empty ⇒ use
    // default_calib_path() (env vars / docker-compose convention). Useful
    // for out-of-Docker runs and tests that pin a specific YAML file.
    std::string    ft_calib_path;
    // When true (--ft-controllers-raw), force admittance / hybrid to consume
    // the *raw* sensor stream even if a payload calibration YAML loaded
    // successfully at boot. Useful for A/B comparisons or running closed-loop
    // force tasks before fully trusting a fresh calibration. The wire still
    // carries both compensated (`wrenchFt`) and raw (`wrenchFtRaw`) — only
    // the controllers' input changes.
    bool           ft_controllers_raw{false};
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(k + " needs a value");
            return argv[++i];
        };
        if (k == "--robot")           a.robot_ip   = next();
        else if (k == "--cmd-port")   a.cmd_port   = std::stoi(next());
        else if (k == "--state-port") a.state_port = std::stoi(next());
        else if (k == "--initial-controller") {
            std::string v = next();
            if      (v == "idle")                a.initial_controller = ControllerType::GravityCompensation;
            else if (v == "cartesian_impedance") a.initial_controller = ControllerType::CartesianImpedance;
            else if (v == "joint_impedance")     a.initial_controller = ControllerType::JointImpedance;
            else if (v == "admittance")          a.initial_controller = ControllerType::CartesianAdmittance;
            else if (v == "hybrid")              a.initial_controller = ControllerType::HybridForceMotion;
            else throw std::runtime_error(
                "--initial-controller must be idle|cartesian_impedance|joint_impedance|admittance|hybrid");
        }
        // Shorthand for --initial-controller. Useful from the CLI; the
        // long form stays the canonical one used by docker-compose.
        else if (k == "--cartesian" || k == "--cart")
            a.initial_controller = ControllerType::CartesianImpedance;
        else if (k == "--joint")
            a.initial_controller = ControllerType::JointImpedance;
        else if (k == "--admittance" || k == "--adm")
            a.initial_controller = ControllerType::CartesianAdmittance;
        else if (k == "--hybrid")
            a.initial_controller = ControllerType::HybridForceMotion;
        else if (k == "--idle")
            a.initial_controller = ControllerType::GravityCompensation;
        else if (k == "--ft-sensor-kind")   a.ft_sensor_kind   = next();
        else if (k == "--ft-sensor-config") a.ft_sensor_config = next();
        else if (k == "--ft-calib")         a.ft_calib_path    = next();
        else if (k == "--ft-controllers-raw") a.ft_controllers_raw = true;
        // Backwards-compat for the original Bota-only flags. Equivalent to
        // --ft-sensor-kind bota --ft-sensor-config <dir>/<json>.
        else if (k == "--bota-config-dir") {
            a.ft_sensor_kind = "bota";
            a.ft_sensor_config = next() + "/" +
                (a.ft_sensor_config.empty() ? std::string("ethercat.json")
                                            : a.ft_sensor_config);
        }
        else if (k == "--bota-driver-config") {
            // Deprecated: combined into --ft-sensor-config; only meaningful
            // alongside --bota-config-dir which already builds the full path.
            a.ft_sensor_config = next();
        }
        else if (k == "--help" || k == "-h") {
            std::cout << "usage: " << argv[0] << " --robot <ip>\n"
                      << "  [--cmd-port 5555]    workstation->NUC commands\n"
                      << "  [--state-port 5556]  NUC->workstation state\n"
                      << "  [--initial-controller idle|cartesian_impedance|joint_impedance|admittance|hybrid]\n"
                      << "  [--cartesian|--joint|--admittance|--hybrid|--idle]   shorthand for the above\n"
                      << "  [--ft-sensor-kind <kind>]    FT sensor backend (currently: bota)\n"
                      << "  [--ft-sensor-config <str>]   backend-specific config (Bota: full path to driver JSON)\n"
                      << "  [--ft-calib <path>]          payload calibration YAML; default: $FR3_FT_CALIB or\n"
                      << "                               /opt/fr3-stack/calib/ft_calibration.yaml (docker convention)\n"
                      << "  [--ft-controllers-raw]       feed RAW wrench to admittance/hybrid even if calib loaded\n"
                      << "                               (state.wrench_ft on the wire stays compensated)\n"
                      << "  [--bota-config-dir <path>]   deprecated; equivalent to "
                         "--ft-sensor-kind bota --ft-sensor-config <path>/ethercat.json\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown arg: " + k);
        }
    }
    if (a.robot_ip.empty()) throw std::runtime_error("--robot is required");
    return a;
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    try { args = parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::cerr << log_pfx() << "args error: " << e.what() << "\n"; return 1;
    }

    // ---- ZMQ -----------------------------------------------------------------
    zmq::context_t ctx{1};

    zmq::socket_t cmd_sock(ctx, zmq::socket_type::pull);
    cmd_sock.set(zmq::sockopt::conflate, 1);
    cmd_sock.set(zmq::sockopt::rcvtimeo, kCmdRecvTimeoutMs);
    cmd_sock.bind("tcp://*:" + std::to_string(args.cmd_port));

    zmq::socket_t state_sock(ctx, zmq::socket_type::pub);
    state_sock.set(zmq::sockopt::sndhwm, 1);
    state_sock.bind("tcp://*:" + std::to_string(args.state_port));

    std::cout << log_pfx() << "cmd   PULL  on tcp://*:" << args.cmd_port  << "\n";
    std::cout << log_pfx() << "state PUB   on tcp://*:" << args.state_port << "\n";
    std::cout << log_pfx() << "connecting to robot at " << args.robot_ip << " ...\n";
    std::cout.flush();

    // ---- libfranka -----------------------------------------------------------
    franka::Robot robot(args.robot_ip);
    franka::Model model = robot.loadModel();

    // Clear any latent reflex/error state from a previous abrupt shutdown.
    // Without this, the daemon restart-loops with "command not possible in
    // the current mode (Reflex)" and the user has to click recovery in Desk.
    try {
        robot.automaticErrorRecovery();
    } catch (const std::exception& e) {
        std::cerr << log_pfx() << "ERROR: auto error-recovery failed: "
                  << e.what() << "\n"
                  << log_pfx() << "  → open Franka Desk and click "
                     "'Automatic error recovery', then restart.\n";
        return 1;
    }

    // Collision thresholds — Nm on joints, N/Nm on Cartesian. 20 was too
    // tight: tiny gravity drift on first RT tick tripped cartesian_reflex.
    // 80 matches Franka's own example values; still well below joint limits.
    robot.setCollisionBehavior(
        {{80, 80, 80, 80, 80, 80, 80}}, {{80, 80, 80, 80, 80, 80, 80}},
        {{80, 80, 80, 80, 80, 80}},     {{80, 80, 80, 80, 80, 80}});

    franka::RobotState s0 = robot.readOnce();
    std::cout << log_pfx() << "connected to " << args.robot_ip
              << " (FCI server v" << robot.serverVersion() << ")\n";

    // Initial pose summary — useful for confirming joint/EE coordinates are
    // sane (the typical "is it pointed where I think?" check).
    {
        Eigen::Map<const Eigen::Matrix4d> T(s0.O_T_EE.data());
        Eigen::Vector3d    p0 = T.block<3, 1>(0, 3);
        Eigen::Quaterniond q0(T.block<3, 3>(0, 0));

        std::cout << log_pfx() << "q   =";
        for (int i = 0; i < 7; ++i) std::cout << " " << s0.q[i];
        std::cout << "\n";
        std::cout << log_pfx() << "pos = [" << p0.x() << ", " << p0.y()
                  << ", " << p0.z() << "] m\n";
        std::cout << log_pfx() << "quat= [" << q0.x() << ", " << q0.y()
                  << ", " << q0.z() << ", " << q0.w() << "]\n";
    }
    const char* mode_name =
        args.initial_controller == ControllerType::CartesianImpedance  ? "cartesian_impedance" :
        args.initial_controller == ControllerType::JointImpedance      ? "joint_impedance"     :
        args.initial_controller == ControllerType::CartesianAdmittance ? "admittance"          :
        args.initial_controller == ControllerType::HybridForceMotion   ? "hybrid"              :
                                                                         "idle";
    if (args.initial_controller == ControllerType::GravityCompensation)
        std::cout << log_pfx() << "mode = idle (gravity-comp + joint damping, hand-guidable)\n";
    else
        std::cout << log_pfx() << "mode = " << mode_name
                  << " (anchored at current pose/q — arm holds in place)\n";
    std::cout << log_pfx() << "entering RT loop @ 1 kHz\n";
    std::cout.flush();

    // ---- Shared state across threads ----------------------------------------
    PendingCommand     pending{};
    bool               pending_dirty{false};
    std::mutex         pending_mu;

    // For non-idle --initial-controller, the spring target must be the EE
    // pose / q at the first RT tick. We can't anchor with s0 here: readOnce()
    // ran hundreds of ms before robot.control() will start (FT init, thread
    // setup), during which the arm drifts a couple mm under FCI's internal
    // gravity-comp. Using s0 as the anchor snaps the arm back to that stale
    // pose once impedance kicks in. Instead we just set a flag and have the
    // RT callback build `pending` from the LIVE state on its first tick —
    // see the boot_anchor_pending handler in cb() below.
    bool boot_anchor_pending =
        args.initial_controller != ControllerType::GravityCompensation;

    franka::RobotState latest_state    = s0;
    std::string        latest_ctrl_name = mode_name;
    std::mutex         state_mu;

    std::atomic<bool>  rt_running{false};
    std::atomic<bool>  rt_stop_requested{false};
    std::string        rt_last_error;
    std::mutex         err_mu;

    // ---- Optional FT sensor -------------------------------------------------
    // Started here (before the threads) so the worker is publishing frames
    // before the libfranka callback first runs, and so pub_thread can safely
    // capture-and-read ft_src by reference. The factory picks a backend based
    // on --ft-sensor-kind; main.cpp doesn't need the vendor headers.
    //
    // If a payload calibration YAML exists at startup, the source is wrapped
    // in a CompensatedWrenchSource so admittance / hybrid controllers and the
    // wrenchFt state field all see gravity+bias-compensated wrench. The raw
    // signal is preserved via comp_src->read_raw() and published as
    // wrenchFtRaw, so consumers can still inspect the uncompensated data.
    std::unique_ptr<WrenchSource>            ft_src;
    CompensatedWrenchSource*                 comp_src = nullptr;  // non-owning
    bool                                     ft_compensated = false;
    // Constant rotation from libfranka's EE frame to the bota sensor frame.
    // Defaults to identity; overridden from the loaded calib YAML if present.
    // Used both by the state publisher's wrench rotation (R_O_sensor =
    // R_O_EE · R_ee_sensor) and indirectly via comp_src for gravity comp.
    // Identity is correct only when libfranka's EE coincides with the sensor
    // mounting orientation — see docs/postmortems/ft_calibration_2026-05-10.md.
    Eigen::Matrix3d                          ft_R_ee_sensor = Eigen::Matrix3d::Identity();
    if (!args.ft_sensor_kind.empty()) {
        std::cout << log_pfx() << "ft sensor: kind=" << args.ft_sensor_kind
                  << "  config=" << args.ft_sensor_config << "\n";
        try {
            ft_src = make_wrench_source(args.ft_sensor_kind,
                                         args.ft_sensor_config);
            ft_src->start();
            std::cout << log_pfx() << "ft sensor '" << ft_src->kind()
                      << "' ready (worker thread @ ~1 kHz)\n";

            // Try the calibration YAML. Missing file ⇒ run uncompensated;
            // malformed file ⇒ throw, caught below, fall back to raw.
            // --ft-calib overrides the default lookup chain (env vars +
            // docker convention).
            const std::string calib_path =
                args.ft_calib_path.empty() ? default_calib_path()
                                            : args.ft_calib_path;
            try {
                if (auto calib = load_payload_calib(calib_path)) {
                    std::cout << log_pfx()
                              << "ft calib loaded: " << calib_path << "\n"
                              << log_pfx()
                              << "  mass = " << calib->mass << " kg, "
                              << "com = [" << calib->com.x() << ", "
                              << calib->com.y() << ", " << calib->com.z()
                              << "] m\n"
                              << log_pfx()
                              << "  f_bias = [" << calib->f_bias.x() << ", "
                              << calib->f_bias.y() << ", " << calib->f_bias.z()
                              << "] N, "
                              << "t_bias = [" << calib->t_bias.x() << ", "
                              << calib->t_bias.y() << ", " << calib->t_bias.z()
                              << "] Nm\n";
                    if (calib->residual_force_N >= 0.0)
                        std::cout << log_pfx()
                                  << "  residual: f="
                                  << calib->residual_force_N << " N, "
                                  << "t=" << calib->residual_torque_Nm
                                  << " Nm\n";
                    ft_R_ee_sensor = calib->R_ee_sensor;
                    {
                        const Eigen::Vector3d rpy_deg =
                            ft_R_ee_sensor.eulerAngles(2, 1, 0).reverse() * (180.0 / M_PI);
                        std::cout << log_pfx()
                                  << "  rpy_ee_sensor (deg) = ["
                                  << rpy_deg.x() << ", " << rpy_deg.y()
                                  << ", " << rpy_deg.z() << "]"
                                  << ((ft_R_ee_sensor.isApprox(Eigen::Matrix3d::Identity()))
                                       ? " (identity)\n" : "\n");
                    }
                    auto wrapped = std::make_unique<CompensatedWrenchSource>(
                        std::move(ft_src), *calib);
                    comp_src = wrapped.get();
                    ft_src   = std::move(wrapped);
                    ft_compensated = true;
                    std::cout << log_pfx()
                              << "ft compensation: ON — controllers + "
                              << "wrenchFt see gravity+bias-subtracted wrench; "
                              << "wrenchFtRaw carries the uncompensated stream.\n";
                } else {
                    std::cout << log_pfx()
                              << "ft compensation: OFF — no calib at "
                              << (calib_path.empty() ? "(no $HOME)" : calib_path)
                              << ". Run `fr3-ft-calibrate` then restart the "
                              << "daemon to enable.\n";
                }
            } catch (const std::exception& e) {
                std::cerr << log_pfx()
                          << "ERROR: ft calib parse failed: " << e.what()
                          << " — running uncompensated\n";
            }
        } catch (const std::exception& e) {
            std::cerr << log_pfx() << "ERROR: ft sensor: " << e.what()
                      << " — falling back to libfranka O_F_ext_hat_K\n";
            ft_src.reset();
            comp_src = nullptr;
            ft_compensated = false;
        }
    }

    // ---- Command receiver thread ---------------------------------------------
    std::thread cmd_thread([&]() {
        // Throttle parse-error logging — a buggy client streaming the same
        // bad capnp at high rate would otherwise flood stderr. The
        // `rt_last_error` field is still updated every time so clients see
        // the latest error on the wire; only stdout is rate-limited.
        LogThrottle bad_cmd_throttle;
        while (!g_stop.load()) {
            zmq::message_t msg;
            auto res = cmd_sock.recv(msg, zmq::recv_flags::none);
            if (!res) continue;            // timeout
            try {
                PendingCommand c = parse_command(msg.data(), msg.size());
                std::lock_guard<std::mutex> lk(pending_mu);
                // Move (PendingCommand carries a unique_ptr<TargetGenerator>
                // for moveTo). If a previous pending was never picked up by
                // the RT thread, its generator is destroyed here — that's
                // the correct behavior: a newer command supersedes the old.
                pending = std::move(c);
                pending_dirty = true;
                // A user command supersedes the boot anchor: the RT loop
                // must not overwrite `pending.<mode>.target` with the live
                // pose once the user has explicitly chosen one.
                boot_anchor_pending = false;
            } catch (const std::exception& e) {
                const std::string err_msg =
                    std::string("bad command: ") + e.what();
                {
                    std::lock_guard<std::mutex> lk(err_mu);
                    rt_last_error = err_msg;
                }
                bad_cmd_throttle.maybe_log(std::cerr, err_msg);
            }
        }
    });

    // ---- State publisher thread ----------------------------------------------
    std::thread pub_thread([&]() {
        while (!g_stop.load()) {
            franka::RobotState snapshot;
            std::string ctrl_name;
            {
                std::lock_guard<std::mutex> lk(state_mu);
                snapshot  = latest_state;
                ctrl_name = latest_ctrl_name;
            }
            std::string err;
            {
                std::lock_guard<std::mutex> lk(err_mu);
                err = rt_last_error;
            }
            // FT sensor wrench: same rotation as the admittance controller
            // does. Sensor frame → base via R_O_EE from the snapshot. Skip
            // when no source is attached or no frame has arrived yet
            // (serialize_state then leaves wrenchFt as an empty list, which
            // the Python client turns into None).
            //
            // When the source is wrapped in CompensatedWrenchSource, ft_src->
            // read() returns the *compensated* sensor-frame wrench and
            // comp_src->read_raw() returns the uncompensated one. We publish
            // both rotated to base — wrenchFt (compensated when calib loaded,
            // raw otherwise) and wrenchFtRaw (always raw).
            Vector6d wrench_ft_base;
            Vector6d wrench_ft_raw_base;
            const Vector6d* wfp     = nullptr;
            const Vector6d* wfp_raw = nullptr;
            if (ft_src) {
                // R_O_sensor = R_O_EE · R_ee_sensor accounts for the constant
                // rigid offset between libfranka's EE frame and the actual bota
                // mounting orientation (e.g. Desk hand inserts a -45° about z
                // into O_T_EE that the sensor doesn't share). Defaults to
                // identity when no calib is loaded — same behavior as before.
                const Eigen::Matrix3d R_O_EE(
                    Eigen::Matrix4d::Map(snapshot.O_T_EE.data())
                        .block<3, 3>(0, 0));
                const Eigen::Matrix3d Rb = R_O_EE * ft_R_ee_sensor;
                Vector6d F_sensor;
                if (ft_src->read(F_sensor)) {
                    wrench_ft_base.head<3>() = Rb * F_sensor.head<3>();
                    wrench_ft_base.tail<3>() = Rb * F_sensor.tail<3>();
                    wfp = &wrench_ft_base;
                }
                if (comp_src) {
                    Vector6d F_raw;
                    if (comp_src->read_raw(F_raw)) {
                        wrench_ft_raw_base.head<3>() = Rb * F_raw.head<3>();
                        wrench_ft_raw_base.tail<3>() = Rb * F_raw.tail<3>();
                        wfp_raw = &wrench_ft_raw_base;
                    }
                } else if (wfp) {
                    // No compensation in play: raw == compensated. Mirror it
                    // so consumers always have wrenchFtRaw available.
                    wrench_ft_raw_base = wrench_ft_base;
                    wfp_raw            = &wrench_ft_raw_base;
                }
            }
            auto words = serialize_state(snapshot, ctrl_name,
                                         rt_running.load(), err,
                                         wfp, wfp_raw, ft_compensated);
            auto bytes = words.asBytes();
            zmq::message_t msg(bytes.begin(), bytes.size());
            state_sock.send(msg, zmq::send_flags::dontwait);
            std::this_thread::sleep_for(std::chrono::milliseconds(kStatePeriodMs));
        }
    });

    // ---- RT control thread ---------------------------------------------------
    GravityCompensationController grav_ctrl;
    CartesianImpedanceController  cart_ctrl;
    JointImpedanceController      joint_ctrl;
    CartesianAdmittanceController adm_ctrl;
    HybridForceMotionController   hybrid_ctrl;
    // Default: controllers read whatever ft_src exposes — that's the
    // compensated stream when comp_src is in play, otherwise raw. With
    // --ft-controllers-raw the admittance / hybrid loops bypass the
    // decorator and consume raw directly (state publisher continues to
    // publish both columns regardless — only the controller input changes).
    WrenchSource* ctrl_src = ft_src.get();
    if (comp_src && args.ft_controllers_raw) {
        ctrl_src = comp_src->inner();
        std::cout << log_pfx() << "ft compensation: controllers=RAW "
                                  "(--ft-controllers-raw); wire stream "
                                  "stays compensated.\n";
    }
    adm_ctrl.set_wrench_source(ctrl_src);                // nullptr → fallback
    hybrid_ctrl.set_wrench_source(ctrl_src);
    Controller* active = &grav_ctrl;
    active->reset(s0);
    std::array<double, 7> tau_prev = s0.tau_J_d;

    // Active motion generator (only for moveTo, drives cart_ctrl). Owned by
    // the RT thread; ownership transferred from `pending.generator` under
    // pending_mu. nullptr ⇒ no generator running, controller uses the user-
    // supplied target directly.
    std::unique_ptr<TargetGenerator> active_gen;

    // Linear interpolator that bridges low-rate streaming cartesian_impedance
    // commands to the 1 kHz tick. See header for the rationale (TLDR: a
    // 200 Hz step train into the LP filter chatters joint 0 audibly; LERP
    // between consecutive received targets removes the step train).
    fr3_stack::StreamingTargetInterpolator stream_interp;

    // Mirror of the active hybrid command's contact-trip thresholds. Tracked
    // separately so the RT-callback's per-tick check doesn't have to touch
    // the controller's private cfg. Updated under pending_mu when a new
    // hybrid cmd lands; cleared whenever the active controller is not hybrid.
    // Zero entry → that axis unbounded; all-zero → feature off.
    Vector6d   hybrid_force_thresh{Vector6d::Zero()};
    Vector7d   hybrid_torque_thresh{Vector7d::Zero()};
    LogThrottle hybrid_trip_throttle;
    auto rt_now = []() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    auto select = [&](ControllerType t) -> Controller* {
        switch (t) {
            case ControllerType::GravityCompensation: return &grav_ctrl;
            case ControllerType::CartesianImpedance:  return &cart_ctrl;
            case ControllerType::JointImpedance:      return &joint_ctrl;
            case ControllerType::CartesianAdmittance: return &adm_ctrl;
            case ControllerType::HybridForceMotion:   return &hybrid_ctrl;
        }
        return &grav_ctrl;
    };

    auto cb = [&](const franka::RobotState& s, franka::Duration) -> franka::Torques {
        // Pump R_O_EE into the compensator before any read() runs this tick.
        // try_lock based, so it never blocks even if pub_thread happens to be
        // mid-read on the same source.
        if (comp_src) {
            const Eigen::Matrix3d R(
                Eigen::Matrix4d::Map(s.O_T_EE.data()).block<3, 3>(0, 0));
            comp_src->set_orientation(R);
        }
        // try_lock pending: never blocks RT.
        {
            std::unique_lock<std::mutex> lk(pending_mu, std::try_to_lock);
            // Boot anchor: if --initial-controller != idle and the user
            // hasn't sent anything yet, build `pending` from the LIVE robot
            // state. Done here (not at startup using s0) because the
            // few-hundred ms gap between readOnce() and the first cb() lets
            // the arm drift a couple mm under FCI's internal gravity-comp;
            // using s0 as the spring anchor would snap the arm back to that
            // stale pose on the first impedance tick ("startup teleport").
            if (lk.owns_lock() && boot_anchor_pending && !pending_dirty) {
                Eigen::Affine3d T_live(Eigen::Matrix4d::Map(s.O_T_EE.data()));
                if (args.initial_controller == ControllerType::CartesianImpedance) {
                    pending.type        = ControllerType::CartesianImpedance;
                    pending.cart.target = T_live;
                } else if (args.initial_controller == ControllerType::JointImpedance) {
                    pending.type = ControllerType::JointImpedance;
                    for (int i = 0; i < 7; ++i) pending.joint.q_target[i] = s.q[i];
                } else if (args.initial_controller == ControllerType::CartesianAdmittance) {
                    pending.type       = ControllerType::CartesianAdmittance;
                    pending.adm.target = T_live;
                } else if (args.initial_controller == ControllerType::HybridForceMotion) {
                    // Defaults: n_af=0, target_wrench=0, Tr=I → pure
                    // admittance with zero force command; the arm holds at
                    // T_live until the client sends a real HybridCmd.
                    pending.type          = ControllerType::HybridForceMotion;
                    pending.hybrid.target = T_live;
                }
                pending_dirty       = true;
                boot_anchor_pending = false;
            }
            if (lk.owns_lock() && pending_dirty) {
                if      (pending.type == ControllerType::GravityCompensation) grav_ctrl.set_cfg(pending.idle);
                else if (pending.type == ControllerType::CartesianImpedance) {
                    cart_ctrl.set_cfg(pending.cart);
                    // Streaming path: feed the interpolator. moveTo carries
                    // a generator and is handled below — its smooth trajectory
                    // already targets every tick, so don't push to interp.
                    // Skip when the user disables LERP via cfg.linear_interp;
                    // also reset state so a later re-enable doesn't blend
                    // against stale prev/latest from before the toggle.
                    if (pending.generator || !pending.cart.linear_interp) {
                        stream_interp.reset();
                    } else {
                        stream_interp.push(pending.cart.target, rt_now());
                    }
                }
                else if (pending.type == ControllerType::JointImpedance)      joint_ctrl.set_cfg(pending.joint);
                else if (pending.type == ControllerType::CartesianAdmittance) adm_ctrl.set_cfg(pending.adm);
                else if (pending.type == ControllerType::HybridForceMotion) {
                    hybrid_ctrl.set_cfg(pending.hybrid);
                    hybrid_force_thresh  = pending.hybrid.force_thresholds;
                    hybrid_torque_thresh = pending.hybrid.torque_thresholds;
                    // Streaming LERP path (same bridge cart uses). Skip when
                    // the user disables it via cfg.linear_interp; also reset
                    // state so a later re-enable doesn't blend against stale
                    // prev/latest from before the toggle.
                    if (!pending.hybrid.linear_interp) {
                        stream_interp.reset();
                    } else {
                        stream_interp.push(pending.hybrid.target, rt_now());
                    }
                }
                // Hybrid thresholds belong to that controller — drop them
                // the moment we leave so a residual cap from a prior hybrid
                // cmd can't trip a subsequent cart-imp / admittance session.
                if (pending.type != ControllerType::HybridForceMotion) {
                    hybrid_force_thresh.setZero();
                    hybrid_torque_thresh.setZero();
                }
                Controller* next = select(pending.type);
                if (next != active) {
                    next->reset(s);
                    active = next;
                    // Stale prev/latest from a previous cart session
                    // shouldn't bleed into a new one. The first push after
                    // re-entering cart_ctrl will re-prime.
                    stream_interp.reset();
                }
                // Generator ownership transfer. moveTo arrives with a
                // generator; any other command type supersedes the in-flight
                // trajectory (active_gen.reset() drops it). start() is
                // called here — at activation, with the live pose — so the
                // trajectory always begins from where the arm actually is.
                if (pending.generator) {
                    active_gen = std::move(pending.generator);
                    Eigen::Affine3d T_now(Eigen::Matrix4d::Map(s.O_T_EE.data()));
                    active_gen->start(T_now.translation(),
                                       Eigen::Quaterniond(T_now.linear()));
                } else {
                    active_gen.reset();
                }
                pending_dirty = false;
            }
        }

        // Advance the generator (if any) and stream the resulting target into
        // the cart-imp controller. We only drive cart_ctrl: moveTo always
        // selects it, and other controllers have their own target sources.
        if (active_gen && active == &cart_ctrl) {
            CartesianTarget t = active_gen->step(0.001);
            Eigen::Affine3d T;
            T.linear()      = t.quat.toRotationMatrix();
            T.translation() = t.pos;
            cart_ctrl.set_target(T);
            // Drop the generator the moment it finishes — controller's cfg
            // already holds the goal (last step output), so subsequent ticks
            // just hold there until the user sends another command.
            if (active_gen->finished()) active_gen.reset();
        } else if (active == &cart_ctrl
                   && cart_ctrl.linear_interp_enabled()
                   && stream_interp.primed()) {
            // Streaming path. Bridge any client rate up to the 1 kHz daemon
            // tick by linearly interpolating between the two most recent
            // received targets. See StreamingTargetInterpolator header.
            // Skipped when cfg.linear_interp is false — controller then
            // tracks the raw cfg_.target directly (set via set_cfg above).
            cart_ctrl.set_target(stream_interp.evaluate(rt_now()));
        } else if (active == &hybrid_ctrl
                   && hybrid_ctrl.linear_interp_enabled()
                   && stream_interp.primed()) {
            // Same bridge for the hybrid outer-pose target. Force-controlled
            // axes ignore the pose component (rigidly tracked via HFVC) so
            // the LERP only smooths the velocity-controlled axes — which is
            // exactly where the step-train chatter showed up.
            hybrid_ctrl.set_target(stream_interp.evaluate(rt_now()));
        }
        // Publish state for the publisher thread.
        {
            std::unique_lock<std::mutex> lk(state_mu, std::try_to_lock);
            if (lk.owns_lock()) {
                latest_state     = s;
                latest_ctrl_name = active->name();
            }
        }

        // Hybrid soft-trip: per-call thresholds enforced here because libfranka
        // forbids re-arming setCollisionBehavior while control() is live.
        // Use libfranka's own external-force / external-torque estimates —
        // noisy, but adequate as a safety floor (the user is expected to
        // budget headroom). A trip switches active to gravity-comp, surfaces
        // the cause via state.lastError, and lets the arm coast on gravity
        // so the operator can recover without a full daemon restart.
        if (active == &hybrid_ctrl
            && (hybrid_force_thresh.array().abs().sum() > 0.0
             || hybrid_torque_thresh.array().abs().sum() > 0.0)) {
            int trip_idx = -1;
            const char* trip_kind = nullptr;
            double trip_val = 0.0, trip_thr = 0.0;
            for (int i = 0; i < 6; ++i) {
                const double thr = hybrid_force_thresh[i];
                if (thr > 0.0 && std::abs(s.O_F_ext_hat_K[i]) > thr) {
                    trip_idx = i; trip_kind = "force";
                    trip_val = s.O_F_ext_hat_K[i]; trip_thr = thr;
                    break;
                }
            }
            if (trip_idx < 0) {
                for (int j = 0; j < 7; ++j) {
                    const double thr = hybrid_torque_thresh[j];
                    if (thr > 0.0 && std::abs(s.tau_ext_hat_filtered[j]) > thr) {
                        trip_idx = j; trip_kind = "torque";
                        trip_val = s.tau_ext_hat_filtered[j]; trip_thr = thr;
                        break;
                    }
                }
            }
            if (trip_idx >= 0) {
                std::string msg = std::string("hybrid: ") + trip_kind
                    + " threshold tripped on axis " + std::to_string(trip_idx)
                    + " (|" + std::to_string(trip_val) + "| > "
                    + std::to_string(trip_thr) + ") — switched to idle";
                {
                    std::lock_guard<std::mutex> lk(err_mu);
                    rt_last_error = msg;
                }
                hybrid_trip_throttle.maybe_log(std::cerr, msg);
                grav_ctrl.reset(s);
                active = &grav_ctrl;
                hybrid_force_thresh.setZero();
                hybrid_torque_thresh.setZero();
            }
        }

        std::array<double, 7> tau_target = active->compute(s, model);
        std::array<double, 7> tau_out    = rate_limit(tau_prev, tau_target);
        tau_prev = tau_out;

        franka::Torques out(tau_out);
        if (rt_stop_requested.load(std::memory_order_relaxed) || g_stop.load())
            return franka::MotionFinished(out);
        return out;
    };

    rt_running = true;
    std::thread rt_thread([&]() {
        try {
            robot.control(cb);
        } catch (const std::exception& e) {
            std::string what = e.what();
            {
                std::lock_guard<std::mutex> lk(err_mu);
                rt_last_error = what;
            }

            // Strip the noisy "libfranka: " prefix and stop at first newline
            // so the cause line stays short and grep-friendly.
            std::string cause = what;
            if (cause.rfind("libfranka: ", 0) == 0) cause = cause.substr(11);
            auto nl = cause.find('\n');
            if (nl != std::string::npos) cause = cause.substr(0, nl);

            // One-line actionable hint based on the error text.
            const char* hint = nullptr;
            if (cause.find("Reflex") != std::string::npos
             || cause.find("reflex") != std::string::npos
             || cause.find("control_command_success_rate") != std::string::npos)
                hint = "robot is in Reflex state. Open Franka Desk → 'Automatic "
                       "error recovery', or use FR3_INITIAL_CONTROLLER=idle.";
            else if (cause.find("communication") != std::string::npos
                  || cause.find("timeout")       != std::string::npos)
                hint = "lost FCI connection. Check Desk → Settings → Activate FCI.";
            else if (cause.find("joint")  != std::string::npos
                  && cause.find("limit")  != std::string::npos)
                hint = "joint at hard limit. Move arm into a valid pose via Desk.";

            std::cerr << log_pfx() << "ERROR: RT loop aborted\n"
                      << log_pfx() << "  cause: " << cause << "\n";
            if (hint)
                std::cerr << log_pfx() << "  hint:  " << hint << "\n";

            // Last-known robot state at abort — dq is the smoking gun for
            // joint_velocity_violation, q for joint_position / cartesian
            // limit reflexes. Printed unconditionally so it shows up for
            // any reflex type without us having to enumerate them.
            franka::RobotState s_at_abort;
            bool have_state = false;
            {
                std::unique_lock<std::mutex> lk(state_mu, std::try_to_lock);
                if (lk.owns_lock()) {
                    s_at_abort  = latest_state;
                    have_state  = true;
                }
            }
            if (have_state) {
                std::cerr << log_pfx() << "  q   =";
                for (int i = 0; i < 7; ++i)
                    std::cerr << " " << s_at_abort.q[i];
                std::cerr << "\n";
                std::cerr << log_pfx() << "  dq  =";
                for (int i = 0; i < 7; ++i)
                    std::cerr << " " << s_at_abort.dq[i];
                std::cerr << "\n";
            }
        }
        rt_running = false;
    });

    while (!g_stop.load() && rt_running.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    {
        std::string err;
        {
            std::lock_guard<std::mutex> lk(err_mu);
            err = rt_last_error;
        }
        // RT-thread already printed the full error+hint; here just say why
        // we're stopping so the log isn't spammed twice with the long message.
        if (!err.empty())
            std::cout << log_pfx() << "shutting down (rt loop aborted, see above)\n";
        else if (g_stop.load())
            std::cout << log_pfx() << "shutting down (signal received)\n";
        else
            std::cout << log_pfx() << "shutting down\n";
    }
    rt_stop_requested = true;
    if (rt_thread.joinable())  rt_thread.join();
    g_stop = true;
    if (cmd_thread.joinable()) cmd_thread.join();
    if (pub_thread.joinable()) pub_thread.join();
    return 0;
}
