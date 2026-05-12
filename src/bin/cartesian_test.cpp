// Real-machine experiment harness for the Cartesian impedance controller.
// Same controller class as the daemon — different updater + CSV logging.
//
// Modes (--mode):
//   hold     : target locked at startup pose; for stability sanity check
//              and disturbance-rejection (push the EE by hand)
//   osc      : sinusoid on --axis (--amp, --freq) — mirrors the original
//              cartesian_impedance demo when run with defaults
//   step     : 2 s settle, then a position step (--step) on --axis
//   disturb  : same target as hold but assumes you'll push by hand —
//              the point of this mode is the CSV log of EE pose + F_ext
//
// Stiffness override: --k kx,ky,kz,krx,kry,krz. Damping is derived from
// --damp-ratio using D_i = 2*sqrt(K_i)*ratio (M_eff implicitly = 1, which
// is also what the controller defaults assume — K=200 D=28 → ratio ≈ 0.99).
// D is only recomputed when --k or --damp-ratio is given, so a no-arg run
// reproduces the default tuning exactly.
//
// CSV: 1 kHz ring-buffer in the RT callback, flushed at exit. No file I/O
// on the RT thread. Layout:
//   t,p_d_x,p_d_y,p_d_z,p_x,p_y,p_z,Fx,Fy,Fz,Tx,Ty,Tz
//
// Build: cmake -B build-test -DFR3_BUILD_TEACHING=ON
//        cmake --build build-test --target cartesian_test
// Run:   sudo ./cartesian_test <robot-ip> [options]

#include <franka/exception.h>
#include <franka/robot.h>

#include <fr3_stack/controllers/cartesian_impedance_controller.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

enum class Mode { Hold, Osc, Step, Disturb };

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::Hold:    return "hold";
        case Mode::Osc:     return "osc";
        case Mode::Step:    return "step";
        case Mode::Disturb: return "disturb";
    }
    return "?";
}

struct Args {
    std::string                 ip;
    Mode                        mode{Mode::Hold};
    int                         axis{2};      // 0=x 1=y 2=z
    double                      amp{0.05};
    double                      freq{0.25};
    double                      step{0.05};
    double                      duration{0.0};   // 0 = manual SIGINT
    bool                        have_K{false};
    Eigen::Matrix<double, 6, 1> K;
    double                      damp_ratio{0.9};
    bool                        damp_ratio_set{false};
    std::string                 csv_path;
};

void usage(const char* prog) {
    std::cerr <<
      "usage: " << prog << " <robot-ip> [options]\n"
      "  --mode hold|osc|step|disturb     default: hold\n"
      "  --axis x|y|z                     default: z\n"
      "  --amp <m>                        default: 0.05\n"
      "  --freq <hz>                      default: 0.25\n"
      "  --step <m>                       default: 0.05\n"
      "  --k kx,ky,kz,krx,kry,krz         override stiffness (N/m, Nm/rad)\n"
      "  --damp-ratio <z>                 default: 0.9; D = 2*sqrt(K)*z\n"
      "  --duration <s>                   default: 0 (manual SIGINT)\n"
      "  --csv <path>                     log 1 kHz samples to CSV\n";
}

bool parse_K(const std::string& s, Eigen::Matrix<double, 6, 1>& out) {
    int    idx = 0;
    size_t pos = 0;
    while (pos <= s.size() && idx < 6) {
        size_t      comma = s.find(',', pos);
        std::string tok =
            s.substr(pos, comma == std::string::npos ? std::string::npos
                                                     : comma - pos);
        try {
            out[idx++] = std::stod(tok);
        } catch (...) {
            return false;
        }
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    return idx == 6;
}

bool parse_args(int argc, char** argv, Args& a) {
    if (argc < 2) return false;
    a.ip = argv[1];
    if (a.ip.empty() || a.ip[0] == '-') return false;

    auto need = [&](int& i, const char* name) -> const char* {
        if (i + 1 >= argc) {
            std::cerr << name << " requires a value\n";
            return nullptr;
        }
        return argv[++i];
    };

    for (int i = 2; i < argc; ++i) {
        std::string k = argv[i];
        if (k == "--mode") {
            const char* v = need(i, "--mode");
            if (!v) return false;
            std::string s = v;
            if (s == "hold")         a.mode = Mode::Hold;
            else if (s == "osc")     a.mode = Mode::Osc;
            else if (s == "step")    a.mode = Mode::Step;
            else if (s == "disturb") a.mode = Mode::Disturb;
            else { std::cerr << "bad --mode: " << s << "\n"; return false; }
        } else if (k == "--axis") {
            const char* v = need(i, "--axis");
            if (!v) return false;
            std::string s = v;
            if (s == "x")      a.axis = 0;
            else if (s == "y") a.axis = 1;
            else if (s == "z") a.axis = 2;
            else { std::cerr << "bad --axis: " << s << "\n"; return false; }
        } else if (k == "--amp") {
            const char* v = need(i, "--amp");  if (!v) return false;
            a.amp = std::stod(v);
        } else if (k == "--freq") {
            const char* v = need(i, "--freq"); if (!v) return false;
            a.freq = std::stod(v);
        } else if (k == "--step") {
            const char* v = need(i, "--step"); if (!v) return false;
            a.step = std::stod(v);
        } else if (k == "--k") {
            const char* v = need(i, "--k");    if (!v) return false;
            if (!parse_K(v, a.K)) {
                std::cerr << "bad --k: need 6 comma-separated numbers\n";
                return false;
            }
            a.have_K = true;
        } else if (k == "--damp-ratio") {
            const char* v = need(i, "--damp-ratio"); if (!v) return false;
            a.damp_ratio     = std::stod(v);
            a.damp_ratio_set = true;
        } else if (k == "--duration") {
            const char* v = need(i, "--duration"); if (!v) return false;
            a.duration = std::stod(v);
        } else if (k == "--csv") {
            const char* v = need(i, "--csv");      if (!v) return false;
            a.csv_path = v;
        } else if (k == "-h" || k == "--help") {
            return false;
        } else {
            std::cerr << "unknown arg: " << k << "\n";
            return false;
        }
    }
    return true;
}

std::atomic<bool> g_stop{false};
void              on_sigint(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        usage(argv[0]);
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    try {
        franka::Robot robot(args.ip);
        franka::Model model = robot.loadModel();

        // Conservative collision thresholds — same as franka_example_controllers.
        robot.setCollisionBehavior(
            {{20, 20, 20, 20, 20, 20, 20}}, {{20, 20, 20, 20, 20, 20, 20}},
            {{20, 20, 20, 20, 20, 20}},     {{20, 20, 20, 20, 20, 20}});

        franka::RobotState s0 = robot.readOnce();
        Eigen::Affine3d    T0(Eigen::Matrix4d::Map(s0.O_T_EE.data()));

        CartesianImpedanceCfg cfg0;
        cfg0.target = T0;
        if (args.have_K) cfg0.K = args.K;
        if (args.have_K || args.damp_ratio_set) {
            for (int i = 0; i < 6; ++i)
                cfg0.D[i] = 2.0 * std::sqrt(cfg0.K[i]) * args.damp_ratio;
        }

        std::cout << "mode    = " << mode_name(args.mode) << "\n"
                  << "axis    = " << "xyz"[args.axis] << "\n"
                  << "K       = " << cfg0.K.transpose() << "\n"
                  << "D       = " << cfg0.D.transpose() << "\n";
        if (args.duration > 0)
            std::cout << "duration= " << args.duration << " s\n";
        if (!args.csv_path.empty())
            std::cout << "csv     = " << args.csv_path << "\n";

        CartesianImpedanceController ctrl;
        ctrl.set_cfg(cfg0);
        ctrl.reset(s0);

        // Pre-allocated ring buffer; only sized when CSV requested.
        struct Sample {
            double t;
            double p_d[3];
            double p[3];
            double F[6];
        };
        const size_t        LOG_CAP = args.csv_path.empty() ? 0 : 1024 * 1024;
        std::vector<Sample> log(LOG_CAP);
        std::atomic<size_t> log_n{0};

        // Off-RT updater: rewrites `latest.target` per mode at 100 Hz.
        std::mutex            cfg_mu;
        CartesianImpedanceCfg latest = cfg0;
        std::thread           updater([&] {
            using clock        = std::chrono::steady_clock;
            const auto t_start = clock::now();
            while (!g_stop.load()) {
                double t =
                    std::chrono::duration<double>(clock::now() - t_start).count();
                if (args.duration > 0.0 && t >= args.duration) {
                    g_stop.store(true);
                    break;
                }
                CartesianImpedanceCfg c = cfg0;
                switch (args.mode) {
                    case Mode::Hold:
                    case Mode::Disturb:
                        break;
                    case Mode::Osc:
                        c.target.translation()[args.axis] +=
                            args.amp * std::sin(2.0 * M_PI * args.freq * t);
                        break;
                    case Mode::Step:
                        if (t >= 2.0)
                            c.target.translation()[args.axis] += args.step;
                        break;
                }
                {
                    std::lock_guard<std::mutex> lk(cfg_mu);
                    latest = c;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        // Mirror of latest.target.translation() living on the RT thread —
        // updated only when the try-lock succeeds, so the log doesn't need
        // to read `latest` outside the lock.
        Eigen::Vector3d p_d_last = T0.translation();
        const auto      rt_start = std::chrono::steady_clock::now();

        auto callback = [&](const franka::RobotState& s,
                            franka::Duration) -> franka::Torques {
            {
                std::unique_lock<std::mutex> lk(cfg_mu, std::try_to_lock);
                if (lk.owns_lock()) {
                    ctrl.set_cfg(latest);
                    p_d_last = latest.target.translation();
                }
            }
            franka::Torques tau(ctrl.compute(s, model));

            if (!log.empty()) {
                size_t i = log_n.fetch_add(1, std::memory_order_relaxed);
                if (i < log.size()) {
                    Sample& sm = log[i];
                    sm.t       = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - rt_start)
                                .count();
                    sm.p_d[0] = p_d_last.x();
                    sm.p_d[1] = p_d_last.y();
                    sm.p_d[2] = p_d_last.z();
                    sm.p[0]   = s.O_T_EE[12];
                    sm.p[1]   = s.O_T_EE[13];
                    sm.p[2]   = s.O_T_EE[14];
                    for (int j = 0; j < 6; ++j) sm.F[j] = s.O_F_ext_hat_K[j];
                }
            }

            if (g_stop.load()) tau.motion_finished = true;
            return tau;
        };

        std::cout << "Cartesian test running. Ctrl+C to stop.\n";
        robot.control(callback);

        g_stop = true;
        updater.join();

        if (!args.csv_path.empty()) {
            size_t        n = std::min(log_n.load(), log.size());
            std::ofstream f(args.csv_path);
            if (!f) {
                std::cerr << "could not open " << args.csv_path
                          << " for writing\n";
                return 1;
            }
            f << "t,p_d_x,p_d_y,p_d_z,p_x,p_y,p_z,Fx,Fy,Fz,Tx,Ty,Tz\n";
            for (size_t i = 0; i < n; ++i) {
                const auto& sm = log[i];
                f << sm.t;
                for (int j = 0; j < 3; ++j) f << "," << sm.p_d[j];
                for (int j = 0; j < 3; ++j) f << "," << sm.p[j];
                for (int j = 0; j < 6; ++j) f << "," << sm.F[j];
                f << "\n";
            }
            std::cout << "wrote " << n << " samples to " << args.csv_path
                      << "\n";
        }
    } catch (const franka::Exception& e) {
        std::cerr << "franka exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
