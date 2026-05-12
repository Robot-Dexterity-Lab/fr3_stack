// fr3-ft — standalone Bota FT-sensor smoke test daemon.
//
// Runs only the FT sensor worker (no libfranka, no ZMQ, no capnp). Polls
// the latest raw wrench at the requested rate and prints CSV lines to
// stdout. Use it to verify the EtherCAT/Bota wiring before bringing up
// the full daemon, or when the FR3 arm isn't available.
//
// IMPORTANT: this binary does NOT do any payload compensation — output is
// the sensor-frame raw wrench (no internal tare; sensor's full electrical
// zero is present). Full mass/CoM calibration still needs the arm (R_O_EE
// → see fr3-ft-calibrate).

#include <fr3_stack/sensors/bota/wrench_source.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr const char* kDefaultConfig =
    "/opt/bota/driver_config/bota_binary.json";
constexpr double kDefaultHz = 250.0;

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

void print_usage(const char* argv0) {
    std::cout
        << "fr3-ft — Bota FT smoke test (no robot, raw sensor wrench).\n"
        << "\n"
        << "usage: " << argv0 << " [--config <path>] [--hz <rate>]\n"
        << "\n"
        << "  --config <path>  Bota driver JSON (default: " << kDefaultConfig << ")\n"
        << "  --hz <rate>      stdout CSV rate in Hz (default: " << kDefaultHz << ")\n"
        << "\n"
        << "Output (stdout, sensor frame, no payload compensation):\n"
        << "  t,fx,fy,fz,tx,ty,tz\n"
        << "where t is monotonic seconds since startup.\n";
}

struct Args {
    std::string config = kDefaultConfig;
    double      hz     = kDefaultHz;
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(k + " needs a value");
            return argv[++i];
        };
        if      (k == "--config") a.config = next();
        else if (k == "--hz")     a.hz     = std::stod(next());
        else if (k == "--help" || k == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown arg: " + k);
        }
    }
    if (a.hz <= 0.0) throw std::runtime_error("--hz must be > 0");
    return a;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT,  on_sigint);
    std::signal(SIGTERM, on_sigint);

    Args args;
    try { args = parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::cerr << "fr3-ft: args error: " << e.what() << "\n";
        return 1;
    }

    std::cerr << "fr3-ft: opening bota driver: " << args.config << "\n";
    std::unique_ptr<WrenchSource> ft;
    try {
        ft = make_wrench_source("bota", args.config);
        ft->start();
    } catch (const std::exception& e) {
        std::cerr << "fr3-ft: ERROR: " << e.what() << "\n";
        return 1;
    }
    std::cerr << "fr3-ft: ready (worker @ ~1 kHz). printing CSV at "
              << args.hz << " Hz to stdout. Ctrl+C to stop.\n";

    std::cout << "t,fx,fy,fz,tx,ty,tz\n";
    std::cout.flush();

    using clock = std::chrono::steady_clock;
    const auto t0     = clock::now();
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1.0e6 / args.hz));
    auto next = clock::now();

    WrenchSource::Vector6d w        = WrenchSource::Vector6d::Zero();
    bool                   have_frame      = false;
    bool                   waiting_logged  = false;

    while (!g_stop.load(std::memory_order_relaxed)) {
        next += period;

        WrenchSource::Vector6d w_new;
        if (ft->read(w_new)) {
            w          = w_new;
            have_frame = true;
        }

        if (have_frame) {
            const double t_s =
                std::chrono::duration<double>(clock::now() - t0).count();
            std::printf("%.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f\n",
                        t_s, w[0], w[1], w[2], w[3], w[4], w[5]);
            std::fflush(stdout);
        } else if (!waiting_logged) {
            std::cerr << "fr3-ft: waiting for first frame…\n";
            waiting_logged = true;
        }

        std::this_thread::sleep_until(next);
    }

    std::cerr << "fr3-ft: shutting down\n";
    ft->stop();
    return 0;
}
