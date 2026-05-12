// Standalone Cartesian impedance binary — same controller class as the
// fr3-stack daemon, just wrapped with a local demo updater instead of
// the ZMQ command stream. Useful for sanity-check and teaching: bring up
// libfranka, hold pose, oscillate Z by ±5 cm at 0.25 Hz to confirm the
// loop is closing. No ZMQ / capnp / FT sensor stack needed.
//
// Build: see CMakeLists.txt (-DFR3_BUILD_TEACHING=ON)
// Run:   sudo ./cartesian_impedance <robot-ip>

#include <franka/exception.h>
#include <franka/robot.h>

#include <fr3_stack/controllers/cartesian_impedance_controller.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <robot-ip>\n";
        return 1;
    }
    try {
        franka::Robot robot(argv[1]);
        franka::Model model = robot.loadModel();

        // Conservative collision thresholds — same as franka_example_controllers.
        robot.setCollisionBehavior(
            {{20, 20, 20, 20, 20, 20, 20}}, {{20, 20, 20, 20, 20, 20, 20}},
            {{20, 20, 20, 20, 20, 20}},     {{20, 20, 20, 20, 20, 20}});

        franka::RobotState s0 = robot.readOnce();
        Eigen::Affine3d    T0(Eigen::Matrix4d::Map(s0.O_T_EE.data()));

        // Default cfg from the header is the production tuning — same K/D as
        // the daemon would seed. Anchor the spring at startup pose.
        CartesianImpedanceCfg cfg0;
        cfg0.target = T0;
        // To experiment with friction comp on the same arm:
        // cfg0.use_friction = true;

        CartesianImpedanceController ctrl;
        ctrl.set_cfg(cfg0);
        ctrl.reset(s0);

        // Demo updater (off-RT): publish an oscillating Z target at 100 Hz.
        // Mirrors the ZMQ command stream in the daemon — replace this body
        // with your own for a different motion profile.
        std::atomic<bool>     stop{false};
        std::mutex            cfg_mu;
        CartesianImpedanceCfg latest = cfg0;
        std::thread updater([&] {
            using clock = std::chrono::steady_clock;
            const auto t_start = clock::now();
            while (!stop.load()) {
                double t = std::chrono::duration<double>(clock::now() - t_start).count();
                CartesianImpedanceCfg c = cfg0;
                c.target.translation().z() += 0.05 * std::sin(2.0 * M_PI * 0.25 * t);
                {
                    std::lock_guard<std::mutex> lk(cfg_mu);
                    latest = c;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        auto callback = [&](const franka::RobotState& s,
                            franka::Duration) -> franka::Torques {
            // try-lock so the RT path never blocks. On contention we just
            // reuse the previous cfg — same pattern as the production daemon.
            {
                std::unique_lock<std::mutex> lk(cfg_mu, std::try_to_lock);
                if (lk.owns_lock()) ctrl.set_cfg(latest);
            }
            return franka::Torques(ctrl.compute(s, model));
        };

        std::cout << "Cartesian impedance running. Ctrl+C to stop.\n";
        robot.control(callback);

        stop = true;
        updater.join();
    } catch (const franka::Exception& e) {
        std::cerr << "franka exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
