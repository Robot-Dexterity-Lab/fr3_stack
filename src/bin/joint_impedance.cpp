// Standalone joint-impedance binary — same controller class as the daemon,
// wrapped with a demo updater that wiggles joint 4 by ±0.1 rad at 0.25 Hz.
//
// Build: see CMakeLists.txt (-DFR3_BUILD_TEACHING=ON)
// Run:   sudo ./joint_impedance <robot-ip>

#include <franka/exception.h>
#include <franka/robot.h>

#include <fr3_stack/controllers/joint_impedance_controller.hpp>

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

        robot.setCollisionBehavior(
            {{20, 20, 20, 20, 20, 20, 20}}, {{20, 20, 20, 20, 20, 20, 20}},
            {{20, 20, 20, 20, 20, 20}},     {{20, 20, 20, 20, 20, 20}});

        franka::RobotState s0 = robot.readOnce();

        JointImpedanceCfg cfg0;
        for (int i = 0; i < 7; ++i) cfg0.q_target[i] = s0.q[i];

        JointImpedanceController ctrl;
        ctrl.set_cfg(cfg0);
        ctrl.reset(s0);

        std::atomic<bool> stop{false};
        std::mutex        cfg_mu;
        JointImpedanceCfg latest = cfg0;
        std::thread updater([&] {
            using clock = std::chrono::steady_clock;
            const auto t_start = clock::now();
            while (!stop.load()) {
                double t = std::chrono::duration<double>(clock::now() - t_start).count();
                JointImpedanceCfg c = cfg0;
                // joint 4 is mid-arm — easy to see the motion, no shoulder
                // / wrist edge cases.
                c.q_target[3] += 0.1 * std::sin(2.0 * M_PI * 0.25 * t);
                {
                    std::lock_guard<std::mutex> lk(cfg_mu);
                    latest = c;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        auto callback = [&](const franka::RobotState& s,
                            franka::Duration) -> franka::Torques {
            {
                std::unique_lock<std::mutex> lk(cfg_mu, std::try_to_lock);
                if (lk.owns_lock()) ctrl.set_cfg(latest);
            }
            return franka::Torques(ctrl.compute(s, model));
        };

        std::cout << "Joint impedance running. Ctrl+C to stop.\n";
        robot.control(callback);

        stop = true;
        updater.join();
    } catch (const franka::Exception& e) {
        std::cerr << "franka exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
