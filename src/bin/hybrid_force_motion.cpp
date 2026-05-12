// Standalone hybrid force/motion binary — same controller class as the
// daemon. Default cfg holds n_af=0 (pure admittance, follows the user's
// hand) so the demo is safe out of the box. Edit `cfg0` below to enable
// force-controlled axes.
//
// Build: see CMakeLists.txt (-DFR3_BUILD_TEACHING=ON)
// Run:   sudo ./hybrid_force_motion <robot-ip>
//
// SAFETY: see others/force_control/README.md before bringing this up on
// real hardware. Start with one translational force axis, low PID gains,
// damping > 0. Get a feel for the sign / sensor calibration before scaling.

#include <franka/exception.h>
#include <franka/robot.h>

#include <fr3_stack/controllers/hybrid_force_motion_controller.hpp>

#include <atomic>
#include <chrono>
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
        Eigen::Affine3d    T0(Eigen::Matrix4d::Map(s0.O_T_EE.data()));

        HybridForceMotionCfg cfg0;
        cfg0.target = T0;
        // Pure admittance default. To enable a +z 10 N push:
        //   cfg0.n_af = 1;                       // first Tr-row force-controlled
        //   cfg0.target_wrench_Tr[0] = -10.0;     // 10 N along Tr's first axis
        //   cfg0.P_trans = 0.001;                 // start tiny, ramp up
        // Tr defaults to identity so first axis = world +x; rotate Tr to
        // align with the surface normal.

        HybridForceMotionController ctrl;
        // ctrl.set_wrench_source(...) here if you have a real FT sensor
        // (the daemon does this from --ft-sensor-kind / --ft-sensor-config).
        // Fallback uses libfranka O_F_ext_hat_K (~3-5 N noise floor).
        ctrl.set_cfg(cfg0);
        ctrl.reset(s0);

        std::atomic<bool>    stop{false};
        std::mutex           cfg_mu;
        HybridForceMotionCfg latest = cfg0;
        std::thread updater([&] {
            // No motion in the demo — hybrid reacts to contact, not to a
            // moving target. Replace with your own update logic.
            while (!stop.load()) {
                {
                    std::lock_guard<std::mutex> lk(cfg_mu);
                    latest = cfg0;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

        std::cout << "Hybrid (HFVC) running. Default n_af=0 (admittance). Ctrl+C to stop.\n";
        robot.control(callback);

        stop = true;
        updater.join();
    } catch (const franka::Exception& e) {
        std::cerr << "franka exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
