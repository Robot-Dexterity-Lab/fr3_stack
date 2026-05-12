// Pure-libfranka clone of franka_ros2's gravity_compensation_example_controller.
//
// franka_ros2 wires its torque interface through:
//   active_control = robot.startTorqueControl();           // active control API
//   loop: state = active_control->readOnce();
//         tau   = franka::Torques({0,0,0,0,0,0,0});
//         tau.tau_J = franka::limitRate(kMaxTorqueRate, tau.tau_J, state.tau_J_d);
//         active_control->writeOnce(tau);
//
// We reproduce that here verbatim — no setCollisionBehavior, no setLoad,
// no impedance setup, no damping. The point is to A/B against the daemon's
// callback-API idle controller and answer one question:
//
//   "Does FR3 firmware reflex behavior depend on which libfranka control
//    API (callback vs active) we use, holding the torque content equal?"
//
// If this binary lets you hand-guide the arm without tripping
// `joint_velocity_violation`, the answer is yes — the daemon should be
// refactored to use startTorqueControl(). If this binary trips the same
// reflex within a few seconds of any push, the answer is no — and we
// keep the mass-weighted damping idle in the daemon.
//
// Build: cmake -B build -DFR3_BUILD_TEACHING=ON && cmake --build build
//        --target grav_comp_franka_style
// Run:   sudo ./build/grav_comp_franka_style <robot-ip>

#include <franka/active_control_base.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <atomic>
#include <csignal>
#include <iostream>

static std::atomic<bool> g_stop{false};

static void on_signal(int) { g_stop.store(true); }

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <robot-ip>\n";
        return 1;
    }
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    try {
        franka::Robot robot(argv[1]);

        // Match franka_ros2 exactly: clear any latent reflex (they call this
        // on exception, we do it once at startup so the user doesn't have to
        // hit "Automatic error recovery" between attempts).
        robot.automaticErrorRecovery();

        // No setCollisionBehavior / setJointImpedance / setLoad — same as
        // franka_ros2's torque interface init path.

        std::cout << "[franka_ros2-style] connected to " << argv[1]
                  << " (FCI v" << robot.serverVersion() << ")\n";
        std::cout << "[franka_ros2-style] starting torque control "
                     "(literal zero torque + rate limit)\n";
        std::cout << "[franka_ros2-style] try to hand-guide. Ctrl+C to stop.\n";

        auto control = robot.startTorqueControl();

        const std::array<double, 7> zeros{};
        while (!g_stop.load()) {
            auto [state, dt] = control->readOnce();
            (void)dt;

            franka::Torques cmd(zeros);
            cmd.tau_J = franka::limitRate(
                franka::kMaxTorqueRate, cmd.tau_J, state.tau_J_d);
            control->writeOnce(cmd);
        }
        std::cout << "\n[franka_ros2-style] stop requested, exiting.\n";
        return 0;
    } catch (const franka::Exception& e) {
        std::cerr << "[franka_ros2-style] FRANKA EXCEPTION: "
                  << e.what() << "\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "[franka_ros2-style] EXCEPTION: " << e.what() << "\n";
        return 2;
    }
}
