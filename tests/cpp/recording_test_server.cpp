// Hardware-free service harness. Invoked and reaped by test_recording_client.py.
#include <fr3_stack/recording_server.hpp>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

static std::atomic<bool> stop_requested{false};
static void stop_handler(int) { stop_requested.store(true); }

int main() {
    std::signal(SIGTERM, stop_handler);
    std::signal(SIGINT, stop_handler);
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.set(zmq::sockopt::rcvtimeo, 50);
    socket.set(zmq::sockopt::sndtimeo, 50);
    socket.set(zmq::sockopt::linger, 0);
    socket.bind("tcp://127.0.0.1:*");
    fr3_stack::RecordingManager manager;
    std::thread producer([&]() {
        std::uint64_t tick = 0;
        while (!stop_requested.load()) {
            ++tick;  // simulated control continues even when logging is off
            manager.record([&](std::uint64_t seq) {
                fr3_stack::RingLogFrame f{};
                f.seq = seq;
                f.t_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                f.robot_time_s = tick * 0.001;
                f.control_period_s = 0.001;
                f.q[0] = static_cast<double>(tick);
                f.joint_target_valid = 1;
                f.controller = 2;
                return f;
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::cout << socket.get(zmq::sockopt::last_endpoint) << std::endl;
    try {
        fr3_stack::serve_recording(socket, manager, stop_requested);
    } catch (const zmq::error_t& e) {
        if (!stop_requested.load()) std::cerr << e.what() << '\n';
    }
    stop_requested.store(true);
    producer.join();
    manager.shutdown();
}
