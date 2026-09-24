#pragma once

#include <atomic>
#include <zmq.hpp>
#include <fr3_stack/recording.hpp>

namespace fr3_stack {
// Own this REP socket on one non-RT thread. Configure finite receive/send
// timeouts before entering. The manager never sends commands to controllers.
void serve_recording(zmq::socket_t& socket, RecordingManager& manager,
                     const std::atomic<bool>& stop);
}  // namespace fr3_stack
