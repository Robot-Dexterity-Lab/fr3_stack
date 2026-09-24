#include <fr3_stack/recording_server.hpp>
#include <fr3.capnp.h>
#include <capnp/message.h>
#include <capnp/serialize.h>
#include <cstring>
#include <stdexcept>

namespace fr3_stack {
namespace {
void fill_status(::RecordingStatus::Builder out, const RecordingStatus& s) {
    out.setRecordingId(s.recording_id);
    out.setPath(s.path);
    out.setActive(s.active);
    out.setOk(s.ok);
    out.setFinished(s.finished);
    out.setComplete(s.complete);
    out.setWritten(s.written);
    out.setDiscarded(s.discarded);
    out.setDropped(s.dropped);
    out.setError(s.error);
    out.setErrorCode(s.error_code);
}
}  // namespace

void serve_recording(zmq::socket_t& socket, RecordingManager& manager,
                     const std::atomic<bool>& stop) {
    while (!stop.load()) {
        zmq::message_t message;
        const auto received = socket.recv(message, zmq::recv_flags::none);
        if (!received) continue;
        bool multipart = socket.get(zmq::sockopt::rcvmore);
        while (socket.get(zmq::sockopt::rcvmore)) {
            zmq::message_t extra;
            (void)socket.recv(extra, zmq::recv_flags::none);
        }
        capnp::MallocMessageBuilder response;
        auto reply = response.initRoot<::RecordingReply>();
        try {
            if (multipart) throw std::invalid_argument("recording request must be a single frame");
            if (message.size() == 0 || message.size() > 65536 || message.size() % sizeof(capnp::word))
                throw std::invalid_argument("invalid recording request size");
            auto words = kj::heapArray<capnp::word>(message.size() / sizeof(capnp::word));
            std::memcpy(words.begin(), message.data(), message.size());
            capnp::ReaderOptions options;
            options.traversalLimitInWords = 8192;
            capnp::FlatArrayMessageReader reader(words.asPtr(), options);
            const auto request = reader.getRoot<::RecordingRequest>();
            RecordingStatus state;
            const auto id = request.getRecordingId();
            switch (request.getAction()) {
                case RecordingAction::STATUS:
                    state = manager.status();
                    break;
                case RecordingAction::START: {
                    const auto path = request.getPath();
                    state = manager.start(std::string(id.begin(), id.size()),
                                          std::string(path.begin(), path.size()));
                    break;
                }
                case RecordingAction::STOP:
                    state = manager.stop(std::string(id.begin(), id.size()));
                    break;
                default: throw std::invalid_argument("unknown recording action");
            }
            reply.setAccepted(true);
            fill_status(reply.initStatus(), state);
        } catch (const kj::Exception& e) {
            reply.setAccepted(false);
            reply.setError(e.getDescription());
            fill_status(reply.initStatus(), manager.status());
        } catch (const std::exception& e) {
            reply.setAccepted(false);
            reply.setError(e.what());
            fill_status(reply.initStatus(), manager.status());
        }
        const auto words = capnp::messageToFlatArray(response);
        const auto bytes = words.asBytes();
        // A disconnected client may miss the reply; retrying the same ID never
        // reopens/truncates a recording or stops a later session.
        while (!stop.load()) {
            if (socket.send(zmq::buffer(bytes.begin(), bytes.size()), zmq::send_flags::none)) break;
        }
    }
}
}  // namespace fr3_stack
