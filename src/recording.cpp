#include <fr3_stack/recording.hpp>

#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace fr3_stack {
namespace {
std::FILE* open_exclusive(const char* path, const char*) { return std::fopen(path, "wx"); }
void validate_id(const std::string& id) {
    if (id.empty() || id.size() > 128) throw std::invalid_argument("invalid recording ID");
    for (unsigned char c : id)
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '-' || c == '_'))
            throw std::invalid_argument("recording ID must contain only letters, digits, - or _");
}
}  // namespace

RingLogFileOps exclusive_recording_file_ops() {
    RingLogFileOps ops;
    ops.open = open_exclusive;
    return ops;
}

RecordingManager::RecordingManager(std::size_t capacity_frames, RingLogFileOps ops)
    : ring_(capacity_frames), ops_(ops) {}
RecordingManager::~RecordingManager() { shutdown(); }

RecordingStatus RecordingManager::status_locked() const {
    RecordingStatus s;
    s.recording_id = recording_id_;
    s.active = enabled_.load(std::memory_order_seq_cst);
    if (writer_) {
        s.path = writer_->path();
        s.ok = writer_->ok();
        s.finished = writer_->finished();
        s.complete = writer_->complete();
        s.written = writer_->written();
        s.discarded = writer_->discarded();
        s.dropped = writer_->dropped();
        if (writer_->error() != RingLogWriter::Error::None) {
            s.error = std::string("recording I/O failure: ") + RingLogWriter::error_name(writer_->error());
            s.error_code = writer_->error_code();
        }
    }
    if (!start_error_.empty()) s.error = start_error_;
    return s;
}

RecordingStatus RecordingManager::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_locked();
}

RecordingStatus RecordingManager::start(const std::string& id, const std::string& path) {
    validate_id(id);
    if (path.empty() || path.front() != '/' || path.size() > 4096 || path.find('\0') != std::string::npos)
        throw std::invalid_argument("recording path must be an absolute NUC path without NUL, at most 4096 bytes");
    std::lock_guard<std::mutex> lock(mutex_);
    if (id == recording_id_ && writer_ && path == writer_->path()) return status_locked();
    if (used_ids_.count(id)) throw std::runtime_error("recording ID already used; do not restart an old request");
    if (enabled_.load(std::memory_order_seq_cst)) throw std::runtime_error("a recording is already active");
    used_ids_.insert(id);
    recording_id_ = id;
    start_error_.clear();
    writer_.reset();  // previous session was already stopped
    ring_.reset();
    seq_ = 0;
    t0_ = -1.0;
    try {
        writer_ = std::make_unique<RingLogWriter>(ring_, path, ops_);
    } catch (const std::exception& e) {
        start_error_ = e.what();
        failed_ = true;
        return status_locked();
    }
    while (!writer_->ok() && writer_->error() == RingLogWriter::Error::None)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!writer_->ok()) {
        writer_->stop();
        failed_ = true;
    } else {
        enabled_.store(true, std::memory_order_seq_cst);
    }
    return status_locked();
}

void RecordingManager::stop_locked() {
    enabled_.store(false, std::memory_order_seq_cst);
    while (producer_busy_.load(std::memory_order_seq_cst))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (writer_) {
        writer_->stop();
        failed_ = failed_ || !writer_->complete();
    }
}

RecordingStatus RecordingManager::stop(const std::string& id) {
    validate_id(id);
    std::lock_guard<std::mutex> lock(mutex_);
    if (id != recording_id_) throw std::runtime_error("recording ID mismatch; current recording was not stopped");
    stop_locked();
    return status_locked();
}

void RecordingManager::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_locked();
}

bool RecordingManager::had_failed_recordings() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
}

}  // namespace fr3_stack
