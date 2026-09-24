#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <fr3_stack/ring_log_writer.hpp>

namespace fr3_stack {

struct RecordingStatus {
    std::string recording_id, path, error;
    bool active{false}, ok{false}, finished{false}, complete{false};
    std::uint64_t written{0}, discarded{0}, dropped{0};
    int error_code{0};
};

// Atomic exclusive creation: existing recordings are never truncated.
RingLogFileOps exclusive_recording_file_ops();

class RecordingManager {
  public:
    explicit RecordingManager(std::size_t capacity_frames = 16384,
                              RingLogFileOps ops = exclusive_recording_file_ops());
    ~RecordingManager();  // producer must be joined first

    // Non-RT operations. start acknowledges successful header flush; stop
    // disables capture, waits for an in-flight push, drains and closes.
    RecordingStatus start(const std::string& recording_id, const std::string& path);
    RecordingStatus stop(const std::string& recording_id);
    RecordingStatus status() const;
    void shutdown();
    bool had_failed_recordings() const;

    // Exactly one RT producer. The permanent ring is never freed by stop().
    // SC ordering guarantees that stop either observes this producer busy,
    // or this producer observes capture disabled. No locks, waits or I/O here.
    // Fill is invoked only while recording and returns a frame with absolute
    // host steady time in t_s; this method converts it to session-relative time.
    template <typename Fill>
    void record(Fill&& fill) {
        struct Guard {
            std::atomic<bool>& busy;
            ~Guard() { busy.store(false, std::memory_order_seq_cst); }
        } guard{producer_busy_};
        producer_busy_.store(true, std::memory_order_seq_cst);
        if (!enabled_.load(std::memory_order_seq_cst)) return;
        auto frame = fill(seq_++);
        if (t0_ < 0.0) t0_ = frame.t_s;
        frame.t_s -= t0_;
        ring_.push(frame);
    }

  private:
    RecordingStatus status_locked() const;
    void stop_locked();
    RingLog ring_;
    RingLogFileOps ops_;
    mutable std::mutex mutex_;  // non-RT only
    std::unique_ptr<RingLogWriter> writer_;
    std::string recording_id_, start_error_;
    std::unordered_set<std::string> used_ids_;
    bool failed_{false};
    std::atomic<bool> enabled_{false}, producer_busy_{false};
    std::uint64_t seq_{0};
    double t0_{-1.0};
};

}  // namespace fr3_stack
