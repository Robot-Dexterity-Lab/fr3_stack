#pragma once

// Non-realtime CSV writer for the 1 kHz sysid ring (see ring_log.hpp).
//
// Owns the ring and the thread that drains it. The RT callback touches only
// `ring().push(...)`; everything that can block -- formatting, the file, the
// allocation of either -- happens on the writer thread.
//
// The file is written continuously rather than dumped at the end, so a run
// is bounded by disk, not by the ring. The ring only has to absorb
// filesystem hiccups.

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <fr3_stack/ring_log.hpp>

namespace fr3_stack {

class RingLogWriter {
  public:
    // Opens `path` and starts the writer thread. Check ok() afterwards: a
    // failed open is reported, not thrown, so the daemon can carry on
    // controlling the robot without a log.
    RingLogWriter(const std::string& path, std::size_t capacity_frames = 16384);
    ~RingLogWriter();

    RingLogWriter(const RingLogWriter&)            = delete;
    RingLogWriter& operator=(const RingLogWriter&) = delete;

    bool ok() const noexcept { return ok_.load(std::memory_order_acquire); }

    // The RT side pushes frames here. Never blocks.
    RingLog& ring() noexcept { return ring_; }

    // Drains what is still buffered, joins the thread and closes the file.
    // Idempotent; also called by the destructor.
    void stop() noexcept;

    std::uint64_t written() const noexcept {
        return written_.load(std::memory_order_relaxed);
    }
    // Frames the RT side offered that the ring could not accept. Non-zero
    // means the CSV has gaps -- visible as jumps in its `seq` column.
    std::uint64_t dropped() const noexcept { return ring_.dropped(); }

    const std::string& path() const noexcept { return path_; }

    // Column header written as the first CSV line. Exposed so a test can
    // assert the schema without opening a file.
    static const char* csv_header() noexcept;

  private:
    void run();

    RingLog                   ring_;
    std::string               path_;
    std::atomic<bool>         ok_{false};
    std::atomic<bool>         stop_{false};
    std::atomic<bool>         stopped_{false};
    std::atomic<std::uint64_t> written_{0};
    std::thread               thread_;
};

}  // namespace fr3_stack
