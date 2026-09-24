#pragma once

// Non-realtime CSV writer for the 1 kHz sysid ring (see ring_log.hpp).
//
// Owns the thread and either owns or borrows its ring. The RT callback touches only
// `ring().push(...)`; everything that can block -- formatting, the file, the
// allocation of either -- happens on the writer thread.
//
// The file is written continuously rather than dumped at the end, so a run
// is bounded by disk, not by the ring. The ring only has to absorb
// filesystem hiccups.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <fr3_stack/ring_log.hpp>

namespace fr3_stack {

// Invoked on the writer thread; injectable for deterministic I/O failure tests.
struct RingLogFileOps {
    std::FILE* (*open)(const char*, const char*) = std::fopen;
    std::size_t (*write)(const void*, std::size_t, std::size_t, std::FILE*) = std::fwrite;
    int (*flush)(std::FILE*) = std::fflush;
    int (*close)(std::FILE*) = std::fclose;
};

class RingLogWriter {
  public:
    enum class Error { None, Open, HeaderWrite, Write, Flush, Close };

    // Asynchronous open/header/flush. ok() starts false, becomes true after the
    // header flush, and latches false on I/O failure without changing control.
    RingLogWriter(const std::string& path, std::size_t capacity_frames = 16384,
                  RingLogFileOps file_ops = {});
    // External ring must outlive the writer; enables safe reuse between runs.
    RingLogWriter(RingLog& ring, const std::string& path, RingLogFileOps file_ops = {});
    ~RingLogWriter();

    RingLogWriter(const RingLogWriter&)            = delete;
    RingLogWriter& operator=(const RingLogWriter&) = delete;

    bool ok() const noexcept { return ok_.load(std::memory_order_acquire); }
    bool finished() const noexcept { return finished_.load(std::memory_order_acquire); }
    // I/O completion only; does not certify robot timing or experiment validity.
    bool complete() const noexcept { return finished() && ok() && dropped() == 0; }
    Error error() const noexcept { return error_.load(std::memory_order_acquire); }
    int error_code() const noexcept {
        (void)error();  // acquire publication of the first error
        return error_code_.load(std::memory_order_relaxed);
    }
    static const char* error_name(Error error) noexcept;

    // The RT side pushes frames here. Never blocks.
    RingLog& ring() noexcept { return ring_; }

    // Drains what is still buffered, joins the thread and closes the file.
    // Stop the producer FIRST. Idempotent for one owner, not concurrent callers.
    // Also called by the destructor. Check complete() after stop().
    void stop() noexcept;

    // Full rows covered by successful fflush. Not an fsync/durability promise;
    // a later close failure still invalidates the recording.
    std::uint64_t written() const noexcept {
        return written_.load(std::memory_order_relaxed);
    }
    // Accepted frames without confirmed flush, or drained after I/O failure.
    std::uint64_t discarded() const noexcept {
        return discarded_.load(std::memory_order_relaxed);
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
    void fail(Error error, int code) noexcept;

    std::unique_ptr<RingLog>  owned_ring_;
    RingLog&                 ring_;
    std::string               path_;
    RingLogFileOps            file_ops_;
    std::atomic<bool>         ok_{false};
    std::atomic<bool>         stop_{false};
    std::atomic<bool>         stopped_{false};
    std::atomic<bool>         finished_{false};
    std::atomic<Error>        error_{Error::None};
    std::atomic<int>          error_code_{0};
    std::atomic<std::uint64_t> written_{0};
    std::atomic<std::uint64_t> discarded_{0};
    std::thread               thread_;
};

}  // namespace fr3_stack
