#pragma once

// Lock-free 1 kHz state ring for system identification.
//
// Why this exists: the daemon runs its controller at 1 kHz but publishes
// state over ZMQ at ~200 Hz (see docs/architecture.md). System
// identification fits against per-tick data and resolves an actuation delay
// of a few milliseconds, which a 5 ms publish period cannot observe at all.
// Raising the publish rate is the wrong fix -- the wire conflates by design
// and per-frame Cap'n Proto serialization is exactly what ~200 Hz avoids.
//
// So the RT callback writes one frame per tick into this ring, and a
// non-realtime writer thread drains it to disk. The ring is the only thing
// the RT side touches, and it obeys the RT rules:
//
//   - `push()` never blocks, never allocates, never throws, and never waits
//     on the consumer. Storage is reserved once at construction.
//   - When the consumer falls behind, `push()` drops the arriving frame and
//     counts it, so a log can say exactly where its gaps are rather than
//     stalling the control loop.
//   - Single producer, single consumer. One RT thread in, one writer out.
//     Two producers or two consumers is undefined behaviour.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fr3_stack {

// One control tick. Trivially copyable POD so the RT side is a plain store
// and the writer can treat it as a fixed-width record.
//
// Targets are included because identification replays the *commanded*
// trajectory in simulation: without the setpoint the measured response
// cannot be reproduced. `tau_cmd` is the torque actually sent, after the
// daemon's final rate limiting, for the same reason.
struct RingLogFrame {
    std::uint64_t seq;                   // monotonic tick counter since start
    double        t_s;                   // seconds since logging started
    double        q[7];                  // measured joint position [rad]
    double        dq[7];                 // measured joint velocity [rad/s]
    double        tau_J[7];              // measured link-side torque [N.m]
    double        tau_cmd[7];            // commanded torque after rate limit
    double        ee_pos[3];             // O_T_EE translation [m]
    double        ee_quat_xyzw[4];       // O_T_EE rotation, xyzw (wire order)
    double        target_pos[3];         // active controller target [m]
    double        target_quat_xyzw[4];   // active controller target, xyzw
    std::uint32_t controller;            // active ControllerType, as uint
    std::uint32_t pad;                   // explicit: keeps the struct hole-free
};

class RingLog {
  public:
    // `capacity_frames` is rounded up to a power of two and clamped to a
    // usable minimum. The ring holds capacity() - 1 frames; one slot is
    // sacrificed so that full and empty stay distinguishable without a
    // separate count that both sides would have to write.
    //
    // At 1 kHz the default holds ~16 s of backlog, which is far more than a
    // writer draining to a local file needs -- the margin is there to absorb
    // filesystem hiccups, not to buffer a whole run.
    explicit RingLog(std::size_t capacity_frames = 16384)
        : capacity_(round_up_pow2(capacity_frames < 2 ? 2 : capacity_frames)),
          mask_(capacity_ - 1),
          buffer_(capacity_) {}

    RingLog(const RingLog&)            = delete;
    RingLog& operator=(const RingLog&) = delete;

    // Producer side. Returns false if the ring is full, having counted the
    // drop; the caller is the RT loop and must not retry or wait.
    bool push(const RingLogFrame& frame) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & mask_;
        if (next == tail_.load(std::memory_order_acquire)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buffer_[head] = frame;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns false if the ring is empty.
    bool pop(RingLogFrame& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        out = buffer_[tail];
        tail_.store((tail + 1) & mask_, std::memory_order_release);
        return true;
    }

    // Frames currently buffered. Safe to call from either side, but it is a
    // snapshot: the other side may have moved on by the time it returns.
    std::size_t size() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & mask_;
    }

    // Slot count (a power of two); the ring holds one fewer frame than this.
    std::size_t capacity() const noexcept { return capacity_; }

    // Frames the producer offered and the ring could not accept. A log with a
    // non-zero count here has gaps, and should say so rather than be trusted
    // as a contiguous 1 kHz record.
    std::uint64_t dropped() const noexcept {
        return dropped_.load(std::memory_order_relaxed);
    }

  private:
    static std::size_t round_up_pow2(std::size_t n) noexcept {
        std::size_t p = 1;
        while (p < n) p <<= 1;
        return p;
    }

    const std::size_t         capacity_;
    const std::size_t         mask_;
    std::vector<RingLogFrame> buffer_;
    std::atomic<std::size_t>  head_{0};   // written by the producer only
    std::atomic<std::size_t>  tail_{0};   // written by the consumer only
    std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace fr3_stack
