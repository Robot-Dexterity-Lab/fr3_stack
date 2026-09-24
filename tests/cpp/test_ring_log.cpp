// Unit tests for the 1 kHz RT ring log (include/fr3_stack/ring_log.hpp).
//
// The ring exists so the RT control callback can record one frame per tick
// without ever blocking, allocating, or touching the filesystem. A non-RT
// writer thread drains it. These tests pin that contract:
//
//   - the producer side never blocks and never fails except by dropping,
//   - drops are counted, so a log can state where its gaps are,
//   - FIFO order survives concurrent single-producer/single-consumer use,
//   - the frame is a trivially copyable POD (memcpy-able into the ring).
//
// Build (CMake):
//     cmake -B build -DFR3_BUILD_TESTS=ON
//     cmake --build build --target test_ring_log
//     ./build/test_ring_log
//
// Build (manual, no CMake, no Eigen or libfranka needed):
//     g++ -std=c++17 -O2 -I include tests/cpp/test_ring_log.cpp src/ring_log_writer.cpp
//         -o /tmp/test_ring_log -lpthread
//     /tmp/test_ring_log

#include <atomic>
#include <cerrno>
#include <cmath>
#include <unordered_map>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fr3_stack/ring_log.hpp>
#include <fr3_stack/ring_log_writer.hpp>

using fr3_stack::RingLog;
using fr3_stack::RingLogFrame;
using fr3_stack::RingLogWriter;

// ============================================================================
// Tiny test framework (same shape as tests/cpp/test_controller_math.cpp)
// ============================================================================
static int         g_pass = 0, g_fail = 0;
static std::string g_section;

static void section(const std::string& s) {
    g_section = s;
    std::cout << "\n[ " << s << " ]\n";
}

#define CHECK(cond, msg) do {                                                  \
    if (cond) { ++g_pass; std::cout << "  PASS: " << msg << "\n"; }            \
    else { ++g_fail; std::cerr << "  FAIL [" << g_section << "]: " << msg      \
                               << " (" << __FILE__ << ":" << __LINE__ << ")\n"; }\
} while (0)

// A frame whose every field is a known function of `seq`, so a consumer can
// verify it received exactly the frame the producer sent.
static RingLogFrame make_frame(std::uint64_t seq) {
    RingLogFrame f{};
    f.seq = seq;
    f.t_s = 0.001 * static_cast<double>(seq);
    for (int i = 0; i < 7; ++i) {
        f.q[i]       = static_cast<double>(seq) + 0.01 * i;
        f.dq[i]      = static_cast<double>(seq) + 0.02 * i;
        f.tau_J[i]   = static_cast<double>(seq) + 0.03 * i;
        f.tau_cmd[i] = static_cast<double>(seq) + 0.04 * i;
        f.q_target[i] = 10.0 + seq + i * 0.1;
        f.q_target_filtered[i] = 20.0 + seq + i * 0.2;
        f.K_joint[i] = 100.0 + i;
        f.D_joint[i] = 10.0 + i;
    }
    for (int i = 0; i < 3; ++i) {
        f.ee_pos[i]     = static_cast<double>(seq) + 0.05 * i;
        f.target_pos[i] = static_cast<double>(seq) + 0.06 * i;
    }
    for (int i = 0; i < 4; ++i) {
        f.ee_quat_xyzw[i]     = static_cast<double>(seq) + 0.07 * i;
        f.target_quat_xyzw[i] = static_cast<double>(seq) + 0.08 * i;
    }
    f.controller = static_cast<std::uint32_t>(seq % 5);
    f.joint_target_valid = 1;
    f.joint_use_friction = seq % 2;
    f.joint_reset_count = seq + 10;
    f.filter_alpha = 0.05;
    f.robot_time_s = 1000.0 + f.t_s;
    f.control_period_s = 0.001;
    return f;
}

static bool frames_equal(const RingLogFrame& a, const RingLogFrame& b) {
    return std::memcmp(&a, &b, sizeof(RingLogFrame)) == 0;
}

// ============================================================================
// Frame layout
// ============================================================================
static void test_frame_is_pod() {
    section("frame layout");
    CHECK(std::is_trivially_copyable<RingLogFrame>::value,
          "RingLogFrame is trivially copyable (memcpy-able from RT)");
    CHECK(std::is_standard_layout<RingLogFrame>::value,
          "RingLogFrame is standard layout (stable for a CSV writer)");
    // Padding would make the memcmp comparison above unreliable, and would
    // silently bloat the ring. Every member is 8-byte or 4-byte aligned by
    // construction, so the struct should have no interior holes.
    constexpr std::size_t expected =
        sizeof(std::uint64_t)          // seq
        + sizeof(double)               // t_s
        + 4 * 7 * sizeof(double)       // q, dq, tau_J, tau_cmd
        + 3 * sizeof(double)           // ee_pos
        + 4 * sizeof(double)           // ee_quat_xyzw
        + 3 * sizeof(double)           // target_pos
        + 4 * sizeof(double)           // target_quat_xyzw
        + 2 * sizeof(std::uint32_t)    // controller + pad
        + 31 * sizeof(double)          // joint targets/gains, alpha, two times
        + sizeof(std::uint64_t)        // reset counter
        + 2 * sizeof(std::uint32_t);   // joint validity + compensation
    CHECK(sizeof(RingLogFrame) == expected, "RingLogFrame has no interior padding");
}

// ============================================================================
// Producer contract
// ============================================================================
static void test_push_is_noexcept() {
    section("producer contract");
    RingLog ring(8);
    RingLogFrame f = make_frame(0);
    CHECK(noexcept(ring.push(f)), "push() is noexcept (safe in the RT callback)");
    RingLogFrame out{};
    CHECK(noexcept(ring.pop(out)), "pop() is noexcept");
}

static void test_roundtrip_preserves_frame() {
    section("roundtrip");
    RingLog ring(8);
    RingLogFrame out{};
    CHECK(!ring.pop(out), "pop() on an empty ring returns false");

    const RingLogFrame in = make_frame(42);
    CHECK(ring.push(in), "push() into an empty ring succeeds");
    CHECK(ring.pop(out), "pop() after a push succeeds");
    CHECK(frames_equal(in, out), "the popped frame is byte-identical to the pushed one");
    CHECK(!ring.pop(out), "the ring is empty again after draining");
    CHECK(ring.dropped() == 0, "no drops on an uncontended roundtrip");
}

static void test_capacity_and_overflow() {
    section("overflow");
    // Capacity is rounded up to a power of two; a ring of N slots holds N-1
    // frames so that full and empty stay distinguishable.
    RingLog ring(4);
    CHECK(ring.capacity() == 4, "capacity(4) stays 4 (already a power of two)");

    CHECK(ring.push(make_frame(0)), "fill 1/3");
    CHECK(ring.push(make_frame(1)), "fill 2/3");
    CHECK(ring.push(make_frame(2)), "fill 3/3");
    CHECK(!ring.push(make_frame(3)), "push() on a full ring returns false rather than blocking");
    CHECK(ring.dropped() == 1, "the rejected frame is counted as dropped");
    CHECK(!ring.push(make_frame(4)), "a second overflow also fails");
    CHECK(ring.dropped() == 2, "drops accumulate");

    // The frames already accepted must survive an overflow: the ring drops
    // the newest arrival, never a frame it already acknowledged.
    RingLogFrame out{};
    bool contiguous = true;
    for (std::uint64_t i = 0; i < 3; ++i) {
        if (!ring.pop(out) || !frames_equal(out, make_frame(i))) contiguous = false;
    }
    CHECK(contiguous, "frames accepted before the overflow are retained in order");
    CHECK(!ring.pop(out), "nothing else is buffered");
}

static void test_capacity_rounds_up_to_power_of_two() {
    section("capacity rounding");
    CHECK(RingLog(5).capacity() == 8, "capacity(5) rounds up to 8");
    CHECK(RingLog(1000).capacity() == 1024, "capacity(1000) rounds up to 1024");
    CHECK(RingLog(0).capacity() >= 2, "capacity(0) is clamped to a usable minimum");
}

static void test_wraparound() {
    section("wraparound");
    RingLog ring(4);
    bool ok = true;
    // Push and drain one frame at a time for several laps around the buffer;
    // the indices must wrap without corrupting anything.
    for (std::uint64_t i = 0; i < 100; ++i) {
        if (!ring.push(make_frame(i))) { ok = false; break; }
        RingLogFrame out{};
        if (!ring.pop(out) || !frames_equal(out, make_frame(i))) { ok = false; break; }
    }
    CHECK(ok, "100 single-frame laps wrap cleanly with no drops or corruption");
    CHECK(ring.dropped() == 0, "no drops while the consumer keeps up");
}

// ============================================================================
// The actual contract: concurrent SPSC use
// ============================================================================
static void test_concurrent_spsc_preserves_order() {
    section("concurrent SPSC");
    constexpr std::uint64_t kFrames = 200000;
    RingLog ring(1024);

    std::atomic<bool> producer_done{false};
    std::uint64_t     pushed = 0;

    std::thread producer([&]() {
        for (std::uint64_t i = 0; i < kFrames; ++i) {
            if (ring.push(make_frame(i))) ++pushed;
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::vector<std::uint64_t> received;
    received.reserve(kFrames);
    bool payload_ok = true;
    RingLogFrame out{};
    while (!producer_done.load(std::memory_order_acquire) || ring.size() > 0) {
        while (ring.pop(out)) {
            if (!frames_equal(out, make_frame(out.seq))) payload_ok = false;
            received.push_back(out.seq);
        }
    }
    producer.join();
    while (ring.pop(out)) {
        if (!frames_equal(out, make_frame(out.seq))) payload_ok = false;
        received.push_back(out.seq);
    }

    CHECK(payload_ok, "every received frame's payload matches its sequence number");
    CHECK(received.size() == pushed,
          "the consumer receives exactly the frames the producer accepted");
    CHECK(pushed + ring.dropped() == kFrames,
          "accepted + dropped accounts for every frame the producer offered");

    bool ascending = true;
    for (std::size_t i = 1; i < received.size(); ++i)
        if (received[i] <= received[i - 1]) { ascending = false; break; }
    CHECK(ascending, "received sequence numbers are strictly ascending (FIFO preserved)");
}


// ============================================================================
// CSV writer
// ============================================================================
static std::vector<std::string> read_lines(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream in(path);
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);
    return lines;
}

static void test_writer_roundtrip() {
    section("writer roundtrip");
    const std::string path = "/tmp/fr3_ring_log_test.csv";
    std::remove(path.c_str());

    constexpr std::uint64_t kN = 500;
    {
        RingLogWriter writer(path, 1024);
        // The writer opens the file on its own thread; give it a moment so
        // ok() reflects the real open result rather than the initial false.
        for (int i = 0; i < 200 && !writer.ok(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        CHECK(writer.ok(), "writer opens its file and reports ok()");
        CHECK(!writer.complete(), "open writer is not a completed recording");
        CHECK(writer.path() == path, "writer reports the path it was given");

        for (std::uint64_t i = 0; i < kN; ++i) {
            while (!writer.ring().push(make_frame(i)))
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        writer.stop();
        CHECK(writer.complete(), "successful flush and close mark I/O complete");
        CHECK(writer.discarded() == 0, "healthy recording discards no frames");
        CHECK(writer.written() == kN, "every pushed frame is written");
        CHECK(writer.dropped() == 0, "no drops when the producer yields on full");
        writer.stop();  // must be safe twice
        CHECK(true, "stop() is idempotent");
    }

    const std::vector<std::string> lines = read_lines(path);
    CHECK(lines.size() == kN + 1, "file has one header line plus one row per frame");
    CHECK(!lines.empty() && lines[0] == RingLogWriter::csv_header(),
          "first line is the documented CSV header");

    // Column count must match the header, or downstream parsing silently
    // mis-assigns fields.
    const auto count_cols = [](const std::string& s) {
        std::size_t n = 1;
        for (char c : s) if (c == ',') ++n;
        return n;
    };
    bool widths_ok = true;
    for (std::size_t i = 1; i < lines.size(); ++i)
        if (count_cols(lines[i]) != count_cols(lines[0])) { widths_ok = false; break; }
    CHECK(widths_ok, "every row has exactly as many columns as the header");
    CHECK(count_cols(lines[0]) == 80, "CSV v2 appends 35 columns to the original 45");
    bool joint_payload_ok = true;
    for (std::uint64_t i = 0; i < kN; ++i) {
        std::istringstream names(lines[0]), values(lines[i + 1]);
        std::unordered_map<std::string, double> row;
        std::string name, value;
        while (std::getline(names, name, ',') && std::getline(values, value, ','))
            row[name] = std::stod(value);
        const auto f = make_frame(i);
        for (int j = 0; j < 7; ++j) {
            const auto n = std::to_string(j);
            joint_payload_ok &= row.at("q_target" + n) == f.q_target[j];
            joint_payload_ok &= row.at("q_target_filtered" + n) == f.q_target_filtered[j];
            joint_payload_ok &= row.at("K_joint" + n) == f.K_joint[j];
            joint_payload_ok &= row.at("D_joint" + n) == f.D_joint[j];
        }
        joint_payload_ok &= row.at("schema_version") == 2;
        joint_payload_ok &= row.at("joint_target_valid") == 1;
        joint_payload_ok &= row.at("filter_alpha") == f.filter_alpha;
        joint_payload_ok &= row.at("joint_use_friction") == f.joint_use_friction;
        joint_payload_ok &= row.at("joint_reset_count") == f.joint_reset_count;
        joint_payload_ok &= std::abs(row.at("robot_time_s") - f.robot_time_s) < 1e-9;
        joint_payload_ok &= row.at("control_period_s") == f.control_period_s;
    }
    CHECK(joint_payload_ok, "all v2 target/configuration/timing columns survive CSV roundtrip");

    // Rows arrive in order and carry the right payload.
    bool order_ok = true, payload_ok = true;
    for (std::uint64_t i = 0; i < kN; ++i) {
        std::istringstream row(lines[static_cast<std::size_t>(i) + 1]);
        std::string cell;
        std::getline(row, cell, ',');
        if (std::stoull(cell) != i) { order_ok = false; break; }
        std::getline(row, cell, ',');                 // t_s
        if (std::abs(std::stod(cell) - 0.001 * static_cast<double>(i)) > 1e-12)
            payload_ok = false;
        std::getline(row, cell, ',');                 // q0 == seq
        if (std::abs(std::stod(cell) - static_cast<double>(i)) > 1e-9)
            payload_ok = false;
    }
    CHECK(order_ok, "rows are written in ascending seq order");
    CHECK(payload_ok, "t_s and q0 survive the text round trip");

    std::remove(path.c_str());
}

static void test_writer_reports_gaps() {
    section("writer gap accounting");
    const std::string path = "/tmp/fr3_ring_log_gaps.csv";
    std::remove(path.c_str());

    // A deliberately tiny ring with a producer that refuses to wait: this is
    // the RT contract, so some frames must be dropped and counted rather
    // than stalling the producer.
    RingLogWriter writer(path, 4);
    for (int i = 0; i < 200 && !writer.ok(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::uint64_t offered = 0, accepted = 0;
    for (std::uint64_t i = 0; i < 100000; ++i) {
        ++offered;
        if (writer.ring().push(make_frame(i))) ++accepted;
    }
    writer.stop();

    CHECK(!writer.complete() && writer.ok(), "overflow invalidates completeness, not I/O health");
    CHECK(writer.dropped() > 0, "a saturated tiny ring does drop frames");
    CHECK(accepted + writer.dropped() == offered,
          "written + dropped accounts for every offered frame");
    CHECK(writer.written() == accepted,
          "the file contains exactly the accepted frames");

    std::remove(path.c_str());
}

static void test_writer_survives_bad_path() {
    section("writer bad path");
    // An unwritable path must not take the daemon down, and must not leave
    // the RT side pushing into a ring nobody drains.
    RingLogWriter writer("/nonexistent-directory-fr3/ring.csv", 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    CHECK(!writer.ok(), "a failed open is reported through ok(), not thrown");

    for (std::uint64_t i = 0; i < 10000; ++i) writer.ring().push(make_frame(i));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const std::size_t backlog = writer.ring().size();
    writer.stop();
    CHECK(backlog == 0, "failed-open writer continues to drain the ring");
    CHECK(writer.error() == RingLogWriter::Error::Open && writer.error_code() == ENOENT,
          "open failure retains operation and errno");
    CHECK(writer.finished() && !writer.complete(), "open failure cannot finalize successfully");
}

// Each fixture runs alone. The open gate queues all three rows before the
// writer can drain, making batch-flush/final-flush injection deterministic.
static std::atomic<bool> io_open_gate{false};
static int io_write_calls, io_flush_calls, io_close_calls;
static int io_short_write_at, io_flush_failure_at;
static bool io_close_failure;

static std::FILE* injected_open(const char* path, const char* mode) {
    while (!io_open_gate.load(std::memory_order_acquire))
        std::this_thread::yield();
    return std::fopen(path, mode);
}
static std::size_t injected_write(const void* p, std::size_t size,
                                  std::size_t count, std::FILE* f) {
    if (++io_write_calls == io_short_write_at) {
        const auto n = std::fwrite(p, size, count - 1, f);
        errno = ENOSPC;
        return n;
    }
    return std::fwrite(p, size, count, f);
}
static int injected_flush(std::FILE* f) {
    const int result = std::fflush(f);
    if (++io_flush_calls == io_flush_failure_at) {
        errno = EIO;
        return EOF;
    }
    return result;
}
static int injected_close(std::FILE* f) {
    ++io_close_calls;
    const int result = std::fclose(f);
    if (io_close_failure) { errno = EBADF; return EOF; }
    return result;
}

static void test_writer_injected_failure(const char* name, int write_at,
                                         int flush_at, bool close_failure,
                                         RingLogWriter::Error expected,
                                         std::uint64_t expected_written) {
    section(name);
    const std::string path = "/tmp/fr3_ring_log_injected.csv";
    io_open_gate.store(false);
    io_write_calls = io_flush_calls = io_close_calls = 0;
    io_short_write_at = write_at;
    io_flush_failure_at = flush_at;
    io_close_failure = close_failure;
    fr3_stack::RingLogFileOps ops{injected_open, injected_write, injected_flush, injected_close};
    RingLogWriter writer(path, 16, ops);
    for (std::uint64_t i = 0; i < 3; ++i) writer.ring().push(make_frame(i));
    io_open_gate.store(true, std::memory_order_release);
    writer.stop();
    CHECK(!writer.ok() && writer.finished() && !writer.complete(),
          "I/O failure latches invalid through shutdown");
    CHECK(writer.error() == expected, "first failure operation survives cleanup");
    const int expected_errno = write_at ? ENOSPC : flush_at ? EIO : EBADF;
    CHECK(writer.error_code() == expected_errno, "first errno survives cleanup");
    CHECK(writer.written() == expected_written, "only successfully flushed rows count as written");
    CHECK(writer.written() + writer.discarded() == 3 && writer.dropped() == 0,
          "every accepted frame is accounted for after failure");
    CHECK(writer.ring().size() == 0, "failure path drains all remaining frames");
    writer.stop();
    CHECK(io_close_calls == 1, "failed stream is closed exactly once, including repeated stop");
    std::remove(path.c_str());
}

static void test_writer_dev_full() {
    section("real filesystem write failure");
#ifdef __linux__
    RingLogWriter writer("/dev/full", 16);
    for (std::uint64_t i = 0; i < 3; ++i) writer.ring().push(make_frame(i));
    writer.stop();
    CHECK(!writer.ok() && !writer.complete(), "/dev/full cannot report a successful recording");
    CHECK(writer.error_code() == ENOSPC, "/dev/full records disk-full errno");
    CHECK(writer.written() == 0 && writer.discarded() == 3,
          "failed header flush cannot count any data row as written");
#endif
}

int main() {
    test_frame_is_pod();
    test_push_is_noexcept();
    test_roundtrip_preserves_frame();
    test_capacity_and_overflow();
    test_capacity_rounds_up_to_power_of_two();
    test_wraparound();
    test_concurrent_spsc_preserves_order();

    test_writer_roundtrip();
    test_writer_reports_gaps();
    test_writer_survives_bad_path();
    using E = RingLogWriter::Error;
    test_writer_injected_failure("short header", 1, 0, false, E::HeaderWrite, 0);
    test_writer_injected_failure("short row", 2, 0, false, E::Write, 0);
    test_writer_injected_failure("unflushed prefix then short row", 3, 0, true, E::Write, 0);
    test_writer_injected_failure("header flush", 0, 1, false, E::Flush, 0);
    test_writer_injected_failure("batch flush", 0, 2, true, E::Flush, 0);
    test_writer_injected_failure("final flush", 0, 3, false, E::Flush, 3);
    test_writer_injected_failure("close", 0, 0, true, E::Close, 3);
    test_writer_dev_full();

    std::cout << "\n========== " << g_pass << " passed, " << g_fail
              << " failed ==========\n";
    return g_fail == 0 ? 0 : 1;
}
