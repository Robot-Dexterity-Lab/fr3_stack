#include <fr3_stack/recording.hpp>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

static int passed = 0, failed = 0;
#define CHECK(condition, message) do { \
    if (condition) { ++passed; } \
    else { ++failed; std::cerr << "FAIL: " << message << '\n'; } \
} while (false)

static fr3_stack::RingLogFrame frame(std::uint64_t seq, double t = 10.0) {
    fr3_stack::RingLogFrame f{};
    f.seq = seq; f.t_s = t;
    return f;
}

static bool throws_start(fr3_stack::RecordingManager& m, const std::string& id,
                         const std::string& path) {
    try { m.start(id, path); } catch (const std::exception&) { return true; }
    return false;
}

static int fail_close(std::FILE* file) {
    std::fclose(file);
    errno = EIO;
    return EOF;
}

int main() {
    char directory[] = "/tmp/fr3-recording-test-XXXXXX";
    if (!mkdtemp(directory)) return 1;
    const std::string root(directory);
    std::vector<std::string> files;
    auto path = [&](const std::string& name) { files.push_back(root + "/" + name); return files.back(); };
    {
        fr3_stack::RecordingManager m;
        int fills = 0;
        m.record([&](auto seq) { ++fills; return frame(seq); });
        CHECK(fills == 0 && !m.status().active, "off by default; capture is not evaluated");
        CHECK(throws_start(m, "", path("bad.csv")), "empty run ID rejected");
        CHECK(throws_start(m, "id", "relative.csv"), "relative path rejected");
        const auto p1 = path("one.csv");
        auto s = m.start("one", p1);
        CHECK(s.active && s.ok && !s.finished && !s.complete, "start acknowledges open before capture");
        CHECK(m.start("one", p1).active, "duplicate start is idempotent");
        CHECK(throws_start(m, "two", path("busy.csv")), "active recording is not replaced");
        for (int i = 0; i < 5; ++i)
            m.record([&](auto seq) { return frame(seq, 10.0 + i * 0.001); });
        bool mismatch = false;
        try { m.stop("other"); } catch (const std::exception&) { mismatch = true; }
        CHECK(mismatch && m.status().active, "stale stop cannot stop current session");
        s = m.stop("one");
        CHECK(!s.active && s.finished && s.complete && s.written == 5, "stop drains and closes with exact count");
        CHECK(m.stop("one").written == 5, "repeated stop is idempotent");
        CHECK(!m.start("one", p1).active, "duplicate completed start cannot restart recording");
        m.record([&](auto seq) { ++fills; return frame(seq); });
        CHECK(fills == 0, "capture remains disabled after stop");
        m.start("two", path("two.csv"));
        m.record([](auto seq) { return frame(seq, 20.0); });
        s = m.stop("two");
        CHECK(s.complete && s.written == 1 && s.dropped == 0, "second recording resets ring counters");
        CHECK(throws_start(m, "one", p1), "old request cannot reopen an earlier recording");
        std::ifstream input(s.path);
        std::string header, row;
        std::getline(input, header); std::getline(input, row);
        CHECK(row.rfind("0,0,", 0) == 0, "second recording resets sequence and relative time");
        auto refused = m.start("existing", p1);
        CHECK(!refused.active && !refused.ok && refused.error_code == EEXIST,
              "existing files are rejected without truncation");
        std::ifstream existing(p1); int rows = 0;
        while (std::getline(existing, row)) ++rows;
        CHECK(rows == 6, "original recording remains intact");
        CHECK(m.had_failed_recordings(), "I/O failure retained for daemon exit status");
        m.start("recovered", path("recovered.csv"));
        CHECK(m.stop("recovered").complete, "new session may start after failed file open");
    }
    {
        // Force stop to overlap a live capture; it must wait for this one push
        // while leaving the producer/controller free to continue after release.
        fr3_stack::RecordingManager m;
        m.start("overlap", path("overlap.csv"));
        std::atomic<bool> entered{false}, release{false}, stopped{false};
        std::thread producer([&]() {
            m.record([&](auto seq) {
                entered.store(true);
                while (!release.load()) std::this_thread::yield();
                return frame(seq);
            });
        });
        while (!entered.load()) std::this_thread::yield();
        fr3_stack::RecordingStatus result;
        std::thread stopper([&]() { result = m.stop("overlap"); stopped.store(true); });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CHECK(!stopped.load(), "stop cannot close a writer during an in-flight push");
        release.store(true);
        producer.join(); stopper.join();
        CHECK(result.complete && result.written == 1, "in-flight frame is included before stop acknowledgement");
    }
    {
        fr3_stack::RecordingManager m;
        std::atomic<bool> done{false};
        std::atomic<unsigned> ticks{0};
        std::thread producer([&]() {
            while (!done.load()) {
                ++ticks;
                m.record([](auto seq) { return frame(seq); });
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });
        bool sessions_ok = true;
        for (int n = 0; n < 20; ++n) {
            const auto id = "repeat-" + std::to_string(n);
            m.start(id, path(id + ".csv"));
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            auto s = m.stop(id);
            sessions_ok &= s.complete && !s.active && s.written > 0;
        }
        const auto before = ticks.load();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        CHECK(ticks.load() > before, "producer continues while recording is stopped");
        done.store(true); producer.join();
        CHECK(sessions_ok, "concurrent producer survives repeated start/stop with clean per-run data");
    }
    {
        auto ops = fr3_stack::exclusive_recording_file_ops();
        ops.close = fail_close;
        fr3_stack::RecordingManager m(16, ops);
        m.start("close-failure", path("close-failure.csv"));
        m.record([](auto seq) { return frame(seq); });
        const auto s = m.stop("close-failure");
        CHECK(!s.active && s.finished && !s.ok && !s.complete && s.error_code == EIO,
              "stop acknowledges finalization but exposes close failure");
        CHECK(s.error.find("close") != std::string::npos && m.had_failed_recordings(),
              "close failure retained in status and daemon outcome");
    }
    for (const auto& f : files) std::remove(f.c_str());
    std::remove(root.c_str());
    std::cout << passed << " passed, " << failed << " failed\n";
    return failed ? 1 : 0;
}
