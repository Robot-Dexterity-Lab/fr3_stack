#include <fr3_stack/ring_log_writer.hpp>

#include <chrono>
#include <cstdio>
#include <iostream>

namespace fr3_stack {
namespace {

// Long enough that a double survives the round trip through text, which
// matters when the fit is chasing millimetres and milliradians.
constexpr int kPrecision = 12;

// Appends `n` doubles as ",v0,v1,..." -- snprintf rather than iostreams so
// the writer stays allocation-light on a per-frame path.
void append_doubles(std::string& out, const double* v, int n, char* scratch,
                    std::size_t scratch_len) {
    for (int i = 0; i < n; ++i) {
        std::snprintf(scratch, scratch_len, ",%.*g", kPrecision, v[i]);
        out += scratch;
    }
}

}  // namespace

const char* RingLogWriter::csv_header() noexcept {
    return "seq,t_s,"
           "q0,q1,q2,q3,q4,q5,q6,"
           "dq0,dq1,dq2,dq3,dq4,dq5,dq6,"
           "tau_J0,tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,"
           "tau_cmd0,tau_cmd1,tau_cmd2,tau_cmd3,tau_cmd4,tau_cmd5,tau_cmd6,"
           "ee_px,ee_py,ee_pz,ee_qx,ee_qy,ee_qz,ee_qw,"
           "tgt_px,tgt_py,tgt_pz,tgt_qx,tgt_qy,tgt_qz,tgt_qw,"
           "controller";
}

RingLogWriter::RingLogWriter(const std::string& path, std::size_t capacity_frames)
    : ring_(capacity_frames), path_(path) {
    thread_ = std::thread([this]() { run(); });
}

RingLogWriter::~RingLogWriter() { stop(); }

void RingLogWriter::stop() noexcept {
    if (stopped_.exchange(true)) return;
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
}

void RingLogWriter::run() {
    std::FILE* f = std::fopen(path_.c_str(), "w");
    if (!f) {
        std::cerr << "[ring-log] cannot open '" << path_
                  << "' for writing — 1 kHz logging is OFF for this run\n";
        ok_.store(false, std::memory_order_release);
        // Keep draining so the RT side never sees the ring fill up and
        // start counting drops for a log nobody is writing.
        while (!stop_.load(std::memory_order_acquire)) {
            RingLogFrame frame{};
            while (ring_.pop(frame)) {}
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return;
    }
    std::fprintf(f, "%s\n", csv_header());
    ok_.store(true, std::memory_order_release);

    std::string line;
    line.reserve(1024);
    char scratch[64];

    // `drain` empties whatever is buffered; the loop below alternates it
    // with a short sleep so an idle writer does not spin a core.
    const auto drain = [&]() {
        RingLogFrame fr{};
        while (ring_.pop(fr)) {
            line.clear();
            std::snprintf(scratch, sizeof(scratch), "%llu",
                          static_cast<unsigned long long>(fr.seq));
            line += scratch;
            std::snprintf(scratch, sizeof(scratch), ",%.*g", kPrecision, fr.t_s);
            line += scratch;
            append_doubles(line, fr.q, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.dq, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.tau_J, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.tau_cmd, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.ee_pos, 3, scratch, sizeof(scratch));
            append_doubles(line, fr.ee_quat_xyzw, 4, scratch, sizeof(scratch));
            append_doubles(line, fr.target_pos, 3, scratch, sizeof(scratch));
            append_doubles(line, fr.target_quat_xyzw, 4, scratch, sizeof(scratch));
            std::snprintf(scratch, sizeof(scratch), ",%u", fr.controller);
            line += scratch;
            line += '\n';
            std::fwrite(line.data(), 1, line.size(), f);
            written_.fetch_add(1, std::memory_order_relaxed);
        }
    };

    while (!stop_.load(std::memory_order_acquire)) {
        drain();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    drain();  // whatever the RT side pushed between the last drain and stop

    std::fflush(f);
    std::fclose(f);

    const std::uint64_t lost = ring_.dropped();
    std::cerr << "[ring-log] wrote " << written_.load() << " frames to '"
              << path_ << "'";
    if (lost > 0) {
        std::cerr << " — WARNING: " << lost
                  << " frame(s) dropped; the log has gaps (look for jumps in"
                     " the seq column) and is not a contiguous 1 kHz record";
    }
    std::cerr << "\n";
}

}  // namespace fr3_stack
