#include <fr3_stack/ring_log_writer.hpp>

#include <chrono>
#include <cstdio>
#include <cerrno>
#include <cstring>

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
           "controller,schema_version,joint_target_valid,"
           "q_target0,q_target1,q_target2,q_target3,q_target4,q_target5,q_target6,"
           "q_target_filtered0,q_target_filtered1,q_target_filtered2,q_target_filtered3,"
           "q_target_filtered4,q_target_filtered5,q_target_filtered6,"
           "K_joint0,K_joint1,K_joint2,K_joint3,K_joint4,K_joint5,K_joint6,"
           "D_joint0,D_joint1,D_joint2,D_joint3,D_joint4,D_joint5,D_joint6,"
           "filter_alpha,joint_use_friction,joint_reset_count,robot_time_s,control_period_s";
}

RingLogWriter::RingLogWriter(const std::string& path, std::size_t capacity_frames,
                             RingLogFileOps file_ops)
    : owned_ring_(std::make_unique<RingLog>(capacity_frames)), ring_(*owned_ring_),
      path_(path), file_ops_(file_ops) {
    thread_ = std::thread([this]() { run(); });
}

RingLogWriter::RingLogWriter(RingLog& ring, const std::string& path, RingLogFileOps file_ops)
    : ring_(ring), path_(path), file_ops_(file_ops) {
    thread_ = std::thread([this]() { run(); });
}

RingLogWriter::~RingLogWriter() { stop(); }

void RingLogWriter::stop() noexcept {
    if (stopped_.exchange(true)) return;
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
}

const char* RingLogWriter::error_name(Error error) noexcept {
    switch (error) {
        case Error::None: return "none";
        case Error::Open: return "open";
        case Error::HeaderWrite: return "header write";
        case Error::Write: return "row write";
        case Error::Flush: return "flush";
        case Error::Close: return "close";
    }
    return "unknown";
}

void RingLogWriter::fail(Error error, int code) noexcept {
    // Only the writer thread publishes errors. Preserve the first cause.
    if (error_.load(std::memory_order_relaxed) != Error::None) return;
    error_code_.store(code ? code : EIO, std::memory_order_relaxed);
    error_.store(error, std::memory_order_release);
    ok_.store(false, std::memory_order_release);
    std::fprintf(stderr, "[ring-log] ERROR: %s '%s': %s; recording INVALID\n",
                 error_name(error), path_.c_str(), std::strerror(code ? code : EIO));
}

void RingLogWriter::run() {
    errno = 0;
    std::FILE* f = file_ops_.open(path_.c_str(), "w");
    if (!f) {
        fail(Error::Open, errno);
    } else {
        const std::string header = std::string(csv_header()) + '\n';
        errno = 0;
        if (file_ops_.write(header.data(), 1, header.size(), f) != header.size()
            || std::ferror(f)) {
            fail(Error::HeaderWrite, errno);
        } else {
            errno = 0;
            if (file_ops_.flush(f) != 0) fail(Error::Flush, errno);
            else ok_.store(true, std::memory_order_release);
        }
    }

    std::string line;
    line.reserve(2048);
    char scratch[96];
    const auto drain = [&]() {
        std::uint64_t pending = 0;
        RingLogFrame fr{};
        // Bound each batch so continuous arrivals cannot defer flush forever.
        for (int n = 0; n < 512 && ring_.pop(fr); ++n) {
            if (!ok()) {
                discarded_.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            line.clear();
            std::snprintf(scratch, sizeof(scratch), "%llu,%.*g",
                          static_cast<unsigned long long>(fr.seq), kPrecision, fr.t_s);
            line += scratch;
            append_doubles(line, fr.q, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.dq, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.tau_J, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.tau_cmd, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.ee_pos, 3, scratch, sizeof(scratch));
            append_doubles(line, fr.ee_quat_xyzw, 4, scratch, sizeof(scratch));
            append_doubles(line, fr.target_pos, 3, scratch, sizeof(scratch));
            append_doubles(line, fr.target_quat_xyzw, 4, scratch, sizeof(scratch));
            std::snprintf(scratch, sizeof(scratch), ",%u,2,%u", fr.controller, fr.joint_target_valid);
            line += scratch;
            append_doubles(line, fr.q_target, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.q_target_filtered, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.K_joint, 7, scratch, sizeof(scratch));
            append_doubles(line, fr.D_joint, 7, scratch, sizeof(scratch));
            append_doubles(line, &fr.filter_alpha, 1, scratch, sizeof(scratch));
            std::snprintf(scratch, sizeof(scratch), ",%u,%llu", fr.joint_use_friction,
                          static_cast<unsigned long long>(fr.joint_reset_count));
            line += scratch;
            append_doubles(line, &fr.robot_time_s, 1, scratch, sizeof(scratch));
            append_doubles(line, &fr.control_period_s, 1, scratch, sizeof(scratch));
            line += '\n';
            errno = 0;
            if (file_ops_.write(line.data(), 1, line.size(), f) != line.size()
                || std::ferror(f)) {
                fail(Error::Write, errno);
                discarded_.fetch_add(pending + 1, std::memory_order_relaxed);
                pending = 0;
            } else {
                ++pending;
            }
        }
        if (pending != 0) {
            errno = 0;
            if (file_ops_.flush(f) != 0) {
                fail(Error::Flush, errno);
                discarded_.fetch_add(pending, std::memory_order_relaxed);
            } else {
                written_.fetch_add(pending, std::memory_order_relaxed);
            }
        }
    };

    // Failed files still drain and count discarded frames. Nothing on the RT
    // producer performs filesystem operations, reports errors, or retries.
    while (!stop_.load(std::memory_order_acquire)) {
        drain();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    while (ring_.size() != 0) drain();  // producer stopped before stop()

    if (f) {
        errno = 0;
        if (file_ops_.flush(f) != 0) fail(Error::Flush, errno);
        errno = 0;
        if (file_ops_.close(f) != 0) fail(Error::Close, errno);
    }
    finished_.store(true, std::memory_order_release);
    std::fprintf(stderr,
        "[ring-log] %s: flushed=%llu discarded=%llu dropped=%llu path='%s'\n",
        complete() ? "I/O complete" : "INVALID recording",
        static_cast<unsigned long long>(written()),
        static_cast<unsigned long long>(discarded()),
        static_cast<unsigned long long>(dropped()), path_.c_str());
}

}  // namespace fr3_stack
