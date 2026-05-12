// Tiny logging helpers shared by the daemon and any controller that wants
// to emit one-shot warnings (e.g. on activation). Off-RT only — never call
// from inside a libfranka callback unless the call site has its own
// throttling, since formatting strings allocates and `std::cerr` is a
// blocking sink.

#pragma once

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>

inline std::string ts() {
    using namespace std::chrono;
    const auto now     = system_clock::now();
    const auto t       = system_clock::to_time_t(now);
    const auto ms_part = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d",
                  tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<int>(ms_part.count()));
    return std::string(buf);
}

inline std::string log_pfx() { return "[fr3 " + ts() + "] "; }

// Suppress consecutive duplicate messages within a window; flush a
// "(repeated N×)" summary on the next distinct message or window expiry.
// Used for parse-error floods (buggy clients) and per-tick guards inside
// RT controllers (1 kHz worst case).
class LogThrottle {
 public:
    void maybe_log(std::ostream& os, const std::string& msg,
                   std::chrono::milliseconds window = std::chrono::seconds(1)) {
        const auto now = std::chrono::steady_clock::now();
        if (msg == last_msg_ && (now - last_time_) < window) {
            ++suppressed_;
            return;
        }
        if (suppressed_ > 0) {
            os << log_pfx() << "(previous message repeated "
               << suppressed_ << "× within 1s)\n";
        }
        os << log_pfx() << msg << "\n";
        last_msg_   = msg;
        last_time_  = now;
        suppressed_ = 0;
    }

 private:
    std::string                              last_msg_;
    std::chrono::steady_clock::time_point    last_time_{};
    int                                      suppressed_{0};
};
