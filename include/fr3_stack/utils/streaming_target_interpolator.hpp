#pragma once

// Linear / SLERP interpolator that bridges a low-rate streaming client
// (e.g. SpaceMouse at 100-200 Hz, a learning policy at 5-20 Hz) to the
// daemon's 1 kHz controller tick.
//
// Why this exists
// ---------------
// Without interpolation, a client streaming at *N* Hz produces an *N* Hz
// **step train** at the daemon: cfg_.target stays constant for 1000/N
// ticks, then jumps. The controller's first-order LP smoother
// (filter_alpha=0.05, τ ≈ 20 ms) blurs the steps but leaves derivative
// kinks at every step boundary. The kink series concentrates spectral
// energy at the client rate (and harmonics) — directly in the audible
// band — and the Jacobian routes that into joint 0 for any horizontal
// EE motion, producing the "sawtooth" chatter on FR3.
//
// Linear interpolation between the two most recent received targets
// removes the step train entirely: any client rate produces a continuous,
// piecewise-linear (resp. piecewise-SLERP) target at the daemon tick.
// Equivalent to deoxys's LinearPoseTrajInterpolator and the LERP half of
// UMI's PoseTrajectoryInterpolator.
//
// Semantics (the subtle part)
// ---------------------------
// At time t we don't yet know what the *next* received target will be,
// so "interpolate between current and next" is impossible. Instead we
// run **one segment behind**:
//
//   * Each push records the new latest target and starts a new segment
//     from "where the daemon was currently aiming" toward that latest,
//     over the same duration as the previous inter-push interval.
//   * Each tick evaluates lerp(prev, latest, α) where
//     α = clamp((t_now − t_segment_start) / segment_duration, 0, 1).
//   * α saturates at 1 past the segment end → output holds at latest if
//     no new command arrives. Never extrapolates.
//
// Effect: at the moment a new command arrives, the segment starts at
// α = 0 (= where the daemon already was → smooth handoff, no jump),
// and α reaches 1 around the time the next command would normally
// arrive (assuming roughly steady client rate). If a new command
// arrives early, the next segment starts from the *current
// interpolated value* (snapshot before mutating state), still no jump.
// If a command arrives late, the daemon holds at latest.
//
// Trade-off: one segment of latency (≈ 1 / client_rate). At 200 Hz
// client that's 5 ms. For teleop this is well under sensorimotor
// thresholds; for a 10 Hz policy chunk it's 100 ms — there you'd want
// a fixed shorter segment_max (a future knob).
//
// Thread model
// ------------
// Not internally synchronized. The caller (RT thread) owns it and
// calls push() + evaluate() from the same thread.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>

namespace fr3_stack {

class StreamingTargetInterpolator {
public:
    void reset() {
        primed_ = false;
        t_last_push_ = 0.0;
    }

    bool primed() const { return primed_; }

    // Record a received target sampled at t_recv (seconds, monotonic).
    void push(const Eigen::Affine3d& T, double t_recv) {
        Eigen::Vector3d    p_new = T.translation();
        Eigen::Quaterniond q_new(T.linear());

        // Hemisphere flip so SLERP takes the short way around SO(3).
        if (latest_q_.coeffs().dot(q_new.coeffs()) < 0.0)
            q_new.coeffs() = -q_new.coeffs();

        if (!primed_) {
            // First push: the segment is degenerate (zero length) —
            // evaluate() will return latest until a second push gives
            // us a real interval to plan against.
            prev_p_   = latest_p_   = p_new;
            prev_q_   = latest_q_   = q_new;
            t_start_  = t_end_      = t_recv;
            t_last_push_ = t_recv;
            primed_   = true;
            return;
        }

        // Snapshot the *current* interpolated value as the new segment's
        // start. Crucial: if a new command arrives mid-segment, this
        // prevents a jump back to the previous latest. If the previous
        // segment had already settled (α=1 clamped), this just equals
        // latest, which is what we want.
        Eigen::Affine3d cur = evaluate(t_recv);
        prev_p_ = cur.translation();
        prev_q_ = Eigen::Quaterniond(cur.linear());

        latest_p_ = p_new;
        latest_q_ = q_new;

        // Estimate the new segment's duration from the previous
        // inter-push interval — assumes roughly steady client rate.
        // Bounded: too short risks numerical issues; too long lets
        // the daemon coast toward a stale target across a real pause.
        double interval = t_recv - t_last_push_;
        if (interval < 1e-3) interval = 1e-3;   // 1 ms floor
        if (interval > 0.050) interval = 0.050; // 50 ms ceiling

        t_start_ = t_recv;
        t_end_   = t_recv + interval;
        t_last_push_ = t_recv;
    }

    // Evaluate the interpolated target at t_now. Caller must check
    // primed() first; otherwise returns identity (sentinel).
    Eigen::Affine3d evaluate(double t_now) const {
        Eigen::Affine3d out = Eigen::Affine3d::Identity();
        if (!primed_) return out;

        const double dt    = t_end_ - t_start_;
        const double alpha = (dt > 1e-9)
            ? std::clamp((t_now - t_start_) / dt, 0.0, 1.0)
            : 1.0;

        out.translation() = prev_p_ + alpha * (latest_p_ - prev_p_);
        out.linear()      = prev_q_.slerp(alpha, latest_q_).toRotationMatrix();
        return out;
    }

private:
    Eigen::Vector3d    prev_p_   {Eigen::Vector3d::Zero()};
    Eigen::Vector3d    latest_p_ {Eigen::Vector3d::Zero()};
    Eigen::Quaterniond prev_q_   {Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond latest_q_ {Eigen::Quaterniond::Identity()};
    // Wall-clock window for the current segment. evaluate() ramps α
    // from 0 to 1 as t_now traverses [t_start_, t_end_].
    double             t_start_     {0.0};
    double             t_end_       {0.0};
    // Wall-clock of the most recent push — used to derive the next
    // segment's duration from the observed inter-push interval.
    double             t_last_push_ {0.0};
    bool               primed_      {false};
};

}  // namespace fr3_stack
