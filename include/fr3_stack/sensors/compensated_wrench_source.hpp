// Decorator that wraps any WrenchSource and subtracts payload gravity +
// constant bias before downstream consumers see the wrench. Output stays in
// the SENSOR frame (matches the WrenchSource contract — caller still rotates
// to base via R_O_EE).
//
// Math (sensor frame, matches fr3_stack/sensors/bota/_common.py):
//
//     R_O_sensor   = R_O_EE · R_ee_sensor       (constant offset from calib)
//     g_s          = R_O_sensor^T · [0, 0, -m·g]
//     f_compensated = f_raw - g_s            - f_bias
//     t_compensated = t_raw - com × g_s      - t_bias
//
// Threading:
//   * set_orientation(R) is called from the RT thread each tick BEFORE the
//     controller's read(). Uses try_lock — never blocks. If contended (rare;
//     the only other locker is read() itself), the previous R is retained
//     for one more tick. 1 ms staleness is well below what affects the
//     subsequent tick's force computation in any practical sense.
//   * read() also uses try_lock — same RT-safety pattern as BotaWrenchSource.
//
// Construction takes ownership of the wrapped source. start()/stop() delegate.

#pragma once

#include <fr3_stack/sensors/bota/wrench_source.hpp>
#include <fr3_stack/sensors/payload_calib.hpp>

#include <Eigen/Dense>
#include <memory>
#include <mutex>

inline constexpr double kStandardGravity = 9.80665;  // m/s² — match Python solver

class CompensatedWrenchSource : public WrenchSource {
 public:
    CompensatedWrenchSource(std::unique_ptr<WrenchSource> inner,
                            PayloadCalib                  calib);
    ~CompensatedWrenchSource() override;

    CompensatedWrenchSource(const CompensatedWrenchSource&)            = delete;
    CompensatedWrenchSource& operator=(const CompensatedWrenchSource&) = delete;

    void  start()                  override;
    void  stop()  noexcept         override;
    bool  read(Vector6d& out) const override;
    const char* kind()       const override;

    // Delegate mount metadata to the wrapped source — the compensator is a
    // transparent decorator from the frame-resolution perspective.
    const char* mount_frame_name() const override {
        return inner_->mount_frame_name();
    }
    Eigen::Matrix3d R_mount_sensor() const override {
        return inner_->R_mount_sensor();
    }

    // Set the EE orientation (R_O_EE) used to project the gravity vector
    // into the sensor frame. Pumped from the RT thread each tick. Try-lock,
    // non-blocking.
    void  set_orientation(const Eigen::Matrix3d& R);

    // Raw passthrough — main.cpp uses this to publish wrenchFtRaw alongside
    // the compensated stream so consumers can still see the uncompensated
    // signal (e.g. fr3-ft-plot's Raw / Both toggle).
    bool  read_raw(Vector6d& out) const;

    // Non-owning view of the wrapped source. main.cpp uses this when
    // --ft-controllers-raw routes admittance / hybrid past the compensator
    // while the state publisher continues to publish both columns through
    // the decorator.
    WrenchSource* inner() { return inner_.get(); }
    const WrenchSource* inner() const { return inner_.get(); }

    const PayloadCalib& calib() const { return calib_; }

 private:
    std::unique_ptr<WrenchSource> inner_;
    PayloadCalib                  calib_;
    double                        mg_;       // calib_.mass * kStandardGravity
    Eigen::Matrix3d               R_ee_sensor_;  // cached from calib_

    mutable std::mutex            orient_mu_;
    Eigen::Matrix3d               R_{Eigen::Matrix3d::Identity()};
};
