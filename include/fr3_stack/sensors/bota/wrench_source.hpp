// Vendor-neutral interface for an off-RT 6-axis force/torque sensor source.
//
// Concrete backends (Bota / ATI / Robotiq / ...) live in their own .cpp/.hpp
// pair and inherit from WrenchSource. main.cpp sees only this header and the
// factory below; it never includes vendor SDKs.
//
// All backends must:
//   * own a worker thread that polls the sensor (vendor SDKs typically block
//     on EtherCAT / serial / network I/O — must NOT happen on libfranka's
//     1 kHz RT thread);
//   * publish the latest frame under a small mutex;
//   * expose read() which RT calls via try_lock — never block.
//
// The wrench published by read() is in the SENSOR FRAME, NOT compensated for
// gravity on a tool past the sensor. The caller rotates into the base frame
// using R_base_sensor — see sensors/wrench_frame.hpp for the helper that
// looks up the mount pose via libfranka and applies the sensor's optional
// fixed body-frame rotation.

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>

class WrenchSource {
 public:
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    virtual ~WrenchSource() = default;

    // Configure → start the worker thread. Throws std::runtime_error on
    // hardware / state-machine failure. Backends must NOT run an internal
    // tare — the published stream carries the sensor's electrical zero and
    // the offline calibration's f_bias absorbs it.
    virtual void start() = 0;

    // Stop the worker, deactivate the sensor. Idempotent and noexcept.
    virtual void stop() noexcept = 0;

    // RT-safe. Returns true and writes [Fx,Fy,Fz,Tx,Ty,Tz] (sensor frame) to
    // `out` iff (a) at least one frame has been published and (b) the
    // publication mutex was acquired without blocking. Returns false on
    // contention OR not-yet-ready (caller should reuse its previous value).
    virtual bool read(Vector6d& out) const = 0;

    // Vendor identifier ("bota", "ati", ...). Used in log messages.
    virtual const char* kind() const = 0;

    // Libfranka frame on which this sensor is rigidly mounted. Returned as
    // a string so this header stays libfranka-free (ft_dump and other
    // standalone tools shouldn't pull franka/model.h). Allowed values:
    //   "flange"        — kFlange (the mechanical flange / link8)
    //   "end_effector"  — kEndEffector (Desk-configured EE / TCP)
    //   "stiffness"     — kStiffness
    //   "joint1".."joint7"
    // sensors/wrench_frame.hpp resolves the string to franka::Frame.
    virtual const char* mount_frame_name() const = 0;

    // Fixed rotation from the mount frame's axes to the sensor body axes.
    // Identity when the sensor body is aligned with its mount (e.g. bota
    // bolted to the flange with X/Y matching the flange's X/Y). Override
    // for sensors mounted via an adapter that turns them (e.g. an ATI on
    // a 45° bracket). R_base_sensor = R_O_mount · R_mount_sensor.
    virtual Eigen::Matrix3d R_mount_sensor() const {
        return Eigen::Matrix3d::Identity();
    }
};

// Factory. `kind` selects the backend; `config` is a vendor-specific string
// (Bota: full path to driver JSON; ATI: TBD; ...). The caller still needs to
// call start() on the returned object — the factory only constructs.
//
// Throws std::runtime_error for unknown kind. Vendor-specific construction
// failures propagate from the concrete constructor.
//
// To add a new sensor:
//   1) write nuc/<vendor>_wrench_source.{hpp,cpp}, declaring a class that
//      inherits from WrenchSource;
//   2) register the kind string in wrench_source.cpp;
//   3) add the .cpp to nuc/CMakeLists.txt and link any vendor library there.
std::unique_ptr<WrenchSource> make_wrench_source(
    const std::string& kind,
    const std::string& config);
