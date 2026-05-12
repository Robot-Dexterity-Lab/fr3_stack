// Bota Systems FT sensor backend for the WrenchSource interface.
//
// See wrench_source.hpp for the contract every backend must satisfy. This
// concrete implementation:
//   * owns a worker thread that polls bota_driver_cpp at ~1 kHz;
//   * publishes the latest raw wrench under pub_mu_;
//   * is RT-safe via try_lock in read().
//
// The published wrench is sensor frame, NOT payload-compensated (this lib has
// the driver only, no HWI/compensator). No internal tare runs — raw stream
// carries the sensor's full electrical zero; offline calibration's f_bias
// absorbs it. Callers rotate into the base frame.
//
// pimpl'd on bota types so main.cpp doesn't need to pull in bota headers.

#pragma once

#include <fr3_stack/sensors/bota/wrench_source.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class BotaWrenchSource : public WrenchSource {
 public:
    // driver_config: path to the JSON consumed by bota::BotaDriver, e.g.
    //                ".../bota_driver_cpp/driver_config/ethercat.json".
    explicit BotaWrenchSource(std::string driver_config);
    ~BotaWrenchSource() override;

    BotaWrenchSource(const BotaWrenchSource&)            = delete;
    BotaWrenchSource& operator=(const BotaWrenchSource&) = delete;

    void start() override;
    void stop() noexcept override;
    bool read(Vector6d& out) const override;
    const char* kind() const override { return "bota"; }

    // The bota is bolted to the mechanical flange and its X/Y body axes are
    // aligned with the flange's. R_mount_sensor = I (the base-class default).
    const char* mount_frame_name() const override { return "flange"; }

 private:
    void worker_loop();

    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::string driver_config_;

    std::thread       worker_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> ready_{false};

    mutable std::mutex pub_mu_;
    Vector6d           latest_{Vector6d::Zero()};
};
