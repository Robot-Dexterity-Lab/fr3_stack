#include <fr3_stack/sensors/bota/bota_wrench_source.hpp>

#include "bota_driver.hpp"

#include <chrono>
#include <stdexcept>

struct BotaWrenchSource::Impl {
    std::unique_ptr<bota::BotaDriver> driver;
    bool                              active = false;
};

BotaWrenchSource::BotaWrenchSource(std::string driver_config)
    : impl_(std::make_unique<Impl>()),
      driver_config_(std::move(driver_config)) {}

BotaWrenchSource::~BotaWrenchSource() { stop(); }

void BotaWrenchSource::start() {
    impl_->driver = std::make_unique<bota::BotaDriver>(driver_config_);

    if (!impl_->driver->configure())
        throw std::runtime_error("bota: configure() failed");
    if (!impl_->driver->activate()) {
        (void)impl_->driver->shutdown();
        throw std::runtime_error("bota: activate() failed");
    }
    impl_->active = true;

    stop_   = false;
    worker_ = std::thread(&BotaWrenchSource::worker_loop, this);
}

void BotaWrenchSource::stop() noexcept {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    if (impl_ && impl_->active && impl_->driver) {
        try { (void)impl_->driver->deactivate(); } catch (...) {}
        try { (void)impl_->driver->shutdown();   } catch (...) {}
        impl_->active = false;
    }
}

bool BotaWrenchSource::read(Vector6d& out) const {
    if (!ready_.load(std::memory_order_acquire)) return false;
    std::unique_lock<std::mutex> lk(pub_mu_, std::try_to_lock);
    if (!lk.owns_lock()) return false;
    out = latest_;
    return true;
}

// Polls readFrame() (non-blocking buffer read) at 1 kHz — example1 pattern.
// We avoid readFrameBlocking() so stop() doesn't have to interrupt a blocked
// driver call: the worker checks stop_ every tick and exits cleanly.
void BotaWrenchSource::worker_loop() {
    using clock = std::chrono::steady_clock;
    auto next   = clock::now();
    while (!stop_.load(std::memory_order_relaxed)) {
        const bota::BotaFrame frame = impl_->driver->readFrame();
        if (!frame.status.bits.invalid) {
            Vector6d w;
            w << static_cast<double>(frame.force[0]),
                 static_cast<double>(frame.force[1]),
                 static_cast<double>(frame.force[2]),
                 static_cast<double>(frame.torque[0]),
                 static_cast<double>(frame.torque[1]),
                 static_cast<double>(frame.torque[2]);
            {
                std::lock_guard<std::mutex> lk(pub_mu_);
                latest_ = w;
            }
            ready_.store(true, std::memory_order_release);
        }
        next += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next);
    }
}
