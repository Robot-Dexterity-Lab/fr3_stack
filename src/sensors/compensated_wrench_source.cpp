#include <fr3_stack/sensors/compensated_wrench_source.hpp>

#include <utility>

CompensatedWrenchSource::CompensatedWrenchSource(
    std::unique_ptr<WrenchSource> inner,
    PayloadCalib                  calib)
    : inner_(std::move(inner)),
      calib_(std::move(calib)),
      mg_(calib_.mass * kStandardGravity),
      R_ee_sensor_(calib_.R_ee_sensor) {}

CompensatedWrenchSource::~CompensatedWrenchSource() = default;

void CompensatedWrenchSource::start() { inner_->start(); }

void CompensatedWrenchSource::stop() noexcept { inner_->stop(); }

const char* CompensatedWrenchSource::kind() const { return inner_->kind(); }

bool CompensatedWrenchSource::read(Vector6d& out) const {
    // First grab the raw frame. If the inner source has nothing fresh OR
    // can't acquire its own lock, propagate that — the caller already knows
    // how to handle a false return (reuse the previous F_ext_cache_).
    if (!inner_->read(out)) return false;

    Eigen::Matrix3d R_O_EE;
    {
        std::unique_lock<std::mutex> lk(orient_mu_, std::try_to_lock);
        if (!lk.owns_lock()) return false;
        R_O_EE = R_;
    }

    // Gravity vector projected into sensor frame. Sign matches the LSQ in
    // the Python solver (f_raw = -m·g_s + f_bias, with g_s = R_O_sensor^T·e3·g).
    // R_O_sensor = R_O_EE · R_ee_sensor accounts for any rigid offset between
    // libfranka's EE frame and the actual sensor mounting (e.g. Desk hand 45°).
    const Eigen::Matrix3d R_O_sensor = R_O_EE * R_ee_sensor_;
    const Eigen::Vector3d g_s = R_O_sensor.transpose() * Eigen::Vector3d(0.0, 0.0, -mg_);

    out.head<3>() -= g_s + calib_.f_bias;
    out.tail<3>() -= calib_.com.cross(g_s) + calib_.t_bias;
    return true;
}

bool CompensatedWrenchSource::read_raw(Vector6d& out) const {
    return inner_->read(out);
}

void CompensatedWrenchSource::set_orientation(const Eigen::Matrix3d& R) {
    // Try-lock. If contended (read() is in flight on another thread),
    // skip — the next RT tick will pump a fresh R one ms later.
    std::unique_lock<std::mutex> lk(orient_mu_, std::try_to_lock);
    if (lk.owns_lock()) R_ = R;
}
