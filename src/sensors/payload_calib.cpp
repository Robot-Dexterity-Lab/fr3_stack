#include <fr3_stack/sensors/payload_calib.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace {

Eigen::Vector3d read_vec3(const YAML::Node& node, const std::string& key) {
    const YAML::Node v = node[key];
    if (!v || !v.IsSequence() || v.size() != 3)
        throw std::runtime_error(
            "payload_calib: expected '" + key + "' to be a 3-element list");
    return Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
}

}  // namespace

std::optional<PayloadCalib> load_payload_calib(const std::string& path) {
    if (path.empty() || !std::filesystem::exists(path))
        return std::nullopt;

    YAML::Node doc;
    try {
        doc = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            "payload_calib: failed to parse '" + path + "': " + e.what());
    }
    if (!doc.IsMap())
        throw std::runtime_error(
            "payload_calib: top-level YAML must be a mapping in '" + path + "'");

    PayloadCalib c;
    if (!doc["mass"])
        throw std::runtime_error("payload_calib: missing 'mass' in '" + path + "'");
    c.mass   = doc["mass"].as<double>();
    c.com    = read_vec3(doc, "center_of_mass");
    c.f_bias = read_vec3(doc, "force_bias");
    c.t_bias = read_vec3(doc, "torque_bias");
    if (doc["rpy_ee_sensor"]) {
        const Eigen::Vector3d rpy = read_vec3(doc, "rpy_ee_sensor");
        // ZYX intrinsic (yaw·pitch·roll), matching scipy / common conventions.
        c.R_ee_sensor =
            (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    }
    if (doc["mean_force_residual_N"])
        c.residual_force_N = doc["mean_force_residual_N"].as<double>();
    if (doc["mean_torque_residual_Nm"])
        c.residual_torque_Nm = doc["mean_torque_residual_Nm"].as<double>();
    return c;
}

std::string default_calib_path() {
    namespace fs = std::filesystem;
    // 1. Explicit single-file override.
    if (const char* p = std::getenv("FR3_FT_CALIB"); p && *p)
        return p;
    // 2. Override directory (mirrors the Python-side $FR3_FT_CALIB_DIR).
    if (const char* d = std::getenv("FR3_FT_CALIB_DIR"); d && *d)
        return (fs::path(d) / "ft_calibration.yaml").string();
    // 3. Convention: docker-compose bind-mounts the host's
    //    fr3_stack/sensors/bota/config/ here. Out-of-Docker runs that
    //    don't set FR3_FT_CALIB(_DIR) just see the file missing →
    //    daemon boots uncompensated, which is the correct fallback.
    return "/opt/fr3-stack/calib/ft_calibration.yaml";
}
