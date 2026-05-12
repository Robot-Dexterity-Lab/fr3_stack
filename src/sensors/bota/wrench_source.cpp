// Single dispatch point for all FT-sensor backends. main.cpp depends only on
// this translation unit (and wrench_source.hpp); vendor-specific includes
// stay confined to each backend's .cpp file.

#include <fr3_stack/sensors/bota/wrench_source.hpp>

#include <fr3_stack/sensors/bota/bota_wrench_source.hpp>
// Add new backends here, e.g.:
//   #include "ati_wrench_source.hpp"

#include <stdexcept>

std::unique_ptr<WrenchSource> make_wrench_source(
    const std::string& kind, const std::string& config) {

    if (kind == "bota") {
        return std::make_unique<BotaWrenchSource>(config);
    }
    // Future:
    //   if (kind == "ati") {
    //       return std::make_unique<AtiWrenchSource>(config);
    //   }

    throw std::runtime_error(
        "make_wrench_source: unknown ft-sensor kind '" + kind +
        "' (supported: bota)");
}
