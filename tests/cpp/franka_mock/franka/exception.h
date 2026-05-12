// Mock franka::Exception — unused inside the controllers' compute path,
// but the headers reference it. Just satisfy the symbol.

#pragma once

#include <stdexcept>

namespace franka {
class Exception : public std::runtime_error {
 public:
    using std::runtime_error::runtime_error;
};
}  // namespace franka
