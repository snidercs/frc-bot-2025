#pragma once

#include <string>

namespace snider {

/** An indicator as how a game pad should behave. If a new mode is added, then
    total_pad_modes also needs updated.
 */
enum class PadMode : int {
    Standard
};

/** Total number of pad modes known. */
static constexpr int total_pad_modes = (int) PadMode::Standard + 1;

} // namespace snider

namespace std {

/** @private */
inline static std::string to_string (snider::PadMode mode) noexcept {
    using snider::PadMode;

    std::string res;

    // clang-format off
    switch (mode) {
        case PadMode::Standard: res = "Standard"; break;
    }
    // clang-format on

    return res;
}

} // namespace std
