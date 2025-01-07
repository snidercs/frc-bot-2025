
#include <cmath>
#include <vector>

#include "include/parameters.hpp"

/** Enable to log value changes in the axis controls. */
#ifndef BOT_LOG_AXES
#    define BOT_LOG_AXES 0
#endif

void Parameters::reset() noexcept {
    values.reset();
    lastValues.reset();
}

void Parameters::process (const Context& context) noexcept {
    values = context;

    static const std::vector<int> sticks { LeftStickX, LeftStickY, RightStickX, RightStickY };

    for (auto i : sticks) {
        if (std::abs (values.axis[i]) <= DeadZone) {
            values.axis[i] = 0;
        }
    }

#if BOT_LOG_AXES
    for (int i = 0; i < 6; ++i)
        if (lastValues.axis[i] != values.axis[i])
            std::clog << "[bot] axis #" << i << " = " << values.axis[i] << std::endl;

    for (int i = 0; i < 1; ++i)
        if (lastValues.povs[i] != values.povs[i])
            std::clog << "[bot] dpad #" << i << " = " << values.povs[i] << std::endl;
#endif

    // Save the context in the previous one for change detection and future
    // interpolations...
    lastValues = values;
}
