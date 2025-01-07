#include <cmath>
#include <cstdlib>

#include <gtest/gtest.h>

#include "include/parameters.hpp"

namespace detail {

/** Generate a ranged pseudo-random number.
    @tparam T a floating point type (float, or double)
    @param low The low limit inclusive.
    @param high The high limit inclusive.
    @return The random number.
*/
template <typename T>
static T random (T low, T high) {
    if (low >= high)
        throw std::runtime_error ("random(): low is greater or equal to high");

    return low + static_cast<T> (std::rand()) / (static_cast<T> (RAND_MAX / (high - low)));
}

/** Make a context with random values set. */
static Parameters::Context randomContext() {
    Parameters::Context c;

    // random joystick values
    for (auto& a : c.axis)
        a = detail::random (-1.0, 1.0);

    // even buttons on.
    for (int i = 0; i < Parameters::MaxButtons; ++i)
        c.buttons[i] = i % 2 == 0;

    int numNonZero = 0;
    for (auto a : c.axis)
        if (a != 0.0)
            ++numNonZero;

    if (numNonZero <= 0)
        throw std::runtime_error ("randomContext(): didn't generate random numbers");

    return c;
}

static bool axisUsesDeadZone (int a) {
    // clang-format off
    return a == Parameters::LeftStickX || 
        a == Parameters::LeftStickY || 
        a == Parameters::RightStickX || 
        a == Parameters::RightStickY;
    // clang-format on
}

} // namespace detail

class ParametersTest : public testing::Test {
protected:
    Parameters params;
};

TEST_F (ParametersTest, ResetClearsAll) {
    params.process (detail::randomContext());
    params.reset();
    for (int a = 0; a < Parameters::MaxAxes; ++a)
        EXPECT_EQ (params.getAxisValue (a), 0.0);
    for (int b = 0; b < Parameters::MaxButtons; ++b)
        EXPECT_EQ (params.getButtonValue (b), false);
    for (int d = 0; d < Parameters::MaxPOVs; ++d)
        EXPECT_EQ (params.getPOVValue (d), 0);
}

TEST_F (ParametersTest, DeadZone) {
    const auto oc = detail::randomContext();
    params.reset();
    params.process (oc);

    for (int a = 0; a < Parameters::MaxAxes; ++a) {
        const auto original = oc.axis[a];
        const auto value    = params.getAxisValue (a);
        if (detail::axisUsesDeadZone (a) && std::abs (original) <= Parameters::DeadZone)
            EXPECT_EQ (value, 0.0);
        else
            EXPECT_EQ (value, original);
    }
}

TEST_F (ParametersTest, Helpers) {
    const auto oc = detail::randomContext();
    params.reset();
    params.process (oc);

    EXPECT_EQ (params.getButtonValue (Parameters::ButtonA), params.getAButton());
    EXPECT_EQ (params.getButtonValue (Parameters::ButtonB), params.getBButton());
    EXPECT_EQ (params.getButtonValue (Parameters::ButtonX), params.getXButton());
    EXPECT_EQ (params.getButtonValue (Parameters::ButtonY), params.getYButton());

    EXPECT_EQ (params.getAxisValue (Parameters::LeftStickX), params.getLeftStickX());
    EXPECT_EQ (params.getAxisValue (Parameters::LeftStickY), params.getLeftStickY());
    EXPECT_EQ (params.getAxisValue (Parameters::RightStickX), params.getRightStickX());
    EXPECT_EQ (params.getAxisValue (Parameters::RightStickY), params.getRightStickY());
    EXPECT_EQ (params.getAxisValue (Parameters::TriggerLeft), params.getTriggerLeft());
    EXPECT_EQ (params.getAxisValue (Parameters::TriggerRight), params.getTriggerRight());
}
