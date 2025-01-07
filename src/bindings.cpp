#include <frc/XboxController.h>

#include "include/scripting.hpp"
#include "sol/sol.hpp"

#include "include/parameters.hpp"
#include "include/robot.hpp"

namespace lua {

using function_list = std::initializer_list<const char*>;

namespace detail {

/** Returns the 'cxx' module in the root Lua state. */
sol::table cxx_table (sol::state& ls) {
    auto& G = ls;

    if (G["cxx"].get_type() != sol::type::table)
        G["cxx"] = ls.create_table();
    sol::table tbl = G["cxx"];
    return tbl;
}

/** Erase bound functions from Lua tables */
template <typename Ls>
void clear_function_bindings (Ls& L, std::string_view mod,
                              function_list functions) {
    sol::table cxx    = cxx_table (L);
    sol::object proxy = cxx[mod];
    if (! proxy.valid())
        return;
    sol::table M = proxy;
    for (const auto f : functions)
        M.set (f, []() {});
}

} // namespace detail

void bind_gamepad (frc::XboxController* self) {
    auto& L  = state();
    auto cxx = detail::cxx_table (L);

    // bind/unbind 'cxx.gamepad' global module.
    if (self != nullptr) {
        auto M = state().create_table();

        M["STICK_LEFT_X"]  = (int) Parameters::LeftStickX;
        M["STICK_LEFT_Y"]  = (int) Parameters::LeftStickY;
        M["STICK_RIGHT_X"] = (int) Parameters::RightStickX;
        M["STICK_RIGHT_Y"] = (int) Parameters::RightStickY;

        M["TRIGGER_LEFT"]  = (int) Parameters::TriggerLeft;
        M["TRIGGER_RIGHT"] = (int) Parameters::TriggerRight;

        // frc buttons are 1-indexed.  So is lua....
        M["BUTTON_A"] = (int) Parameters::ButtonA + 1;
        M["BUTTON_B"] = (int) Parameters::ButtonB + 1;
        M["BUTTON_X"] = (int) Parameters::ButtonX + 1;
        M["BUTTON_Y"] = (int) Parameters::ButtonY + 1;

        M["BUMPER_LEFT"]  = (int) Parameters::ButtonLeftBumper + 1;
        M["BUMPER_RIGHT"] = (int) Parameters::ButtonRightBumper + 1;

        M["raw_axis"] = [self] (int index) {
            return self->GetRawAxis (index);
        };

        M["raw_button"] = [self] (int button) {
            return self->GetRawButton (button);
        };

        M["raw_button_pressed"] = [self] (int button) {
            return self->GetRawButtonPressed (button);
        };

        M["raw_button_released"] = [self] (int button) {
            return self->GetRawButtonReleased (button);
        };

        M["pov"] = [self]() { return self->GetPOV (0); };

        cxx["gamepad"] = M;
    } else {
        // clang-format off
        detail::clear_function_bindings (L, "gamepad", { 
            "raw_axis", "raw_button", "raw_button_pressed", 
            "raw_button_released" 
        });
        // clang-format on
    }
}

} // namespace lua

using namespace lua;

//=============================================================================
bool Parameters::bind (Parameters* self) {
    auto& L  = state();
    auto cxx = detail::cxx_table (L);

    // bind/unbind 'cxx.params' global module.
    if (self != nullptr) {
        auto M = state().create_table();

        M["speed"]    = [self]() -> lua_Number { return self->getSpeed(); };
        M["rotation"] = [self]() -> lua_Number { return self->getAngularSpeed(); };
        M["brake"]    = [self]() -> lua_Number { return self->getBrake(); };

        cxx["params"] = M;
    } else {
        // clang-format off
        detail::clear_function_bindings (L, "params", { 
            "speed", "rotation", "brake" 
        });
        // clang-format on
    }

    return true;
}
//=============================================================================
void Drivetrain::bind (Drivetrain* self) {
    auto& L  = lua::state();
    auto cxx = detail::cxx_table (L);

    // bind/unbind 'cxx.drivetrain' global module.
    if (self != nullptr) {
        auto M = L.create_table();

        M["drive"] = [self] (double speed, double rot) {
            self->driveNormalized (speed, 0.0, rot);
        };

        cxx["drivetrain"] = M;
    } else {
        detail::clear_function_bindings (L, "drivetrain", { "drive" });
    }
}

//=============================================================================
// experimental...
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
namespace lua {

inline static int frc_Rotation2d (lua_State* state) {
    using frc::Rotation2d;
    sol::state_view L (state);
    auto T = L.create_table();

    // clang-format off
    T.new_usertype<Rotation2d> ("Rotation2d", sol::no_constructor,
        "new", []() { return Rotation2d(); },
        "rotateBy", &Rotation2d::RotateBy,
        "cos", sol::readonly_property (&Rotation2d::Cos),
        "sin", sol::readonly_property (&Rotation2d::Sin),
        "tan", sol::readonly_property (&Rotation2d::Tan),
        "degrees", sol::readonly_property ([](const Rotation2d& self) { return self.Degrees().value(); }),
        "radians", sol::readonly_property ([](const Rotation2d& self) { return self.Radians().value(); })
    );
    // clang-format on

    sol::stack::push (L, T["Rotation2d"]);
    return 0;
}

inline static int frc_Pose2d (lua_State* state) {
    using frc::Pose2d;
    sol::state_view L (state);
    auto T = L.create_table();
    // clang-format off
    T.new_usertype<Pose2d> ("Pose2d", sol::no_constructor,
        "new", []() { return Pose2d(); },
        "x", sol::readonly_property ([](Pose2d& self) { return self.X().value(); }),
        "y", sol::readonly_property ([](Pose2d& self) { return self.Y().value(); }),
        "rotation", sol::readonly_property(&Pose2d::Rotation)
    );
    // clang-format on
    sol::stack::push (L, T["Pose2d"]);
    return 1;
}

} // namespace lua
