
#include <filesystem>

#include <frc/TimedRobot.h>

#include <gtest/gtest.h>

#include "include/config.hpp"
#include "include/scripting.hpp"
#include "sol/sol.hpp"

namespace fs = std::filesystem;
namespace cfg = lua::config;
using std::string_view;

class ConfigTest : public testing::Test {
public:
    ConfigTest() {}
    
    // expect just a valid value.
    void expectValid (string_view cat, string_view sym) {
        EXPECT_TRUE (cfg::get (cat, sym).valid());
    }
    
    // expect a valid number that can cast to integer
    void expectValidInt(string_view cat, string_view sym) {
        auto obj = cfg::get (cat, sym);
        EXPECT_NO_THROW (obj.as<lua_Integer>());
        EXPECT_NO_THROW (obj.as<int>());
        EXPECT_NO_THROW (obj.as<lua_Number>());
        EXPECT_NO_THROW (obj.as<double>());
    }

    // expect a valid number that can cast to integer
    void expectValidNumber(string_view cat, string_view sym) {
        auto obj = cfg::get (cat, sym);
        EXPECT_NO_THROW (obj.as<lua_Number>());
        EXPECT_NO_THROW (obj.as<double>());
    }
};

TEST_F (ConfigTest, Get) {
    EXPECT_TRUE (cfg::get ("team_name").valid());
    EXPECT_FALSE (cfg::get ("general_fake", "fake_symbol").valid());
    auto tn = cfg::get ("team_number");
    EXPECT_EQ (tn.get_type(), sol::type::number);
    EXPECT_EQ (tn.as<lua_Integer>(), 9431);
}


TEST_F (ConfigTest, EngineLatency) {
    expectValid ("engine", "period");    
    auto obj = cfg::get ("engine", "period");
    expectValidInt ("engine", "period");
    EXPECT_GT(obj.as<int>(), 0);
}

TEST_F (ConfigTest, GetOr) {
    // placeholder
}

TEST_F (ConfigTest, GetDouble) {
    EXPECT_EQ (config::gamepad_skew_factor(),
               config::number ("gamepad", "skew_factor"));
    EXPECT_EQ (config::number ("gamepad", "fake", 1001.0), 1001.0);
    EXPECT_EQ (config::number ("fake", "alsofake", -1002.0), -1002.0);
}

TEST_F (ConfigTest, Gamepad) {
    const auto skew_factor = config::gamepad_skew_factor();
    EXPECT_GE (skew_factor, 0.0);
    EXPECT_LE (skew_factor, 1.0);
    EXPECT_EQ (skew_factor, config::number ("gamepad", "skew_factor"));
}

TEST_F (ConfigTest, Ports) {
    const std::vector<std::string> symbols {
        "gamepad",
        "joystick",

        "front_left_drive_motor",
        "front_left_steer_motor",
        "front_left_encoder",
        "front_right_drive_motor",
        "front_right_steer_motor",
        "front_right_encoder",
        "back_left_drive_motor",
        "back_left_steer_motor",
        "back_left_encoder",
        "back_right_drive_motor",
        "back_right_steer_motor",
        "back_right_encoder",

    };

    EXPECT_EQ (config::num_ports(), (int) symbols.size());
    for (const auto& sym : symbols) {
        expectValidInt ("ports", sym);
        EXPECT_GE (config::port (sym), 0);
    }
}

TEST_F (ConfigTest, NoExceptions) {
    try {
        EXPECT_LT (config::port (""), 0);
        EXPECT_LT (config::port ("invalid_____index____not_good"), 0);
        EXPECT_EQ (config::team_name(), "The Gold Standard");
        EXPECT_EQ (config::team_number(), 9431);
    } catch (...) {
        std::cerr << "exception in: " << __FILE__ << std::endl;
        EXPECT_TRUE (false);
    }
}
