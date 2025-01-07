
#include <filesystem>

#include <gtest/gtest.h>

#include "test.hpp"
#include "include/scripting.hpp"
#include "sol/sol.hpp"

namespace fs = std::filesystem;
namespace config = lua::config;

class EngineTest : public testing::Test {
public:
    EngineTest() {
        if (gTimedRobot != nullptr)
            gTimedRobot->RobotInit();
    }
};

TEST_F (EngineTest, Period) {
    EXPECT_NE(gTimedRobot, nullptr);
    double actual = units::time::millisecond_t (gTimedRobot->GetPeriod()).value();
    EXPECT_EQ (config::get ("engine", "period").as<double>(), actual);
}

TEST_F (EngineTest, RunAutonomous) {
    // run the bot in test mode for one second.
    gTimedRobot->AutonomousInit();
    int period = std::max (1, static_cast<int> (units::time::millisecond_t (gTimedRobot->GetPeriod()).value()));
    int ticks = 1000 / period;
    while (--ticks >= 0) {
        gTimedRobot->AutonomousPeriodic();
        gTimedRobot->RobotPeriodic();
    }
    gTimedRobot->AutonomousExit();
}

TEST_F (EngineTest, RunTeleop) {
    // run the bot in test mode for one second.
    gTimedRobot->TeleopInit();
    int period = std::max (1, static_cast<int> (units::time::millisecond_t (gTimedRobot->GetPeriod()).value()));
    int ticks = 1000 / period;
    while (--ticks >= 0) {
        gTimedRobot->TeleopPeriodic();
        gTimedRobot->RobotPeriodic();
    }
    gTimedRobot->TeleopExit();
}

TEST_F (EngineTest, RunTest) {
    // run the bot in test mode for one second.
    gTimedRobot->TestInit();
    int period = std::max (1, static_cast<int> (units::time::millisecond_t (gTimedRobot->GetPeriod()).value()));
    int ticks = 1000 / period;
    while (--ticks >= 0) {
        gTimedRobot->TestPeriodic();
        gTimedRobot->RobotPeriodic();
    }
    gTimedRobot->TestExit();
}

TEST_F (EngineTest, RequireAll) {
    auto& L { lua::state() };
    EXPECT_NO_THROW (
        L.script (R"(
            require ('config')
            require ('params')
            require ('gamepad')
            require ('robot')
        )")
    );
}
