#include <filesystem>

#include <hal/HAL.h>

#include <frc/TimedRobot.h>

#include "gtest/gtest.h"
#include "include/scripting.hpp"

/** Same as the firmware: lua needs to be in global scope so it stays alive 
    when FRC is using a global static robot base with no shutdown system.
*/
static lua::Lifecycle engine;

extern frc::TimedRobot* instantiate_robot();
frc::TimedRobot* gTimedRobot = nullptr;

int main (int argc, char** argv) {
    // clang-format off
    auto robot_dir = std::filesystem::path (__FILE__)
        .parent_path()
        .parent_path()
        .make_preferred() / "robot";
    // clang-format on

    lua::set_path (robot_dir.string());
    if (! lua::bootstrap())
        throw std::runtime_error ("lua could not be bootstrapped for unit testing");
     
    HAL_Initialize (500, 1);
    
    try {
        if (auto bot = std::unique_ptr<frc::TimedRobot> (instantiate_robot())) {
            gTimedRobot = bot.get();
            ::testing::InitGoogleTest (&argc, argv);
            int ret = RUN_ALL_TESTS();
            gTimedRobot = nullptr;
            bot.reset();
            return 0;
        }
    } catch (sol::error e) {
        std::clog << e.what() << std::endl;
        return -2000;
    }

    return -1000;
}
