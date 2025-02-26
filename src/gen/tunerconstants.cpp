#include "tunerconstants.hpp"
#include "commandswervedrivetrain.hpp"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain() {
    return { DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight };
}
