// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "commandswervedrivetrain.hpp"
#include "telemetry.hpp"
#include "lifter.hpp" // Include the Lifter header file

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed =
        TunerConstants::kSpeedAt12Volts * .5; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate =
        0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive =
        swerve::requests::FieldCentric {}
            .WithDeadband (MaxSpeed * 0.1)
            .WithRotationalDeadband (MaxAngularRate * 0.1) // Add a 10% deadband
            .WithDriveRequestType (
                swerve::DriveRequestType::
                    OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake {};
    swerve::requests::PointWheelsAt point {};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
   *       define a destructor to un-register the telemetry from the drivetrain
   */
    Telemetry logger { MaxSpeed };

    frc2::CommandXboxController joystick { 0 };

public:
    subsystems::CommandSwerveDrivetrain drivetrain {
        TunerConstants::CreateDrivetrain()
    };
    Lifter lifter; // Instantiate the Lifter subsystem

    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();
};
