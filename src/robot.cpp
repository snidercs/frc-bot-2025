// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// saved for reference only

#include "robot.hpp"
#include "lifter.hpp" // Include the Lifter header file

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container->GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_container == nullptr) {
        m_container.reset (new RobotContainer());
    }
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {
    m_container->drivetrain.Periodic();
    m_container->lifter.Periodic(); // Call the Lifter's Periodic method
}

void Robot::TeleopExit() {
}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
