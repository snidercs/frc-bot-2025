#pragma once

// saved for reference.

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Lifter : public frc2::SubsystemBase {
public:
    Lifter();

    void Periodic() override;
    void LiftUp();
    void LiftDown();
    void Stop();

private:
    ctre::phoenix6::hardware::TalonFX m_liftMotor1{0}; // Assuming first motor is connected to CAN ID 0
    ctre::phoenix6::hardware::TalonFX m_liftMotor2{1}; // Assuming second motor is connected to CAN ID 1
    frc::XboxController m_controller{0}; // Assuming controller is on port 0
};