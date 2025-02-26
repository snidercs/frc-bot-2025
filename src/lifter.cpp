// saved for reference only.

#include "lifter.hpp"

Lifter::Lifter() {
    // Initialize motors and controller
    m_liftMotor2.SetInverted(true); // Invert the second motor if necessary
}

void Lifter::Periodic() {
    // This method will be called once per scheduler run
    if (m_controller.GetAButton()) {
        LiftUp();
    } else if (m_controller.GetBButton()) {
        LiftDown();
    } else {
        Stop();
    }

    // Update SmartDashboard with motor status
    frc::SmartDashboard::PutNumber("Lift Motor 1 Speed", m_liftMotor1.Get());
    frc::SmartDashboard::PutNumber("Lift Motor 2 Speed", m_liftMotor2.Get());
}

void Lifter::LiftUp() {
    m_liftMotor1.Set(0.2); // Set motors to 20% speed up
    m_liftMotor2.Set(0.2); // Set motors to 20% speed up
}

void Lifter::LiftDown() {
    m_liftMotor1.Set(-0.2); // Set motors to 20% speed down
    m_liftMotor2.Set(-0.2); // Set motors to 20% speed down
}

void Lifter::Stop() {
    m_liftMotor1.Set(0.0); // Stop the motors
    m_liftMotor2.Set(0.0); // Stop the motors
}