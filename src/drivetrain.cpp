#include "include/robot.hpp"
#include <iostream>

Drivetrain::Drivetrain() {

    // Set motors to brake mode
    frontLeftDriveMotor.Set(0.0);
    frontRightDriveMotor.Set(0.0);
    rearLeftDriveMotor.Set(0.0);
    rearRightDriveMotor.Set(0.0);

    frontLeftSteerMotor.Set(0.0);
    frontRightSteerMotor.Set(0.0);
    rearLeftSteerMotor.Set(0.0);
    rearRightSteerMotor.Set(0.0);

    // Reset odometry with correct arguments
    odometry.ResetPosition(frc::Rotation2d(), {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()}, frc::Pose2d());
}

Drivetrain::~Drivetrain() {
    // Destructor implementation (if needed)
}

void Drivetrain::drive(MetersPerSecond xSpeed, MetersPerSecond ySpeed, RadiansPerSecond rot) {
    auto states = kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometry.GetPose().Rotation()));
    kinematics.DesaturateWheelSpeeds(&states, maxSpeed);

    frontLeftState = states[0];
    frontRightState = states[1];
    rearLeftState = states[2];
    rearRightState = states[3];

    frontLeftDriveMotor.Set(frontLeftState.speed.to<double>() / maxSpeed.to<double>());
    frontRightDriveMotor.Set(frontRightState.speed.to<double>() / maxSpeed.to<double>());
    rearLeftDriveMotor.Set(rearLeftState.speed.to<double>() / maxSpeed.to<double>());
    rearRightDriveMotor.Set(rearRightState.speed.to<double>() / maxSpeed.to<double>());

    frontLeftSteerMotor.Set(frontLeftState.angle.Radians().to<double>());
    frontRightSteerMotor.Set(frontRightState.angle.Radians().to<double>());
    rearLeftSteerMotor.Set(rearLeftState.angle.Radians().to<double>());
    rearRightSteerMotor.Set(rearRightState.angle.Radians().to<double>());
}

void Drivetrain::driveNormalized(double xSpeed, double ySpeed, double rotation) noexcept {
    drive(MetersPerSecond{xSpeed * maxSpeed.to<double>()}, MetersPerSecond{ySpeed * maxSpeed.to<double>()}, RadiansPerSecond{rotation * maxAngularSpeed.to<double>()});
}

void Drivetrain::resetOdometry(const frc::Pose2d& pose) {
    odometry.ResetPosition(frc::Rotation2d(), {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()}, pose);
    #ifdef SWERVE_DRIVE_SIM_SUPPORTED
    if (simulation)
        simulation->onOdometryReset(pose);
    #endif
}

void Drivetrain::initializeSimulation() {
    #ifdef SWERVE_DRIVE_SIM_SUPPORTED
    if (simulation != nullptr)
        return;
    simulation = std::make_unique<Simulation>(*this);
    #endif
}

void Drivetrain::updateSimulation() {
    #ifdef SWERVE_DRIVE_SIM_SUPPORTED
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    if (simulation)
        simulation->update();
    #endif
}

void Drivetrain::postProcess() {
    // Implementation of postProcess method
}
