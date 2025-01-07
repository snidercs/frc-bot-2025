#pragma once

#include <algorithm>
#include <array>
#include <numbers>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include "config.hpp"
#include "normalisablerange.hpp"
#include "types.hpp"

// Check if swerve drive simulation is supported
#ifdef SWERVE_DRIVE_SIM_SUPPORTED
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/EncoderSim.h>
#endif

/** Represents a swerve drive style drivetrain. */
class Drivetrain {
public:
    Drivetrain();
    ~Drivetrain();

    /** Drive the bot by FRC units. */
    void drive (MetersPerSecond xSpeed, MetersPerSecond ySpeed, RadiansPerSecond rot);

    /** Drive the bot by normalized speed and rotation (-1.0 to 1.0) */
    void driveNormalized (double xSpeed, double ySpeed, double rotation) noexcept;

    /** Reset odometry. */
    void resetOdometry (const frc::Pose2d& pose);

    /** Get estimated field position. */
    frc::Pose2d estimatedPosition() const { return odometry.GetPose(); }

    /** Initialize simulation components. */
    void initializeSimulation();

    /** Update simulation components. */
    void updateSimulation();

private:
    friend class RobotMain;
    static void bind (Drivetrain*);

    const MetersPerSecond maxSpeed {
        config::number ("drivetrain", "max_speed")
    };
    const RadiansPerSecond maxAngularSpeed {
        config::number ("drivetrain", "max_angular_speed")
    };
    const double wheelRadius {
        config::number ("drivetrain", "wheel_radius")
    };
    const int encoderResolution {
        static_cast<int> (config::number ("drivetrain", "encoder_resolution"))
    };

    frc::SwerveDriveKinematics<4> kinematics {
        frc::Translation2d {(units::meter_t) config::number("drivetrain", "wheel_positions.front_left_x"),(units::meter_t) config::number("drivetrain", "wheel_positions.front_left_y")},
        frc::Translation2d {(units::meter_t)config::number("drivetrain", "wheel_positions.front_right_x"),(units::meter_t) config::number("drivetrain", "wheel_positions.front_right_y")},
        frc::Translation2d {(units::meter_t)config::number("drivetrain", "wheel_positions.rear_left_x"),(units::meter_t) config::number("drivetrain", "wheel_positions.rear_left_y")},
        frc::Translation2d {(units::meter_t)config::number("drivetrain", "wheel_positions.rear_right_x"),(units::meter_t) config::number("drivetrain", "wheel_positions.rear_right_y")}
    };
    
    frc::SwerveDriveOdometry<4> odometry { kinematics, frc::Rotation2d(), {frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition(), frc::SwerveModulePosition()} };
    frc::Field2d field;

    // Swerve module objects
    ctre::phoenix6::hardware::TalonFX frontLeftDriveMotor { config::port("front_left_drive") };
    ctre::phoenix6::hardware::TalonFX frontRightDriveMotor { config::port("front_right_drive")};
    ctre::phoenix6::hardware::TalonFX rearLeftDriveMotor { config::port("rear_left_drive") };
    ctre::phoenix6::hardware::TalonFX rearRightDriveMotor { config::port("rear_right_drive")};

    ctre::phoenix6::hardware::TalonFX frontLeftSteerMotor { config::port("front_left_steer")};
    ctre::phoenix6::hardware::TalonFX frontRightSteerMotor { config::port("front_right_steer")};
    ctre::phoenix6::hardware::TalonFX rearLeftSteerMotor { config::port("rear_left_steer")};
    ctre::phoenix6::hardware::TalonFX rearRightSteerMotor { config::port("rear_right_steer")};

    // Encoders and other necessary components
    ctre::phoenix6::hardware::CANcoder frontLeftEncoder { config::port("front_left_encoder") };
    ctre::phoenix6::hardware::CANcoder frontRightEncoder { config::port("front_right_encoder") };
    ctre::phoenix6::hardware::CANcoder rearLeftEncoder { config::port("rear_left_encoder") };
    ctre::phoenix6::hardware::CANcoder rearRightEncoder { config::port("rear_right_encoder") };

    frc::PIDController frontLeftPIDController { config::number("drivetrain", "steer_pid.kP"), config::number("drivetrain", "steer_pid.kI"), config::number("drivetrain", "steer_pid.kD") };
    frc::PIDController frontRightPIDController { config::number("drivetrain", "steer_pid.kP"), config::number("drivetrain", "steer_pid.kI"), config::number("drivetrain", "steer_pid.kD") };
    frc::PIDController rearLeftPIDController { config::number("drivetrain", "steer_pid.kP"), config::number("drivetrain", "steer_pid.kI"), config::number("drivetrain", "steer_pid.kD") };
    frc::PIDController rearRightPIDController { config::number("drivetrain", "steer_pid.kP"), config::number("drivetrain", "steer_pid.kI"), config::number("drivetrain", "steer_pid.kD") };
    
    // Swerve module states
    frc::SwerveModuleState frontLeftState;
    frc::SwerveModuleState frontRightState;
    frc::SwerveModuleState rearLeftState;
    frc::SwerveModuleState rearRightState;

    // Gains are for example purposes only: must be determined for your own bot!
    frc::SimpleMotorFeedforward<units::meters> feedforward { 1_V, 3_V / 1_mps };

    struct SpeedRange : public juce::NormalisableRange<double> {
        using range_type = juce::NormalisableRange<double>;
        SpeedRange() : range_type (-1.0, 1.0, 0.0, config::gamepad_skew_factor(), true) {}
        ~SpeedRange() = default;
    } speedRange;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0  to 1.
    // This is also called parameter smoothing.
    frc::SlewRateLimiter<units::scalar> speedLimiter { 3 / 1_s };
    frc::SlewRateLimiter<units::scalar> rotLimiter { 3 / 1_s };

    void postProcess();
    const MetersPerSecond calculateSpeed (double value) noexcept;
    const RadiansPerSecond calculateRotation (double value) noexcept;
    void setSpeeds (const frc::SwerveDriveWheelSpeeds<4>& speeds);
    void updateOdometry();

#ifdef SWERVE_DRIVE_SIM_SUPPORTED
    //==========================================================================
    class Simulation {
    
    public:
        Simulation() = delete;
        Simulation (Drivetrain& o) : owner (o) {
            frc::SmartDashboard::PutData ("Field", &fieldSim);
        }

        /** Update the overall state of the simulation. */
        void update() {
            // To update our simulation, we set motor voltage inputs, update the
            // simulation, and write the simulated positions and velocities to our
            // simulated encoder and gyro.
            swerveSimulator.SetInputs(
                {owner.frontLeftDriveMotor.Get(), owner.frontRightDriveMotor.Get(), owner.rearLeftDriveMotor.Get(), owner.rearRightDriveMotor.Get()},
                {owner.frontLeftSteerMotor.Get(), owner.frontRightSteerMotor.Get(), owner.rearLeftSteerMotor.Get(), owner.rearRightSteerMotor.Get()});

            auto engineMillis = units::time::millisecond_t(enginePeriodMs);
            swerveSimulator.Update(engineMillis);

            frontLeftEncoderSim.SetDistance(swerveSimulator.GetModulePosition(0).value());
            frontLeftEncoderSim.SetRate(swerveSimulator.GetModuleVelocity(0).value());
            frontRightEncoderSim.SetDistance(swerveSimulator.GetModulePosition(1).value());
            frontRightEncoderSim.SetRate(swerveSimulator.GetModuleVelocity(1).value());
            rearLeftEncoderSim.SetDistance(swerveSimulator.GetModulePosition(2).value());
            rearLeftEncoderSim.SetRate(swerveSimulator.GetModuleVelocity(2).value());
            rearRightEncoderSim.SetDistance(swerveSimulator.GetModulePosition(3).value());
            rearRightEncoderSim.SetRate(swerveSimulator.GetModuleVelocity(3).value());
            gyroSim.SetAngle(-swerveSimulator.GetHeading().Degrees().value());
        }

        /** Called from the drive train during "RobotPeriodic" */
        void onPostProcess() {
            fieldSim.SetRobotPose(owner.odometry.GetPose());
        }

        /** Called when the drivetrain resets its Odometry. */
        void onOdometryReset(const frc::Pose2d& pose) {
            swerveSimulator.SetPose(pose);
        }

    private:
        const int enginePeriodMs { static_cast<int> (config::number ("engine", "period")) };

        Drivetrain& owner;
        frc::sim::AnalogGyroSim gyroSim { owner.gyro };
        frc::sim::EncoderSim frontLeftEncoderSim { owner.frontLeftEncoder };
        frc::sim::EncoderSim frontRightEncoderSim { owner.frontRightEncoder };
        frc::sim::EncoderSim rearLeftEncoderSim { owner.rearLeftEncoder };
        frc::sim::EncoderSim rearRightEncoderSim { owner.rearRightEncoder };
        frc::Field2d fieldSim;
        frc::sim::SwerveDriveSim swerveSimulator {
            frc::LinearSystemId::IdentifySwerveDriveSystem(
                1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_mps, 0.3_V / 1_mps_sq),
            frc::DCMotor::CIM(2), 8, 2_in, 4
        };
    };

    std::unique_ptr<Simulation> simulation;


    void initializeSimulation();
    void updateSimulation();
    #endif
};
