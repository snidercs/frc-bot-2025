#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.units import rotationsToRadians
from lifter import Lifter  # Import the Lifter class
import wpilib
from autolink import AutonomousCommand
import logging
import math
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds

# Configure logging
logging.basicConfig(level=logging.DEBUG)


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts * .72
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75 * .35
            #maybe limit this rotational
        )  # 3/4 of a rotation per second max angular velocity

        # Drive Inversion multiplier
        self._driveMultiplier = -1.0

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Initialize the elevator with motor IDs
        self.elevator = Lifter(20, 16)
        # Initialize the intake with motor IDs
        self.intake = Lifter(18, 22)
        # making it closer to 1 will make the decay slower and smoother.

        # Initialize PID controllers for smoothing
        self._pid_x = PIDController(0.8, 0.0, 0.0)
        self._pid_y = PIDController(0.8, 0.0, 0.0)
        self._pid_rot = PIDController(1.6, 0.0, 0.0)

        # Configure PID controllers
        self._pid_x.setTolerance(0.01)
        self._pid_y.setTolerance(0.01)
        self._pid_rot.setTolerance(0.01)

        # Setpoint initialization
        self._pid_x.setSetpoint(0)
        self._pid_y.setSetpoint(0)
        self._pid_rot.setSetpoint(0)

        # Setup telemetry
        self._registerTelemetery()

    def configureButtonBindings(self) -> None:
        """
        Setup which buttons do what.
        """
        if not hasattr(self, '_joystick') or self._joystick is None:
            self._joystick = commands2.button.CommandXboxController(0)

        # Cache the multiplier
        self._driveMultiplier = -1.0 if self.isRedAlliance() else 1.0

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        self.drivetrain.get_chassis_speed().vx + (self._driveMultiplier * self._smooth_input(self._joystick.getLeftX() * self._max_speed, self.drivetrain.get_chassis_speed().vx, 'x') )
                    )  # Drive forward with negative Y (forward) and left trigger for acceleration
                    .with_velocity_y(
                        self.drivetrain.get_chassis_speed().vy + (self._driveMultiplier * self._smooth_input(self._joystick.getLeftY() * self._max_speed, self.drivetrain.get_chassis_speed().vy, 'y') )
                    )  # Drive left with negative X (left) and left trigger for acceleration
                    .with_rotational_rate(
                        self._driveMultiplier * self._smooth_input(self._joystick.getRightX() * self._max_angular_rate, self.drivetrain.get_chassis_speed().omega, 'rot') 
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        print(f"Post Speeds: {self.drivetrain.get_chassis_speed()}")
        # reset the field-centric heading on left bumper press
        self._joystick.x().onTrue(
            self.drivetrain.runOnce(lambda: self.resetHeading())
        )

        # Configure buttons for elevator control
        self._joystick.y().whileTrue(commands2.cmd.startEnd(
            lambda: self.elevator.moveUp(),
            lambda: self.elevator.stop()
        ))

        self._joystick.a().whileTrue(commands2.cmd.startEnd(
            lambda: self.elevator.moveDown(),
            lambda: self.elevator.stop()
        ))

        self._joystick.rightTrigger().onTrue(commands2.cmd.runOnce(self.elevator.stop, self.elevator))

        # Configure buttons for intake control
        self._joystick.rightBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(.8),
            lambda: self.intake.stop()
        ))

        # Theres a chance red vs blue has changed, so do this now.

    def resetHeading(self):
        self.drivetrain.seed_field_centric()
    
    def _registerTelemetery (self):
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self, selected_traj_file: str) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return AutonomousCommand(self.drivetrain, 
                                 self.intake, 
                                 selected_traj_file, 
                                 is_red_alliance=self.isRedAlliance())
    
    def isRedAlliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def _smooth_input(self, set_point, chassis_speeds, axis):
        return 0.8 * (set_point - chassis_speeds)
        
        print(f"Pre Speeds: {self.drivetrain.get_chassis_speed()}")
        if abs(set_point) <= 0.1:
            smoothed_value = 0
        elif axis == 'x':
            
            smoothed_value = self._pid_x.calculate(chassis_speeds, set_point)
        elif axis == 'y':
            smoothed_value = self._pid_y.calculate(chassis_speeds, set_point)
        elif axis == 'rot':
            smoothed_value = self._pid_rot.calculate(chassis_speeds, set_point)

        print(f"Corrections of {axis}: {smoothed_value}")
        return smoothed_value