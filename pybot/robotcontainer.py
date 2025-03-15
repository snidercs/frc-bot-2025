#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
#from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve#, SignalLogger
from wpimath.units import rotationsToRadians
from lifter import Lifter  # Import the Lifter class
import wpilib
import logging
import math

# Configure logging
logging.basicConfig(level=logging.DEBUG)

MAX_SPEED_SCALING = 0.35
DEAD_BAND = 0.1
ELEVATOR_MOTOR_ID_1 = 20
ELEVATOR_MOTOR_ID_2 = 16

INTAKE_MOTOR_ID_TOP = 18
INTAKE_MOTOR_ID_BOTTOM = 22

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts * MAX_SPEED_SCALING
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            TunerConstants.angular_at_12_volts * .1
        )

        # Setting up bindings for necessary control of the swerve drive platform
        self._deadband = DEAD_BAND # The input deadband
        self._exponent = 3.0 # Exponential factor (try 2, 2.5, or 3)

        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * self._deadband)
            .with_rotational_deadband(
                self._max_angular_rate * self._deadband
            ) 
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Initialize the elevator with motor IDs
        self.elevator = Lifter(ELEVATOR_MOTOR_ID_1, ELEVATOR_MOTOR_ID_2)
        # Initialize the intake with motor IDs
        self.intake = Lifter(INTAKE_MOTOR_ID_TOP, INTAKE_MOTOR_ID_BOTTOM)
        
        # Setup telemetry
        self._registerTelemetry()

    def calculateJoystick(self) -> tuple[float, float]:
            x0, y0 = self._joystick.getLeftX(), self._joystick.getLeftY()
            magnitude = self.applyExponential(math.hypot(x0, y0), self._deadband, self._exponent) * self._max_speed
            theta = math.atan2(y0, x0)

            x1 = magnitude * math.cos(theta)
            y1 = magnitude * math.sin(theta)

            return x1, y1
    
    def applyRequest(self) -> swerve.requests.SwerveRequest:
            (new_vx, new_vy) = self.calculateJoystick()

            return (self._drive.with_velocity_x(self._driveMultiplier * new_vy # Drive left with negative X (left)
            )  .with_velocity_y(self._driveMultiplier * new_vx) # Drive forward with negative Y (forward)
            .with_rotational_rate(
                self._driveMultiplier * self.applyExponential(self._joystick.getRightX(), self._deadband, self._exponent) * self._max_angular_rate
            ))  # Drive counterclockwise with negative X (left)


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
        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(self.applyRequest))
        # reset the field-centric heading on x button press
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

        self._joystick.leftBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(.3),
            lambda: self.intake.stop()
        ))
        # Configure buttons for intake control
        self._joystick.rightBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(1),
            lambda: self.intake.stop()
        ))

    def resetHeading(self) -> None:
        self.drivetrain.seed_field_centric()

    def getAutonomousCommand(self, selected: str) -> commands2.Command:
        from autos import FollowTrajectory
        return FollowTrajectory (self.drivetrain,
                                 self.intake,
                                 selected,
                                 is_red_alliance = self.isRedAlliance())
    
    def isRedAlliance(self) -> bool:
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    def _registerTelemetry (self) -> None:
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def applyExponential(self, input: float, deadband: float, exponent: float) -> float:
        """
        Apply an exponential response curve with a deadband on the input
        such that the final output goes smoothly from 0 -> ±1 without
        a deadband in the output.

        :param input: The raw joystick input in [-1, 1].
        :param deadband: The input deadband (e.g. 0.1).
        :param exponent: The exponent for the curve (e.g. 2.0 for a squared curve).
        :return: A smoothly scaled & exponentiated value in [-1, 1].
        """
        if abs(input) < deadband:
            return 0.0

        sign = math.copysign(1, input)  # Use math.copysign for clarity
        scaled = (abs(input) - deadband) / (1.0 - deadband)
        return sign * math.pow(scaled, exponent)
