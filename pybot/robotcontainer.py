#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from wpimath.units import rotationsToRadians
from lifter import Lifter  # Import the Lifter class
import wpilib
import logging

import math

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
        self._deadband = 0.05 # The input deadband
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
        self.elevator = Lifter(20, 16)
        # Initialize the intake with motor IDs
        self.intake = Lifter(18, 22)
        # making it closer to 1 will make the decay slower and smoother.

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

        def calculateJoystick():
            x0 = self._joystick.getLeftX()
            y0 = self._joystick.getLeftY()

            mag = self.apply_exponential(math.sqrt((x0 * x0) + (y0 * y0)), self._deadband, self._exponent) * self._max_speed
            theta = math.atan2(y0, x0)

            x1 = mag * math.cos(theta)
            y1 = mag * math.sin(theta)

            print('0: %f, %f    1: %f %f' % (x0, y0, x1, y1))

            return (x1, y1)

        def applyRequest():
            (new_vx, new_vy) = calculateJoystick()

            return (self._drive.with_velocity_x(self._driveMultiplier * new_vy)  # Drive forward with negative Y (forward)
            .with_velocity_y(self._driveMultiplier * new_vx)  # Drive left with negative X (left)
            .with_rotational_rate(
                self._driveMultiplier * self.apply_exponential(self._joystick.getRightX(), self._deadband, self._exponent) * self._max_angular_rate
            ))  # Drive counterclockwise with negative X (left)

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(applyRequest))
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

    def getAutonomousCommand(self, selected: str) -> commands2.Command:
        from autos import FollowTrajectory
        return FollowTrajectory (self.drivetrain,
                                 self.intake,
                                 selected,
                                 is_red_alliance = self.isRedAlliance())
    
    def isRedAlliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    def _registerTelemetery (self):
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def apply_exponential(self, input: float, deadband: float, exponent: float) -> float:
        """
        Apply an exponential response curve with a deadband on the input
        such that the final output goes smoothly from 0 -> ±1 without
        a deadband in the output.

        :param input: The raw joystick input in [-1, 1].
        :param deadband: The input deadband (e.g. 0.1).
        :param exponent: The exponent for the curve (e.g. 2.0 for a squared curve).
        :return: A smoothly scaled & exponentiated value in [-1, 1].
        """
        # 1. Apply input deadband (stick within ±deadband => output = 0)
        if abs(input) < deadband:
            return 0.0

        # 2. Preserve sign and work with magnitude
        sign = 1 if input > 0 else -1
        magnitude = abs(input)

        # 3. Scale from [deadband .. 1] to [0 .. 1]
        scaled = (magnitude - deadband) / (1.0 - deadband)  # in [0..1]

        # 4. Apply exponential. e.g. exponent=2 => x^2, exponent=3 => x^3
        curved = scaled ** exponent

        # 5. Reapply sign to restore forward/backward
        return sign * curved
