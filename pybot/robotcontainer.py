#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
import numpy as np

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
            TunerConstants.speed_at_12_volts * 0.72
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75 * .35
            #maybe limit this rotational
        )  # 3/4 of a rotation per second max angular velocity

        # Drive Inversion multiplier
        self._driveMultiplier = 1.0

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            # NOTE: Using FieldCentric prevents the use of the PIDController because the chassis speeds are of the
            #       robot. FieldCentric focus on the field so the desired direction is a different unit of measure than y(t)
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed / 10)
            .with_rotational_deadband(
                math.pi / 16
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

        # Setup telemetry
        self._registerTelemetery()

    def configureButtonBindings(self) -> None:
        """
        Setup which buttons do what.
        """
        if not hasattr(self, '_joystick') or self._joystick is None:
            self._joystick = commands2.button.CommandXboxController(0)

        # Cache the multiplier
        #self._driveMultiplier = -1.0 if self.isRedAlliance() else 1.0
        self._driveMultiplier = 1.0

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        floor_threshold = 0.15
        def floor(value):
            if (math.fabs(value) < floor_threshold):
                return 0
            
            return value
        
        def getNormalizedHeading():
            raw_heading = self.drivetrain.get_state().raw_heading.radians()
            return raw_heading % (2 * math.pi)
        
        def mapCoordinates(curX, curY, heading):
            newX = curX * math.cos(heading - 90) - curY * math.sin(heading - 90)
            newY = curX * math.sin(heading - 90) + curY * math.cos(heading - 90)

            return (newX, newY)
        
        def field_to_robot(t_vx, t_vy, theta):
            R = np.array([
                [np.cos(theta), np.sin(theta)],
                [-np.sin(theta), np.cos(theta)]
            ])
            v_field = np.array([t_vx, t_vy])
            v_robot = R.dot(-v_field)

            return (v_robot[0], v_robot[1])

        controller_vx = PIDController(0.5, 0.1, 0.0)
        controller_vy = PIDController(0.5, 0.1, 0.0)
        controller_omega = PIDController(0.5, 0.1, 0.0)

        def calculate_velocity_x(vx_current, vx_target):
            return vx_current + controller_vx.calculate(vx_current, vx_target)
        def calculate_velocity_y(vy_current, vy_target):
            return vy_current + controller_vy.calculate(vy_current, vy_target)
        def calculate_rotational_rate():
            omega_current = self.drivetrain.get_chassis_speed().omega
            omega_target = floor(self._driveMultiplier * self._joystick.getRightX()) * self._max_angular_rate

            return omega_current + controller_omega.calculate(omega_current, omega_target)

        def calculate_request():
            heading = getNormalizedHeading()
            speeds = self.drivetrain.get_chassis_speed()
            vx_current = speeds.vx
            vy_current = speeds.vy
            vx_target = floor(self._driveMultiplier * self._joystick.getLeftY()) * self._max_speed
            vy_target = floor(self._driveMultiplier * self._joystick.getLeftX()) * self._max_speed

            #(vx_target, vy_target) = mapCoordinates(vx_target, vy_target, heading)
            #(vx_current, vy_current) = mapCoordinates(vx_current, vy_current, heading)

            (vx_target, vy_target) = field_to_robot(vx_target, vy_target, heading)

            vx = calculate_velocity_x(vx_current, vx_target)
            vy = calculate_velocity_y(vy_current, vy_target)
            rot = calculate_rotational_rate()

            print('Heading: %f, Target: (%f %f), Curr: (%f %f), New: (%f %f)' % (heading, vx_target, vy_target, vx_current, vy_current, vx, vy))

            return (self._drive
                # Drive left with negative X (left) and left trigger for acceleration
                .with_velocity_x(vx)
                # Drive forward with negative Y (forward) and left trigger for acceleration
                .with_velocity_y(vy)
                # Drive counterclockwise with negative X (left)
                .with_rotational_rate(rot))
            
        def calculate_request_raw():
            vx_target = floor(self._driveMultiplier * self._joystick.getLeftY()) * self._max_speed
            vy_target = floor(self._driveMultiplier * self._joystick.getLeftX()) * self._max_speed
            omega = floor(self._driveMultiplier * self._joystick.getRightX()) * self._max_angular_rate

            #heading = getNormalizedHeading()
            heading = self.drivetrain.get_state().raw_heading.radians()
            print('%f' % heading)

            return (self._drive.with_velocity_x(vx_target).with_velocity_y(vy_target).with_rotational_rate(omega))
        
        # Drivetrain will execute this command periodically
        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(calculate_request))

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