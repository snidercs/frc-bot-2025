#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from wpimath.geometry import Pose2d # , Rotation2d
import commands2
import commands2.button
import commands2.cmd
# from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from phoenix6 import swerve#, SignalLogger
from wpimath.units import rotationsToRadians
from lifter import Lifter  # Import the Lifter class
import wpilib
import logging
from limelightinit import LimelightSubsystem  # Import the LimelightSubsystem class
import math
# from wpimath.controller import PIDController
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
            TunerConstants.speed_at_12_volts * .35
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            TunerConstants.angular_at_12_volts * .1
        )

        # Setting up bindings for necessary control of the swerve drive platform
        self._deadband = 0.1 # The input deadband
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

        self.dummy_pose = False

        # Setup telemetry
        self._registerTelemetery()
    
    def calculateJoystick(self):
        x0, y0 = self._joystick.getLeftX(), self._joystick.getLeftY()
        magnitude = self.apply_exponential(math.hypot(x0, y0), self._deadband, self._exponent) * self._max_speed
        theta = math.atan2(y0, x0)

        x1 = magnitude * math.cos(theta)
        y1 = magnitude * math.sin(theta)

        return x1, y1

    def configureButtonBindings(self) -> None:
        """
        Setup which buttons do what.
        """
        if not hasattr(self, '_joystick') or self._joystick is None:
            self._joystick = commands2.button.CommandXboxController(0)

        # Cache the multiplier
        self._driveMultiplier = -1.0 if self.isRedAlliance() else 1.0

        

        def applyRequest():
            (new_vx, new_vy) = self.calculateJoystick()

            return (self._drive.with_velocity_x(self._driveMultiplier * new_vy)  # Drive forward with negative Y (forward)
            .with_velocity_y(self._driveMultiplier * new_vx)  # Drive left with negative X (left)
            .with_rotational_rate(
                self._driveMultiplier * self.apply_exponential(self._joystick.getRightX(), self._deadband, self._exponent) * self._max_angular_rate
            ))  # Drive counterclockwise with negative X (left)

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(applyRequest))
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
            lambda: self.intake.setMotor(-.3),
            lambda: self.intake.stop()
        ))
        # Configure buttons for intake control
        self._joystick.rightBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(-1),
            lambda: self.intake.stop()
        ))

        #if self.limelight_subsystem.limelight1 is not None:
            # Configure button to call PointAtCoordinateCommand with Limelight data
        self._joystick.b().whileTrue(
            commands2.cmd.run(self.create_point_at_coordinate_request(), self.drivetrain)
        )
        self._joystick.povDown().whileTrue(
            commands2.cmd.run(self.create_go_to_coordinate_request(), self.drivetrain)
        )
    
    def create_go_to_coordinate_request(self):
        target_pose = Pose2d(2, 1, 1/2)  # Define your target pose here

        def apply_request():
            self.drivetrain.go_to_coordinate(target_pose)

        return apply_request

    def create_point_at_coordinate_request(self):
        if self.dummy_pose:
            target_pose = Pose2d(2, 1, 1/2)
        else:
            if self.limelight_subsystem.limelight1 is None:
                logging.warning("Limelight1 is not initialized")
                return lambda: None

            fiducial_id = self.limelight_subsystem.get_primary_fiducial_id(self.limelight_subsystem.limelight1)
            target_pose_data = self.limelight_subsystem.get_target_pose(fiducial_id)

            if fiducial_id == -1 or target_pose_data is None:
                logging.warning("No valid fiducial ID or target pose")
                return lambda: None

            target_pose = self.limelight_subsystem.to_pose2d(target_pose_data)

        def apply_request():
            joyvalues = self.calculateJoystick()
            forward, strafe = joyvalues[1], joyvalues[0]
            current_pose = self.drivetrain.get_pose()

            heading_to_target = self.compute_heading_to_target(current_pose, target_pose)
            current_heading = current_pose.rotation().radians()

            self.drivetrain.heading_controller.setSetpoint(heading_to_target)
            turn_command = self.drivetrain.heading_controller.calculate(current_heading)

            request = (
                swerve.requests.ApplyFieldSpeeds()
                .with_speeds(ChassisSpeeds(forward, strafe, turn_command))
                .with_drive_request_type(swerve.swerve_module.SwerveModule.DriveRequestType.VELOCITY)
                .with_steer_request_type(swerve.swerve_module.SwerveModule.SteerRequestType.POSITION)
                .with_desaturate_wheel_speeds(True)
            )
            self.drivetrain.set_control(request)

        return apply_request

    @staticmethod
    def compute_heading_to_target(current_pose: Pose2d, target_pose: Pose2d) -> float:
        relative_pose = current_pose.relativeTo(target_pose)
        return math.atan2(relative_pose.Y(), relative_pose.X())

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

        sign = math.copysign(1, input)  # Use math.copysign for clarity
        scaled = (abs(input) - deadband) / (1.0 - deadband)
        return sign * math.pow(scaled, exponent)

    @property
    def limelight_subsystem(self):
        if not hasattr(self, '_limelight_subsystem'):
            self._limelight_subsystem = LimelightSubsystem()
        return self._limelight_subsystem