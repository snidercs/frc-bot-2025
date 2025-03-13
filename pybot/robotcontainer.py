#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from wpimath.geometry import Pose2d, Rotation2d
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
from limelightinit import LimelightSubsystem  # Import the LimelightSubsystem class
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

        # Initialize the Limelight subsystem
        self.limelight_subsystem = LimelightSubsystem()

        # Setup telemetry
        self._registerTelemetery()
    
    def calculateJoystick(self):
            x0 = self._joystick.getLeftX()
            y0 = self._joystick.getLeftY()

            mag = self.apply_exponential(math.sqrt((x0 * x0) + (y0 * y0)), self._deadband, self._exponent) * self._max_speed
            theta = math.atan2(y0, x0)

            x1 = mag * math.cos(theta)
            y1 = mag * math.sin(theta)

            return (x1, y1)

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

        self._joystick.leftBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(.3),
            lambda: self.intake.stop()
        ))
        # Configure buttons for intake control
        self._joystick.rightBumper().whileTrue(commands2.cmd.startEnd(
            lambda: self.intake.setMotor(1),
            lambda: self.intake.stop()
        ))

        #if self.limelight_subsystem.limelight1 is not None:
            # Configure button to call PointAtCoordinateCommand with Limelight data
        self._joystick.b().whileTrue(
            commands2.cmd.run(self.create_point_at_coordinate_request(), self.drivetrain)
        )

    def create_point_at_coordinate_request(self):
        dummy_pose = True
        # Use the LimelightSubsystem to get the primary fiducial ID and target pose
        if not dummy_pose:
            if self.limelight_subsystem.limelight1 is None:
                logging.warning("Limelight1 is not initialized")
                return commands2.cmd.none()
            else:
                fiducial_id = self.limelight_subsystem.get_primary_fiducial_id(self.limelight_subsystem.limelight1)
                target_pose = self.limelight_subsystem.get_target_pose(fiducial_id)
                if fiducial_id == -1 or target_pose is None:
                    logging.warning("No valid fiducial ID or target pose")
                    return commands2.cmd.none()

        # Create and configure our heading PID. We only use P here, but you may want D or I.
        heading_pid = PIDController(1.6, 0.0, 0.05)
        # Enable continuous input so angles wrap at ±π
        heading_pid.enableContinuousInput(-math.pi, math.pi)

        def apply_request():
            # 1) Driver’s XY input
            joyvalues = self.calculateJoystick()
            forward = joyvalues[1]
            strafe = joyvalues[0]

            # 2) Current robot pose from swerve
            current_pose = self.drivetrain.get_pose()

            if dummy_pose:
                heading_to_target = self.compute_heading_to_target(current_pose, Pose2d(2, 1, 1/2))

            else:
            # 3) Compute angle from robot to target
                heading_to_target = self.compute_heading_to_target(current_pose, Pose2d(target_pose[0], target_pose[1], target_pose[5]))
            
            current_heading = current_pose.rotation().radians()


            # 4) Update the heading PID setpoint each iteration
            heading_pid.setSetpoint(heading_to_target)

            # 5) Calculate turn_command = PID output
            turn_command = heading_pid.calculate(current_heading)

            # 6) Build a SwerveRequest for field-centric velocity
            request = swerve.requests.ApplyFieldSpeeds().with_speeds(
                ChassisSpeeds(forward, strafe, turn_command)
            ).with_drive_request_type(swerve.swerve_module.SwerveModule.DriveRequestType.VELOCITY \
            ).with_steer_request_type(swerve.swerve_module.SwerveModule.SteerRequestType.POSITION \
            ).with_desaturate_wheel_speeds(True)

            # 7) Send to swerve
            self.drivetrain.set_control(request)

        return apply_request

    @staticmethod
    def compute_heading_to_target(current_pose: Pose2d, target_pose: Pose2d) -> float:
        dx = target_pose.X() - current_pose.X()
        dy = target_pose.Y() - current_pose.Y()
        return math.atan2(dy, dx)

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
        scaled = (magnitude - deadband) / (1.0 - deadband)

        # 4. Apply the exponential curve
        exponentiated = math.pow(scaled, exponent)

        # 5. Restore the sign and return the result
        return sign * exponentiated