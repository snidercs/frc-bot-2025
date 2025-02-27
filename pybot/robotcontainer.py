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

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from lifter import Lifter  # Import the Lifter class
import choreo  # Make sure to import choreo
import wpilib


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.timer = wpilib.Timer()
        self._max_speed = (
            TunerConstants.speed_at_12_volts * .3
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

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
        self.elevator = Lifter(20, 14)
        """
        20: .18
        14: .97


        20: -101.67
        14: -100.95
        """

        # Initialize the intake with motor IDs
        self.intake = Lifter(300, 400)

        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        if hasattr(self, '_joystick') and self._joystick is not None:
            return

        self._joystick = commands2.button.CommandXboxController(0)

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        #self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        #self._joystick.b().whileTrue(
        #    self.drivetrain.apply_request(
        #        lambda: self._point.with_module_direction(
        #            Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
        #        )
        #    )
        #)

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftTrigger().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
            
        )
        
        # Configure buttons for elevator control
        self._joystick.y().whileTrue(commands2.cmd.run(self.elevator.move_up, self.elevator))
        
        self._joystick.a().whileTrue(commands2.cmd.run(self.elevator.move_down, self.elevator))
        self._joystick.rightTrigger().onTrue(commands2.cmd.runOnce(self.elevator.stop, self.elevator))

        self._joystick.setRumble(wpilib.interfaces.GenericHID.RumbleType.kBothRumble, 1)
        # Configure buttons for intake control
        #self._joystick.a().whileTrue(commands2.cmd.run(self.intake.move_up, self.intake))
        #self._joystick.b().whileTrue(commands2.cmd.run(self.intake.move_down, self.intake))
        #self._joystick.x().onTrue(commands2.cmd.runOnce(self.intake.stop, self.intake))

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        try:
            trajectory = choreo.load_swerve_trajectory("COMPLEXCRAZY")
        except ValueError:
            trajectory = None

        if trajectory:
            initial_pose = trajectory.get_initial_pose(self.is_red_alliance())
            if initial_pose:
                self.drivetrain.reset_pose(initial_pose)

            return commands2.cmd.run(
                lambda: self.drivetrain.follow_trajectory(
                    trajectory.sample_at(self.timer.get(), self.is_red_alliance())
                ),
                self.drivetrain
            )
        else:
            return commands2.cmd.print_("No autonomous command configured")

    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
