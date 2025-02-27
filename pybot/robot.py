#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing
import choreo 

from robotcontainer import RobotContainer

# class MyRobot(commands2.TimedCommandRobot):
class MyRobot(wpilib.TimedRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None
    def is_red_alliance(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    def robotInit(self) -> None:
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.field = wpilib.Field2d()
        self.timer = wpilib.Timer()
        try:
            self.trajectory = choreo.load_swerve_trajectory("COMPLEXCRAZY")
        except ValueError:
            print("Trajectory not found")
            self.trajectory = None

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        self.scheduler.run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        ##self.autonomousCommand = self.container.getAutonomousCommand()
        #if self.autonomousCommand:
        #    self.autonomousCommand.schedule()
        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance())

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.container.drivetrain.reset_pose(initial_pose)

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance())

            if sample:
                self.container.drivetrain.follow_trajectory(sample)

        print(f"Sample: {sample}\nTimer: {self.timer.get()} \nX Pos: {self.container.drivetrain.get_pose().X()} \nY Pos: {self.container.drivetrain.get_pose().Y()}\nRot: {self.container.drivetrain.get_pose().rotation().degrees()}")
        self.field.setRobotPose(self.container.drivetrain.get_pose())

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # Configure button bindings
        self.container.configureButtonBindings()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        #
        # print(f"X Pos: {self.container.drivetrain.get_pose().X()} \nY Pos: {self.container.drivetrain.get_pose().Y()}\nRot: {self.container.drivetrain.get_pose().rotation().degrees()}")
        #self.field.setRobotPose(self.container.drivetrain.get_pose())

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def simulationInit(self) -> None:
        wpilib.simulation.DriverStationSim.setDsAttached(True)
    def simulationPeriodic(self) -> None:
        print(f"X Pos: {self.container.drivetrain.get_pose().X()} \nY Pos: {self.container.drivetrain.get_pose().Y()}\nRot: {self.container.drivetrain.get_pose().rotation().degrees()}")
        self.field.setRobotPose(self.container.drivetrain.get_pose())
