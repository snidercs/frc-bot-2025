#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import wpilib, commands2

import autos
from robotcontainer import RobotContainer

LATENCY_SECONDS = 0.03

class MyRobot(wpilib.TimedRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None
    chooser: None
    
    def __init__(self):
        super().__init__(LATENCY_SECONDS)
    
    def robotInit(self) -> None:
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.registerTrajectories()

    def registerTrajectories(self) -> None:
        self.chooser = autos.createChooser()
        wpilib.SmartDashboard.putData ('Trajectory Files', self.chooser)

    def selectedTrajectory(self) -> str:
        return self.chooser.getSelected()

    def robotPeriodic(self) -> None:
        self.scheduler.run()

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        selected = self.selectedTrajectory()
        self.autonomousCommand = self.container.getAutonomousCommand (selected)
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        if self.autonomousCommand:
           self.autonomousCommand.cancel()
           self.autonomousCommand = None

        self.container.configureButtonBindings()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        self.scheduler.cancelAll()

    def simulationInit(self) -> None:
        pass

    def simulationPeriodic(self) -> None:
        pass
