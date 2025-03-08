#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import os, typing
import wpilib, commands2
from robotcontainer import RobotContainer

AUTOMODE_DEFAULT = 'outwayred'
LATENCY_SECONDS = 0.02

class MyRobot(wpilib.TimedRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None
    chooser: None
    
    def __init__(self):
        super().__init__(LATENCY_SECONDS)
    
    def robotInit(self) -> None:
        print('Robot Init')

        self.container = RobotContainer()

        if (self.isSimulation()):
            print('Entering Simulation...')
            self.addPeriodic(lambda: self.container.drivetrain.update_sim_state(0.02, 12), 0.02, 0.02)

        self.scheduler = commands2.CommandScheduler.getInstance()
        self.registerTrajectories()

    def registerTrajectories(self) -> None:
        traj_dir = f"{wpilib.getOperatingDirectory()}/deploy/choreo"
        traj_files = []
        for f in os.listdir (traj_dir):
            if f.endswith('.traj'):
                traj_files.append (f)
        
        self.chooser = wpilib.SendableChooser()
        for traj_file in traj_files:
            self.chooser.addOption (traj_file.removesuffix ('.traj'),
                                    traj_file.removesuffix ('.traj'))        
        self.chooser.setDefaultOption (AUTOMODE_DEFAULT, AUTOMODE_DEFAULT)

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

        self.container.configureButtonBindings()

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        self.scheduler.cancelAll()

    def _simulationInit(self) -> None:
        pass

    def _simulationPeriodic(self) -> None:
        pass
