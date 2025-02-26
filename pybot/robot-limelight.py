#!/usr/bin/env python3
#
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import limelight
import limelightresults

class MyRobot(wpilib.TimedRobot):
    """Main robot class."""

    def robotInit(self):
        """Robot-wide initialization code should go here."""
        self.lstick = wpilib.Joystick(0)
        self.motor = wpilib.Talon(3)
        self.timer = wpilib.Timer()
        self.loops = 0
        self.limelight = None
    
    def limelightInit(self):
        # save for reference
        print("----------- LIMELIGHT -----------")
        addy = limelight.discover_limelights()[0]
        self.limelight = ll = limelight.Limelight(addy)
        result = ll.get_results()
        print(result)
        print("-----------")
        parsed_result = limelightresults.parse_results(result)
        
        for result in parsed_result.fiducialResults:
            print("fiducial_id")
            print(result.fiducial_id)
        print("----------- LIMELIGHT -----------")

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def disabledInit(self):
        self.logger.info("%d loops / %f seconds", self.loops, self.timer.get())

    def disabledPeriodic(self):
        pass

    def teleopInit (self):
        self.loops = 0
        self.timer.reset()
        self.timer.start()

    def teleopPeriodic (self):
        if self.limelight != None:
            status = self.limelight.get_status()
            result = self.limelight.get_results()
            parsed_result = limelightresults.parse_results(result)
            for fiducial_result in parsed_result.fiducialResults:
                print(f"fiducial_id: {fiducial_result.fiducial_id}, cpu: {status['cpu']}")
