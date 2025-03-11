from commands2 import Command
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d

import math

class PointAtCoordinateCommand(Command):

    def __init__(self, drivetrain, target_pose: Pose2d):
        super().__init__()
        self.drivetrain = drivetrain
        self.target_pose = target_pose

        # Create and configure our heading PID. We only use P here, but you may want D or I.
        self.heading_pid = PIDController(5.0, 0.0, 0.0)
        # Enable continuous input so angles wrap at ±π
        self.heading_pid.enableContinuousInput(-math.pi, math.pi)

        self.addRequirements(drivetrain)

    def initialize(self):
        print(f"[PointAtCoordinate] Target: {self.target_pose}")
        # Optionally reset the PID's internal state
        self.heading_pid.reset()

    def execute(self):
        print("IVE BEEN CALLED")
        # 1) Driver’s XY input
        joyvalues = self.drivetrain.calculateJoystick()
        forward = joyvalues[0]
        strafe = joyvalues[1]

        # 2) Current robot pose from swerve
        current_pose = self.drivetrain.get_pose()

        # 3) Compute angle from robot to target
        heading_to_target = self.compute_heading_to_target(current_pose, self.target_pose)
        current_heading = current_pose.rotation().radians()

        # 4) Update the heading PID setpoint each iteration
        self.heading_pid.setSetpoint(heading_to_target)

        # 5) Calculate turn_command = PID output
        turn_command = self.heading_pid.calculate(current_heading)

        # 6) Build a SwerveRequest for field-centric velocity
        request = self.drivetrain.requests.ApplyFieldSpeeds().with_speeds(
            ChassisSpeeds(forward, strafe, turn_command)
        ).with_drive_request_type(
            self.drivetrain.swerve_module.SwerveModule.DriveRequestType.VELOCITY
        ).with_steer_request_type(
            self.drivetrain.swerve_module.SwerveModule.SteerRequestType.POSITION
        ).with_desaturate_wheel_speeds(True)

        # 7) Send to swerve
        self.swerve.set_control(request)

    def isFinished(self):
        return False

    def end(self, interrupted):
        pass

    @staticmethod
    def compute_heading_to_target(current_pose: Pose2d, target_pose: Pose2d) -> float:
        dx = target_pose.X() - current_pose.X()
        dy = target_pose.Y() - current_pose.Y()
        return math.atan2(dy, dx)