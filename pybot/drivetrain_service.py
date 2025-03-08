import commands2
import commands2.button
import commands2.cmd
import numpy as np

from generated.tuner_constants import TunerConstants
from phoenix6 import swerve
from wpimath.units import rotationsToRadians
import wpilib
import math
from wpimath.controller import PIDController
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

floor_threshold = 0.15
def floor(value):
    if (math.fabs(value) < floor_threshold):
        return 0
    
    return value

def field_to_robot(t_vx, t_vy, theta):
    R = np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)]
    ])
    v_field = np.array([t_vx, t_vy])
    v_robot = R.dot(-v_field)

    return (v_robot[0], v_robot[1])

def calculate(controller, current, target):
    return current + controller.calculate(current, target)

class DrivetrainService:
  def __init__(self, drive_request: swerve.requests.RobotCentric, drivetrain: CommandSwerveDrivetrain) -> None:
    self._max_speed = (
        TunerConstants.speed_at_12_volts * 0.72
    )  # speed_at_12_volts desired top speed

    self._max_angular_rate = rotationsToRadians(
        0.75 * .35
        #maybe limit this rotational
    )  # 3/4 of a rotation per second max angular velocity
    self._drive = drive_request
    self.drivetrain = drivetrain

    self.controller_vx = PIDController(0.5, 0.1, 0.0)
    self.controller_vy = PIDController(0.5, 0.1, 0.0)
    self.controller_omega = PIDController(0.5, 0.1, 0.0)

  def getNormalizedHeading(self):
      raw_heading = self.drivetrain.get_state().raw_heading.radians()
      return raw_heading % (2 * math.pi)

  def calculate_request(self, joystick: commands2.button.CommandXboxController):
      alliance_multiplier = -1.0 if self.isRedAlliance() else 1.0

      heading = self.getNormalizedHeading()
      speeds = self.drivetrain.get_chassis_speed()
      vx_current = speeds.vx
      vy_current = speeds.vy
      omega_current = speeds.omega
      vx_target = floor(alliance_multiplier * joystick.getLeftY()) * self._max_speed
      vy_target = floor(alliance_multiplier * joystick.getLeftX()) * self._max_speed
      omega_target = floor(alliance_multiplier * joystick.getRightX()) * self._max_angular_rate

      (vx_target, vy_target) = field_to_robot(vx_target, vy_target, heading)

      vx = calculate(self.controller_vx, vx_current, vx_target)
      vy = calculate(self.controller_vy, vy_current, vy_target)
      rot = calculate(self.controller_omega, omega_current, omega_target)

      # NOTE: For debugging purposes
      # print('Heading: %f, Target: (%f %f), Curr: (%f %f), New: (%f %f)' % (heading, vx_target, vy_target, vx_current, vy_current, vx, vy))

      return (self._drive
          # Drive left with negative X (left) and left trigger for acceleration
          .with_velocity_x(vx)
          # Drive forward with negative Y (forward) and left trigger for acceleration
          .with_velocity_y(vy)
          # Drive counterclockwise with negative X (left)
          .with_rotational_rate(rot))
  
  def isRedAlliance(self):
      return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed