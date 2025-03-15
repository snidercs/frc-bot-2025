from dualmotor import DualMotor
from phoenix6.controls import DutyCycleOut# , NeutralOut
from phoenix6.configs import TalonFXConfiguration

HOLDING_POWER = 0.02
GOING_UP_POWER = 0.7
GOING_DOWN_POWER = -0.5

class Elevator(DualMotor):
    def __init__(self, motor1_id, motor2_id):
        super().__init__(motor1_id, motor2_id)
        # Additional initialization for the elevator can go here

        # I want to have it torque out at mins and max's, not sure if I'm doing this right here though lol

        # Configure current limits
        #motor_config = TalonFXConfiguration()
        #motor_config.SupplyCurrentLimit = 70  # Default limit of 70 A
        #motor_config.SupplyCurrentLowerLimit = 40  # Reduce to 40 A if at 70 A for 1 second
        #motor_config.SupplyCurrentLowerTime = 1  # Time in seconds
        #motor_config.SupplyCurrentLimitEnable = True  # Enable supply current limiting
        
        #motor_config.StatorCurrentLimit = 120  # Limit stator to 120 A
        #motor_config.StatorCurrentLimitEnable = True  # Enable stator current limiting
        
        # Apply configuration to both motors
        #self.motor.configurator = motor_config
        #self.follower.configurator = motor_config

    def move_to_position(self, position):
        # Implement logic to move the elevator to a specific position. Not sure on the units yet
        pass

    def moveUp(self) -> None:
        self.setMotor(GOING_UP_POWER)

    def moveDown(self) -> None:
        self.setMotor(GOING_DOWN_POWER)

    def stop(self) -> None:
        self.motor.stopMotor()
        self.motor.set_control(DutyCycleOut(HOLDING_POWER))