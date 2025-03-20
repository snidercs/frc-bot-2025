from dualmotor import DualMotor
from phoenix6.controls import DutyCycleOut# , NeutralOut
from phoenix6.configs import TalonFXConfiguration

HOLDING_POWER = 0.015
GOING_UP_POWER = 0.7
GOING_DOWN_POWER = -0.5

class Elevator(DualMotor):
    def __init__(self, motor1_id, motor2_id):
        super().__init__(motor1_id, motor2_id)

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