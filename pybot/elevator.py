from dualmotor import DualMotor
from phoenix6.controls import DutyCycleOut#, NeutralOut

HOLDING_POWER = 0.02  # Holding power to prevent the elevator from falling
GOING_UP_POWER = 0.7  # Power to move up
GOING_DOWN_POWER = -0.5  # Power to move down
STALL_CURRENT_THRESHOLD_UP = .18  # Amperage threshold for stall detection
STALL_CURRENT_THRESHOLD_DOWN = .9  # Amperage threshold for stall detection

class Elevator(DualMotor):
    def __init__(self, motor1_id, motor2_id):
        super().__init__(motor1_id, motor2_id)

    def move_to_position(self, position):
        # No encoders, so this method is unused for now
        pass

    def moveUp(self) -> None:
        while not self.check_stall(True):
            self.motor.set(GOING_UP_POWER)

    def moveDown(self) -> None:
        while not self.check_stall(False):
            self.motor.set(GOING_DOWN_POWER)

    def stop(self) -> None:
        """Stops the motor and applies holding power to prevent gravity drop."""
        self.motor.set_control(DutyCycleOut(HOLDING_POWER))  

    def check_stall(self, going_up: bool) -> bool:
        """Detects if the elevator is stalled by checking current draw."""
        current = self.motor.get_supply_current().value_as_double  # Get motor current draw
        print(f"Current supply current: {current}")
        
        if going_up:

            if current >= STALL_CURRENT_THRESHOLD_UP:
                print(f"Stall detected GOING UP! Stopping motor. stall current: {current}")
                self.stop()
                return True  # Stall detected, stop moving
            
        else:
            if current <= STALL_CURRENT_THRESHOLD_DOWN:
                print(f"Stall detected GOING DOWN! Stopping motor. stall current: {current}")
                self.stop()
                return True  # Stall detected, stop moving

        return False  # No stall, keep moving
