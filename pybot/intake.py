from dualmotor import DualMotor

SHOOTING_POWER = -.30
LOADING_POWER = -.15


class Intake(DualMotor):
    def __init__(self, motor1_id, motor2_id):
        super().__init__(motor1_id, motor2_id)

    def shoot(self):
        """Shoots the coral."""
        self.setMotor(SHOOTING_POWER)
    
    def load(self):
        """Loads the coral."""  
        self.setMotor(LOADING_POWER)