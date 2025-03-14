import logging
from phoenix6.hardware import talon_fx
from phoenix6.controls.follower import Follower
from phoenix6.signals import NeutralModeValue

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Lifter:
    def __init__(self, master_id, follower_id):
        self.motor = talon_fx.TalonFX(master_id)
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)
        self.follower = talon_fx.TalonFX(follower_id)
        self.follower.set_control(Follower(master_id, True))
        self.follower.setNeutralMode(NeutralModeValue.BRAKE)

    def moveDown(self):
            self.motor.set(-0.5)  # Gentle downward movement

    def moveUp(self):
            self.motor.set(0.7)  # Upward movement


    def stop(self):
            self.motor.stopMotor()
        
    def setMotor(self, value):
            self.motor.set(value)
