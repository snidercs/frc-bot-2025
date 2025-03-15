from phoenix6.hardware import talon_fx
from phoenix6.controls.follower import Follower
from phoenix6.signals import NeutralModeValue

class DualMotor:
    def __init__(self, master_id, follower_id) -> None:
        self.motor = talon_fx.TalonFX(master_id)
        self.follower = talon_fx.TalonFX(follower_id)
        
        # Set neutral mode
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)
        self.follower.setNeutralMode(NeutralModeValue.BRAKE)
        self.follower.set_control(Follower(master_id, True))


    def stop(self) -> None:
        self.motor.stopMotor()
        
    def setMotor(self, value) -> None:
        self.motor.set(value)
