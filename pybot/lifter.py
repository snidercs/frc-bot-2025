from phoenix6.hardware import talon_fx
from phoenix6.controls.follower import Follower
from phoenix6.signals import NeutralModeValue

class Lifter:
    def __init__(self, master_id, follower_id) -> None:
        self.motor = talon_fx.TalonFX(master_id)
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)
        self.follower = talon_fx.TalonFX(follower_id)
        self.follower.set_control(Follower(master_id, True))
        self.follower.setNeutralMode(NeutralModeValue.BRAKE)

    def moveDown(self) -> None:
                self.motor.set(-0.2)

    def moveUp(self) -> None:
                self.motor.set(0.3)

    def stop(self) -> None:
            self.motor.stopMotor()
        
    def setMotor(self, value) -> None:
            self.motor.set(value)
