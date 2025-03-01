import logging
from phoenix6.hardware import talon_fx

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Lifter:
    def __init__(self, master_id):
        self.motor = talon_fx.TalonFX(master_id)

    def moveDown(self):
        try:
            if self.motor.get_reverse_limit() == 0:
                logger.warning("Reverse limit reached; cannot move down")
                self.stop()
            else:
                logger.info("Moving down")
                self.motor.set(-0.05)  # Gentle downward movement
        except Exception as e:
            logger.error(f"Error moving down: {e}")

    def moveUp(self):
        try:
            if self.motor.get_forward_limit() == 0:
                logger.warning("Forward limit reached; cannot move up")
                self.stop()
            else:
                logger.info("Moving up")
                self.motor.set(0.10)  # Upward movement
        except Exception as e:
            logger.error(f"Error moving up: {e}")


    def stop(self):
        try:
            self.motor.stopMotor()
        except Exception as e:
            logger.error(f"Error stopping motor: {e}")
        
    def setMotor(self, value):
        try:
            self.motor.set(value)
            logging.info(f"SETTING TO {value}")
        except Exception as e:
            logger.error(f"Error setting motor: {e}")
