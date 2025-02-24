from phoenix6.hardware import talon_fx
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration, MotorOutputConfigs
from phoenix6.signals import InvertedValue

class Lifter:
    def __init__(self, motor_ids):
        self.motors = {id: talon_fx.TalonFX(id) for id in motor_ids}
        
        self._configure_motors()

    def _configure_motors(self):
        try:
            if 20 in self.motors.keys():
                motor_20 = self.motors[20]
                motor_20.configFactoryDefault()
                config_20 = TalonFXConfiguration()
                motor_output_config_20 = MotorOutputConfigs()
                motor_output_config_20.inverted = InvertedValue.CLOCKWISE_POSITIVE
                config_20.with_motor_output(motor_output_config_20)
                motor_20.getConfigurator().apply(config_20)
                motor_20.configPeakCurrentLimit(40)
                config.with_motor_output(motor_output_config)
                
                # Apply the configuration to the motor
                motor.getConfigurator().apply(config)
                
                motor.configPeakCurrentLimit(40)
                motor.configContinuousCurrentLimit(30)
                motor.enableCurrentLimit(True)
            if 14 in self.motors.keys():
                motor_14 = self.motors[14]
                motor_14.configFactoryDefault()
                config_14 = TalonFXConfiguration()
                motor_output_config_14 = MotorOutputConfigs()
                motor_output_config_14.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                config_14.with_motor_output(motor_output_config_14)
                motor_14.getConfigurator().apply(config_14)
                motor_14.configPeakCurrentLimit(40)
                config.with_motor_output(motor_output_config)
                
                # Apply the configuration to the motor
                motor.getConfigurator().apply(config)
                
                motor.configPeakCurrentLimit(40)
                motor.configContinuousCurrentLimit(30)
                motor.enableCurrentLimit(True)
        except Exception as e:
            pass

    def move_up(self):
        try:
            for motor in self.motors:
                motor.set(0.1)
        except Exception as e:
            pass

    def move_down(self):
        try:
            for motor in self.motors:
                motor.set(-0.1)
        except Exception as e:
            pass

    def stop(self):
        try:
            for motor in self.motors:
                motor.set(0.0)
        except Exception as e:
            pass

    def get_positions(self):
        try:
            positions = [motor.get_position().value for motor in self.motors]
            return positions
        except Exception as e:
            return []