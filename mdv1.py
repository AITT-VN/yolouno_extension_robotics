import machine
from struct import pack

from setting import *
from utility import *

MD4C_DEFAULT_I2C_ADDRESS = 0x30
# Motor Index
MDV1_M1 = const(0)
MDV1_M2 = const(1)
MDV1_M3 = const(2)
MDV1_M4 = const(3)

MDV1_STEPPER1 = const(0)
MDV1_STEPPER2 = const(1)

MD4C_REG_CH1 = const(0)
MD4C_REG_CH2 = const(1)
MD4C_REG_CH3 = const(2)
MD4C_REG_CH4 = const(3)
# Direction
DIR_FORWARD = const(0)
DIR_BACKWARD = const(1)
# Stepper Style Controls
STEPPER_STYLE_SINGLE = const(0)
STEPPER_STYLE_DOUBLE = const(1)
STEPPER_STYLE_INTERLEAVE = const(2)
# Stepper Mode
STEPPER_MODE_SPEED = const(0)
STEPPER_MODE_STEP = const(1)
# Max Speed
DC_MOTOR_MAX_SPEED = 100
STEPPER_MOTOR_MAX_SPEED = 255

# motor type
MOTOR_DC = const(0x00)
MOTOR_STEPPER = const(0x01)

class MotorDriverV1():
    def __init__(self):
        # Grove port: GND VCC SCL SDA
        scl_pin = machine.Pin(SCL_PIN)
        sda_pin = machine.Pin(SDA_PIN)
        self._i2c = machine.SoftI2C(scl=scl_pin, sda=sda_pin, freq=100000)
        
        self._addr = MD4C_DEFAULT_I2C_ADDRESS
        self._motor_type = MOTOR_DC

        self._steps_per_rev = 200
        
        # enable motors
        self.set_motors(MDV1_M1, 0)
        self.set_motors(MDV1_M2, 0)
        self.set_motors(MDV1_M3, 0)
        self.set_motors(MDV1_M4, 0)
        print("Motor driver V1 initialized")

    def set_motors(self, motor_index, speed):
        if motor_index not in (MDV1_M1, MDV1_M2, MDV1_M3, MDV1_M4):
            raise RuntimeError('Invalid motor number')

        speed = max(min(DC_MOTOR_MAX_SPEED, int(speed)), -DC_MOTOR_MAX_SPEED)
        
        if speed < 0:
            direction = DIR_BACKWARD
            speed = - speed
        else:
            direction = DIR_FORWARD

        self._write(MOTOR_DC, pack('BBBBBBBB', motor_index, direction, speed >> 8, speed & 0xFF , 0, 0, 0, 0))

    def stop(self, motors):
        self.set_motors(motors, 0)

    def brake(self, motors):
        self.set_motors(motors, 0)
    
    def stepper_speed(self, stepper, steps_per_rev, speed, style=STEPPER_STYLE_INTERLEAVE):

        if stepper not in (MDV1_STEPPER1, MDV1_STEPPER2):
            raise RuntimeError('Invalid motor number')

        self._steps_per_rev = steps_per_rev
        speed = max(min(STEPPER_MOTOR_MAX_SPEED, int(speed)), 0)

        direction = DIR_FORWARD

        self._write(self._motor_type, pack('BBBBBBBB', stepper , self._steps >> 8, (self._steps)& 0xFF , style , STEPPER_MODE_SPEED , direction, speed >> 8, speed & 0xFF))


    def stepper_step(self, stepper, steps, style=STEPPER_STYLE_DOUBLE):
        if stepper not in (MDV1_STEPPER1, MDV1_STEPPER2):
            raise RuntimeError('Invalid stepper motor port')

        dir = DIR_FORWARD
        if steps < 0:
            dir = DIR_BACKWARD
            steps = -steps

        self._write(self._motor_type, pack('BBBBBBBB', stepper , self._steps_per_rev >> 8, (self._steps_per_rev)& 0xFF , style, STEPPER_MODE_STEP, dir, steps >> 8, steps & 0xFF))

    def _write(self, register, data):
        # Write 1 byte of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, data)