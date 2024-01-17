import machine
from time import sleep
from struct import pack, unpack

from setting import *
from utility import *

MD4C_DEFAULT_I2C_ADDRESS = 0x30
# Motor Index
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

class MotorDriver():
    def __init__(self):
        # Grove port: GND VCC SCL SDA
        scl_pin = machine.Pin(SCL_PIN)
        sda_pin = machine.Pin(SDA_PIN)
        self._i2c = machine.SoftI2C(scl=scl_pin, sda=sda_pin, freq=100000)
        
        self._addr = MD4C_DEFAULT_I2C_ADDRESS
        self._motor_type = 0x00
        #check connection, not for now
        who_am_i = MD4C_DEFAULT_I2C_ADDRESS # turn off who am i if while not implemented
        
        if who_am_i != MD4C_DEFAULT_I2C_ADDRESS:
            print(who_am_i)
            raise RuntimeError("Could not find motor driver at address 0x{:X}".format(address))
        else:
            # enable motors
            self.setSpeed(MD4C_REG_CH1, 0)
            self.setSpeed(MD4C_REG_CH2, 0)
            self.setSpeed(MD4C_REG_CH3, 0)
            self.setSpeed(MD4C_REG_CH4, 0)
            print("Motor driver 4 channel initialized")

    def setSpeed(self, motor_index, speed):
        if motor_index not in (MD4C_REG_CH1,MD4C_REG_CH2,MD4C_REG_CH3,MD4C_REG_CH4):
            raise RuntimeError('Invalid motor number')

        if speed < -DC_MOTOR_MAX_SPEED:
            speed = -DC_MOTOR_MAX_SPEED
        elif speed > DC_MOTOR_MAX_SPEED:
            speed = DC_MOTOR_MAX_SPEED
        
        speed = int(speed)
            
            
        if speed < 0:
            direction = 1
            speed = - speed
        else:
            direction = 0
        
        # if direction not in (DIR_FORWARD,DIR_BACKWARD):
        #    raise RuntimeError('Direction is not valid') 

        self._write(self._motor_type, pack('BBBBBBBB', motor_index, direction, speed >> 8,speed & 0xFF , 0, 0, 0, 0))
    
    def set_motor_time(self, motor_index, speed, t = None):
        if motor_index not in (MD4C_REG_CH1,MD4C_REG_CH2,MD4C_REG_CH3,MD4C_REG_CH4):
            raise RuntimeError('Invalid motor number')
        
        if speed < -DC_MOTOR_MAX_SPEED:
            speed = -DC_MOTOR_MAX_SPEED
        elif speed > DC_MOTOR_MAX_SPEED:
            speed = DC_MOTOR_MAX_SPEED
        
        if speed < 0:
            direction = 1
            speed = - speed
        else:
            direction = 0
        self._write(self._motor_type, pack('BBBBBBBB', motor_index, direction, speed >> 8,speed & 0xFF , 0, 0, 0, 0))
        if t != None:
            sleep(t)
            self.setSpeed(motor_index,0)

    def fullOn(self, motor_index, direction):
        self.setSpeed(motor_index, direction , 100)

    def fullOff(self, motor_index, direction):
        self.setSpeed(motor_index, direction, 0)

    def _write(self, register, data):
        # Write 1 byte of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, data)


class StepperMotor():
    def __init__(self, number_step=200):
        # Grove port: GND VCC SCL SDA
        scl_pin = machine.Pin(SCL_PIN)
        sda_pin = machine.Pin(SDA_PIN)
        self._i2c = machine.SoftI2C(scl=scl_pin, sda=sda_pin, freq=100000)
        
        self._addr = MD4C_DEFAULT_I2C_ADDRESS
        self._motor_type = 0x01
        self._number_step = number_step
        #check connection, not for now
        who_am_i = MD4C_DEFAULT_I2C_ADDRESS # turn off who am i if while not implemented
        
        if who_am_i != MD4C_DEFAULT_I2C_ADDRESS:
            print(who_am_i)
            raise RuntimeError("Could not find motor driver at address 0x{:X}".format(address))
        else:
            # enable motors
            self.setSpeed(MD4C_REG_CH1, DIR_FORWARD, 0)
            self.setSpeed(MD4C_REG_CH1, DIR_FORWARD, 0)
            print("Stepper Motor 2 channel initialized")

    def setSpeed(self, motor_index, direction, speed, style = STEPPER_STYLE_INTERLEAVE):
        
        if motor_index not in (MD4C_REG_CH1,MD4C_REG_CH2):
            raise RuntimeError('Invalid motor number')

        if (speed < 0) or (speed > STEPPER_MOTOR_MAX_SPEED):
            raise RuntimeError('Speed is out of range')
        
        if direction not in (DIR_FORWARD,DIR_BACKWARD):
            raise RuntimeError('Direction is not valid') 

        self._write(self._motor_type, pack('BBBBBBBB', motor_index , self._number_step >> 8, (self._number_step )& 0xFF , style , STEPPER_MODE_SPEED , direction, speed >> 8, speed & 0xFF))
    

    def step(self, motor_index, direction, step , style = STEPPER_STYLE_DOUBLE):
        
        if motor_index not in (MD4C_REG_CH1,MD4C_REG_CH2):
            raise RuntimeError('Invalid motor number')

        if direction not in (DIR_FORWARD,DIR_BACKWARD):
            raise RuntimeError('Direction is not valid') 

        self._write(self._motor_type, pack('BBBBBBBB', motor_index , self._number_step >> 8, (self._number_step )& 0xFF , style , STEPPER_MODE_STEP , direction, step >> 8, step & 0xFF))
    
    def onestep(self, motor_index, direction, style = STEPPER_STYLE_DOUBLE):
        self.step(motor_index, direction, style)

    def release(self, motor_index):
        self.setSpeed(motor_index, DIR_FORWARD, 0)

    def _write(self, register, data):
        # Write 1 byte of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, data)
