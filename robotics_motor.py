from machine import PWM, Pin
from i2c_motors_driver import MotorDriver
from utility import *

class DCMotor:
    def __init__(self):
        pass

    def speed(self, value):    
        pass

    def brake(self):    
        pass


class DCMotor_I2C:
    def __init__(self, driver: MotorDriver, index):
        self._driver = driver
        self._index = index
        super().__init__()

    def speed(self, value):
        value = max(min(100, value),-100)
        self._driver.setSpeed(self._index, value)
    
    def brake(self):
        self.speed(0)

class DCMotor_2PIN:
    def __init__(self, in1, in2):
        # motor pins
        self._in1 = PWM(Pin(in1), freq=500, duty=0)
        self._in2 = PWM(Pin(in2), freq=500, duty=0)
        super().__init__()

    def speed(self, value):
        value = max(min(100, value),-100)  

        if value > 0:
            # Forward
            self._in1.duty(int(translate(abs(value), 0, 100, 0, 1023)))
            self._in2.duty(0)
        elif value < 0:
            # Backward
            self._in1.duty(0)
            self._in2.duty(int(translate(abs(value), 0, 100, 0, 1023)))
        else:
            # Release
            self._in1.duty(0)
            self._in2.duty(0)
    
    def brake(self):
        self._in1.duty(1023)
        self._in2.duty(1023)

class DCMotor_3PIN:
    def __init__(self, in1, in2, pwm, stby=None):
        # motor pins
        self._in1 = Pin(in1, mode=Pin.OUT, pull=None)
        self._in2 = Pin(in2, mode=Pin.OUT, pull=None)
        self._pwm = PWM(pwm, freq=500, duty=0)
        
        if stby:
            self._stby = Pin(stby, mode=Pin.OUT, pull=None)
            self._stby.value(1)
        super().__init__()

    def speed(self, value):
        value = max(min(100, value),-100)  

        if value > 0:
            # Forward
            self._in1.value(1)
            self._in2.value(0)
        elif value < 0:
            # Backward
            self._in1.value(0)
            self._in2.value(1)
        else:
            # Release
            self._in1.duty(0)
            self._in2.duty(0)
        
        self._pwm.duty(int(translate(abs(value), 0, 100, 0, 1023)))
    
    def brake(self):
        self._in1.value(1)
        self._in2.value(1)
        self._pwm.duty(0)



