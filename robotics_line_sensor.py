from machine import Pin, SoftI2C
from utility import *
from setting import *
from micropython import const
import robotics_pcf8574


LINE_LEFT3 = const(-3)
LINE_LEFT2 = const(-2)
LINE_LEFT = const(-1)
LINE_CENTER = const(0)
LINE_RIGHT = const(1)
LINE_RIGHT2 = const(2)
LINE_RIGHT3 = const(3)
LINE_CROSS = const(4)
LINE_END = const(5)

class LineSensor:
    def __init__(self):
        pass

    '''
        Chheck robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self): 
        return 0

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):    
        return 0


class LineSensor2P(LineSensor):
    def __init__(self, s1, s2):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        super().__init__()

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self):
        state = self.read()
        if state == (0, 0):
            return LINE_CENTER
        elif state == (0, 1):
            return LINE_LEFT2
        elif state == (1, 0):
            return LINE_RIGHT2
        else:
            return LINE_CROSS

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):
        if index == 0:
            return self._s1.value()
        elif index == 1:
            return self._s2.value()
        else:
            return (self._s1.value(), self._s2.value())

class LineSensor3P:
    def __init__(self, s1, s2, s3):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        self._s3 = Pin(s3, Pin.IN)
        super().__init__()

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self):
        state = self.read()
        if state == (1, 1, 1):
            return LINE_CROSS
        elif state == (0, 0, 0):
            return LINE_END
        elif state == (1, 1, 0):
            return LINE_RIGHT
        elif state == (0, 1, 1):
            return LINE_LEFT
        elif state == (1, 0, 0):
            return LINE_RIGHT2
        elif state == (0, 0, 1):
            return LINE_LEFT2
        else:
            return LINE_CENTER

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):
        if index == 0:
            return self._s1.value()
        elif index == 1:
            return self._s2.value()
        else:
            return (self._s1.value(), self._s2.value())


class LineSensor_PCF8574(LineSensor):
    def __init__(self, address=0x23):
        scl_pin = Pin(SCL_PIN)
        sda_pin = Pin(SDA_PIN)
        self.i2c_pcf = SoftI2C(scl=scl_pin, sda=sda_pin, freq=100000)
        self.address = address

        try:
            self.pcf = robotics_pcf8574.PCF8574(self.i2c_pcf, self.address)
        except:
            self.pcf = None
            print('Line sensor not found')

    def read(self, index=None):
        # 0 white, 1 black
        if self.pcf == None:
            return 0

        if index == None:
            return (self.pcf.pin(0), self.pcf.pin(1), self.pcf.pin(2), self.pcf.pin(3))

        return self.pcf.pin(index)

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self): 
        now = self.read()
        #print(now)
        if now == (0, 0, 0, 0):
            return LINE_END
        elif now == (1, 1, 1, 1):
            return LINE_CROSS
        elif (now[1], now[2]) == (1, 1) or now == (1, 0, 0, 1):
            return LINE_CENTER
        elif (now[0], now[1]) == (1, 1): 
            return LINE_RIGHT2
        elif (now[2], now[3]) == (1, 1): 
            return LINE_LEFT2
        elif now == (0, 0, 1, 0): 
            return LINE_RIGHT
        elif now == (0, 1, 0, 0): 
            return LINE_LEFT
        elif now[1] == 1: 
            return LINE_RIGHT2
        elif now[2] == 1:
            return LINE_LEFT2
        elif now[0] == 1: 
            return LINE_RIGHT3
        elif now[3] == 1: 
            return LINE_LEFT3