import asyncio
from machine import PWM, Pin
from utility import *
from mdv1 import *
from mdv2 import *

class DCMotor:
    def __init__(self, driver, port, reversed=False):
        self._driver = driver
        self.port = port

        self._encoder_enabled = False
        self._rpm = 0
        self._ppr = 0 # pulses per revolution
        self._gears = 0 # pulses per revolution
        self._cpr = 0 # count pulses per revolution
        self._max_pps = 0 # max count pulses per second

        if reversed:
            self._reversed = -1
        else:
            self._reversed = 1
        
        self.reset_angle()

    def reverse(self):
        if self._reversed == 1:
            self._reversed = -1
        else:
            self._reversed = 1

    def set_encoder(self, rpm, ppr, gears):
        if rpm <= 0 or ppr <= 0 or gears <= 0:
            raise Exception('Invalid encoder pulses config')

        if isinstance(self._driver, MotorDriverV1):
            raise Exception('Motor driver V1 not supported encoder config')

        self._encoder_enabled = True
        self._rpm = rpm
        self._ppr = ppr # pulses per revolution
        self._gears = gears # pulses per revolution
        self._cpr = ppr * 4 * gears # count pulses per revolution
        self._max_pps = rpm * ppr * 4 * gears

        self._driver.set_max_speed(self.port, rpm, ppr, gears)
    
    '''
        Run the motor with given speed.

        The speed is % duty cyle of maximum speed. 
        
        Parameters:
            speed (Number, %) - Speed of the motor to run.

    '''
    def run(self, speed):
        speed = max(min(100, int(speed)),-100)
        self._driver.set_motors(self.port, speed*self._reversed)

    '''
        Actively brakes the motor.

        The motor stops due to being locked by H-bridge IC.
    '''
    def brake(self):
        self._driver.brake(self.port)

    '''
        Passively stop the motor.

        The motor stops due to friction, plus the voltage that is generated while the motor is still moving.
    '''
    def stop(self):
        self._driver.stop(self.port)

    ############## Measuring #################
    
    '''
        Gets the rotation angle of the motor.

        Returns: Motor angle.
    '''
    def angle(self):
        if not self._encoder_enabled:
            return 0
        ticks = self._driver.get_encoder(self.port)
        rotations = (ticks*360*self._reversed)/self._cpr
        return round(rotations, 1)

    '''
        Sets the accumulated rotation angle of the motor to 0.
    '''
    def reset_angle(self):
        if not self._encoder_enabled:
            return
        self._driver.reset_encoder(self.port)

    '''
        Gets the current encoder ticks of the motor.

        Returns: Encoder ticks.
    '''
    def encoder_ticks(self):
        if not self._encoder_enabled:
            return 0
        ticks = self._driver.get_encoder(self.port)
        return ticks
    
    '''
        Get the speed of the motor.

        The speed is measured as the change in the motor angle during last 100ms. 

        Returns:
            Motor speed (rpm).
    '''
    def speed(self):
        if not self._encoder_enabled:
            return 0

        return round(self._driver.get_speed(self.port)/self._cpr, 1)
    
    '''
        Runs the motor at a constant speed towards a given target angle.

        The direction of rotation is automatically selected based on the target angle. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            angle (Number, deg) - Angle by which the motor should rotate.

            wait (bool) - Wait for the motor to reach the target before continuing with the rest of the program.

    '''
    async def run_angle(self, speed, angle, wait=True):
        if not self._encoder_enabled:
            return 0

        target_ticks = int(angle * self._cpr / 360)
        self.reset_angle()
        self.run(speed, target_ticks)

        if wait:
            while not self._driver.get_done(self.port):
                await asyncio.sleep_ms(50)
    
    '''
        Runs the motor at a constant speed towards a given target rotations.

        The direction of rotation is automatically selected based on the target rotations. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            rotation (Number) - Number rotation by which the motor should rotate.

            wait (bool) - Wait for the motor to reach the target before continuing with the rest of the program.

    '''
    async def run_rotation(self, speed, rotation, wait=True):
        if not self._encoder_enabled:
            return 0

        target_angle = rotation*360
        await self.run_angle(speed, target_angle, wait)

class DCMotor2PIN (DCMotor):
    def __init__(self, in1, in2, reversed=False):
        # motor pins
        self._in1 = PWM(Pin(in1), freq=500, duty=0)
        self._in2 = PWM(Pin(in2), freq=500, duty=0)
        super().__init__(reversed)

    def run(self, value):
        value = int(max(min(100, value),-100))

        value = value*self._reversed

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
    
    def stop(self):
        self.run(0)
    
    def brake(self):
        self._in1.duty(1023)
        self._in2.duty(1023)

class DCMotor3PIN (DCMotor):
    def __init__(self, in1, in2, pwm, stby=None, reversed=False):
        # motor pins
        self._in1 = Pin(in1, mode=Pin.OUT, pull=None)
        self._in2 = Pin(in2, mode=Pin.OUT, pull=None)
        self._pwm = PWM(pwm, freq=500, duty=0)
        
        if stby:
            self._stby = Pin(stby, mode=Pin.OUT, pull=None)
            self._stby.value(1)
        super().__init__(reversed)

    def run(self, value):
        value = int(max(min(100, value),-100))
        value = value*self._reversed

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
    
    def stop(self):
        self.run(0)

    def brake(self):
        self._in1.value(1)
        self._in2.value(1)
        self._pwm.duty(0)

'''
md = MotorDriverV2()
m1 = DCMotor(md, MDV2_M1)
m2 = DCMotor(md, MDV2_M2)
e1 = EncoderMotor(md, MDV2_E1, 250, 11, 34)
e2 = EncoderMotor(md, MDV2_E2, 250, 11, 34)

while True:
    try:
        e1.run(100)
        m1.run(100)
        sleep(1)
        e1.run(-100)
        m1.run(-100)
        sleep(1)
        e1.brake()
        m1.brake()
        sleep(1)
    except:
        e1.brake()
        m1.brake()
        break
'''