import asyncio
from time import ticks_ms
from machine import PWM, Pin
from utility import *
from constants import *
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
        self.ticks_per_rev = 0 # encoder ticks count per revolution
        self._max_pps = 0 # max count pulses per second

        # stalled config
        self._stalled_speed = 0.05 # < 5% of max speed considered stalled
        self._stalled_time = 1000 # 1 seconds

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
        
        self._driver.reverse_encoder(self.port)

    def set_encoder(self, rpm, ppr, gears):
        if rpm <= 0 or ppr <= 0 or gears <= 0:
            raise Exception('Invalid encoder pulses config')

        self._encoder_enabled = True
        self._rpm = rpm
        self._ppr = ppr # pulses per revolution
        self._gears = gears # pulses per revolution
        self.ticks_per_rev = ppr * 4 * gears # encoder ticks count per revolution
        self._max_pps = rpm * ppr * 4 * gears / 60
    
    def stall_tolerances(self, speed, time):
        self._stalled_speed = speed
        self._stalled_time = time
    
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
        rotations = (ticks*360*self._reversed)/self.ticks_per_rev
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
        return self._driver.get_encoder(self.port)
    
    '''
        Get the speed of the motor.

        The speed is measured as the change in the motor angle during last 100ms. 

        Returns:
            Motor speed (rpm).
    '''
    def speed(self):
        if not self._encoder_enabled:
            return 0

        return round(self._driver.get_speed(self.port)*60/self.ticks_per_rev, 1)
    
    '''
        Runs the motor at a constant speed towards a given target angle.

        The direction of rotation is automatically selected based on the target angle. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            angle (Number, deg) - Angle by which the motor should rotate.

            wait (bool) - Wait for the motor to reach the target before continuing with the rest of the program.

    '''
    async def run_time(self, speed, time, then=STOP):
        if time <= 0:
            return

        start = ticks_ms()
        self.run(speed)

        while True:
            if abs(ticks_ms() - start) >= time:
                break
            await asyncio.sleep_ms(10)
        
        if then == STOP:
            self.stop()
        elif then == BRAKE:
            self.brake()
        else:
            pass

    '''
        Runs the motor at a constant speed towards a given target angle.

        The direction of rotation is automatically selected based on the target angle. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            angle (Number, deg) - Angle by which the motor should rotate.

            wait (bool) - Wait for the motor to reach the target before continuing with the rest of the program.

    '''
    async def run_angle(self, speed, angle, then=BRAKE):
        if not self._encoder_enabled:
            return 0

        target_ticks = int(angle * self.ticks_per_rev / 360)
        start_ticks = self.encoder_ticks()
        self.run(speed)

        while True:
            if abs(self.encoder_ticks() - start_ticks) >= target_ticks:
                break
            await asyncio.sleep_ms(10)
        
        if then == STOP:
            self.stop()
        elif then == BRAKE:
            self.brake()
        else:
            pass
    
    '''
        Runs the motor at a constant speed towards a given target rotations.

        The direction of rotation is automatically selected based on the target rotations. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            rotation (Number) - Number rotation by which the motor should rotate.

            wait (bool) - Wait for the motor to reach the target before continuing with the rest of the program.

    '''
    async def run_rotation(self, speed, rotation, then=BRAKE):
        if not self._encoder_enabled:
            return 0

        target_angle = rotation*360
        await self.run_angle(speed, target_angle)
    
    async def run_until_stalled(self, speed, then=STOP):
        if not self._encoder_enabled:
            return 0
        stalled_start = 0
        stalled = False
        threshold = int(self._max_pps * self._stalled_speed * 60 / self.ticks_per_rev)
        self.run(speed)
        while True:
            if abs(self.speed()) <= threshold:
                if not stalled:
                    stalled = True
                    stalled_start = ticks_ms()
            else:
                stalled = False
            
            if stalled and (ticks_ms() - stalled_start) > self._stalled_time:
                break
            
            await asyncio.sleep_ms(200)
        
        if then == STOP:
            self.stop()
        elif then == BRAKE:
            self.brake()
        else:
            pass



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
m2 = DCMotor(md, MDV2_M4)
m1.set_encoder(250, 11, 34)
m2.set_encoder(250, 11, 34)
md.reverse_encoder(MDV2_M1)

async def main():
    try:
        #m1.run(100)
        #await asleep_ms(1000)
        #m1.run(-100)
        #await asleep_ms(1000)
        #m1.brake()
        #await asleep_ms(1000)
        await m1.run_until_stalled(70)
    except KeyboardInterrupt as e:
        m1.brake()

run_loop(main())
'''