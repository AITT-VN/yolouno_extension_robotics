import asyncio
from machine import PWM, Pin
from i2c_motors_driver import MotorDriver
import robotics_motor_driver_v2 as mdv2
from utility import *

class DCMotor:
    def __init__(self, reversed=False):
        if reversed:
            self._reversed = -1
        else:
            self._reversed = 1

    def speed(self, value):    
        pass

    def brake(self):    
        pass

    def reverse(self):
        if self._reversed == 1:
            self._reversed = -1
        else:
            self._reversed = 1

class DCMotorI2CV1 (DCMotor):
    def __init__(self, driver: MotorDriver, port, reversed=False):
        self._driver = driver
        self._port = port
        super().__init__(reversed)

    def speed(self, value):
        value = int(max(min(100, value),-100))
        self._driver.setSpeed(self._port, value*self._reversed)
    
    def brake(self):
        self.speed(0)

class DCMotorI2CV2 (DCMotor):
    def __init__(self, driver: mdv2.MotorDriver, port, reversed=False):
        self._driver = driver
        self._port = port
        self._ppr = 4680
        super().__init__(reversed)
    
    '''
        Gets the rotation angle of the motor.

        Returns: Motor angle.
    '''
    def angle(self):
        if self._port not in (mdv2.MOTOR_E1, mdv2.MOTOR_E2):
            return 0

        ticks = self._driver.get_encoder(self._port)
        rotations = ticks/self._ppr
        return rotations * 360 * self._reversed

    '''
        Sets the accumulated rotation angle of the motor to 0.
    '''
    def reset_angle(self):
        if self._port not in (mdv2.MOTOR_E1, mdv2.MOTOR_E2):
            return
        self._driver.reset_encoder(self._port)

    '''
        Sets or gets the speed of the motor.

        The speed is measured as the change in the motor angle during last 100ms. 
        
        Parameters:
            speed (Number, %) - Speed of the motor if set.

        Returns:
            Motor speed (rpm).
    '''
    def speed(self, value=None):
        if value == None:
            if self._port not in (mdv2.MOTOR_E1, mdv2.MOTOR_E2):
                return 0
            return self._driver.get_speed(self._port)*self._reversed/self._ppr
        else:
            value = int(max(min(100, value),-100))
            self._driver.set_motor(self._port, value*self._reversed)


    ############## Stopping #################

    '''
        Passively brakes the motor.

        The motor stops due to friction, plus the voltage that is generated while the motor is still moving.
    '''
    def brake(self):
        self._driver.brake(self._port)
        if self._port == mdv2.MOTOR_E1 or self._port == mdv2.MOTOR_E2:
            self._driver.reset_encoder(self._port)
    
    
    '''
        Stops the motor using given method.
    '''
    async def stop_then(self, then):
        if then == mdv2.STOP_COAST:
            self.speed(0)
        elif then == mdv2.STOP_BRAKE:
            self.brake()
            #await asyncio.sleep_ms(200)
            #self.speed(0)
        elif then == mdv2.STOP_HOLD:
            # TODO: Support stop hold position
            pass
        else: # None: Do nothing when approaching the target position
            return
    
    '''
        Runs the motor at a constant speed towards a given target angle.

        The direction of rotation is automatically selected based on the target angle. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            angle (Number, deg) - Angle by which the motor should rotate.

            then (Stop) - What to do after coming to a standstill.

    '''
    async def run_angle(self, speed, angle, then=mdv2.STOP_COAST):
        if self._port not in (mdv2.MOTOR_E1, mdv2.MOTOR_E2):
            return

        self.reset_angle()

        self.speed(speed)

        while abs(self.angle()) < abs(angle):
            await asyncio.sleep_ms(5)

        await self.stop_then(then)
    
    '''
        Runs the motor at a constant speed towards a given target rotations.

        The direction of rotation is automatically selected based on the target rotations. It does not matter if speed is positive or negative.

        Parameters:
            speed (Number, %) - Speed of the motor.

            rotation (Number, int) - Number rotation by which the motor should rotate.

            then (Stop) - What to do after coming to a standstill.

    '''
    async def run_rotation(self, speed, rotation, then=mdv2.STOP_COAST):
        if self._port not in (mdv2.MOTOR_E1, mdv2.MOTOR_E2):
            return

        self.reset_angle()

        self.speed(speed)

        while (abs(self.angle())/360) < abs(rotation):
            print(abs(self.angle())/360)
            await asyncio.sleep_ms(5)

        await self.stop_then(then)
        print(abs(self.angle())/360)


class DCMotor2PIN (DCMotor):
    def __init__(self, in1, in2, reversed=False):
        # motor pins
        self._in1 = PWM(Pin(in1), freq=500, duty=0)
        self._in2 = PWM(Pin(in2), freq=500, duty=0)
        super().__init__(reversed)

    def speed(self, value):
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

    def speed(self, value):
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
    
    def brake(self):
        self._in1.value(1)
        self._in2.value(1)
        self._pwm.duty(0)



