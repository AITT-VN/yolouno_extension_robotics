import asyncio
from time import ticks_ms, ticks_diff
from machine import PWM, Pin
from utility import *
from constants import *
from mdv1 import *
from mdv2 import *
from mdv3 import *


class DCMotor:
    def __init__(self, driver, port, reversed=False):
        self.driver = driver
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
    
    def reverse_encoder(self):        
        self.driver.reverse_encoder(self.port)

    def set_encoder(self, rpm, ppr, gears):
        if rpm <= 0 or ppr <= 0 or gears <= 0:
            raise Exception('Invalid encoder pulses config')

        self._encoder_enabled = True
        self._rpm = rpm
        self._ppr = ppr # pulses per revolution
        self._gears = gears # pulses per revolution
        self.ticks_per_rev = ppr * 4 * gears # encoder ticks count per revolution
        self._max_pps = rpm * ppr * 4 * gears / 60
        self.reset_angle()
    
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
        speed = max(min(100, speed), -100)
        self.driver.set_motors(self.port, speed*self._reversed)

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
            if ticks_diff(ticks_ms(), start) >= time:
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
            
            if stalled and ticks_diff(ticks_ms(), stalled_start) > self._stalled_time:
                break
            
            await asyncio.sleep_ms(200)
        
        if then == STOP:
            self.stop()
        elif then == BRAKE:
            self.brake()
        else:
            pass
    
    '''
        Actively brakes the motor.

        The motor stops due to being locked by H-bridge IC.
    '''
    def brake(self):
        self.driver.brake(self.port)

    '''
        Passively stop the motor.

        The motor stops due to friction, plus the voltage that is generated while the motor is still moving.
    '''
    def stop(self):
        self.driver.stop(self.port)

    ############## Measuring #################
    
    '''
        Gets the rotation angle of the motor.

        Returns: Motor angle.
    '''
    def angle(self):
        if not self._encoder_enabled:
            return 0
        ticks = self.driver.get_encoder(self.port)
        rotations = (ticks*360*self._reversed)/self.ticks_per_rev
        return round(rotations, 1)

    '''
        Sets the accumulated rotation angle of the motor to 0.
    '''
    def reset_angle(self):
        if not self._encoder_enabled:
            return
        self.driver.reset_encoder(self.port)

    '''
        Gets the current encoder ticks of the motor.

        Returns: Encoder ticks.
    '''
    def encoder_ticks(self):
        if not self._encoder_enabled:
            return 0
        return self.driver.get_encoder(self.port)
    
    '''
        Get the speed of the motor.

        The speed is measured as the change in the motor angle during last 100ms. 

        Returns:
            Motor speed (rpm).
    '''
    def speed(self):
        if not self._encoder_enabled:
            return 0

        return round(self.driver.get_speed(self.port)*60/self.ticks_per_rev, 1)

    ############## Closed-loop control (Motor Driver V3) #################
    # These run the PID inside Motor Driver V3 (firmware 0.2+) and need
    # set_encoder() for the RPM / degree conversion. On ORC Hub, Motor Driver
    # V1/V2 or plain pin motors they fall back to the open-loop functions
    # above (speed in RPM converted to % of the encoder's rated RPM) and never
    # raise, so a program keeps working whichever driver it runs on.

    def closed_loop(self):
        # True when this motor gets real closed-loop control
        return self._encoder_enabled and self._driver_closed_loop()

    def _driver_closed_loop(self):
        return self.driver is not None and getattr(self.driver, 'closed_loop', False)

    def _prepare_closed_loop(self):
        # untuned motor: use the rated speed from set_encoder() as feed-forward
        if not getattr(self, '_cl_ready', False):
            self._cl_ready = True
            if self._max_pps > 0:
                self.driver.set_max_speed(self.port, self._max_pps, only_if_untuned=True)

    def _rpm_to_cps(self, rpm):
        return int(rpm * self.ticks_per_rev / 60) * self._reversed

    def _rpm_to_percent(self, rpm):
        # open-loop fallback; without encoder data the value is taken as %
        if self._rpm > 0:
            return max(-100, min(100, rpm * 100 / self._rpm))
        return max(-100, min(100, rpm))

    def _deg_to_counts(self, degrees):
        return int(round(degrees * self.ticks_per_rev / 360)) * self._reversed

    async def _wait_done(self, counts, rpm):
        # longest time the move can take, plus margin for ramps and settling
        cps = abs(self._rpm_to_cps(rpm)) or 1
        timeout = int(min(60000, 3000 + 1500 * abs(counts) / cps))
        if not await self.driver.wait(self.port, timeout):
            self.brake()

    async def _fallback_angle(self, speed_rpm, degrees, then):
        # open-loop: turn by degrees (sign = direction), then STOP or BRAKE
        if degrees == 0:
            return
        speed = abs(self._rpm_to_percent(speed_rpm)) or 50
        if degrees < 0:
            speed = -speed
        await self.run_angle(speed, abs(degrees), then=STOP if then == STOP else BRAKE)

    '''
        Keeps the motor at a constant speed whatever the load or battery
        level (PID in Motor Driver V3).

        Parameters:
            rpm (Number) - Speed in revolutions per minute, sign = direction.
            accel (Number) - Acceleration in RPM per second, 0 = automatic.
    '''
    def run_speed(self, rpm, accel=0):
        if self.closed_loop():
            self._prepare_closed_loop()
            self.driver.run_speed(self.port, self._rpm_to_cps(rpm), abs(self._rpm_to_cps(accel)))
        else:
            self.run(self._rpm_to_percent(rpm))

    '''
        Turns the motor by an angle and stops exactly there (closed loop on
        Motor Driver V3; open loop elsewhere).

        Parameters:
            speed (Number, rpm) - Speed of the motor.
            angle (Number, deg) - Angle to turn by, sign = direction.
            then - BRAKE, HOLD (keep the position) or STOP (coast).
    '''
    async def run_angle_precise(self, speed, angle, then=BRAKE):
        if not self._encoder_enabled:
            return
        if not self.closed_loop():
            await self._fallback_angle(speed, angle, then)
            return
        self._prepare_closed_loop()
        counts = self._deg_to_counts(angle)
        self.driver.move(self.port, counts, abs(self._rpm_to_cps(speed)), 0, mdv3_end(then))
        await asyncio.sleep_ms(20)
        await self._wait_done(counts, speed)

    async def run_rotation_precise(self, speed, rotation, then=BRAKE):
        await self.run_angle_precise(speed, rotation * 360, then)

    '''
        Turns the motor to an absolute angle (0 = where reset_angle() was
        called), choosing the direction automatically.

        Parameters:
            speed (Number, rpm) - Speed of the motor.
            angle (Number, deg) - Target angle.
            then - BRAKE, HOLD or STOP.
    '''
    async def run_target(self, speed, angle, then=BRAKE):
        if not self._encoder_enabled:
            return
        if not self.closed_loop():
            await self._fallback_angle(speed, angle - self.angle(), then)
            return
        self._prepare_closed_loop()
        target = self._deg_to_counts(angle)
        distance = target - self.encoder_ticks()
        self.driver.move_to(self.port, target, abs(self._rpm_to_cps(speed)), 0, mdv3_end(then))
        await asyncio.sleep_ms(20)
        await self._wait_done(distance, speed)

    '''
        Holds the current position: the motor pushes back when turned by
        hand or by a load. Brakes on drivers without closed loop.
    '''
    def hold(self):
        if self.closed_loop():
            self._prepare_closed_loop()
            self.driver.hold(self.port)
        else:
            self.brake()

    '''
        Slows the motor down smoothly, then brakes, holds or coasts.
    '''
    async def stop_smooth(self, then=BRAKE):
        if self.closed_loop() and self.driver.control_mode(self.port) != 0:
            self.driver.stop_smooth(self.port, 0, mdv3_end(then))
            await asyncio.sleep_ms(20)
            await self.driver.wait(self.port, 5000)
        elif then == STOP:
            self.stop()
        elif then == HOLD:
            self.hold()
        else:
            self.brake()

    '''
        Measures the motor (max speed, starting power, response time) so the
        closed loop fits it. The motor runs both ways at full power for about
        5 s: lift the robot. Returns True when done, False on other drivers.
    '''
    async def tune(self):
        if not self._driver_closed_loop():
            return False
        self._cl_ready = True   # measured values replace the set_encoder() estimate
        return await self.driver.tune(self.port)

    def set_pid(self, kp, ki, kd):
        if self._driver_closed_loop():
            self.driver.set_pid(self.port, kp, ki, kd)

    def set_current(self, ma):
        # current limit of this port in mA (Motor Driver V3)
        f = getattr(self.driver, 'set_current', None) if self.driver else None
        if f:
            f(self.port, ma)

    def done(self):
        # True when no closed-loop move is running
        if self._driver_closed_loop():
            return not self.driver.busy(self.port)
        return True

    def is_stalled(self):
        # True if Motor Driver V3 stopped this motor because it was blocked
        if self._driver_closed_loop():
            return self.driver.stalled(self.port)
        return False

    def save_settings(self):
        # store tune / PID / current values in Motor Driver V3's flash
        if self._driver_closed_loop():
            return self.driver.save_settings()
        return False



class DCMotor2PIN (DCMotor):
    def __init__(self, in1, in2, reversed=False):
        # motor pins
        self._in1 = PWM(Pin(in1), freq=500, duty=0)
        self._in2 = PWM(Pin(in2), freq=500, duty=0)
        super().__init__(None, 0, reversed) # no driver board, no encoder

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
        self._pwm = PWM(Pin(pwm), freq=500, duty=0)
        
        if stby:
            self._stby = Pin(stby, mode=Pin.OUT, pull=None)
            self._stby.value(1)
        super().__init__(None, 0, reversed) # no driver board, no encoder

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
            self._in1.value(0)
            self._in2.value(0)
        
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