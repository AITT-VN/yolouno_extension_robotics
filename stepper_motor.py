import asyncio
from time import ticks_ms, ticks_diff
from constants import *

'''
    Stepper motor on a motor driver board.

    Motor Driver V3: full control in the driver (acceleration ramps, micro-
    stepping, position counter, smooth stop, holding current), STEPPER1 on
    connector STEP1 (M1 + M2), STEPPER2 on STEP2 (M3 + M4).
    Motor Driver V1: the MD4C stepper commands (constant speed, steps, release);
    the position is counted by the library.
    Other drivers (ORC Hub, Motor Driver V2) have no stepper output: the
    functions print a note once and do nothing, so a program never stops on
    an error there.

    Speeds are in RPM, angles in degrees; steps are steps of the selected mode
    (STEP_HALF: 2 x steps_per_rev per turn).
'''

_MICRO = (1, 1, 2, 4, 8, 16)   # steps per full step for STEP_WAVE .. STEP_1_16

# Motor Driver V3 stepper commands (mdv3.MDV3_STEP_*)
_RUN = 1
_MOVE = 2
_MOVE_TO = 3
_STOP = 4
_HALT = 5
_RELEASE = 6
_SET_POS = 7

_V1_MAX_RPM = 255


class StepperMotor:
    def __init__(self, driver, port, steps_per_rev=200, reversed=False):
        if port not in (STEPPER1, STEPPER2):
            raise ValueError('Invalid stepper port, use STEPPER1 or STEPPER2')
        if steps_per_rev <= 0:
            raise ValueError('steps_per_rev must be > 0')
        self.driver = driver
        self.port = port
        self.steps_per_rev = int(steps_per_rev)
        self._dir = -1 if reversed else 1
        self._mode = STEP_FULL
        self._accel = 0          # RPM per second, 0 = no ramp
        self._noted = False

        if driver is not None and getattr(driver, 'supports_stepper', False):
            self._kind = 3
            self._mode = driver.stepper_mode(port)
            if self._mode >= len(_MICRO):
                self._mode = STEP_FULL
        elif driver is not None and getattr(driver, 'legacy_stepper', False):
            self._kind = 1
            self._pos = 0        # counted here, the MD4C protocol has no read-back
            self._busy_until = ticks_ms()
            self._last_rpm = None   # speed last sent to the board
        else:
            self._kind = 0
        self._req = self._mode   # mode asked for; _mode is the one the driver uses
        self._timeout = 5000     # ms, longest wait for the last move (see wait())

    ######################## Configuration #####################

    '''
        Step mode and acceleration.

        Parameters:
            mode - STEP_WAVE, STEP_FULL, STEP_HALF, STEP_1_4, STEP_1_8, STEP_1_16
                   (micro steps need Motor Driver V3; Motor Driver V1 uses half step)
            accel (Number, RPM/s) - speed gained per second, 0 = start at full speed
    '''
    def config(self, mode=None, accel=None):
        if mode is not None:
            if mode not in (STEP_WAVE, STEP_FULL, STEP_HALF, STEP_1_4, STEP_1_8, STEP_1_16):
                raise ValueError('Invalid step mode')
            if self._kind == 1 and mode > STEP_HALF:
                print('Motor Driver V1: no micro steps, using half step')
                mode = STEP_HALF
            if self._kind == 1:
                self._pos = self._pos * _MICRO[mode] // _MICRO[self._mode]
                self._mode = mode
            self._req = mode
        if accel is not None:
            self._accel = max(0, accel)
        if self._kind == 3:
            self._apply_now()

    '''
        Phase current while running and holding (Motor Driver V3).

        Parameters:
            run_ma (Number, mA) - current limit of the two ports, 200 to 1500
            hold_percent (Number, %) - current kept when stopped, 0 = free shaft
    '''
    def set_current(self, run_ma=None, hold_percent=None):
        if self._kind != 3:
            return
        if run_ma is not None:
            self.driver.set_current(M1 | M2 if self.port == STEPPER1 else M3 | M4, run_ma)
        if hold_percent is not None:
            self.driver.stepper_config(self.port, hold=hold_percent)
            self._apply_now()

    ######################## Moving #####################

    '''
        Turns continuously at the given speed (negative = backwards).
        0 slows down to a stop and holds.
    '''
    def run(self, speed):
        if not self._check():
            return
        if self._kind == 3:
            self._cmd(_RUN, 0, self._sps(speed) * self._dir)
        else:
            rpm = max(-_V1_MAX_RPM, min(_V1_MAX_RPM, speed))
            if rpm:
                self._last_rpm = abs(rpm)
            self.driver.stepper_speed(self.port, self.steps_per_rev, rpm * self._dir, self._style())
            self._busy_until = ticks_ms()   # running: the position is not counted

    '''
        Turns by a number of steps (negative = backwards), then holds.

        Parameters:
            speed (Number, RPM) - top speed
            steps (Number) - steps of the current mode
            wait (bool) - wait until the move is done
    '''
    async def run_steps(self, speed, steps, wait=True):
        if not self._check():
            return
        steps = int(round(steps))
        if self._kind == 3:
            self._cmd(_MOVE, steps * self._dir, abs(self._sps(speed)))
        else:
            self._v1_move(speed, steps)
        self._timeout = self._estimate(steps, speed)
        if wait:
            await self.wait()

    async def run_angle(self, speed, angle, wait=True):
        if self._check():   # brings the mode up to date before converting
            await self.run_steps(speed, self._deg_to_steps(angle), wait)

    async def run_rotation(self, speed, rotation, wait=True):
        if self._check():
            await self.run_steps(speed, self._deg_to_steps(rotation * 360), wait)

    '''
        Turns to an absolute angle (0 = where reset_position() was called).
    '''
    async def run_target(self, speed, angle, wait=True):
        if not self._check():
            return
        target = self._deg_to_steps(angle)
        distance = abs(target - self.position())
        if self._kind == 3:
            self._cmd(_MOVE_TO, target * self._dir, abs(self._sps(speed)))
        else:
            self._v1_move(speed, target - self._pos)
        self._timeout = self._estimate(distance, speed)
        if wait:
            await self.wait()

    '''
        Stops the motor.

        Parameters:
            smooth (bool) - slow down with the acceleration (Motor Driver V3)
                            instead of stopping at once
        The motor keeps its holding current afterwards; release() frees the shaft.
    '''
    async def stop(self, smooth=True):
        if self._kind == 3:
            if smooth:
                self._cmd(_STOP)
                await self.wait(None, 0, 5000)
            else:
                self.driver.stepper_command(self.port, _HALT)
        elif self._kind == 1:
            self.release()

    def release(self):
        # no current in the coils: the shaft turns freely
        if self._kind == 3:
            self.driver.stepper_command(self.port, _RELEASE)
        elif self._kind == 1:
            self.driver.stepper_speed(self.port, self.steps_per_rev, 0, self._style())
            self._busy_until = ticks_ms()

    ######################## Measuring #####################

    def position(self):
        # steps of the current mode
        if self._kind == 3:
            return self.driver.stepper_position(self.port) * self._dir
        if self._kind == 1:
            return self._pos
        return 0

    def angle(self):
        return round(self.position() * 360 / (self.steps_per_rev * _MICRO[self._mode]), 1)

    def reset_position(self):
        if self._kind == 3:
            self.driver.stepper_command(self.port, _SET_POS, 0)
        elif self._kind == 1:
            self._pos = 0

    def done(self):
        # True when no move is running
        if self._kind == 3:
            return not self.driver.stepper_busy(self.port)
        if self._kind == 1:
            return ticks_diff(ticks_ms(), self._busy_until) >= 0
        return True

    async def wait(self, steps=None, speed=0, timeout=None):
        # without arguments: wait for the last move, with the time it was given
        if timeout is None:
            timeout = self._timeout if steps is None else self._estimate(steps, speed)
        start = ticks_ms()
        while not self.done():
            if ticks_diff(ticks_ms(), start) > timeout:
                if self._kind == 3:
                    self.driver.stepper_command(self.port, _HALT)
                return False
            await asyncio.sleep_ms(10)
        return True

    ######################## internal #####################

    def _check(self):
        if self._kind == 3 and self._req != self._mode:
            self._apply_now()   # a mode set while moving: apply it now if stopped
        if self._kind:
            return True
        if not self._noted:
            self._noted = True
            print('Stepper motors need Motor Driver V3 or V1, this driver has no stepper output')
        return False

    def _estimate(self, steps, speed):
        # ms: time of the move plus the ramps, with margin
        sps = abs(self._sps(speed)) or 1
        a = self._accel_sps2()
        t = abs(steps) / sps + (2 * sps / a if a else 0)
        return int(min(300000, 2000 + 1500 * t))

    def _sps(self, rpm):
        return rpm * self.steps_per_rev * _MICRO[self._mode] / 60

    def _accel_sps2(self, mode=None):
        return self._accel * self.steps_per_rev * _MICRO[self._mode if mode is None else mode] / 60

    def _deg_to_steps(self, degrees):
        return int(round(degrees * self.steps_per_rev * _MICRO[self._mode] / 360))

    def _cmd(self, cmd, target=0, speed=0):
        # Mode and acceleration go with every command (the acceleration in
        # steps/s^2 depends on the mode). The driver applies the mode only
        # while stopped and writes back the one in use; the position is in
        # steps of that mode.
        mode = self._mode
        if self._req != self._mode and not self.driver.stepper_busy(self.port):
            mode = self._req   # will be applied by this command
        self.driver.stepper_config(self.port, mode=self._req, accel=self._accel_sps2(mode))
        self.driver.stepper_command(self.port, cmd, target, speed)
        actual = self.driver.stepper_mode(self.port)
        if actual < len(_MICRO):
            self._mode = actual

    def _apply_now(self):
        # a STOP on a stopped stepper applies mode / hold current without moving
        if self.driver.stepper_busy(self.port):
            self.driver.stepper_config(self.port, accel=self._accel_sps2())
        else:
            self._cmd(_STOP)

    def _style(self):
        # MD4C styles: 0 single (wave), 1 double (full), 2 interleave (half)
        return min(self._mode, STEP_HALF)

    def _v1_move(self, speed, steps):
        rpm = max(1, min(_V1_MAX_RPM, abs(speed) or self._last_rpm or 30))
        if rpm != self._last_rpm:
            # the MD4C step command moves at the last speed: set it first
            self.driver.stepper_speed(self.port, self.steps_per_rev, rpm, self._style())
            self._last_rpm = rpm
        self.driver.stepper_step(self.port, steps * self._dir, self._style())
        self._pos += steps
        ms = abs(steps) * 60000 // (rpm * self.steps_per_rev * _MICRO[self._mode])
        self._busy_until = ticks_ms() + int(ms)
