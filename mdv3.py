import asyncio
from micropython import const
from machine import SoftI2C, Pin
from time import ticks_ms, ticks_diff, sleep_ms
from struct import pack, unpack
from setting import *
from utility import *
from constants import *

'''
    Motor Driver V3: I2C 4-Ch Motor Stepper Driver (RP2040 + 4 x DRV8870).
    Firmware: https://github.com/AITT-VN/firmware_motor_driver_4_channel

    Same interface as MotorDriverV2 (set_motors, stop, brake, get_encoder,
    reset_encoder, reverse_encoder, get_speed, battery), so DCMotor and
    DriveBase work with it unchanged. All four ports M1..M4 have an encoder;
    E1 and E2 are accepted as M1 and M2 so programs written for Motor Driver V2
    keep their port numbers.

    Firmware 0.2 adds closed-loop control done by the driver itself: speed
    hold (PID), moves to a position, position hold, smooth stop, auto-tune.
    Units here are raw encoder counts (x4) and counts/s; DCMotor converts
    them to degrees and RPM.
'''

MDV3_DEFAULT_I2C_ADDRESS = const(0x30)   # + DIP switch: 0x30..0x33
MDV3_DEVICE_ID = const(0x4D)

MDV3_STEPPER1 = const(0)    # connector STEP1 = M1 + M2
MDV3_STEPPER2 = const(1)    # connector STEP2 = M3 + M4

# registers
_REG_DEVICE_ID = const(0x10)
_REG_FW_MINOR = const(0x11)
_REG_STATUS = const(0x14)
_REG_VM_MV = const(0x16)
_REG_ENCODER = const(0x18)
_REG_SPEED = const(0x28)
_REG_MOTOR_SPEED = const(0x40)
_REG_BRAKE = const(0x48)
_REG_COAST = const(0x49)
_REG_ENC_RESET = const(0x4A)
_REG_ENC_REVERSE = const(0x4B)
_REG_CURRENT_MA = const(0x50)
_REG_STEPPER1 = const(0x60)
# closed loop (firmware 0.2)
_REG_CTL = const(0x80)          # + 16 * channel
_REG_PID = const(0xC0)          # + 8 * channel
_REG_STALL_MS = const(0xE0)
_REG_CTL_BUSY = const(0xE2)
_REG_CTL_STALL = const(0xE3)
_REG_CTL_MODE = const(0xE4)
_REG_TUNE_FAIL = const(0xE5)
_REG_SETTINGS = const(0xE6)
_REG_DEADBAND = const(0xE8)
_REG_TAU_MS = const(0xEC)

_CB_TOL = const(0x0D)
_CB_CMD = const(0x0F)

_CTL_SPEED = const(1)
_CTL_MOVE = const(2)
_CTL_MOVE_TO = const(3)
_CTL_STOP = const(4)
_CTL_HOLD = const(5)
_CTL_TUNE = const(7)

# what a closed-loop move / smooth stop ends with (firmware END codes)
MDV3_END_BRAKE = const(0)
MDV3_END_HOLD = const(1)
MDV3_END_COAST = const(2)

# stepper block commands
_STEP_RUN = const(1)
_STEP_MOVE = const(2)
_STEP_RELEASE = const(6)

_STATUS_VM_OK = const(0x01)
_PID_DEFAULT_VMAX = const(10000)   # firmware value before any tune

# settings status (save_settings)
MDV3_SETTINGS_SAVED = const(2)
MDV3_SETTINGS_BUSY = const(3)


def mdv3_end(then):
    # STOP / BRAKE / HOLD of the robotics library -> firmware END code
    if then == HOLD:
        return MDV3_END_HOLD
    if then == STOP:
        return MDV3_END_COAST
    return MDV3_END_BRAKE


class MotorDriverV3():
    def __init__(self, address=MDV3_DEFAULT_I2C_ADDRESS):
        self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self._addr = address

        try:
            device_id = self._read(_REG_DEVICE_ID, 1)[0]
        except OSError:
            device_id = 0
        if device_id != MDV3_DEVICE_ID:
            raise RuntimeError("Motor driver V3 not found at " + hex(address) + " (check the Grove cable and DIP switch)")

        minor, major = self._read(_REG_FW_MINOR, 2)
        self._fw = (major, minor)
        self.closed_loop = self._fw >= (0, 2)   # PID, moves, hold, tune

        self.stop(ALL)
        if self.closed_loop:
            self._write(_REG_CTL_STALL, bytes([0x0F]))

    #################### BASIC  FUNCTIONS ####################

    def fw_version(self):
        return "{}.{}".format(self._fw[0], self._fw[1])

    def battery(self):
        # motor supply (DC jack / battery) in volts
        return round(self._read_u16(_REG_VM_MV) / 1000, 1)

    def power_ok(self):
        # True when the motor supply is on (>= 6.5 V)
        return bool(self._read(_REG_STATUS, 1)[0] & _STATUS_VM_OK)

    def has_encoder(self, motors):
        # every port of this driver has an encoder input
        return _channels(motors) != 0

    #################### MOTOR CONTROL ####################

    def set_motors(self, motors, speed):
        # speed -100..100 %; also ends closed-loop control of these motors
        speed = max(min(100, speed), -100)
        duty = int(round(speed * 10))
        chs = _channels(motors)
        if chs == 0x0F:
            self._write(_REG_MOTOR_SPEED, pack('<hhhh', duty, duty, duty, duty))
        else:
            for ch in range(4):
                if chs & (1 << ch):
                    self._write(_REG_MOTOR_SPEED + 2 * ch, pack('<h', duty))

    def stop(self, motors=ALL):
        # coast
        chs = _channels(motors)
        if chs:
            self._write(_REG_COAST, bytes([chs]))

    def brake(self, motors=ALL):
        chs = _channels(motors)
        if chs:
            self._write(_REG_BRAKE, bytes([chs]))

    def set_current(self, motors, ma):
        # current limit per channel, 200..1500 mA (the firmware clamps it)
        ma = int(max(0, min(1500, ma)))
        chs = _channels(motors)
        for ch in range(4):
            if chs & (1 << ch):
                self._write(_REG_CURRENT_MA + 2 * ch, pack('<H', ma))

    def get_encoder(self, motors=ALL):
        values = list(unpack('<iiii', self._read(_REG_ENCODER, 16)))
        if motors == ALL:
            return values
        ch = _first_channel(motors)
        return values[ch] if ch >= 0 else 0

    def reset_encoder(self, motors=ALL):
        chs = _channels(motors)
        if chs:
            self._write(_REG_ENC_RESET, bytes([chs]))

    def reverse_encoder(self, motors):
        # reverse the count direction of these encoders. Calling it again
        # (e.g. when the program restarts) keeps them reversed: the board
        # keeps its state while the Yolo UNO restarts.
        mask = self._read(_REG_ENC_REVERSE, 1)[0] | _channels(motors)
        self._write(_REG_ENC_REVERSE, bytes([mask & 0x0F]))

    def get_speed(self, motors=ALL):
        # counts/s, measured over 50 ms
        values = list(unpack('<iiii', self._read(_REG_SPEED, 16)))
        if motors == ALL:
            return values
        ch = _first_channel(motors)
        return values[ch] if ch >= 0 else 0

    #################### CLOSED LOOP (firmware 0.2) ####################
    # Every function returns quietly (False / None) on firmware 0.1.

    def run_speed(self, motors, cps, accel=0):
        # hold cps counts/s with the PID; accel counts/s^2 (0 = automatic)
        return self._ctl(motors, _CTL_SPEED, 0, cps, accel)

    def move(self, motors, counts, cps=0, accel=0, end=MDV3_END_BRAKE):
        # move by counts from the current set-point (repeated moves do not drift)
        return self._ctl(motors, _CTL_MOVE, counts, cps, accel, end)

    def move_to(self, motors, position, cps=0, accel=0, end=MDV3_END_BRAKE):
        return self._ctl(motors, _CTL_MOVE_TO, position, cps, accel, end)

    def stop_smooth(self, motors, accel=0, end=MDV3_END_BRAKE):
        return self._ctl(motors, _CTL_STOP, 0, 0, accel, end)

    def hold(self, motors):
        return self._ctl(motors, _CTL_HOLD, 0, 0, 0)

    def busy(self, motors=ALL):
        if not self.closed_loop:
            return False
        return bool(self._read(_REG_CTL_BUSY, 1)[0] & _channels(motors))

    async def wait(self, motors=ALL, timeout=60000):
        start = ticks_ms()
        while self.busy(motors):
            if ticks_diff(ticks_ms(), start) > timeout:
                return False
            await asyncio.sleep_ms(10)
        return True

    def control_mode(self, motors):
        # 0 open loop, 1 speed, 2 moving, 3 holding
        if not self.closed_loop:
            return 0
        ch = _first_channel(motors)
        return (self._read(_REG_CTL_MODE, 1)[0] >> (2 * ch)) & 3 if ch >= 0 else 0

    def stalled(self, motors=ALL):
        # True if one of these motors was stopped by the stall / wrong-direction protection
        if not self.closed_loop:
            return False
        return bool(self._read(_REG_CTL_STALL, 1)[0] & _channels(motors))

    def clear_stall(self, motors=ALL):
        if self.closed_loop:
            self._write(_REG_CTL_STALL, bytes([_channels(motors)]))

    def set_stall_time(self, ms):
        # stop a closed-loop motor blocked this long (0 = off, default 1000 ms)
        if self.closed_loop:
            self._write(_REG_STALL_MS, pack('<H', int(max(0, min(65535, ms)))))

    def set_tolerance(self, motors, counts):
        # a move is done within +-counts (default 10)
        if not self.closed_loop:
            return
        chs = _channels(motors)
        for ch in range(4):
            if chs & (1 << ch):
                self._write(_REG_CTL + 16 * ch + _CB_TOL, bytes([int(max(1, min(255, counts)))]))

    def set_pid(self, motors, kp=None, ki=None, kd=None):
        # kp [1/s], ki [1/s^2], kd; defaults 20, 5, 1.5
        if not self.closed_loop:
            return
        chs = _channels(motors)
        for ch in range(4):
            if chs & (1 << ch):
                for off, g in ((0, kp), (2, ki), (4, kd)):
                    if g is not None:
                        self._write(_REG_PID + 8 * ch + off, pack('<H', int(max(0, min(65535, round(g * 100))))))

    def pid(self, motors):
        # control parameters of the first motor in motors, or None
        ch = _first_channel(motors)
        if not self.closed_loop or ch < 0:
            return None
        kp, ki, kd, vmax = unpack('<HHHH', self._read(_REG_PID + 8 * ch, 8))
        return {'kp': kp / 100, 'ki': ki / 100, 'kd': kd / 100, 'vmax': vmax,
                'deadband': self._read(_REG_DEADBAND + ch, 1)[0],
                'tau_ms': self._read(_REG_TAU_MS + ch, 1)[0]}

    def set_max_speed(self, motors, cps, only_if_untuned=False):
        # counts/s at full power (feed-forward). tune() measures it; with
        # only_if_untuned the value is written only if the motor was never tuned
        if not self.closed_loop:
            return
        cps = int(max(1, min(65535, cps)))
        chs = _channels(motors)
        for ch in range(4):
            if not chs & (1 << ch):
                continue
            if only_if_untuned:
                p = self.pid(1 << ch)
                if p['vmax'] != _PID_DEFAULT_VMAX or p['deadband'] or p['tau_ms']:
                    continue
            self._write(_REG_PID + 8 * ch + 6, pack('<H', cps))

    async def tune(self, motors):
        # Measure max speed, deadband and time constant of each motor. The
        # motor runs both ways at full power for about 5 s: lift the robot.
        # An encoder that counts against the motor is reversed first.
        # Returns True if every motor was tuned.
        if not self.closed_loop:
            return False
        if not self.power_ok():
            print('Motor driver V3: motor power is off, cannot tune')
            return False
        ok = True
        chs = _channels(motors)
        for ch in range(4):
            if not chs & (1 << ch):
                continue
            m = 1 << ch
            # direction check: +60 % must count up
            start = self.get_encoder(m)
            self.set_motors(m, 60)
            await asyncio.sleep_ms(300)
            delta = self.get_encoder(m) - start
            self.stop(m)
            await asyncio.sleep_ms(500)
            if abs(delta) < 30:
                print('Motor driver V3: M%d has no encoder counts' % (ch + 1))
                ok = False
                continue
            if delta < 0:
                rev = self._read(_REG_ENC_REVERSE, 1)[0] ^ m
                self._write(_REG_ENC_REVERSE, bytes([rev & 0x0F]))
            self._write(_REG_CTL + 16 * ch + _CB_CMD, bytes([_CTL_TUNE]))
            await asyncio.sleep_ms(50)
            await self.wait(m, 20000)
            if self._read(_REG_TUNE_FAIL, 1)[0] & m:
                print('Motor driver V3: M%d tune failed' % (ch + 1))
                ok = False
        return ok

    def save_settings(self):
        # Store reverse masks, current limits, PID and tune values in the
        # driver's flash (loaded at every power-up). Motors must be stopped.
        if not self.closed_loop:
            return False
        self._write(_REG_SETTINGS, bytes([0xA5]))
        sleep_ms(100)   # the flash write takes a few tens of ms
        return self._read(_REG_SETTINGS, 1)[0] == MDV3_SETTINGS_SAVED

    def _ctl(self, motors, cmd, target, speed, accel, end=None):
        if not self.closed_loop:
            return False
        data = pack('<iiI', int(target), int(speed), int(max(0, accel)))
        if end is not None:
            data += bytes([end])
        chs = _channels(motors)
        for ch in range(4):
            if chs & (1 << ch):
                base = _REG_CTL + 16 * ch
                self._write(base, data)
                self._write(base + _CB_CMD, bytes([cmd]))
        return True

    #################### STEPPERS ####################
    # STEPPER1 uses M1 + M2, STEPPER2 uses M3 + M4. A DC command on one of
    # these ports releases the stepper.

    def stepper_speed(self, stepper, steps_per_rev, speed):
        # turn continuously at speed RPM (sign = direction), 0 = release
        if stepper not in (MDV3_STEPPER1, MDV3_STEPPER2):
            raise RuntimeError('Invalid stepper motor port')
        base = _REG_STEPPER1 + 16 * stepper
        if speed == 0:
            self._write(base + 0x0F, bytes([_STEP_RELEASE]))
            return
        sps = int(max(-10000, min(10000, speed * steps_per_rev / 60)))
        self._write(base + 4, pack('<hH', sps, 0))
        self._write(base + 0x0F, bytes([_STEP_RUN]))

    def stepper_step(self, stepper, steps, speed=200):
        # move steps (sign = direction) at speed steps/s
        if stepper not in (MDV3_STEPPER1, MDV3_STEPPER2):
            raise RuntimeError('Invalid stepper motor port')
        base = _REG_STEPPER1 + 16 * stepper
        self._write(base, pack('<ihH', int(steps), int(min(10000, abs(speed))), 0))
        self._write(base + 0x0F, bytes([_STEP_MOVE]))

    #################### I2C ####################

    def _write(self, register, data):
        self._i2c.writeto_mem(self._addr, register, data)

    def _read(self, register, n):
        # register pointer, then the read: two transactions, max 16 bytes
        self._i2c.writeto(self._addr, bytes([register]))
        return self._i2c.readfrom(self._addr, n)

    def _read_u16(self, register):
        d = self._read(register, 2)
        return d[0] | (d[1] << 8)


def _channels(motors):
    # port mask (M1..M4, E1 = M1, E2 = M2, ALL) -> channel bits 0..3
    chs = motors & 0x0F
    if motors & E1:
        chs |= 0x01
    if motors & E2:
        chs |= 0x02
    return chs


def _first_channel(motors):
    chs = _channels(motors)
    for ch in range(4):
        if chs & (1 << ch):
            return ch
    return -1
