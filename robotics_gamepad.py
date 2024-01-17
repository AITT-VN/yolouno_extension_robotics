from machine import Pin, SoftI2C
from time import sleep, ticks_ms
import math
from micropython import const
from utility import *

_GAMEPAD_RECEIVER_ADDR = const(0x55)

_REG_SET_LED_COLOR = const(0x01)
_REG_SET_LED_PLAYER = const(0x02)
_REG_SET_RUMBLE = const(0x03)

_DPAD_UP = const(0)
_DPAD_DOWN = const(1)
_DPAD_RIGHT = const(2)
_DPAD_LEFT = const(3)

_BUTTON_A = const(0)
_BUTTON_B = const(1)
_BUTTON_X = const(2)
_BUTTON_Y = const(3)
_BUTTON_SHOULDER_L = const(4)
_BUTTON_SHOULDER_R = const(5)
_BUTTON_TRIGGER_L = const(6)
_BUTTON_TRIGGER_R = const(7)
_BUTTON_THUMB_L = const(8)
_BUTTON_THUMB_R = const(9)

_MISC_BUTTON_SYSTEM = const(0)  # AKA: PS, Xbox, etc.
_MISC_BUTTON_M1 = const(1)      # AKA: Select, Share, -
_MISC_BUTTON_M2 = const(2)      # AKA: Start, Options, +


class GamePadReceiver:
    def __init__(self, i2c):
        self._i2c = i2c

        self._verbose = False
        self._last_print = ticks_ms()
        self._isconnected = False

        self.data = {
            'dpad': 0,
            'dpad_left': 0,
            'dpad_right': 0,
            'dpad_up': 0,
            'dpad_down': 0,
            'dpad_left': 0,
            'alx': 0,
            'aly': 0,
            'arx': 0,
            'ary': 0,
            'a': 0,
            'b': 0,
            'x': 0,
            'y': 0,
            'l1': 0,
            'r1': 0,
            'l2': 0,
            'r2': 0,
            'al2': 0,  # 0-1020
            'ar2': 0,  # 0-1020
            'm1': 0,
            'm2': 0,
        }

        self.dpad = 0

        # Joystick
        self.aLX = 0
        self.aLY = 0

        self.aRX = 0
        self.aRY = 0

        # Buttons
        self.al2 = 0
        self.ar2 = 0

        self.buttons = 0
        self.misc_buttons = 0

    def _read_16(self, b1, b2):
        # Read and return a 16-bit signed little endian value from 2 bytes
        raw = (b1 << 8) | b2
        if (raw & (1 << 15)):  # sign bit is set
            return (raw - (1 << 16))
        else:
            return raw

    def _read_32(self, b1, b2, b3, b4):
        # Read and return a 32-bit signed little endian value from 4 bytes

        raw = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4
        if (raw & (1 << 31)):  # sign bit is set
            return (raw - (1 << 32))
        else:
            return raw

    def _write(self, address, value):
        self._i2c.writeto_mem(_GAMEPAD_RECEIVER_ADDR, address, value)

    def update(self):
        result = self._i2c.readfrom(_GAMEPAD_RECEIVER_ADDR, 30)
        has_data = result[0]

        if has_data == 1:
            self._isconnected = True
        else:
            self._isconnected = False

        if has_data:
            self.dpad = result[1]
            self.aLX = self._read_32(
                result[2], result[3], result[4], result[5])
            self.aLY = self._read_32(
                result[6], result[7], result[8], result[9])
            self.aRX = self._read_32(
                result[10], result[11], result[12], result[13])
            self.aRY = self._read_32(
                result[14], result[15], result[16], result[17])
            self.al2 = self._read_32(
                result[18], result[19], result[20], result[21])
            self.ar2 = self._read_32(
                result[22], result[23], result[24], result[25])
            self.buttons = self._read_16(result[26], result[27])
            self.misc_buttons = self._read_16(result[28], result[29])
        else:
            self.dpad = 0
            self.aLX = 0
            self.aLY = 0
            self.aRX = 0
            self.aRY = 0
            self.al2 = 0
            self.ar2 = 0
            self.buttons = 0
            self.misc_buttons = 0

        self._convert_data()

        if self._verbose:
            if ticks_ms() - self._last_print > 100:
                if has_data:
                    print('dpad=', self.dpad, ' aX=', self.aLX, ' aY=', self.aLY, ' aRX=', self.aRX, ' aRY=', self.aRY,
                          ' l2=', self.al2, ' r2=', self.ar2, ' buttons=', self.buttons, ' misc=', self.misc_buttons)
                self._last_print = ticks_ms()

    def _convert_data(self):
        self.data = {
            'dpad': self.dpad,
            'dpad_left': (self.dpad >> _DPAD_LEFT) & 1,
            'dpad_right': (self.dpad >> _DPAD_RIGHT) & 1,
            'dpad_up': (self.dpad >> _DPAD_UP) & 1,
            'dpad_down': (self.dpad >> _DPAD_DOWN) & 1,
            'alx': self.aLX,
            'aly': self.aLY,
            'arx': self.aRX,
            'ary': self.aRY,
            'thumbl': (self.buttons >> _BUTTON_THUMB_L) & 1,
            'thumbr': (self.buttons >> _BUTTON_THUMB_R) & 1,
            'a': (self.buttons >> _BUTTON_A) & 1,
            'b': (self.buttons >> _BUTTON_B) & 1,
            'x': (self.buttons >> _BUTTON_X) & 1,
            'y': (self.buttons >> _BUTTON_Y) & 1,
            'l1': (self.buttons >> _BUTTON_SHOULDER_L) & 1,
            'r1': (self.buttons >> _BUTTON_SHOULDER_R) & 1,
            'l2': (self.buttons >> _BUTTON_TRIGGER_L) & 1,
            'r2': (self.buttons >> _BUTTON_TRIGGER_R) & 1,
            'al2': self.al2,  # 0-1020
            'ar2': self.ar2,  # 0-1020
            'm1': (self.misc_buttons >> _MISC_BUTTON_M1) & 1,
            'm2': (self.misc_buttons >> _MISC_BUTTON_M2) & 1,
            'sys': (self.misc_buttons >> _MISC_BUTTON_SYSTEM) & 1,
        }

    def set_led_color(self, color):
        if len(color) != 3 or color[0] < 0 or color[0] > 255 or color[1] < 0 or color[1] > 255 or color[2] < 0 or color[2] > 255:
            return
        self._write(_REG_SET_LED_COLOR, bytearray(
            [color[0], color[1], color[2]]))

    def set_player_led(self, led):
        if led < 0 or led > 255:
            return
        self._write(_REG_SET_LED_PLAYER, bytearray([led]))

    def set_rumble(self, force, duration):
        if force < 0 or force > 255:
            return

        if duration < 0 or duration > 255:
            return
        self._write(_REG_SET_RUMBLE, bytearray([force, duration]))

    def calculate_direction(self, angle):
        # calculate direction based on angle
        #         90(3)
        #   135(4) |  45(2)
        # 180(5)---+----Angle=0(dir=1)
        #   225(6) |  315(8)
        #         270(7)
        dir = 1
        if 0 <= angle < 22.5 or angle >= 337.5:
            dir = 1
        elif 22.5 <= angle < 67.5:
            dir = 2
        elif 67.5 <= angle < 112.5:
            dir = 3
        elif 112.5 <= angle < 157.5:
            dir = 4
        elif 157.5 <= angle < 202.5:
            dir = 5
        elif 202.5 <= angle < 247.5:
            dir = 6
        elif 247.5 <= angle < 292.5:
            dir = 7
        elif 292.5 <= angle < 337.5:
            dir = 8

        return dir

    def read_joystick(self, index=0):  # 0=left, 1=right
        x = y = 0

        if index == 0:
            if self.aLX < 0:
                x = round(translate(self.aLX, -512, 0, 100, 0))
            else:
                x = round(translate(self.aLX, 0, 512, 0, -100))

            if self.aLY < 0:
                y = round(translate(self.aLY, -512, 0, 100, 0))
            else:
                y = round(translate(self.aLY, 0, 512, 0, -100))
        else:
            if self.aRX < 0:
                x = round(translate(self.aRX, -512, 0, 100, 0))
            else:
                x = round(translate(self.aRX, 0, 512, 0, -100))

            if self.aLY < 0:
                y = round(translate(self.aRY, -512, 0, 100, 0))
            else:
                y = round(translate(self.aRY, 0, 512, 0, -100))

        # joystick drag distance (robot speed)
        j_distance = int(math.sqrt(x*x + y*y))

        angle = int((math.atan2(y, x) - math.atan2(0, 100)) * 180 / math.pi)
        if angle < 0:
            angle += 360

        if j_distance < 15:
            j_distance = 0
            angle = -1
        elif j_distance > 100:
            j_distance = 100

        return(
            x,
            y,
            angle,
            self.check_dir(angle),  # direction
            j_distance
        )

    def check_dir(self, angle):

        # calculate direction based on angle

        #         90(3)

        #   135(4) |  45(2)

        # 180(5)---+----Angle=0(dir=1)

        #   225(6) |  315(8)

        #         270(7)

        dir = 0

        if 0 <= angle < 22.5 or angle >= 337.5:
            dir = 1
        elif 22.5 <= angle < 67.5:
            dir = 2
        elif 67.5 <= angle < 112.5:
            dir = 3
        elif 112.5 <= angle < 157.5:
            dir = 4
        elif 157.5 <= angle < 202.5:
            dir = 5
        elif 202.5 <= angle < 247.5:
            dir = 6
        elif 247.5 <= angle < 292.5:
            dir = 7
        elif 292.5 <= angle < 337.5:
            dir = 8

        return dir


# i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)
# gamepad = GamePadReceiver(i2c)
