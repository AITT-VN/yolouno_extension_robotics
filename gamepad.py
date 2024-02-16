import math, asyncio
from micropython import const
from time import ticks_ms
from utility import *
from ble import *
from ps4_receiver import PS4GamepadReceiver

BTN_UP = 'U'
BTN_DOWN = 'D'
BTN_LEFT = 'L'
BTN_RIGHT = 'R'

BTN_SQUARE = 'SQ'
BTN_TRIANGLE = 'TR'
BTN_CROSS = 'CR'
BTN_CIRCLE = 'CI'

BTN_L1 = 'L1'
BTN_R1 = 'R1'
BTN_L2 = 'L2'
BTN_R2 = 'R2'
AL = 'AL'
ALX = 'ALX'
ALY = 'ALY'
AL_DIR = 'AL_DIR'
AL_DISTANCE = 'AL_DISTANCE'
AR = 'AR'
ARX = 'ARX'
ARY = 'ARY'
AR_DIR = 'AR_DIR'
AR_DISTANCE = 'AR_DISTANCE'

# direction
DIR_FW = const(0) # forward
DIR_RF = const(1) # right forward
DIR_R = const(2) # turn right
DIR_RB = const(3) # right backward
DIR_BW = const(4) # backward
DIR_LB = const(5) # left backward
DIR_L = const(6) # turn left
DIR_LF = const(7) # left forward
DIR_SL = const(8) # side left
DIR_SR = const(9) # side right

class Gamepad:
    def __init__(self):
        self._verbose = False
        self._last_print = 0

        self.data = {
            BTN_UP: 0,
            BTN_DOWN: 0,
            BTN_LEFT: 0,
            BTN_RIGHT: 0,
            BTN_SQUARE: 0,
            BTN_TRIANGLE: 0,
            BTN_CROSS: 0,
            BTN_CIRCLE: 0,
            BTN_L1: 0,
            BTN_R1: 0,
            BTN_L2: 0,
            BTN_R2: 0,
            AL: 0,
            ALX: 0,
            ALY: 0,
            AL_DIR: -1,
            AL_DISTANCE: 0,
            AR: 0,
            ARX: 0,
            ARY: 0,
            AR_DIR: -1,
            AR_DISTANCE: 0,
        }

        # remote control
        self._cmd = None
        self._last_cmd = None
        self._run_speed = 0
        self._cmd_handlers = {}

        # enable PS4 gamepad receiver
        try:
            self._ps4_gamepad = PS4GamepadReceiver()
        except:
            print('PS4 gamepad receiver not found. Ignore it.')
            self._ps4_gamepad = None
        
        # enable BLE gamepad on OhStem App
        ble.on_receive_msg('name_value', self.on_ble_cmd)
    
    async def on_ble_cmd(self, name, value):
        #print(name + '=' + str(value))
        if name not in list(self.data.keys()):
            return
        self.data[name] = value

        if name == AL or name == AR:
            value = int(value)
            print(value)
            ax = value >> 8
            ay = value & 0xFF
            if ay > 100:
                ay = -(-value & 0xFF)

            if name == AL:
                self.data[ALX] = ax
                self.data[ALY] = ay
                dir, distance = self._calculate_joystick(self.data[ALX], self.data[ALY])
                self.data[AL_DIR] = dir
                self.data[AL_DISTANCE] = distance
            elif name == AR:
                self.data[ARX] = ax
                self.data[ARY] = ay
                dir, distance = self._calculate_joystick(self.data[ARX], self.data[ARY])
                self.data[AR_DIR] = dir
                self.data[AR_DISTANCE] = distance

    def on_button_pressed(self, button, callback):
        self._cmd_handlers[button] = callback
        
    async def run(self):
        while True:
            if self._ps4_gamepad:
                self._ps4_gamepad.update()
                if self._ps4_gamepad.is_connected:
                    self.data[BTN_UP] = self._ps4_gamepad.data['dpad_up']
                    self.data[BTN_DOWN] = self._ps4_gamepad.data['dpad_down']
                    self.data[BTN_LEFT] = self._ps4_gamepad.data['dpad_left']
                    self.data[BTN_RIGHT] = self._ps4_gamepad.data['dpad_right']
                    self.data[BTN_CROSS] = self._ps4_gamepad.data['a']
                    self.data[BTN_CIRCLE] = self._ps4_gamepad.data['b']
                    self.data[BTN_SQUARE] = self._ps4_gamepad.data['x']
                    self.data[BTN_TRIANGLE] = self._ps4_gamepad.data['y']
                    self.data[BTN_L1] = self._ps4_gamepad.data['l1']
                    self.data[BTN_R1] = self._ps4_gamepad.data['r1']
                    self.data[BTN_L2] = self._ps4_gamepad.data['l2']
                    self.data[BTN_R2] = self._ps4_gamepad.data['r2']
                    alx = self._ps4_gamepad.data['alx']
                    alx = translate(alx, -508, 512, -100, 100)
                    self.data[ALX] = alx
                    aly = self._ps4_gamepad.data['aly']
                    aly = translate(aly, 512, -508, -100, 100)
                    self.data[ALY] = aly
                    dir, distance = self._calculate_joystick(self.data[ALX], self.data[ALY])
                    self.data[AL_DIR] = dir
                    self.data[AL_DISTANCE] = distance
                    arx = self._ps4_gamepad.data['arx']
                    arx = translate(arx, -508, 512, -100, 100)
                    self.data[ARX] = arx
                    ary = self._ps4_gamepad.data['ary']
                    ary = translate(ary, 512, -508, -100, 100)
                    self.data[ARY] = ary
                    dir, distance = self._calculate_joystick(self.data[ARX], self.data[ARY])
                    self.data[AR_DIR] = dir
                    self.data[AR_DISTANCE] = distance
            
            if self._verbose:
                if ticks_ms() - self._last_print > 200:
                    print(self.data)
                    self._last_print = ticks_ms()
            
            await asyncio.sleep_ms(10)

    def _calculate_joystick(self, x, y):
        dir = -1
        distance = int(math.sqrt(x*x + y*y))

        if distance < 15:
            distance = 0
            dir = -1
            return (dir, distance)
        elif distance > 100:
            distance = 100

        # calculate direction based on angle
        #         90
        #   135    |  45
        # 180   ---+----Angle=0
        #   225    |  315
        #         270
        #angle = int((math.atan2(y, x) - math.atan2(0, 100)) * 180 / math.pi)
        angle = int(math.atan2(y, x) * 180 / math.pi)

        if angle < 0:
            angle += 360

        if 0 <= angle < 22.5 or angle >= 337.5:
            dir = DIR_R
        elif 22.5 <= angle < 67.5:
            dir = DIR_RF
        elif 67.5 <= angle < 112.5:
            dir = DIR_FW
        elif 112.5 <= angle < 157.5:
            dir = DIR_LF
        elif 157.5 <= angle < 202.5:
            dir = DIR_L
        elif 202.5 <= angle < 247.5:
            dir = DIR_LB
        elif 247.5 <= angle < 292.5:
            dir = DIR_BW
        elif 292.5 <= angle < 337.5:
            dir = DIR_RB

        #print(x, y, angle, distance, dir)
        return (dir, distance)

'''
gamepad = Gamepad()

async def setup():
  print('App started')
  create_task(ble.wait_for_msg())
  create_task(gamepad.start())

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
'''