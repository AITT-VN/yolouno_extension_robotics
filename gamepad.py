import math, asyncio
from micropython import const
from time import ticks_ms
from utility import *
from ble import *
from constants import *
from ps4_receiver import PS4GamepadReceiver

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

        if 0 <= angle < 10 or angle >= 350:
            dir = DIR_R
        elif 15 <= angle < 75:
            dir = DIR_RF
        elif 80 <= angle < 110:
            dir = DIR_FW
        elif 115 <= angle < 165:
            dir = DIR_LF
        elif 170 <= angle < 190:
            dir = DIR_L
        elif 195 <= angle < 255:
            dir = DIR_LB
        elif 260 <= angle < 280:
            dir = DIR_BW
        elif 285 <= angle < 345:
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