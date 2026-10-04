import math, asyncio
from micropython import const
from time import ticks_ms, ticks_diff
from utility import *
from ble import *
from constants import *
from ps4_receiver import PS4GamepadReceiver

# The App's Gamepad screen repeats a held dpad button / joystick every 250 ms.
# When nothing comes for this long the phone has gone (screen off, app in the
# background, Bluetooth out of range) and the control counts as released, so
# the robot does not keep driving on the last command.
_BLE_TIMEOUT_MS = const(1000)

# Inputs that drive the robot, the ones the timeout above releases
_DRIVE_INPUTS = (BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, AL, AR)

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
            BTN_THUMBL: 0,
            BTN_THUMBR: 0,
            BTN_M1: 0,
            BTN_M2: 0,
            BTN_PS: 0,
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
            # drive mode and speed gear (%) picked on the App, -1 until it sends them
            TELEOP_MODE: -1,
            TELEOP_SPEED: -1,
            TELEOP_GEAR_LIST: '',
        }

        # remote control
        self._cmd = None
        self._last_cmd = None
        self._run_speed = 0
        self._cmd_handlers = {}

        # when each drive input last came from the App, see check_timeout()
        self._ble_seen = {}
        # Only an App that repeats held controls sends MODE: older ones send a
        # joystick once and then nothing while it is held still, so the
        # timeout is armed by the first MODE and disarmed on disconnect.
        self._ble_timeout = False
        self._ble_connected = False
        self._ps4_connected = False

        # enable PS4 gamepad receiver
        try:
            self._ps4_gamepad = PS4GamepadReceiver()
        except:
            print('PS4 gamepad receiver not found. Ignore it.')
            self._ps4_gamepad = None
        
        # enable BLE gamepad on OhStem App
        ble.on_receive_msg('name_value', self.on_ble_cmd)
    
    async def on_ble_cmd(self, name, value):
        self.feed(name, value)

    def feed(self, name, value):
        '''One NAME=value message from the App's Gamepad screen. Not async, so
        firmware code may also call it straight from a BLE handler.'''
        #print(name + '=' + str(value))
        if name not in self.data:
            return

        if name == AL or name == AR:
            try:
                value = int(value)
            except ValueError:
                return
            # x * 256 + y, both -100..100: y is the low byte read as signed,
            # x what is left. (Shifting the value right gave x one too small
            # whenever y was negative.)
            ay = value & 0xFF
            if ay > 127:
                ay -= 256
            ax = (value - ay) >> 8
            self.data[name] = value
            self._set_stick(name, ax, ay)
        else:
            self.data[name] = value
            if name == TELEOP_MODE:
                self._ble_timeout = True

        if name in _DRIVE_INPUTS:
            self._ble_seen[name] = ticks_ms()

    def _set_stick(self, name, x, y):
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        dir, distance = self._calculate_joystick(x, y)
        if name == AL:
            self.data[ALX] = x
            self.data[ALY] = y
            self.data[AL_DIR] = dir
            self.data[AL_DISTANCE] = distance
        else:
            self.data[ARX] = x
            self.data[ARY] = y
            self.data[AR_DIR] = dir
            self.data[AR_DISTANCE] = distance

    def _release(self, name):
        if name == AL or name == AR:
            self.data[name] = 0
            self._set_stick(name, 0, 0)
        else:
            self.data[name] = 0
        self._ble_seen.pop(name, None)

    def release_all(self):
        '''Every button and joystick back to rest. MODE, SPD and GEARS are kept.'''
        for name in self.data:
            if name not in (TELEOP_MODE, TELEOP_SPEED, TELEOP_GEAR_LIST, AL, AR, ALX, ALY, ARX, ARY,
                            AL_DIR, AL_DISTANCE, AR_DIR, AR_DISTANCE):
                self.data[name] = 0
        self._release(AL)
        self._release(AR)
        self._ble_seen = {}

    def check_timeout(self):
        '''Releases what the App holds but has stopped repeating, and
        everything when the App disconnects. run() calls it every 10 ms.'''
        connected = ble.is_connected()
        if connected != self._ble_connected:
            self._ble_connected = connected
            if not connected:
                self._ble_timeout = False
                if not self._ps4_connected:
                    self.release_all()
        if not self._ble_timeout or self._ps4_connected or not self._ble_seen:
            return
        now = ticks_ms()
        for name in list(self._ble_seen):
            if ticks_diff(now, self._ble_seen[name]) > _BLE_TIMEOUT_MS:
                if self.data[name]:
                    print('Gamepad: no news of', name, 'from the App, released')
                self._release(name)

    @property
    def ps4_connected(self):
        return self._ps4_connected

    def feedback(self, color=None, rumble=0, duration=0, player=None):
        '''Light bar colour (r, g, b), rumble 0-255 for duration (x10 ms) and
        player LEDs of a PS4 gamepad on the receiver. Does nothing without one.'''
        if not self._ps4_connected:
            return
        try:
            if color is not None:
                self._ps4_gamepad.set_led_color(color)
            if player is not None:
                self._ps4_gamepad.set_player_led(player)
            if rumble:
                self._ps4_gamepad.set_rumble(rumble, duration)
        except OSError:
            pass

    def on_button_pressed(self, button, callback):
        self._cmd_handlers[button] = callback

    def _read_ps4(self):
        ps4 = self._ps4_gamepad
        ps4.update()
        if not ps4.is_connected:
            if self._ps4_connected:
                # the gamepad switched off or went out of range: do not keep
                # what it last held
                self._ps4_connected = False
                self.release_all()
            return
        self._ps4_connected = True
        d = ps4.data
        self.data[BTN_UP] = d['dpad_up']
        self.data[BTN_DOWN] = d['dpad_down']
        self.data[BTN_LEFT] = d['dpad_left']
        self.data[BTN_RIGHT] = d['dpad_right']
        self.data[BTN_CROSS] = d['a']
        self.data[BTN_CIRCLE] = d['b']
        self.data[BTN_SQUARE] = d['x']
        self.data[BTN_TRIANGLE] = d['y']
        self.data[BTN_L1] = d['l1']
        self.data[BTN_R1] = d['r1']
        self.data[BTN_L2] = d['l2']
        self.data[BTN_R2] = d['r2']
        self.data[BTN_M1] = d['m1']
        self.data[BTN_M2] = d['m2']
        self.data[BTN_PS] = d['sys']
        self.data[BTN_THUMBL] = d['thumbl']
        self.data[BTN_THUMBR] = d['thumbr']
        self._set_stick(AL, translate(d['alx'], -508, 512, -100, 100), translate(d['aly'], 512, -508, -100, 100))
        self._set_stick(AR, translate(d['arx'], -508, 512, -100, 100), translate(d['ary'], 512, -508, -100, 100))

    async def run(self):
        while True:
            if self._ps4_gamepad:
                self._read_ps4()
            self.check_timeout()
            
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
  create_task(gamepad.run())

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
'''