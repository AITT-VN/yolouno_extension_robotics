"""CPython check of Gamepad (packed joystick decoding, App timeout) and DriveBase.run_teleop drive modes.

    python3 tools/teleop/test_teleop.py
"""
import os, sys, types, asyncio, time as _time
LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, LIB)

clock = {'ms': 0}
_time.ticks_ms = lambda: clock['ms']
_time.ticks_us = lambda: clock['ms'] * 1000
_time.ticks_diff = lambda a, b: a - b
_orig_sleep = asyncio.sleep
async def sleep_ms(ms):
    clock['ms'] += ms
    await _orig_sleep(0)
asyncio.sleep_ms = sleep_ms

def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m

class FakeBle:
    connected = True
    sent = []
    def on_receive_msg(self, t, cb): self.cb = cb
    def is_connected(self): return self.connected
    def send_value(self, n, v): self.sent.append((n, v))
ble = FakeBle()
def translate(value, a, b, c, d):
    return round(c + (value - a) / (b - a) * (d - c))
mod('micropython', const=lambda x: x)
mod('machine', Pin=object, SoftI2C=object)
mod('ble', ble=ble)
mod('utility', translate=translate)
mod('setting'); mod('yolo_uno')
class NoRx:
    def __init__(self): raise OSError('none')
mod('ps4_receiver', PS4GamepadReceiver=NoRx)
mod('motor'); mod('line_sensor'); mod('pid', PIDController=lambda *a, **k: None)
sys.modules['utility'].__all__ = ['translate']
sys.modules['ble'].__all__ = ['ble']

from constants import *
import gamepad as G
import drivebase as DB

class Motor:
    def __init__(self, port): self.port = port; self.speed = 0; self.rev = False; self.driver = self
    def reverse(self): self.rev = True
    def run(self, s): self.speed = s
    def set_motors(self, ports, s):
        for m in robot.left + robot.right: m.speed = 0
    def brake(self, ports): self.set_motors(ports, 0)

ok = True
def check(name, cond, info=''):
    global ok
    print(('PASS ' if cond else 'FAIL ') + name, info)
    ok = ok and cond

# ---- packed joystick decode ----
gp = G.Gamepad()
for x, y in [(-50, -50), (50, -50), (-100, 100), (0, -1), (100, 0), (-1, -100), (37, 99)]:
    gp.feed(AL, str(x * 256 + y))
    check('decode %d,%d' % (x, y), (gp.data[ALX], gp.data[ALY]) == (x, y), (gp.data[ALX], gp.data[ALY]))

# ---- timeout only armed after MODE, released after 1 s ----
gp = G.Gamepad()
gp.feed(BTN_UP, 1)
clock['ms'] += 1500; gp.check_timeout()
check('old app: no timeout', gp.data[BTN_UP] == 1)
gp.feed(TELEOP_MODE, 1); gp.feed(BTN_UP, 1); gp.feed(AL, str(80 * 256))
clock['ms'] += 500; gp.check_timeout()
check('new app: held within 1s', gp.data[BTN_UP] == 1 and gp.data[ALX] == 80)
clock['ms'] += 600; gp.check_timeout()
check('new app: released after 1s', gp.data[BTN_UP] == 0 and gp.data[ALX] == 0 and gp.data[AL_DISTANCE] == 0)
gp.feed(BTN_DOWN, 1); ble.connected = False; gp.check_timeout()
check('disconnect releases', gp.data[BTN_DOWN] == 0 and gp.data[TELEOP_MODE] == 1)
ble.connected = True; gp.check_timeout()

# ---- teleop ----
def make(mode=MODE_2WD):
    global robot
    ms = [Motor(M1), Motor(M2), Motor(M3), Motor(M4)]
    robot = DB.DriveBase(mode, *(ms if mode == MODE_MECANUM else ms[:2]))
    robot.speed(80, min_speed=40)
    return robot

def wheels(r):
    return [round(m.speed) for m in r.left + r.right]

async def drive(r, g, ms):
    t = asyncio.create_task(r.run_teleop(g, accel_steps=5))
    end = clock['ms'] + ms
    while clock['ms'] < end:
        await _orig_sleep(0)
    t.cancel()

async def main():
    g = G.Gamepad(); r = make()
    # DPAD legacy: up ramps to top 80
    g.feed(BTN_UP, 1)
    await drive(r, g, 300)
    check('dpad forward ramps to 80', wheels(r) == [80, 80], wheels(r))
    g.feed(BTN_UP, 0); await drive(r, g, 30)
    check('dpad release stops', wheels(r) == [0, 0], wheels(r))

    # gear 40% from the App
    g.feed(TELEOP_SPEED, 40); g.feed(BTN_UP, 1); await drive(r, g, 300)
    check('gear 40 caps at 32', wheels(r) == [32, 32], wheels(r))
    g.feed(BTN_UP, 0); g.feed(TELEOP_SPEED, 100)

    # JOYSTICK: half push forward -> about 40 + 40*(50-15)/85
    g.feed(TELEOP_MODE, DRIVE_JOYSTICK); g.feed(AL, str(0 * 256 + 50)); await drive(r, g, 300)
    exp = round(40 + 40 * 35 / 85)
    check('joystick half forward', wheels(r) == [exp, exp], (wheels(r), exp))
    g.feed(AL, str(100 * 256 + 0)); await drive(r, g, 300)
    w = wheels(r)
    check('joystick full right spins slower', w[0] > 0 and w[1] < 0 and abs(w[0]) == 48, w)
    # near the 22.5 border keeps last sector (hysteresis): 25 deg from right still right
    import math
    x = round(100 * math.cos(math.radians(25))); y = round(100 * math.sin(math.radians(25)))
    check('hysteresis keeps right', r._teleop_stick_dir(x, y, DIR_R) == DIR_R and r._teleop_stick_dir(x, y, -1) == DIR_RF)
    x = round(100 * math.cos(math.radians(35))); y = round(100 * math.sin(math.radians(35)))
    check('past hysteresis -> right forward', r._teleop_stick_dir(x, y, DIR_R) == DIR_RF)
    g.feed(AL, '0'); await drive(r, g, 30)
    check('joystick release stops', wheels(r) == [0, 0], wheels(r))

    # SPLIT: throttle full, no steer -> 80/80 ; steer right at standstill spins
    g.feed(TELEOP_MODE, DRIVE_SPLIT); g.feed(AL, str(0 * 256 + 100)); await drive(r, g, 300)
    check('split straight', wheels(r) == [80, 80], wheels(r))
    g.feed(AR, str(100 * 256 + 0)); await drive(r, g, 300)
    w = wheels(r)
    check('split curve right', w[0] > w[1] > 0, w)
    g.feed(AL, '0'); await drive(r, g, 300)
    w = wheels(r)
    check('split spin right', w[0] > 0 and w[1] < 0, w)
    g.feed(AR, '0'); await drive(r, g, 30)
    check('split release stops', wheels(r) == [0, 0], wheels(r))
    # dpad still drives in split mode
    g.feed(BTN_DOWN, 1); await drive(r, g, 300)
    check('split dpad back', wheels(r) == [-80, -80], wheels(r))
    g.feed(BTN_DOWN, 0)

    # TANK
    g.feed(TELEOP_MODE, DRIVE_TANK); g.feed(AL, str(0 * 256 + 100)); g.feed(AR, str(0 * 256 - 100)); await drive(r, g, 300)
    check('tank opposite', wheels(r) == [80, -80], wheels(r))
    g.feed(AL, '0'); g.feed(AR, '0'); await drive(r, g, 30)

    # OPTIONS is the program's: it does not switch mode unless asked to
    ble.sent.clear()
    g.feed(BTN_M2, 1); await drive(r, g, 30); g.feed(BTN_M2, 0); await drive(r, g, 30)
    check('options left alone', r.teleop_mode() == DRIVE_TANK and not ble.sent, (r.teleop_mode(), ble.sent))
    r.teleop_buttons(mode_button=BTN_M2)
    g.feed(BTN_M2, 1); await drive(r, g, 30); g.feed(BTN_M2, 0); await drive(r, g, 30)
    check('teleop_buttons(BTN_M2): options -> next mode', r.teleop_mode() == DRIVE_DPAD and (TELEOP_MODE, 0) in ble.sent, (r.teleop_mode(), ble.sent))
    r.teleop_buttons()
    # SHARE cycles gear, reported to app
    g.feed(BTN_M1, 1); await drive(r, g, 30); g.feed(BTN_M1, 0); await drive(r, g, 30)
    check('share -> gear 40', r.teleop_gear() == 40, r.teleop_gear())
    # gears set on the App: SHARE steps through those
    g.feed(TELEOP_GEAR_LIST, '30,60,90'); g.feed(TELEOP_SPEED, 60); await drive(r, g, 30)
    g.feed(BTN_M1, 1); await drive(r, g, 30); g.feed(BTN_M1, 0); await drive(r, g, 30)
    check('app gears: 60 -> 90', r.teleop_gears() == (30, 60, 90) and r.teleop_gear() == 90, (r.teleop_gears(), r.teleop_gear()))
    g.feed(BTN_M1, 1); await drive(r, g, 30); g.feed(BTN_M1, 0); await drive(r, g, 30)
    check('app gears wrap: 90 -> 30', r.teleop_gear() == 30, r.teleop_gear())
    g.feed(TELEOP_SPEED, 100)

    # handler runs beside driving, repeats while held
    calls = []
    async def on_l1(): calls.append(clock['ms'])
    r.on_teleop_command(BTN_L1, on_l1)
    g.feed(BTN_UP, 1); g.feed(BTN_L1, 1); await drive(r, g, 500)
    check('drives while handler', wheels(r) == [80, 80], wheels(r))
    check('handler repeats while held', 2 <= len(calls) <= 4, calls)
    g.feed(BTN_L1, 0); g.feed(BTN_UP, 0)
    # handler on UP replaces forward
    r.on_teleop_command(BTN_UP, on_l1); calls.clear()
    g.feed(BTN_UP, 1); await drive(r, g, 100)
    check('UP handler replaces driving', wheels(r) == [0, 0] and calls, (wheels(r), calls))
    g.feed(BTN_UP, 0)

    # battery voltage reported every 2 s when the driver can read it
    ble.sent.clear()
    Motor.battery = lambda self: 7.6
    await drive(r, g, 2100)
    del Motor.battery
    check('reports VBAT', ('VBAT', 7.6) in ble.sent, ble.sent)

    # mecanum split: left stick sideways strafes
    g = G.Gamepad(); r = make(MODE_MECANUM); r.teleop_mode(DRIVE_SPLIT)
    g.feed(AL, str(100 * 256 + 0)); await drive(r, g, 50)
    w = wheels(r)  # left list m1,m3 ; right m2,m4
    check('mecanum strafe right', w[0] > 0 and w[1] < 0 and w[2] < 0 and w[3] > 0, w)

asyncio.run(main())
print('ALL OK' if ok else 'SOME FAILED')
