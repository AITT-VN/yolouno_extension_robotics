"""CPython check of mdv3.py (Motor Driver V3), the closed-loop DCMotor functions,
their fallback on Motor Driver V2 / pin motors, and DriveBase encoder detection.

    python3 tools/mdv3/test_mdv3.py

The I2C bus is simulated: FakeV3 keeps the firmware 0.2 register map and a
simple motor model, FakeV2 the Motor Driver V2 registers.
"""
import os, sys, types, struct, asyncio, time as _time
LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
sys.path.insert(0, LIB)

# ---------------- simulated clock and MicroPython modules ----------------
clock = {'ms': 0}
devices = {}

def advance(ms):
    for _ in range(int(ms)):
        clock['ms'] += 1
        for d in devices.values():
            d.physics(0.001)

_time.ticks_ms = lambda: clock['ms']
_time.ticks_us = lambda: clock['ms'] * 1000
_time.ticks_diff = lambda a, b: a - b
_time.ticks_add = lambda a, b: a + b
_time.sleep_ms = advance
_orig_sleep = asyncio.sleep
async def sleep_ms(ms):
    advance(ms)
    await _orig_sleep(0)
asyncio.sleep_ms = sleep_ms

def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m

class Pin:
    IN = 0; OUT = 1
    def __init__(self, *a, **k): self._v = 0
    def value(self, v=None):
        if v is None: return self._v
        self._v = v

class PWM:
    def __init__(self, *a, **k): self._d = 0
    def duty(self, d=None):
        if d is None: return self._d
        self._d = d

class SoftI2C:
    def __init__(self, *a, **k): self._ptr = {}
    def _dev(self, addr):
        if addr not in devices or clock['ms'] < getattr(devices[addr], 'ready_at', 0): raise OSError(19)
        return devices[addr]
    def scan(self):
        return sorted(a for a, d in devices.items() if clock['ms'] >= getattr(d, 'ready_at', 0))
    def writeto_mem(self, addr, reg, data): self._dev(addr).write(reg, bytes(data))
    def writeto(self, addr, data):
        data = bytes(data)
        d = self._dev(addr)
        if len(data) == 1: d.ptr = data[0]
        else: d.write(data[0], data[1:])
    def readfrom(self, addr, n):
        d = self._dev(addr)
        return d.read(d.ptr, n)

def translate(value, a, b, c, d):
    return c + (value - a) / (b - a) * (d - c)

mod('micropython', const=lambda x: x)
mod('machine', Pin=Pin, PWM=PWM, SoftI2C=SoftI2C)
mod('setting', SCL_PIN=22, SDA_PIN=21, D2_PIN=2, A3_PIN=3, A6_PIN=6, A7_PIN=7)
mod('utility', translate=translate, asleep_ms=sleep_ms)
sys.modules['utility'].__all__ = ['translate', 'asleep_ms']
mod('ble'); mod('yolo_uno'); mod('line_sensor'); mod('gamepad', Gamepad=object)

# ---------------- fake Motor Driver V3 (firmware 0.2 register map) ----------------
class FakeV3:
    def __init__(self, minor=2):
        self.bank = bytearray(256)
        self.bank[0x10] = 0x4D; self.bank[0x11] = minor; self.bank[0x12] = 0
        self.bank[0x14] = 0x01; struct.pack_into('<H', self.bank, 0x16, 7400)
        for ch in range(4):
            struct.pack_into('<HHHH', self.bank, 0xC0 + 8 * ch, 2000, 500, 150, 10000)
            self.bank[0x80 + 16 * ch + 0x0D] = 10
        self.ptr = 0
        self.pos = [0.0] * 4; self.vel = [0.0] * 4
        self.mode = ['open'] * 4; self.duty = [0] * 4
        self.target = [0] * 4; self.speed_ref = [0.0] * 4; self.end = [0] * 4
        self.wiring = [1, 1, 1, 1]   # -1: encoder counts against the motor
        self.vmax_true = 6000.0
        self.tune_t = [0] * 4
        self.log = []
        self.settings = 1
        # steppers: position in steps of the mode in use, like the firmware
        self.bank[0x68] = self.bank[0x78] = 1          # STEP_FULL
        self.bank[0x69] = self.bank[0x79] = 50
        self.st_mode = [1, 1]; self.st_pos = [0.0, 0.0]; self.st_state = ['idle', 'idle']
        self.st_target = [0, 0]; self.st_speed = [0.0, 0.0]; self.st_energized = [False, False]
        self.st_hold = [50, 50]; self.st_cmds = []

    def sign(self, ch):
        rev = -1 if self.bank[0x4B] & (1 << ch) else 1
        return self.wiring[ch] * rev

    def write(self, reg, data):
        self.log.append((reg, data))
        for i, b in enumerate(data): self.bank[reg + i] = b
        end = reg + len(data)
        for ch in range(4):
            if reg <= 0x40 + 2 * ch < end:
                self.mode[ch] = 'open'
                self.duty[ch] = struct.unpack_from('<h', self.bank, 0x40 + 2 * ch)[0]
        if reg <= 0x48 < end or reg <= 0x49 < end:
            for ch in range(4):
                if data[0x48 - reg if reg <= 0x48 < end else 0x49 - reg] & (1 << ch):
                    self.mode[ch] = 'open'; self.duty[ch] = 0
        if reg <= 0x4A < end:
            for ch in range(4):
                if self.bank[0x4A] & (1 << ch):
                    self.target[ch] -= int(self.pos[ch]); self.pos[ch] = 0
            self.bank[0x4A] = 0
        if reg <= 0xE3 < end:
            self.bank[0xE3] &= ~data[0xE3 - reg]
        if reg <= 0xE6 < end and data[0xE6 - reg] == 0xA5:
            self.settings = 2 if all(m == 'open' and d == 0 for m, d in zip(self.mode, self.duty)) else 3
        for ch in range(4):
            base = 0x80 + 16 * ch
            if reg <= base + 0x0F < end:
                self.command(ch, self.bank[base + 0x0F]); self.bank[base + 0x0F] = 0
        for s in range(2):
            base = 0x60 + 16 * s
            if reg <= base + 0x0F < end:
                self.st_command(s, self.bank[base + 0x0F]); self.bank[base + 0x0F] = 0

    MICRO = (1, 1, 2, 4, 8, 16)
    def st_command(self, s, cmd):
        base = 0x60 + 16 * s
        tgt, spd, acc = struct.unpack_from('<ihH', self.bank, base)
        self.st_cmds.append((s, cmd, tgt, spd, acc))
        if cmd == 1 and spd == 0: cmd = 4
        if cmd != 6:
            mode = self.bank[base + 8]
            if self.st_state[s] == 'idle' and mode != self.st_mode[s]:
                self.st_pos[s] = self.st_pos[s] * self.MICRO[mode] / self.MICRO[self.st_mode[s]]
                self.st_mode[s] = mode
            self.st_hold[s] = self.bank[base + 9]
        self.bank[base + 8] = self.st_mode[s]          # firmware writes back the mode in use
        if cmd == 6: self.st_state[s] = 'idle'; self.st_energized[s] = False
        elif cmd == 7: self.st_pos[s] = tgt
        elif cmd in (4, 5): self.st_state[s] = 'idle'
        elif cmd == 1: self.st_state[s] = 'run'; self.st_speed[s] = spd; self.st_energized[s] = True
        elif cmd in (2, 3):
            self.st_target[s] = (int(self.st_pos[s]) + tgt) if cmd == 2 else tgt
            self.st_speed[s] = abs(spd) or 200
            self.st_state[s] = 'move'; self.st_energized[s] = True

    def command(self, ch, cmd):
        base = 0x80 + 16 * ch
        tgt, spd, acc = struct.unpack_from('<iiI', self.bank, base)
        self.end[ch] = self.bank[base + 0x0C]
        if cmd == 1: self.mode[ch] = 'speed'; self.speed_ref[ch] = spd
        elif cmd in (2, 3):
            start = self.target[ch] if self.mode[ch] in ('move', 'hold') else int(self.pos[ch])
            self.target[ch] = tgt if cmd == 3 else start + tgt
            self.speed_ref[ch] = abs(spd) or 3000
            self.mode[ch] = 'move'
        elif cmd == 4: self.mode[ch] = 'stop'
        elif cmd == 5: self.mode[ch] = 'hold'; self.target[ch] = int(self.pos[ch])
        elif cmd == 7: self.mode[ch] = 'tune'; self.tune_t[ch] = 0; self.bank[0xE5] &= ~(1 << ch)

    def physics(self, dt):
        for ch in range(4):
            m = self.mode[ch]
            if m == 'open':
                v = self.duty[ch] / 1000 * self.vmax_true * self.sign(ch)
            elif m == 'speed':
                v = self.speed_ref[ch]
                if self.sign(ch) < 0:          # wrong direction: protection stops it
                    self.bank[0xE3] |= 1 << ch; self.mode[ch] = 'open'; self.duty[ch] = 0; v = 0
            elif m == 'move':
                rem = self.target[ch] - self.pos[ch]
                step = self.speed_ref[ch] * dt
                if abs(rem) <= step:
                    self.pos[ch] = self.target[ch]; v = 0
                    self.mode[ch] = 'hold' if self.end[ch] == 1 else 'open'; self.duty[ch] = 0
                else:
                    v = step / dt * (1 if rem > 0 else -1)
            elif m == 'tune':
                self.tune_t[ch] += 1; v = 0
                if self.tune_t[ch] >= 5000:
                    self.mode[ch] = 'open'
                    if self.sign(ch) < 0: self.bank[0xE5] |= 1 << ch
                    else: struct.pack_into('<H', self.bank, 0xC0 + 8 * ch + 6, int(self.vmax_true))
            else:   # hold, stop
                v = 0
                if m == 'stop': self.mode[ch] = 'hold' if self.end[ch] == 1 else 'open'
            self.vel[ch] = v
            self.pos[ch] += v * dt
        for s in range(2):
            if self.st_state[s] == 'run':
                self.st_pos[s] += self.st_speed[s] * dt
            elif self.st_state[s] == 'move':
                rem = self.st_target[s] - self.st_pos[s]
                step = self.st_speed[s] * dt
                if abs(rem) <= step:
                    self.st_pos[s] = self.st_target[s]; self.st_state[s] = 'idle'
                else:
                    self.st_pos[s] += step if rem > 0 else -step

    def read(self, reg, n):
        for ch in range(4):
            struct.pack_into('<i', self.bank, 0x18 + 4 * ch, int(self.pos[ch]))
            struct.pack_into('<i', self.bank, 0x28 + 4 * ch, int(self.vel[ch]))
        busy = sum(1 << ch for ch in range(4) if self.mode[ch] in ('move', 'stop', 'tune'))
        modes = {'open': 0, 'speed': 1, 'move': 2, 'stop': 2, 'tune': 2, 'hold': 3}
        self.bank[0xE2] = busy
        self.bank[0xE4] = sum(modes[self.mode[ch]] << (2 * ch) for ch in range(4))
        self.bank[0xE6] = self.settings
        for s in range(2):
            struct.pack_into('<i', self.bank, 0x38 + 4 * s, int(round(self.st_pos[s])))
        self.bank[0x14] = 0x01 | sum(0x04 << s for s in range(2) if self.st_state[s] != 'idle')
        return bytes(self.bank[reg:reg + n])

# ---------------- fake Motor Driver V2 (ORC Hub / MDV2 registers) ----------------
class FakeV2:
    def __init__(self):
        self.bank = bytearray(64); self.bank[42] = 0x54; self.bank[43] = 74
        self.ptr = 0; self.pos = [0.0, 0.0]; self.duty = [0, 0]; self.braked = 0; self.log = []
    def write(self, reg, data):
        self.log.append((reg, data))
        for i, b in enumerate(data): self.bank[reg + i] = b
        if reg == 16:
            idx, spd = struct.unpack_from('<Hh', self.bank, 16)
            for i, bit in enumerate((16, 32)):
                if idx & bit: self.duty[i] = spd
        if reg == 22: self.braked = data[0]
        if reg == 0:
            for i, bit in enumerate((16, 32)):
                if data[0] & bit: self.pos[i] = 0
    def physics(self, dt):
        for i in range(2): self.pos[i] += self.duty[i] / 1000 * 6000 * dt
    def read(self, reg, n):
        for i in range(2):
            struct.pack_into('<i', self.bank, 44 + 4 * i, int(self.pos[i]))
            struct.pack_into('<h', self.bank, 52 + 2 * i, int(self.duty[i] * 6))
        return bytes(self.bank[reg:reg + n])

# ---------------- tests ----------------
ok = True
def check(name, cond, info=''):
    global ok
    print(('PASS ' if cond else 'FAIL ') + name, info)
    ok = ok and bool(cond)

def run(coro):
    return asyncio.run(coro)

from constants import *
import mdv3, motor, drivebase as DB
from mdv3 import MotorDriverV3
from mdv2 import MotorDriverV2
from motor import DCMotor, DCMotor2PIN

# -- detection: scan 0x30..0x33 unless an address is given
def expect_error(name, f, text):
    try:
        f(); check(name, False, 'no error')
    except RuntimeError as e:
        check(name, text in str(e), str(e))

expect_error('no board -> RuntimeError', MotorDriverV3, 'devices: none')
devices[0x32] = FakeV3()
check('auto: finds the board at 0x32', MotorDriverV3().address == 0x32)
expect_error('explicit 0x30 does not take the board at 0x32', lambda: MotorDriverV3(0x30), 'answers at 0x32')
check('explicit 0x32', MotorDriverV3(0x32).address == 0x32)
devices[0x31] = FakeV3()
check('auto: two boards -> lowest address', MotorDriverV3().address == 0x31)
del devices[0x31], devices[0x32]

class OtherDevice:   # e.g. the old Motor Driver V1 at 0x30: no device ID register
    ready_at = 0
    def __init__(self): self.ptr = 0; self.writes = []
    def write(self, reg, data): self.writes.append(reg)
    def read(self, reg, n): return bytes(n)
    def physics(self, dt): pass
devices[0x30] = OtherDevice()
devices[0x24] = line = OtherDevice()   # 5-channel line sensor
expect_error('other device at 0x30 -> hint V1', MotorDriverV3, 'not a V3')
check('devices outside 0x30..0x33 are never written', line.writes == [])
del devices[0x30], devices[0x24]

late = FakeV3(); late.ready_at = clock['ms'] + 800   # driver still booting
devices[0x30] = late
t0 = clock['ms']
check('waits for a driver that boots late', MotorDriverV3().address == 0x30, '%d ms' % (clock['ms'] - t0))
del devices[0x30]

devices[0x30] = FakeV3(minor=1)
md_old = MotorDriverV3()
check('firmware 0.1: closed loop off', md_old.closed_loop is False and md_old.fw_version() == '0.1')
check('firmware 0.1: run_speed returns False quietly', md_old.run_speed(M1, 100) is False and md_old.busy() is False)
check('firmware 0.1: tune returns False', run(md_old.tune(M1)) is False)

v3 = devices[0x30] = FakeV3()
md = MotorDriverV3()
check('firmware 0.2: closed loop on', md.closed_loop and md.fw_version() == '0.2')
check('battery from VM', md.battery() == 7.4)
check('has_encoder on every port', all(md.has_encoder(p) for p in (M1, M2, M3, M4, E1, E2)))

# -- open-loop interface used by DCMotor / DriveBase
m1 = DCMotor(md, M1); m2 = DCMotor(md, E2, reversed=True); m3 = DCMotor(md, M3)
m1.run(50)
check('run 50 % -> duty 500 on channel 1', v3.duty[0] == 500)
m2.run(30)
check('E2 maps to channel 2, reversed', v3.duty[1] == -300)
md.set_motors(M1 | M2 | M3 | M4, 0)
check('mask M1..M4 -> all channels', v3.duty == [0, 0, 0, 0])
md.brake(E1 | M3)
check('brake E1|M3 -> channel mask 0x05', any(r == 0x48 and d[0] == 0x05 for r, d in v3.log))
m1.set_encoder(rpm=300, ppr=11, gears=30)          # 1320 counts / turn, 6600 counts/s rated
m2.set_encoder(rpm=300, ppr=11, gears=30)
m1.run(100); run(sleep_ms(500)); m1.stop()
check('encoder and speed read back', m1.encoder_ticks() > 2900 and abs(m1.angle() - m1.encoder_ticks() * 360 / 1320) < 0.2)
md.reverse_encoder(M3); md.reverse_encoder(M2)
check('reverse_encoder keeps earlier bits', v3.bank[0x4B] == 0x06)
v3.bank[0x4B] = 0

# -- closed loop
m1.reset_angle()
m1.run_speed(60)                                    # 60 rpm = 1320 counts/s
check('run_speed 60 rpm -> speed mode 1320 counts/s', v3.mode[0] == 'speed' and v3.speed_ref[0] == 1320)
check('untuned motor gets VMAX from set_encoder', struct.unpack_from('<H', v3.bank, 0xC6)[0] == 6600)
m2.run_speed(60)
check('reversed motor -> negative speed', v3.speed_ref[1] == -1320)
run(sleep_ms(1000))
check('speed() in rpm', abs(m1.speed() - 60) < 1, m1.speed())
run(m1.stop_smooth(then=HOLD))
check('stop_smooth then HOLD', v3.mode[0] == 'hold')
m1.run(40)
check('run() leaves the closed loop', v3.mode[0] == 'open' and v3.duty[0] == 400)
m1.stop(); m1.reset_angle()

run(m1.run_angle_precise(60, 90, then=HOLD))
check('run_angle_precise 90 deg', abs(m1.angle() - 90) < 0.5 and v3.mode[0] == 'hold', m1.angle())
run(m1.run_angle_precise(60, -45))
check('relative move chains from the held target', abs(m1.angle() - 45) < 0.5 and v3.mode[0] == 'open', m1.angle())
run(m1.run_rotation_precise(120, 2))
check('run_rotation_precise 2 turns', abs(m1.angle() - 765) < 0.5, m1.angle())
run(m1.run_target(100, 0, then=BRAKE))
check('run_target 0', abs(m1.angle()) < 0.5 and m1.done(), m1.angle())
run(m2.run_target(60, 180))
check('run_target on a reversed motor', abs(m2.angle() - 180) < 0.5, m2.angle())
m1.hold()
check('hold', v3.mode[0] == 'hold')
m1.set_pid(10, 2, 0.5)
check('set_pid x100 into the PID block', struct.unpack_from('<HHH', v3.bank, 0xC0) == (1000, 200, 50))
m1.set_current(800)
check('set_current', struct.unpack_from('<H', v3.bank, 0x50)[0] == 800)

# -- tune: channel 3 has its encoder wired backwards, tune fixes it
v3.wiring[2] = -1
m3.stop()
check('tune M3 (reversed encoder fixed)', run(m3.tune()) is True and v3.bank[0x4B] & 0x04)
check('tune wrote VMAX', md.pid(M3)['vmax'] == 6000)
check('save_settings while M1 holds -> refused', m3.save_settings() is False)
md.stop(ALL)
check('save_settings with motors stopped', m3.save_settings() is True)
v3.bank[0x4B] = 0                                     # wrong direction again
m3.set_encoder(rpm=300, ppr=11, gears=30)
m3.run_speed(50); run(sleep_ms(50))
check('wrong direction -> is_stalled', m3.is_stalled() is True)
md.clear_stall(M3)
check('clear_stall', m3.is_stalled() is False)

# -- no set_encoder: precise functions do nothing, no error
m4 = DCMotor(md, M4)
run(m4.run_angle_precise(60, 90)); run(m4.run_target(60, 90))
check('no encoder config: precise moves are skipped', v3.mode[3] == 'open' and v3.pos[3] == 0)

# -- DriveBase: V3 ports M1/M2 count as encoder motors
md.stop(ALL)
a = DCMotor(md, M1); b = DCMotor(md, M2)
a.set_encoder(300, 11, 30); b.set_encoder(300, 11, 30)
robot = DB.DriveBase(MODE_2WD, a, b)
check('DriveBase finds V3 encoders on M1/M2', robot.left_encoder is a and robot.right_encoder is b)
robot.size(wheel=65, width=150)
check('DriveBase ticks per rev', robot._ticks_per_rev == 1320)
a.run_speed(30); robot.stop()
check('robot.stop coasts both channels', v3.mode[:2] == ['open', 'open'] and v3.duty[:2] == [0, 0])

# -- stepper on Motor Driver V3
from stepper_motor import StepperMotor
md.stop(ALL)
st = StepperMotor(md, STEPPER1, steps_per_rev=200)
st.config(mode=STEP_HALF, accel=60)
check('stepper config applied while stopped', v3.st_mode[0] == 2 and struct.unpack_from('<H', v3.bank, 0x66)[0] == 400,
      (v3.st_mode[0], struct.unpack_from('<H', v3.bank, 0x66)[0]))
check('config does not move or energize', v3.st_state[0] == 'idle' and not v3.st_energized[0])
run(st.run_rotation(60, 1))
check('run_rotation 1 turn half step = 400 steps', st.position() == 400 and st.angle() == 360, (st.position(), st.angle()))
run(st.run_angle(120, -90))
check('run_angle -90', st.angle() == 270, st.angle())
run(st.run_target(60, 0))
check('run_target 0', st.angle() == 0 and st.done(), st.angle())
run(st.run_steps(60, 1000, wait=False))
check('wait=False returns while moving', not st.done() and 0 <= st.position() < 1000, st.position())
st.config(mode=STEP_FULL)
check('mode change while moving is deferred', v3.st_mode[0] == 2 and st._mode == STEP_HALF)
run(st.wait())
check('wait() until done', st.position() == 1000 and st.done(), st.position())
run(st.run_angle(60, 90))     # mode FULL applied now: 90 deg = 50 full steps from 500
check('deferred mode applied before the next move', v3.st_mode[0] == 1 and st.angle() == 990, (v3.st_mode[0], st.angle()))
st.reset_position()
check('reset_position', st.position() == 0 and st.angle() == 0)
st.run(30)
check('run 30 rpm full step = 100 steps/s', v3.st_state[0] == 'run' and v3.st_speed[0] == 100)
run(sleep_ms(1000))
check('runs continuously', 95 <= st.position() <= 105, st.position())
run(st.stop())
check('smooth stop holds', v3.st_state[0] == 'idle' and v3.st_energized[0] and v3.st_cmds[-1][1] == 4)
st.set_current(run_ma=700, hold_percent=30)
check('stepper current on M1+M2, hold 30 %', struct.unpack_from('<HH', v3.bank, 0x50) == (700, 700) and v3.st_hold[0] == 30)
st.release()
check('release', not v3.st_energized[0] and v3.st_cmds[-1][1] == 6)
st2 = StepperMotor(md, STEPPER2, steps_per_rev=48, reversed=True)
run(st2.run_steps(60, 24))
check('STEPPER2 reversed: driver moves -24, position +24', v3.st_pos[1] == -24 and st2.position() == 24 and st2.angle() == 180)
check('micro step mode 1/16', (st2.config(mode=STEP_1_16), v3.st_mode[1])[1] == 5 and st2.position() == 384, st2.position())
try:
    StepperMotor(md, 3); check('invalid port -> ValueError', False)
except ValueError:
    check('invalid port -> ValueError', True)

# -- fallback on Motor Driver V2 (ORC Hub): no closed loop, no error
v2 = devices[0x54] = FakeV2()
md2 = MotorDriverV2()
e1 = DCMotor(md2, E1); e1.set_encoder(rpm=300, ppr=11, gears=30)
check('V2 has no closed loop', e1.closed_loop() is False)
e1.run_speed(150)
check('V2 run_speed 150 rpm -> 50 % open loop', v2.duty[0] == 500)
e1.stop(); e1.reset_angle()
run(e1.run_angle_precise(150, 90, then=HOLD))
check('V2 run_angle_precise falls back to run_angle (brake)', e1.angle() >= 90 and v2.braked & E1, e1.angle())
e1.reset_angle()
run(e1.run_target(150, -90))
check('V2 run_target -90 turns backwards', e1.angle() <= -90, e1.angle())
e1.hold()
check('V2 hold -> brake', v2.braked & E1)
check('V2 tune -> False', run(e1.tune()) is False)
e1.set_pid(1, 2, 3); e1.set_current(500)
check('V2 done / is_stalled / save', e1.done() is True and e1.is_stalled() is False and e1.save_settings() is False)
e1.run(50); run(e1.stop_smooth(then=STOP))
check('V2 stop_smooth STOP', v2.duty[0] == 0)
ra = DCMotor(md2, E1); rb = DCMotor(md2, E2); rc = DCMotor(md2, M1)
robot2 = DB.DriveBase(MODE_2WD, ra, rb)
check('DriveBase on V2 still uses E1/E2', robot2.left_encoder is ra and robot2.right_encoder is rb)
robot3 = DB.DriveBase(MODE_2WD, rc, DCMotor(md2, M2))
check('DriveBase on V2 M1/M2: no encoder', robot3.left_encoder is None and robot3.right_encoder is None)

# -- pin motor (no driver)
pm = DCMotor2PIN(4, 5)
pm.run_speed(50); pm.hold(); run(pm.stop_smooth()); pm.set_current(300); pm.set_pid(1, 1, 1)
check('pin motor: closed-loop calls do not raise', run(pm.tune()) is False and pm.done() and not pm.is_stalled())

# -- stepper on drivers without a stepper output: no error
sv2 = StepperMotor(md2, STEPPER1)
sv2.run(60); run(sv2.run_angle(60, 90)); run(sv2.run_target(60, 0)); run(sv2.stop()); sv2.release()
sv2.config(mode=STEP_HALF, accel=10); sv2.set_current(500, 20); sv2.reset_position()
check('V2 stepper: no-op, no error', sv2.done() and sv2.position() == 0 and sv2.angle() == 0)
check('no driver: no error', StepperMotor(None, STEPPER2).done())

# -- stepper on Motor Driver V1 (MD4C protocol, register 0x01)
class FakeV1:
    ready_at = 0
    def __init__(self): self.ptr = 0; self.log = []
    def write(self, reg, data): self.log.append((reg, bytes(data)))
    def read(self, reg, n): return bytes(n)
    def physics(self, dt): pass
del devices[0x30]
v1 = devices[0x30] = FakeV1()
from mdv1 import MotorDriverV1
md1 = MotorDriverV1()
s1 = StepperMotor(md1, STEPPER1, steps_per_rev=200)
v1.log.clear()
t0 = clock['ms']
run(s1.run_steps(60, 200))
pk = [d for r, d in v1.log if r == 0x01]
check('V1 move: speed packet then step packet on register 0x01',
      len(pk) == 2 and pk[0][4] == 0 and (pk[0][6] << 8 | pk[0][7]) == 60 and pk[1][4] == 1 and (pk[1][6] << 8 | pk[1][7]) == 200, pk)
check('V1 move waits for the estimated time', 1000 <= clock['ms'] - t0 < 1300 and s1.position() == 200, clock['ms'] - t0)
v1.log.clear()
run(s1.run_angle(60, -90))
pk = [d for r, d in v1.log if r == 0x01]
check('V1 backwards: one step packet, direction 1', len(pk) == 1 and pk[0][5] == 1 and (pk[0][6] << 8 | pk[0][7]) == 50 and s1.angle() == 270, pk)
s1.config(mode=STEP_1_8)
check('V1 micro step -> half step', s1._mode == STEP_HALF and s1.position() == 300)
v1.log.clear(); s1.run(-30)
pk = [d for r, d in v1.log if r == 0x01]
check('V1 run -30 rpm', pk[-1][3] == 2 and pk[-1][4] == 0 and pk[-1][5] == 1 and pk[-1][7] == 30, pk)
s1.release()
check('V1 release = speed 0', [d for r, d in v1.log if r == 0x01][-1][7] == 0)
md1.stepper_speed(1, 200, 20)   # old call signature still works
check('V1 stepper_speed old signature', v1.log[-1][0] == 0x01 and v1.log[-1][1][0] == 1)

print('ALL PASS' if ok else 'SOME FAILED')
sys.exit(0 if ok else 1)
