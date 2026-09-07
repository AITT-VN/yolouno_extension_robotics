import sys, math, asyncio, random
import os
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(__file__))
import shims
from shims import clock, mod
mod('motor')
from world import Robot, demo_track
from constants import *
import line_sensor as LS
import drivebase as DB
DB.TUNING_FILE = os.path.join(os.path.dirname(__file__), 'tuning.json')

I2C_US = 700   # one I2C transaction on the bit-banged bus

class FakeDriver:
    def __init__(self, robot): self.robot = robot; self.motors = []
    def set_motors(self, ports, v):
        for m in self.motors:
            if ports & m.port: self.robot.set_cmd(m.side, v)
    def brake(self, ports): self.set_motors(ports, 0)

class FakeMotor:
    WHEEL_CIRC = math.pi * 0.065
    ticks_per_rev = 1400
    def __init__(self, robot, side, port):
        self.robot, self.side, self.port = robot, side, port
        self._reversed = 1
        self.driver = robot.driver if hasattr(robot, 'driver') else FakeDriver(robot)
        robot.driver = self.driver
        self.driver.motors.append(self)
        self._zero = 0.0
    def reverse(self): self._reversed *= -1
    def run(self, v): self.robot.set_cmd(self.side, v)
    def brake(self): self.robot.set_cmd(self.side, 0)
    def stop(self): self.robot.set_cmd(self.side, 0)
    def angle(self):
        return (self.robot.wheel[self.side] - self._zero) / self.WHEEL_CIRC * 360
    def reset_angle(self):
        self._zero = self.robot.wheel[self.side]
    def encoder_ticks(self):
        return int(self.robot.wheel[self.side] / self.WHEEL_CIRC * self.ticks_per_rev)

class FakeAngle:
    """gyro: degrees, unwrapped since reset(); clockwise positive like the real one"""
    def __init__(self, robot): self.robot = robot; self._zero = 0.0
    async def reset(self): self._zero = self.robot.h
    @property
    def angle(self): return -math.degrees(self.robot.h - self._zero)
    @property
    def heading(self):
        a = self.angle % 360
        return a - 360 if a > 180 else a
    def reverse(self): self._reversed *= -1
    def run(self, v): self.robot.set_cmd(self.side, v)
    def brake(self): self.robot.set_cmd(self.side, 0)
    def stop(self): self.robot.set_cmd(self.side, 0)

class SimSensor5(LS.LineSensor5P_I2C):
    def __init__(self, robot):
        LS.LineSensor.__init__(self, 5)
        self.robot = robot; self.ok = True; self.i2c = None; self.address = LINE5_ADDR
        self._analog = False; self._raw = (0,)*5; self._norm = [0.0]*5; self._sig = [0]*5; self._bg = None
        self._cal_min = [4095]*5; self._cal_max = [0]*5; self._cal_learn = True
        self._line_high = None; self._polarity_votes = 0; self._color = None
    def _read(self, reg, n=1):
        clock.advance(I2C_US if n == 1 else I2C_US + 250)
        if reg == LINE5_REG_TUPLE:
            b = self.robot.bits()   # bit0 = S1 -> firmware bit4 = S1
            fw = 0
            for i in range(5):
                if b & (1 << i): fw |= 1 << (4 - i)
            return bytes([fw])
        if reg == LINE5_REG_RAW:
            raw = self.robot.raw()  # S1..S5 -> firmware S5..S1
            out = bytearray()
            for i in range(4, -1, -1):
                out += bytes([raw[i] & 0xFF, raw[i] >> 8])
            return bytes(out)
        return bytes(n)
    def _write(self, reg, val): pass
    def save_calibration(self): pass
    def _load_calibration(self): pass
    def reset_calibration(self):
        self._cal_min = [4095]*5; self._cal_max = [0]*5; self._line_high = None; self._polarity_votes = 0

class SimSensor4(LS.LineSensor):
    """4-eye PCF8574 array: digital only"""
    def __init__(self, robot):
        super().__init__(4)
        self.robot = robot
    def _read_bits(self):
        clock.advance(I2C_US)
        return self.robot.bits()
    check = LS.LineSensorI2C.check

# ---------------- the old discrete follower from main (ported verbatim in behaviour) ----------------
class OldFollower:
    def __init__(self, robot_db, sensor):
        self.db, self.s = robot_db, sensor
        self.last = LINE_CENTER
    def step(self, backward=True, line_state=None):
        db, s = self.db, self.s
        mn = db._min_speed
        if line_state is None:
            line_state = s.check()
        if line_state == LINE_END:
            if backward:
                db.run_speed(-mn, -mn)
        else:
            if line_state == LINE_CENTER:
                if self.last == LINE_CENTER:
                    db.run_speed(db._speed, db._speed)
                else:
                    db.run_speed(mn, mn)
            elif line_state == LINE_CROSS:
                db.run_speed(mn, mn)
            else:
                if line_state == LINE_RIGHT:
                    db.run_speed(mn, int(mn*1.25))
                elif line_state == LINE_RIGHT2:
                    db.run_speed(0, mn)
                elif line_state == LINE_RIGHT3:
                    t0 = clock.us
                    while line_state != LINE_CENTER and line_state != LINE_LEFT and clock.us - t0 < 3e6:
                        db.run_speed(-mn, mn)
                        line_state = s.check()
                    self.last = line_state
                    return
                elif line_state == LINE_LEFT:
                    db.run_speed(int(mn*1.25), mn)
                elif line_state == LINE_LEFT2:
                    db.run_speed(mn, 0)
                elif line_state == LINE_LEFT3:
                    t0 = clock.us
                    while line_state != LINE_CENTER and line_state != LINE_RIGHT and clock.us - t0 < 3e6:
                        db.run_speed(mn, -mn)
                        line_state = s.check()
                    self.last = line_state
                    return
        self.last = line_state
    async def until_cross(self):
        status, count = 1, 0
        t0 = clock.us
        while clock.us - t0 < 30e6:
            st = self.s.check()
            if status == 1:
                if st != LINE_CROSS: status = 2
            elif st == LINE_CROSS:
                count += 1
                if count == 2: break
            self.step(True, st)
            await asyncio.sleep_ms(20 if (status == 2 and count == 1) else 10)
        self.db.stop()
        return clock.us - t0 < 30e6
    async def until_end(self):
        count = 2
        t0 = clock.us
        while clock.us - t0 < 30e6:
            st = self.s.check()
            if st == LINE_END:
                count -= 1
                if count == 0: break
            self.step(False, st)
            await asyncio.sleep_ms(10)
        self.db.stop()
        return clock.us - t0 < 30e6

# ---------------- scenario ----------------
def make(eyes=5, analog=False, cruise=60, slow=40, kp=None, kd=None, slowdown=None, vmax=0.8, seed=1,
         track=None, encoders=True, gyro=False, offset=0, **rk):
    random.seed(seed)
    clock.us = 0
    track = track or demo_track()
    robot = Robot(track, vmax=vmax, eyes=eyes, spacing=0.015 if eyes == 5 else 0.014, **rk)
    clock.physics = robot.step
    m1, m2 = FakeMotor(robot, 0, E1 if encoders else 1), FakeMotor(robot, 1, E2 if encoders else 2)
    db = DB.DriveBase(MODE_2WD, m1, m2)
    db.size(wheel=65, width=150)
    db.speed(cruise, min_speed=slow)
    if gyro:
        db.angle_sensor(FakeAngle(robot))
        db.use_gyro(True)
    if offset:
        db.line_sensor_offset(offset)
    sensor = SimSensor5(robot) if eyes == 5 else SimSensor4(robot)
    if analog:
        sensor.mode('analog')
    db.line_sensor(sensor)
    if kp is not None: db.line_pid(Kp=kp)
    if kd is not None: db.line_pid(Kd=kd)
    if slowdown is not None: db.line_slowdown(slowdown)
    return track, robot, db, sensor

async def run_new(track, robot, db, sensor):
    t0 = clock.us
    ok = await db.follow_line_until_cross(then=BRAKE)
    t_cross = (clock.us - t0) / 1e6
    x_cross = robot.x
    err_cross = robot.max_err
    ok2 = False
    if ok:
        ok2 = await db.follow_line_until_end(then=BRAKE)
    t_end = (clock.us - t0) / 1e6
    return dict(cross=ok, cross_err_mm=(x_cross + robot.sensor_ahead - track.cross_x) * 1000, t_cross=t_cross,
                end=ok2, t_end=t_end, end_x=robot.x, max_err_mm=robot.max_err * 1000, smooth_mm=robot.max_err_smooth * 1000, rms_mm=robot.rms_err() * 1000)

async def run_old(track, robot, db, sensor):
    f = OldFollower(db, sensor)
    t0 = clock.us
    ok = await f.until_cross()
    t_cross = (clock.us - t0) / 1e6
    x_cross = robot.x
    ok2 = False
    if ok:
        ok2 = await f.until_end()
    t_end = (clock.us - t0) / 1e6
    return dict(cross=ok, cross_err_mm=(x_cross + robot.sensor_ahead - track.cross_x) * 1000, t_cross=t_cross,
                end=ok2, t_end=t_end, end_x=robot.x, max_err_mm=robot.max_err * 1000, smooth_mm=robot.max_err_smooth * 1000, rms_mm=robot.rms_err() * 1000)

async def run_T(track, robot, db, sensor):
    t0 = clock.us
    ok = await db.follow_line_until_cross(then=BRAKE)
    axle_dx = (robot.x - track.cross_x) * 1000
    t_cross = (clock.us - t0) / 1e6
    ok2 = await db.turn_until_line_detected(100, then=BRAKE)
    t_turn = (clock.us - t0) / 1e6 - t_cross
    heading = math.degrees(robot.h)
    pos_after = sensor.update()
    ok3 = await db.follow_line_until_end(then=BRAKE)
    t_end = (clock.us - t0) / 1e6
    return dict(cross=ok, axle_dx_mm=axle_dx, t_cross=t_cross, turn=ok2, t_turn=t_turn, heading=heading,
                pos_after=pos_after, end=ok3, end_x=robot.x, end_y=robot.y, t_end=t_end)

def finished(track, robot, r):
    # cross detected near the bar, and end reached near the track end
    return r['cross'] and abs(r['cross_err_mm']) < 60 and r['end'] and abs(r['end_x'] - track.end[0]) < 0.12 and abs(robot.y - track.end[1]) < 0.06

def trial(kind, **kw):
    track, robot, db, sensor = make(**kw)
    r = asyncio.run(run_old(track, robot, db, sensor) if kind == 'old' else run_new(track, robot, db, sensor))
    r['ok'] = finished(track, robot, r)
    return r

if __name__ == '__main__':
    import json
    for kind, kw in [('old', dict(eyes=4)), ('old', dict(eyes=5)),
                     ('new', dict(eyes=4)), ('new', dict(eyes=5)), ('new', dict(eyes=5, analog=True))]:
        for cruise in (40, 60, 80, 100):
            r = trial(kind, cruise=cruise, slow=40 if cruise >= 40 else cruise, **kw)
            print('%-4s %-30s cruise=%3d ok=%-5s cross=%-5s dx=%6.1fmm t_cross=%5.2fs end=%-5s t_end=%5.2fs max_err=%5.1fmm smooth=%5.1fmm rms=%4.1fmm' % (
                kind, kw, cruise, r['ok'], r['cross'], r['cross_err_mm'], r['t_cross'], r['end'], r['t_end'], r['max_err_mm'], r['smooth_mm'], r['rms_mm']))
