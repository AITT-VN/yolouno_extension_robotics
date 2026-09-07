import sys, time
for m in ('line_sensor', 'constants', 'drivebase'):
    sys.modules.pop(m, None)
from constants import *
from mdv2 import MotorDriverV2
from motor import DCMotor
from drivebase import DriveBase
from line_sensor import LineSensorI2C
md = MotorDriverV2()
robot = DriveBase(MODE_2WD, m1=DCMotor(md, M3), m2=DCMotor(md, M4, reversed=True))
s = LineSensorI2C(); s.mode('analog')
robot.line_sensor(s)
robot.speed(0, min_speed=0)   # motors stay still, only the timing matters
t0 = time.ticks_us()
n = 300
for _ in range(n):
    robot.follow_line_step()
dt = time.ticks_diff(time.ticks_us(), t0) / n
robot.stop()
print('follow_line_step: %.0f us  -> %.0f Hz with a 5 ms sleep' % (dt, 1000000 / (dt + 5000)))
t0 = time.ticks_us()
for _ in range(n): s.update()
print('sensor update only: %.0f us' % (time.ticks_diff(time.ticks_us(), t0) / n))
