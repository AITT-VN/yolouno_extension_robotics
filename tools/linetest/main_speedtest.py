# Line-following speed test. Waits for the BOOT button, then follows the line
# without stopping, blinking the RGB LED white at every crossing bar. Every
# 8 s the straight speed goes up by 5 (green double blink) until 100. Stops
# when the line is lost for good or BOOT is pressed again (red), and writes
# what happened to /line_log.txt. Robot settings are at the top.
import time, asyncio
from time import ticks_ms, ticks_diff
from machine import Pin
from setting import BOOT_PIN
from yolo_uno import neopix
from constants import *
from mdv2 import MotorDriverV2
from motor import DCMotor
from drivebase import DriveBase
from line_sensor import LineSensorI2C

# ---- robot ----
LEFT_PORT, RIGHT_PORT = M3, M4
LEFT_REVERSED, RIGHT_REVERSED = False, True
WHEEL_MM, WIDTH_MM = 65, 110
START_SPEED, CURVE_MIN, STEP, STEP_MS, TOP_SPEED = 50, 35, 5, 8000, 100

boot = Pin(BOOT_PIN, Pin.IN, Pin.PULL_UP)
lines = []
def out(*a):
    lines.append(' '.join(str(x) for x in a))

def pressed():
    return boot.value() == 0

md = MotorDriverV2()
robot = DriveBase(MODE_2WD, m1=DCMotor(md, LEFT_PORT, reversed=LEFT_REVERSED),
                  m2=DCMotor(md, RIGHT_PORT, reversed=RIGHT_REVERSED))
robot.size(wheel=WHEEL_MM, width=WIDTH_MM)
sensor = LineSensorI2C()
sensor.mode('analog')
robot.line_sensor(sensor)

async def wait_for_boot():
    # slow blue blink until BOOT is pressed and released
    on = False
    while not pressed():
        on = not on
        neopix.show(0, (0, 0, 40) if on else (0, 0, 0))
        await asyncio.sleep_ms(400)
    while pressed():
        await asyncio.sleep_ms(20)
    neopix.show(0, (0, 0, 0))

async def run():
    cruise = START_SPEED
    curve = CURVE_MIN
    robot.speed(cruise, min_speed=curve)
    robot.line_speed(cruise, min_speed=curve)
    robot._line_reset()
    t0 = ticks_ms()
    t_step = t0
    crossings = 0
    lost_eps = 0
    lost_longest = 0
    was_lost = False
    led_until = None
    out('start speed', cruise, 'curve', curve)
    while True:
        now = ticks_ms()
        if not robot.follow_line_step():
            out('LOST for good at speed', cruise, 'after', ticks_diff(now, t0), 'ms')
            break
        # brief losses (curve recovery) per speed step
        lost_now = robot._line_lost_since is not None
        if lost_now and not was_lost:
            lost_eps += 1
        if lost_now:
            lost_longest = max(lost_longest, robot._line_lost_ms)
        was_lost = lost_now
        if robot.line_crossed():
            crossings += 1
            neopix.show(0, (255, 255, 255))
            led_until = now + 120
            out('cross', crossings, 'speed', cruise, 'at', ticks_diff(now, t0), 'ms')
        if led_until is not None and ticks_diff(now, led_until) >= 0:
            neopix.show(0, (0, 0, 0))
            led_until = None
        if ticks_diff(now, t_step) >= STEP_MS and cruise < TOP_SPEED:
            out('step done: speed', cruise, 'curve', curve, 'lost episodes', lost_eps, 'longest', lost_longest, 'ms')
            cruise = min(TOP_SPEED, cruise + STEP)
            curve = max(CURVE_MIN, int(cruise * 0.6))
            robot.line_speed(cruise, min_speed=curve)
            t_step = now
            lost_eps = 0
            lost_longest = 0
            neopix.show(0, (0, 255, 0))
            led_until = now + 150
        if pressed():
            out('stopped by BOOT at speed', cruise, 'after', ticks_diff(now, t0), 'ms')
            break
        await asyncio.sleep_ms(5)
    robot.stop()
    out('crossings', crossings, 'last step: lost episodes', lost_eps, 'longest', lost_longest, 'ms')
    neopix.show(0, (255, 0, 0))

try:
    asyncio.run(wait_for_boot())
    try:
        asyncio.run(run())
    except Exception as e:
        out('EXC', repr(e))
    finally:
        try:
            robot.stop()
        except Exception:
            pass
        with open('/line_log.txt', 'w') as f:
            f.write('\n'.join(lines) + '\n')
except KeyboardInterrupt:
    # no sys.exit(): a SystemExit here would soft-reboot the VM and hide the
    # chip's real reset reason behind machine.reset_cause() == 5
    neopix.show(0, (0, 0, 0))
    robot.stop()
