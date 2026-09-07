# one-shot line following test: runs when /run_once exists, logs to /line_log.txt
import os, sys, time, asyncio
try:
    os.stat('/run_once')
except OSError:
    sys.exit()
os.remove('/run_once')

# log lines are kept in RAM and written once at the end: every flash write
# costs 100 ms or more on this board and would stall the control loop
lines = []
def out(*a):
    s = ' '.join(str(x) for x in a)
    lines.append(s)
    sys.stdout.write(s + '\n')
INVERT = 1   # -1 if the array is mounted with S1 on the robot's right

try:
    from constants import *
    from mdv2 import MotorDriverV2
    from motor import DCMotor
    import drivebase, line_sensor
    drivebase.print = out
    line_sensor.print = out
    from drivebase import DriveBase
    from line_sensor import LineSensorI2C

    md = MotorDriverV2()
    left = DCMotor(md, M3)
    right = DCMotor(md, M4, reversed=True) # wired the other way round on this robot
    robot = DriveBase(MODE_2WD, m1=left, m2=right)
    robot.size(wheel=65, width=110)
    robot.speed(38, min_speed=30)
    s = LineSensorI2C()
    s.mode('analog')
    robot.line_sensor(s)
    robot.line_speed(38, min_speed=30)
    robot.line_debug(True, 40)
    robot.line_invert(INVERT)

    async def main():
        out('start ms', time.ticks_ms(), 'sensor', type(s).__name__, 'calibrated', s.calibrated())
        await asyncio.sleep(5)
        for i in range(5):
            out('static pos', s.update(), 'pattern', bin(s.pattern()), 'digital', s.read())
            await asyncio.sleep_ms(50)
        t0 = time.ticks_ms()
        ok = await robot.follow_line_until_cross(then=BRAKE)
        out('until_cross ->', ok, 'in', time.ticks_diff(time.ticks_ms(), t0), 'ms, pattern', bin(s.pattern()), 'pos', s.position())
        await asyncio.sleep(1)
        ok2 = await robot.follow_line_by_time(1.5, then=BRAKE)
        out('by_time ->', ok2, 'pattern', bin(s.pattern()))
        robot.stop()
        out('done ms', time.ticks_ms())
    asyncio.run(main())
except Exception as e:
    out('EXC', repr(e))
finally:
    try:
        MotorDriverV2().stop()
    except Exception:
        pass
    with open('/line_log.txt', 'w') as log:
        log.write('\n'.join(lines) + '\n')
