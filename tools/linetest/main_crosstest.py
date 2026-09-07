# one-shot cross + turn test: runs when /run_once exists, logs to /line_log.txt
import os, sys, time, asyncio
try:
    os.stat('/run_once')
except OSError:
    sys.exit()
os.remove('/run_once')

lines = []
def out(*a):
    s = ' '.join(str(x) for x in a)
    lines.append(s)   # no stdout: writing to USB blocks the loop when unplugged

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
    robot = DriveBase(MODE_2WD, m1=DCMotor(md, M3), m2=DCMotor(md, M4, reversed=True))
    robot.size(wheel=65, width=110)
    robot.speed(38, min_speed=30)
    s = LineSensorI2C()
    s.mode('analog')
    robot.line_sensor(s)
    robot.line_speed(38, min_speed=30)
    robot.line_turn_offset(0.5)   # sensor sits 10 cm ahead of the axle
    robot.line_debug(True, 0)   # every control frame

    async def main():
        out('start ms', time.ticks_ms(), 'calibrated', s.calibrated())
        await asyncio.sleep(5)
        for _ in range(3):
            out('static pos', s.update(), 'pattern', bin(s.pattern()), 'sig', s._sig)
            await asyncio.sleep_ms(50)

        t0 = time.ticks_ms()
        ok = await robot.follow_line_until_cross(then=BRAKE)
        out('CROSS ->', ok, 'in', time.ticks_diff(time.ticks_ms(), t0), 'ms, pattern', bin(s.pattern()), 'pos', s.position())
        await asyncio.sleep(1)

        out('--- turning right until the line is found again')
        t1 = time.ticks_ms()
        ok2 = await robot.turn_until_line_detected(100, then=BRAKE)
        out('TURN ->', ok2, 'in', time.ticks_diff(time.ticks_ms(), t1), 'ms, pattern', bin(s.pattern()), 'pos', s.position())
        await asyncio.sleep(1)

        out('--- following the branch for 2 s')
        ok3 = await robot.follow_line_by_time(2, then=BRAKE)
        out('BRANCH ->', ok3, 'pattern', bin(s.pattern()))
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
