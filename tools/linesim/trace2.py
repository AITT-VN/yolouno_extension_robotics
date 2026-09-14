import sys, asyncio, math
from linesim import *
from shims import clock
def traced(every_ms=40, xr=(1.45, 1.8), yr=(0.1, 0.5), **kw):
    track, robot, db, sensor = make(**kw)
    rows = []; last = [-1]; orig = robot.step
    def step(dt):
        orig(dt)
        t = clock.us // 1000
        if t // every_ms != last[0]:
            last[0] = t // every_ms
            rows.append((t, robot.x, robot.y, robot.h, sensor._pattern, sensor._pos, robot.cmd[0], robot.cmd[1], db._line_lost_ms, db._line_side, db._line_abs))
    clock.physics = step
    r = asyncio.run(run_new(track, robot, db, sensor))
    for row in rows:
        if xr[0] < row[1] < xr[1] and yr[0] < row[2] < yr[1]:
            print('%6d x=%.3f y=%.3f h=%6.1f pat=%s pos=%s L=%4d R=%4d lost=%4d side=%2d abs=%.2f' % (row[0], row[1], row[2], math.degrees(row[3]) % 360, format(row[4], '05b')[::-1], 'None' if row[5] is None else '%.2f' % row[5], row[6], row[7], row[8], row[9], row[10]))
    print(r)
kw = eval('dict(' + (sys.argv[1] if len(sys.argv) > 1 else '') + ')')
traced(**kw)
