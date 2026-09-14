import sys, asyncio
import linesim
from linesim import *
from shims import clock

def traced(kind, every_ms=50, until_s=6, **kw):
    track, robot, db, sensor = make(**kw)
    rows = []
    last = [-1]
    orig = robot.step
    def step(dt):
        orig(dt)
        t = clock.us // 1000
        if t // every_ms != last[0]:
            last[0] = t // every_ms
            rows.append((t, robot.x, robot.y, robot.h, sensor._pattern, sensor._pos, robot.cmd[0], robot.cmd[1]))
    clock.physics = step
    async def go():
        if kind == 'old':
            f = OldFollower(db, sensor)
            return await f.until_cross()
        return await db.follow_line_until_cross(then=BRAKE)
    ok = asyncio.run(go())
    for r in rows:
        if r[0] > until_s * 1000: break
        print('%6d x=%.3f y=%.3f h=%6.1f pat=%s pos=%s L=%4d R=%4d' % (r[0], r[1], r[2], math.degrees(r[3]), format(r[4], '05b')[::-1], 'None' if r[5] is None else '%.2f' % r[5], r[6], r[7]))
    print('ok', ok, 'x', robot.x, 'y', robot.y)

if __name__ == '__main__':
    kind = sys.argv[1]
    kw = eval('dict(' + (sys.argv[2] if len(sys.argv) > 2 else '') + ')')
    traced(kind, **kw)
