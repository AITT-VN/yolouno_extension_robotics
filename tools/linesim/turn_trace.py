import asyncio, math, sys
from linesim import *
from world import t_track
from shims import clock
gyro = len(sys.argv) > 1 and sys.argv[1] == 'gyro'
track, robot, db, sensor = make(track=t_track(), eyes=5, analog=True, cruise=60, slow=40, gyro=gyro, offset=60)
rows = []; last = [-1]; orig = robot.step; active = [False]
def step(dt):
    orig(dt)
    t = clock.us // 1000
    if active[0] and t // 20 != last[0]:
        last[0] = t // 20
        rows.append((t, robot.x, robot.y, math.degrees(robot.h), sensor._pattern, sensor._pos, robot.cmd[0], robot.cmd[1]))
clock.physics = step
async def go():
    await db.follow_line_until_cross(then=BRAKE)
    active[0] = True
    ok = await db.turn_until_line_detected(100, then=BRAKE)
    active[0] = False
    return ok
ok = asyncio.run(go())
for r in rows:
    print('%6d x=%.3f y=%.3f h=%7.1f pat=%s pos=%s L=%4d R=%4d' % (r[0], r[1], r[2], r[3], format(r[4], '05b')[::-1], 'None' if r[5] is None else '%.2f' % r[5], r[6], r[7]))
print('found', ok, 'heading', math.degrees(robot.h))
