"""Plain DC motors with a measured duty curve: the xBot Evo (ORC hub, 5-eye
array) on 90 deg corners rounded to 2-4 cm, a U of three corners and the
demo track.

    python3 evo_tracks.py
    python3 evo_tracks.py '{"speeds": [[50, 36]], "db": {"_line_search_speed": 45,
                            "_line_min_wheel": 34, "_line_outer_cap": 55}}'
    python3 evo_tracks.py '{"analog": true}'

The wheels follow what the hub's gyro measured on the robot: from standstill
they start at 33-37 % duty (a little different per wheel), once turning they
keep going down to 24 %, speed grows linearly to 60 % and hardly at all
above. Defaults (2026-10-03, cruise 50 / min 36): 25/32 finished; with the
settings in the second example, which the xBot Evo firmware uses, 32/32.
"""
import sys, os, json, math, asyncio, random
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import linesim as L
from world import Robot, Track, arc, demo_track
from shims import clock
from constants import *
import drivebase as DB

class EvoRobot(Robot):
    """wheel speed vs duty measured on the floor: starts at ~34 %, keeps
    turning down to ~24 %, linear to 60 %, flat above"""
    VSAT = 0.53
    def set_cmd(self, side, v):
        self.cmd[side] = max(-100, min(100, v))
    def step(self, dt_us):
        dt = dt_us / 1e6
        for i in range(2):
            c = self.cmd[i]; a = abs(c)
            moving = abs(self.vel[i]) > 0.03
            if a < (24 if moving else self.start_db[i]):
                tgt = 0.0
            else:
                tgt = self.VSAT * min(1.0, max(0.0, (a - 20) / 40)) * (1 if c > 0 else -1)
            self.vel[i] += (tgt - self.vel[i]) * min(1, dt / self.tau)
        self.wheel[0] += self.vel[0] * dt; self.wheel[1] += self.vel[1] * dt
        v = (self.vel[0] + self.vel[1]) / 2
        w = (self.vel[1] - self.vel[0]) / self.width
        self.x += v * math.cos(self.h) * dt; self.y += v * math.sin(self.h) * dt; self.h += w * dt

def corner_pts(pts, r):
    """polyline with every inner corner rounded to radius r"""
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        (x0, y0), (x1, y1), (x2, y2) = pts[i-1], pts[i], pts[i+1]
        d1 = math.hypot(x1-x0, y1-y0); d2 = math.hypot(x2-x1, y2-y1)
        u1 = ((x1-x0)/d1, (y1-y0)/d1); u2 = ((x2-x1)/d2, (y2-y1)/d2)
        a = (x1 - u1[0]*r, y1 - u1[1]*r); b = (x1 + u2[0]*r, y1 + u2[1]*r)
        for k in range(7):  # quadratic bezier approx of the fillet
            t = k / 6
            out.append(((1-t)**2*a[0] + 2*(1-t)*t*x1 + t*t*b[0], (1-t)**2*a[1] + 2*(1-t)*t*y1 + t*t*b[1]))
    out.append(pts[-1])
    return out

def zig(r, seg=0.3):
    t = Track(); pts = [(0, 0), (0.4, 0)]
    x, y = 0.4, 0.0
    for i, (dx, dy) in enumerate(((0, seg), (seg, 0), (0, -seg), (seg, 0), (0, seg), (seg, 0))):
        x += dx; y += dy; pts.append((x, y))
    pts.append((x + 0.3, y)); t.add(corner_pts(pts, r)); return t

def square(r, side=0.5):
    t = Track(); pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0.15)]
    t.add(corner_pts(pts, r)); return t

TRACKS = [('zig r2', lambda: zig(0.02), 14), ('zig r4', lambda: zig(0.04), 14),
          ('square r3', lambda: square(0.03), 14), ('demo', demo_track, 14)]

SET = json.loads(sys.argv[1]) if __name__ == '__main__' and len(sys.argv) > 1 else {}

def trial(track_fn, secs, ahead, tau, cruise, slow, seed):
    random.seed(seed); clock.us = 0
    track = track_fn()
    robot = EvoRobot(track, vmax=0.53, width=0.115, eyes=5, spacing=0.010, sensor_ahead=ahead, deadband=0, tau=tau)
    robot.start_db = (33 + random.random() * 4, 33 + random.random() * 4)
    clock.physics = robot.step
    db = DB.DriveBase(MODE_2WD, L.FakeMotor(robot, 0, E1), L.FakeMotor(robot, 1, E2))
    db.size(wheel=65, width=115)
    db.speed(cruise, min_speed=slow)
    sens = L.SimSensor5(robot); sens._analog = SET.get('analog', False)
    db.line_sensor(sens)
    for k, v in SET.get('db', {}).items():
        setattr(db, k, v)
    n = len(track.segs)
    st = dict(best=0.0, t_done=None, err=0.0, acc=0)
    step = robot.step
    def physics(dt):
        step(dt); st['acc'] += dt
        if st['acc'] < 5000: return
        st['acc'] = 0
        sx = robot.x + robot.sensor_ahead * math.cos(robot.h); sy = robot.y + robot.sensor_ahead * math.sin(robot.h)
        d = track.dist(sx, sy, bars=False)
        if st['t_done'] is None: st['err'] = max(st['err'], d)
        if d < 0.02:
            pr = track.progress(sx, sy) / n
            if st['best'] < pr < st['best'] + 0.2: st['best'] = pr
            if st['best'] >= 0.95 and st['t_done'] is None: st['t_done'] = clock.us / 1e6
    clock.physics = physics
    asyncio.run(db.follow_line_by_time(secs))
    return st

if __name__ == '__main__':
    tot = done = 0; tsum = 0; esum = 0
    for tau in (0.06, 0.12):
        for ahead in (0.05, 0.08):
            for cruise, slow in SET.get('speeds', [[50, 36]]):
                row = []
                for name, fn, secs in TRACKS:
                    for seed in (1, 2):
                        st = trial(fn, secs, ahead, tau, cruise, slow, seed)
                        tot += 1; ok = st['t_done'] is not None; done += ok
                        tsum += st['t_done'] if ok else secs; esum += st['err']
                        row.append('%s:%s' % (name, ('%4.1fs' % st['t_done']) if ok else ('p%3d%%' % (100*st['best']))))
                print('tau=%.2f arm=%dcm %d/%d | %s' % (tau, ahead*100, cruise, slow, ' '.join(row)))
    print('TOTAL %d/%d time %.1f maxerr_avg %.0f mm' % (done, tot, tsum, 1000*esum/tot))
