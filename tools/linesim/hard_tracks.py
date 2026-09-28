"""Harder line-following scenarios for the 5-channel array (10 mm eye pitch).

    python3 hard_tracks.py                      # the default drivebase.py
    python3 hard_tracks.py '{"kp": 0.8}'        # try line_pid / line_slowdown values
    python3 hard_tracks.py '{"_line_x": 1}'     # or any DriveBase attribute starting with _line

Robots: sensor 6 cm or 16 cm ahead of the axle (a long-arm 3-wheel robot),
motor time constant 0.06 s or 0.15 s (heavier robot / slower gear motors),
dead band 30 % with min_speed 40, cruise 60 and 80.

Tracks: the demo track, a 90 deg zigzag, a 120 deg (acute) zigzag, tight waves
(r = 8 cm) and a notch of 6 cm steps - shorter than a long sensor arm.

Scored on the furthest point the sensor reached while still on the line (a run
that finishes and then searches past the end of the line still counts as
finished), the time to reach 97 % of the track, and the largest distance from
the sensor to the line before that. Baseline with the defaults of 2026-09-28:
32/40 finished; the notch and the 120 deg zigzag with slow motors fail.
"""
import sys, os, math, asyncio, random, json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
sys.path.insert(0, os.path.dirname(__file__))
import linesim as L
from world import Robot, Track, arc, demo_track
from shims import clock
from constants import *
import drivebase as DB

SETTINGS = {}

def zigzag_track(turn_deg=90, seg=0.2):
    """0.4 m straight, 4 corners turning turn_deg, 0.4 m straight."""
    t = Track()
    pts = [(0, 0), (0.4, 0)]
    x, y = 0.4, 0.0
    a = math.radians(turn_deg / 2)
    for i in range(4):
        x += seg * math.cos(a)
        y += seg * math.sin(a) * (1 if i % 2 == 0 else -1)
        pts.append((x, y))
    pts.append((x + 0.4, y))
    t.add(pts)
    return t

def waves_track(r=0.08):
    """0.3 m straight, 4 half circles of radius r turning alternately, 0.3 m straight."""
    t = Track()
    pts = [(0, 0), (0.3, 0)]
    x = 0.3
    for i in range(4):
        cx = x + r
        pts += arc(cx, 0.0, r, math.pi, 0 if i % 2 == 0 else 2 * math.pi)[1:]
        x += 2 * r
    pts.append((x + 0.3, 0.0))
    t.add(pts)
    return t

def notch_track(step=0.06):
    """0.4 m straight, then 90 deg steps of `step` (up, along, down, along...), 0.4 m straight."""
    t = Track()
    pts = [(0, 0), (0.4, 0)]
    x, y = 0.4, 0.0
    for dx, dy in ((0, step), (step, 0), (0, -step), (step, 0), (0, step), (step, 0)):
        x += dx
        y += dy
        pts.append((x, y))
    pts.append((x + 0.4, y))
    t.add(pts)
    return t

TRACKS = [('demo', demo_track, 14), ('zig90', lambda: zigzag_track(90, 0.2), 12),
          ('zig120', lambda: zigzag_track(120, 0.18), 12), ('wave8', waves_track, 12), ('notch', notch_track, 12)]

def make(track, ahead, deadband, cruise, slow, tau, spacing=0.010, vmax=0.8, width=0.15, seed=1):
    random.seed(seed)
    clock.us = 0
    robot = Robot(track, vmax=vmax, width=width, eyes=5, spacing=spacing, sensor_ahead=ahead,
                  deadband=deadband, tau=tau)
    clock.physics = robot.step
    db = DB.DriveBase(MODE_2WD, L.FakeMotor(robot, 0, E1), L.FakeMotor(robot, 1, E2))
    db.size(wheel=65, width=int(width * 1000))
    db.speed(cruise, min_speed=slow)
    db.line_sensor(L.SimSensor5(robot))
    if 'kp' in SETTINGS or 'kd' in SETTINGS:
        db.line_pid(Kp=SETTINGS.get('kp'), Kd=SETTINGS.get('kd'))
    if 'slowdown' in SETTINGS:
        db.line_slowdown(SETTINGS['slowdown'])
    for k, v in SETTINGS.items():
        if k.startswith('_line'):
            setattr(db, k, v)
    return robot, db

def trial(track_fn, secs, **kw):
    track = track_fn()
    robot, db = make(track, **kw)
    n = len(track.segs)
    st = dict(best=0.0, t_done=None, err=0.0, acc=0)
    step = robot.step
    def physics(dt):
        step(dt)
        st['acc'] += dt
        if st['acc'] < 5000:
            return
        st['acc'] = 0
        sx = robot.x + robot.sensor_ahead * math.cos(robot.h)
        sy = robot.y + robot.sensor_ahead * math.sin(robot.h)
        d = track.dist(sx, sy, bars=False)
        if st['t_done'] is None:
            st['err'] = max(st['err'], d)
        if d < 0.02:
            pr = track.progress(sx, sy) / n
            if st['best'] < pr < st['best'] + 0.2:   # no jumping to another part of the track
                st['best'] = pr
            if st['best'] >= 0.97 and st['t_done'] is None:
                st['t_done'] = clock.us / 1e6
    clock.physics = physics
    asyncio.run(db.follow_line_by_time(secs))
    return dict(done=st['t_done'] is not None, prog=round(100 * st['best']), t=st['t_done'],
                max_err=round(st['err'] * 1000))

if __name__ == '__main__':
    if len(sys.argv) > 1:
        SETTINGS.update(json.loads(sys.argv[1]))
    done = total = 0
    time_sum = 0.0
    for tau in (0.06, 0.15):
        for ahead in (0.06, 0.16):
            for cruise, slow in ((60, 40), (80, 40)):
                row = []
                for name, fn, secs in TRACKS:
                    r = trial(fn, secs, ahead=ahead, deadband=30, cruise=cruise, slow=slow, tau=tau)
                    total += 1
                    done += r['done']
                    time_sum += r['t'] if r['done'] else secs
                    row.append('%s:%s e%3d' % (name, ('%4.1fs' % r['t']) if r['done'] else ('p%3d%%' % r['prog']), r['max_err']))
                print('tau=%.2f arm=%2dcm %d/%d | %s' % (tau, ahead * 100, cruise, slow, ' | '.join(row)))
    print('TOTAL finished %d/%d, time sum %.1f s' % (done, total, time_sum))
