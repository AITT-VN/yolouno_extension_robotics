"""Sharp, acute corners: the outgoing line leaves the apex at 30-60 deg to the
line coming in (the robot turns 120-150 deg), no rounding. Two failure modes
are counted separately:

  lost     - never found the line again (or ran out of time)
  reversed - found a line, but it was the one it came from, and followed it
             back (heading against the track's direction while on the line)

    python3 acute_tracks.py                         # defaults
    python3 acute_tracks.py '{"gyro": true}'        # robot has an angle sensor
    python3 acute_tracks.py '{"encoders": true}'    # or wheel encoders
    python3 acute_tracks.py '{"db": {"_line_search_speed": 45}}'

Robots: the xBot Evo motor curve (evo_tracks.py) with the sensor 5 or 8 cm
ahead of the axle, and the generic model (dead band 30 %) with 6 or 16 cm.
"""
import sys, os, json, math, asyncio, random
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import linesim as L
from world import Robot, Track
from shims import clock
from constants import *
import drivebase as DB
from evo_tracks import EvoRobot

SET = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}

def vee(interior_deg, seg=0.35, n=4):
    """0.4 m straight, then n sharp corners alternating left/right; the
    outgoing line makes interior_deg with the incoming one"""
    t = Track()
    pts = [(0.0, 0.0), (0.4, 0.0)]
    h = 0.0
    turn = math.radians(180 - interior_deg)
    for i in range(n):
        h += turn if i % 2 == 0 else -turn
        x, y = pts[-1]
        pts.append((x + seg * math.cos(h), y + seg * math.sin(h)))
    t.add(pts)
    return t

TRACKS = [('V60', lambda: vee(60), 16), ('V45', lambda: vee(45), 16), ('V30', lambda: vee(30), 16)]

def seg_dir(track, px, py):
    best, bd = 1e9, (1, 0)
    for (x1, y1, x2, y2) in track.segs:
        dx, dy = x2 - x1, y2 - y1
        l2 = dx * dx + dy * dy
        tt = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / l2))
        d = math.hypot(px - (x1 + tt * dx), py - (y1 + tt * dy))
        if d < best:
            l = math.sqrt(l2); best, bd = d, (dx / l, dy / l)
    return best, bd

def trial(fn, secs, evo, ahead, tau, cruise, slow, seed):
    random.seed(seed); clock.us = 0
    track = fn()
    if evo:
        robot = EvoRobot(track, vmax=0.53, width=0.115, eyes=5, spacing=0.010, sensor_ahead=ahead, deadband=0, tau=tau)
        robot.start_db = (33 + random.random() * 4, 33 + random.random() * 4)
    else:
        robot = Robot(track, vmax=0.8, width=0.15, eyes=5, spacing=0.010, sensor_ahead=ahead, deadband=30, tau=tau)
    clock.physics = robot.step
    db = DB.DriveBase(MODE_2WD, L.FakeMotor(robot, 0, E1), L.FakeMotor(robot, 1, E2))
    db.size(wheel=65, width=115 if evo else 150)
    db.speed(cruise, min_speed=slow)
    sens = L.SimSensor5(robot); sens._analog = SET.get('analog', False)
    db.line_sensor(sens)
    if not SET.get('encoders'):
        db.left_encoder = db.right_encoder = None # like most of these robots
    if SET.get('gyro'):
        db.angle_sensor(L.FakeAngle(robot))
    for k, v in SET.get('evo' if evo else 'gen', {}).items():
        setattr(db, k, v)
    for k, v in SET.get('db', {}).items():
        setattr(db, k, v)
    n = len(track.segs)
    st = dict(best=0.0, t_done=None, acc=0, rev=0.0)
    step = robot.step
    def physics(dt):
        step(dt); st['acc'] += dt
        if st['acc'] < 5000: return
        st['acc'] = 0
        sx = robot.x + robot.sensor_ahead * math.cos(robot.h); sy = robot.y + robot.sensor_ahead * math.sin(robot.h)
        d, (ux, uy) = seg_dir(track, sx, sy)
        if d < 0.012 and st['t_done'] is None:
            # moving along the line against the track's direction
            v = (robot.vel[0] + robot.vel[1]) / 2
            if v > 0.05 and math.cos(robot.h) * ux + math.sin(robot.h) * uy < -0.7:
                st['rev'] += 0.005
        if d < 0.02:
            pr = track.progress(sx, sy) / n
            if st['best'] < pr < st['best'] + 0.2: st['best'] = pr
            if st['best'] >= 0.95 and st['t_done'] is None: st['t_done'] = clock.us / 1e6
    clock.physics = physics
    asyncio.run(db.follow_line_by_time(secs))
    return st

done = tot = rev = 0; tsum = 0.0
for evo, arms, motor in ((True, (0.05, 0.08), 'evo'), (False, (0.06, 0.16), 'gen')):
    for tau in (0.06, 0.12):
        for ahead in arms:
            cs = SET.get('speeds', {'evo': [50, 36], 'gen': [60, 40]})[motor]
            row = []
            for name, fn, secs in TRACKS:
                for seed in (1, 2):
                    st = trial(fn, secs, evo, ahead, tau, cs[0], cs[1], seed)
                    tot += 1
                    ok = st['t_done'] is not None
                    r = st['rev'] > 0.3
                    done += ok; rev += r
                    tsum += st['t_done'] if ok else secs
                    row.append('%s:%s' % (name, ('%4.1fs' % st['t_done']) if ok else ('REV' if r else 'p%2d%%' % (100 * st['best']))))
            print('%s tau=%.2f arm=%2dcm | %s' % (motor, tau, ahead * 100, ' '.join(row)))
print('TOTAL finished %d/%d, reversed %d, time %.1f' % (done, tot, rev, tsum))
