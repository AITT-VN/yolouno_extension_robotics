"""Differential-drive robot + line track + reflectance array model."""
import math, random
from shims import clock

class Track:
    """Polyline segments (m). Distance queries give the nearest point."""
    def __init__(self):
        self.segs = []      # (x1,y1,x2,y2)
        self.bars = []      # crossing bars: same format
        self.end = None
    def add(self, pts):
        for a, b in zip(pts, pts[1:]):
            self.segs.append((a[0], a[1], b[0], b[1]))
        self.end = pts[-1]
    def add_bar(self, x, y, heading, half=0.1):
        c, s = math.cos(heading + math.pi/2), math.sin(heading + math.pi/2)
        self.bars.append((x - c*half, y - s*half, x + c*half, y + s*half))
    @staticmethod
    def _dist(px, py, seg):
        x1, y1, x2, y2 = seg
        dx, dy = x2 - x1, y2 - y1
        l2 = dx*dx + dy*dy
        t = 0 if l2 == 0 else max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / l2))
        cx, cy = x1 + t*dx, y1 + t*dy
        return math.hypot(px - cx, py - cy)
    def dist(self, px, py, bars=True):
        d = min(self._dist(px, py, s) for s in self.segs)
        if bars and self.bars:
            d = min(d, min(self._dist(px, py, s) for s in self.bars))
        return d
    def progress(self, px, py):
        """fraction of the track length passed (nearest segment index based)"""
        best, bi, bt = 1e9, 0, 0
        for i, (x1, y1, x2, y2) in enumerate(self.segs):
            dx, dy = x2 - x1, y2 - y1
            l2 = dx*dx + dy*dy
            t = 0 if l2 == 0 else max(0, min(1, ((px - x1)*dx + (py - y1)*dy) / l2))
            d = math.hypot(px - (x1 + t*dx), py - (y1 + t*dy))
            if d < best:
                best, bi, bt = d, i, t
        return bi + bt

def arc(cx, cy, r, a0, a1, n=24):
    return [(cx + r*math.cos(a0 + (a1 - a0)*i/n), cy + r*math.sin(a0 + (a1 - a0)*i/n)) for i in range(n + 1)]

def t_track():
    """0.7 m straight ending on a T bar; branches 0.5 m to the left and right (right one is followed)."""
    t = Track()
    t.add([(0, 0), (0.7, 0)])
    t.add_bar(0.7, 0, 0.0, half=0.5)     # the T bar doubles as both branches
    t.cross_x = 0.7
    t.end = (0.7, -0.5)                  # right branch end (turning right = heading -y)
    t.segs.append((0.7, 0, 0.7, -0.5))   # right branch is also part of the track for error stats
    t.segs.append((0.7, 0, 0.7, 0.5))
    return t

def demo_track():
    """1 m straight, 90 deg left curve r=0.3, 0.4 m straight, 180 deg right curve r=0.15,
    0.4 m straight, sharp 90 deg right corner, 0.5 m straight with a crossing bar, 0.4 m to the end."""
    t = Track()
    pts = [(0, 0), (1.0, 0)]
    pts += arc(1.0, 0.3, 0.3, -math.pi/2, 0)[1:]            # ends at (1.3, 0.3) heading +y
    pts += [(1.3, 0.7)]
    pts += arc(1.45, 0.7, 0.15, math.pi, 0)[1:]              # right 180: ends (1.6, 0.7) heading -y
    pts += [(1.6, 0.3)]                                      # straight down
    pts += [(2.1, 0.3)]                                      # sharp 90 left... corner at (1.6,0.3): from heading -y to +x = left turn
    pts += [(2.5, 0.3)]
    t.add(pts)
    t.add_bar(1.9, 0.3, 0.0)                                 # crossing bar at x=1.9
    t.cross_x = 1.9
    return t

class Robot:
    """2-wheel differential drive. speeds in % -> wheel m/s with first-order lag."""
    def __init__(self, track, vmax=0.8, width=0.15, tau=0.06, deadband=6, sensor_ahead=0.06,
                 eyes=5, spacing=0.015, spot=0.006, line_width=0.019, noise=25, analog_black_high=True):
        self.track = track
        self.vmax, self.width, self.tau, self.deadband = vmax, width, tau, deadband
        self.sensor_ahead, self.eyes, self.spacing, self.spot, self.line_width = sensor_ahead, eyes, spacing, spot, line_width
        self.noise, self.black_high = noise, analog_black_high
        self.x, self.y, self.h = 0.0, 0.0, 0.0
        self.cmd = [0.0, 0.0]      # left, right %
        self.vel = [0.0, 0.0]      # actual wheel m/s
        self.wheel = [0.0, 0.0]    # wheel travel m (signed)
        self.max_err = 0.0
        self.max_err_smooth = 0.0
        self.err_sum2 = 0.0; self.err_n = 0
        self.dist = 0.0
        self.log = []
    def set_cmd(self, side, v):
        v = max(-100, min(100, v))
        self.cmd[side] = 0.0 if abs(v) < self.deadband else v
    def step(self, dt_us):
        dt = dt_us / 1e6
        for i in range(2):
            target = self.cmd[i] / 100 * self.vmax
            self.vel[i] += (target - self.vel[i]) * min(1, dt / self.tau)
        self.wheel[0] += self.vel[0] * dt
        self.wheel[1] += self.vel[1] * dt
        v = (self.vel[0] + self.vel[1]) / 2
        w = (self.vel[1] - self.vel[0]) / self.width
        self.x += v * math.cos(self.h) * dt
        self.y += v * math.sin(self.h) * dt
        self.h += w * dt
        self.dist += abs(v) * dt
        e = self.track.dist(self.x, self.y, bars=False)
        self.max_err = max(self.max_err, e)
        self.err_sum2 += e*e; self.err_n += 1
        # smooth sections only: leave out the sharp corner at (1.6, 0.3)
        if not (1.45 < self.x < 1.75 and 0.15 < self.y < 0.45):
            self.max_err_smooth = max(self.max_err_smooth, e)
    # ---- sensor model ----
    def eye_xy(self, i):
        off = (i - (self.eyes - 1) / 2) * self.spacing   # + = right of centre
        # sensor frame: ahead along heading, right = heading - 90deg
        cx = self.x + self.sensor_ahead * math.cos(self.h)
        cy = self.y + self.sensor_ahead * math.sin(self.h)
        rx, ry = math.sin(self.h), -math.cos(self.h)
        return cx + off * rx, cy + off * ry
    def coverage(self, i):
        ex, ey = self.eye_xy(i)
        d = self.track.dist(ex, ey)
        c = (self.line_width / 2 + self.spot / 2 - d) / self.spot
        return max(0.0, min(1.0, c))
    def bits(self):
        b = 0
        for i in range(self.eyes):
            if self.coverage(i) > 0.5:
                b |= 1 << i
        return b
    def raw(self):
        out = []
        for i in range(self.eyes):
            c = self.coverage(i)
            v = 600 + 3000 * c if self.black_high else 3600 - 3000 * c
            out.append(int(max(0, min(4095, v + random.uniform(-self.noise, self.noise)))))
        return out
    def rms_err(self):
        return math.sqrt(self.err_sum2 / max(1, self.err_n))
