import sys, time
for m in ('line_sensor', 'constants'):
    sys.modules.pop(m, None)
from constants import *
from line_sensor import *
s = LineSensorI2C()
s.mode('analog')
dig = {}; ana = {}; rows = []
t0 = time.ticks_ms(); n = 0; last = None
while time.ticks_diff(time.ticks_ms(), t0) < 10000:
    p = s.update(); n += 1
    d = s._read_bits()
    dig[d] = dig.get(d, 0) + 1
    k = (s.pattern(), None if p is None else round(p, 1))
    ana[k] = ana.get(k, 0) + 1
    if k != last:
        rows.append((time.ticks_diff(time.ticks_ms(), t0), d, s.pattern(), k[1], s._raw))
        last = k
    time.sleep_ms(15)
def b(x): return ''.join(str((x >> i) & 1) for i in range(5))
print('updates', n)
print('STM32 digital patterns:', sorted(((b(k), v) for k, v in dig.items()), key=lambda kv: -kv[1])[:10])
print('analog (pattern,pos):', sorted(((b(k[0]), k[1], v) for k, v in ana.items()), key=lambda kv: -kv[2])[:12])
print('black_high', s._line_high, 'votes', s._polarity_votes, 'calibrated', s.calibrated())
print('cal min', s._cal_min, 'max', s._cal_max)
for r in rows[:40]:
    print('%5d ms stm=%s ana=%s pos=%s raw=%s' % (r[0], b(r[1]), b(r[2]), r[3], r[4]))
if s.calibrated():
    s.save_calibration(); print('saved', open('/line_calib.json').read())
