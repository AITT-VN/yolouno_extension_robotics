import sys, time
for m in ('line_sensor', 'constants'):
    sys.modules.pop(m, None)
from constants import *
from line_sensor import *
s = LineSensorI2C()
white = [0]*5; n = 0
rows = []
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 9000:
    r = s.read_raw()
    rows.append(r)
    for i in range(5):
        if r[i] > white[i]: white[i] = r[i]
    time.sleep_ms(20)
best = [0]*5   # deepest dip below that eye's own white
for r in rows:
    for i in range(5):
        d = white[i] - r[i]
        if d > best[i]: best[i] = d
print('samples', len(rows))
print('white per eye ', white)
print('deepest dip   ', best)
noise = [0]*5
for r in rows:
    for i in range(5):
        d = white[i] - r[i]
        if d < best[i]*0.15 and d > noise[i]: noise[i] = d
print('background wobble', noise)
