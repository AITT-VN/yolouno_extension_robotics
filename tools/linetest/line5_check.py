import sys, time
for m in ('line_sensor', 'constants'):
    sys.modules.pop(m, None)
from constants import *
from line_sensor import *
s = LineSensorI2C(); s.mode('analog')
seen = {}
for _ in range(60):
    p = s.update()
    k = (''.join(str((s.pattern() >> i) & 1) for i in range(5)), None if p is None else round(p, 2))
    seen[k] = seen.get(k, 0) + 1
    time.sleep_ms(20)
print('60 samples:', sorted(seen.items(), key=lambda kv: -kv[1])[:6])
print('raw', s._raw, 'bg', [round(b) for b in (s._bg or [])], 'sig', s._sig)
