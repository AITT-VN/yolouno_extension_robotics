import sys, itertools
from linesim import trial
vmax = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
print('vmax', vmax)
best = []
for kp, kd, sd in itertools.product((0.8, 1.2, 1.6, 2.2, 3.0), (0.02, 0.04, 0.07, 0.1, 0.15), (0.8, 1.2, 1.8)):
    res = []
    for cruise in (80, 100):
        r = trial('new', eyes=5, analog=True, cruise=cruise, slow=40, kp=kp, kd=kd, slowdown=sd, vmax=vmax)
        res.append(r)
    ok = all(r['ok'] for r in res)
    print('kp=%.1f kd=%.2f sd=%.1f | 80: ok=%-5s t=%5.2f max=%5.1f rms=%4.1f | 100: ok=%-5s t=%5.2f max=%5.1f rms=%4.1f' % (
        kp, kd, sd, res[0]['ok'], res[0]['t_end'], res[0]['max_err_mm'], res[0]['rms_mm'], res[1]['ok'], res[1]['t_end'], res[1]['max_err_mm'], res[1]['rms_mm']))
