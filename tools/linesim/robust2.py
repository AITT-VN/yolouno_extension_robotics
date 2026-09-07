import itertools
from linesim import trial
for vmax, ahead in itertools.product((0.5, 1.1), (0.04, 0.09)):
    for eyes, analog in ((4, False), (5, False), (5, True)):
        res = [trial('new', eyes=eyes, analog=analog, cruise=c, slow=40, vmax=vmax, sensor_ahead=ahead, tau=0.08) for c in (80, 100)]
        print('vmax=%.1f ahead=%.2f eyes=%d analog=%-5s | ' % (vmax, ahead, eyes, analog) + ' | '.join(
            '%3d: %s t=%5.2f dx=%5.1f smooth=%4.1f' % (c, 'OK ' if r['ok'] else 'BAD', r['t_end'], r['cross_err_mm'], r['smooth_mm']) for c, r in zip((80, 100), res)), flush=True)
