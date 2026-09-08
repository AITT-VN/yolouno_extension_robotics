
import asyncio, math
from linesim import *
from world import t_track
for gyro in (False, True):
    for eyes, analog in ((4, False), (5, False), (5, True)):
        for cruise in (60, 100):
            track, robot, db, sensor = make(track=t_track(), eyes=eyes, analog=analog, cruise=cruise, slow=40, gyro=gyro, offset=60)
            r = asyncio.run(run_T(track, robot, db, sensor))
            print('gyro=%-5s eyes=%d analog=%-5s cruise=%3d | cross=%-5s axle_dx=%6.1fmm t=%.2fs | turn=%-5s heading=%7.1f pos=%s t=%.2fs | end=%-5s at (%.3f,%.3f) t_total=%.2fs' % (
                gyro, eyes, analog, cruise, r['cross'], r['axle_dx_mm'], r['t_cross'], r['turn'], r['heading'], 'None' if r['pos_after'] is None else '%.2f' % r['pos_after'], r['t_turn'], r['end'], r['end_x'], r['end_y'], r['t_end']))
