import asyncio, math
from linesim import *
import drivebase as DB
DB.TUNING_FILE = __import__('os').path.join(__import__('os').path.dirname(__file__), 'tuning.json')
for gyro in (False, True):
    track, robot, db, sensor = make(eyes=5, cruise=60, slow=40, gyro=gyro)
    db.debug = True
    x0 = robot.x
    async def go():
        await db.straight(40, 6, CM, BRAKE)
    asyncio.run(go())
    print('gyro', gyro, 'moved %.1f mm' % ((robot.x - x0) * 1000), 'distance()', db.distance(), 'wheel', robot.wheel)
