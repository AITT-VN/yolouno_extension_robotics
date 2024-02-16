import asyncio
from machine import PWM, Pin
from utility import *
from mdv2 import *

class Servo:
    def __init__(self, *args):
        if len(args) == 2:
            if isinstance(args[0], MotorDriverV2):
                self._driver = args[0]
                self._port = int(args[1])
            else:
                raise ValueError('Invalid servo argument')

        elif len(args) == 1:
            if isinstance(args[0], int):
                self._port = PWM(Pin(int(args[0])))
                self._port.freq(50)
                self._driver = None
            else:
                raise ValueError('Invalid servo argument')

        self._current_angle = None

    def _angle_to_pulse(self, angle, max_angle=180):
        pulse = 25 + int((angle/max_angle)*100)
        return pulse

    def angle(self, angle, max_angle=180):
        angle = int(max(min(max_angle, angle), 0))

        if self._driver:
            self._driver.set_servo(self._port, angle, max_angle)
        else:
            pulse = self._angle_to_pulse(angle, max_angle)
            self._port.duty(pulse)

        self._current_angle = angle

    async def run_angle(self, angle, speed=100, max_angle=180):
        if speed == 100 or self._current_angle == None:
            self.angle(angle, max_angle)
        else:
            delay = translate(speed, 0, 100, 50, 0)
            if self._current_angle > angle:
                for i in range(self._current_angle, angle, -1):
                    self.angle(i, max_angle)
                    await asyncio.sleep_ms(delay)
            else:
                for i in range(self._current_angle, angle, 1):
                    self.angle(i, max_angle)
                    await asyncio.sleep_ms(delay)

    def spin(self, speed):
        speed = int(max(min(100, speed), -100))
        angle = 90 - (speed/100)*90
        self.angle(angle)

''' Test code
md = MotorDriverV2()
s1 = Servo(md, 3)
s1.angle(0)
run_loop(s1.run_angle(180, 70))
sleep(2)
run_loop(s1.run_angle(0, 70))
'''
md = MotorDriverV2()
s1 = Servo(md, MDV2_S1)
