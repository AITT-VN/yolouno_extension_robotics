import asyncio
from machine import PWM, Pin
from utility import *
from mdv2 import *

class Servo:
    def __init__(self, *args):
        if len(args) == 3:
            if isinstance(args[0], MotorDriverV2):
                self._driver = args[0]
                self._port = int(args[1])
                self._max_angle = int(args[2])
            else:
                raise ValueError('Invalid servo argument')

        elif len(args) == 2:
            if isinstance(args[0], int):
                self._port = PWM(Pin(int(args[0])))
                self._port.freq(50)
                self._driver = None
                self._max_angle = int(args[1])
            else:
                raise ValueError('Invalid servo argument')

        self._current_angle = None
        self._limit_min = 0
        self._limit_max = self._max_angle

    def limit(self, min, max):
        if min < 0 or min > max or max > self._max_angle:
            raise ValueError('Invalid servo limit value')

        self._limit_min = min
        self._limit_max = max

    def _angle_to_pulse(self, angle):
        pulse = 25 + int((angle/self._max_angle)*100)
        return pulse

    def angle(self, angle):
        angle = int(max(min(self._limit_max, angle), self._limit_min))

        if self._driver:
            self._driver.set_servo(self._port, angle, self._max_angle)
        else:
            pulse = self._angle_to_pulse(angle, self._max_angle)
            self._port.duty(pulse)

        self._current_angle = angle

    async def run_angle(self, angle, speed=100):
        if speed == 100 or self._current_angle == None:
            self.angle(angle)
        else:
            delay = translate(speed, 0, 100, 10, 0)
            if self._current_angle > angle:
                for i in range(self._current_angle, angle, -1):
                    self.angle(i)
                    await asyncio.sleep_ms(delay)
            else:
                for i in range(self._current_angle, angle, 1):
                    self.angle(i)
                    await asyncio.sleep_ms(delay)
    
    async def run_steps(self, steps, speed=100):
        if self._current_angle == None:
            self._current_angle = 0

        new_pos = self._current_angle + steps
        await self.run_angle(new_pos, speed)

    def spin(self, speed):
        speed = int(max(min(100, speed), -100))
        angle = 90 - (speed/100)*90
        self.angle(angle)

