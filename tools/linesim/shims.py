"""Stub modules so the real drivebase.py / line_sensor.py import under CPython."""
import sys, types, time as _time, asyncio as _asyncio

# ---- simulated clock (microseconds) ----
class Clock:
    us = 0
    physics = None   # callable(dt_us) advancing the world
    def advance(self, us):
        step = 1000
        while us > 0:
            d = min(step, us)
            self.us += d
            if self.physics:
                self.physics(d)
            us -= d
clock = Clock()

_time.ticks_ms = lambda: clock.us // 1000
_time.ticks_us = lambda: clock.us
_time.ticks_diff = lambda a, b: a - b
_time.ticks_add = lambda a, b: a + b
_time.sleep_ms = lambda ms: clock.advance(ms * 1000)

_orig_sleep = _asyncio.sleep
COMPUTE_US = 2500   # loop overhead on the ESP32 between two sleeps

async def sleep_ms(ms):
    clock.advance(ms * 1000 + COMPUTE_US)
    await _orig_sleep(0)
async def sleep(s):
    await sleep_ms(int(s * 1000))
_asyncio.sleep_ms = sleep_ms
_asyncio.sleep = sleep

def mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m

class Pin:
    IN = 0; OUT = 1
    def __init__(self, *a, **k): self._v = 0
    def value(self, v=None):
        if v is None: return self._v
        self._v = v
class SoftI2C:
    def __init__(self, *a, **k): pass
    def scan(self): return []
mod('machine', Pin=Pin, SoftI2C=SoftI2C)
mod('micropython', const=lambda x: x)
mod('utility', asleep_ms=sleep_ms)
mod('setting', SCL_PIN=1, SDA_PIN=2, BOOT_PIN=0)
mod('pcf8574', PCF8574=type('PCF8574', (), {'__init__': lambda self, *a: (_ for _ in ()).throw(OSError())}))
mod('ble', )
mod('yolo_uno', )
mod('gamepad', Gamepad=object)
mod('vector3d', )
