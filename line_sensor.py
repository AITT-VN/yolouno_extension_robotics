from machine import Pin, SoftI2C
from utility import *
from setting import *
from micropython import const
import pcf8574
import asyncio
import time
from constants import *

# where the analog calibration of the 5-channel array is kept between power cycles
LINE_CALIB_FILE = '/line_calib.json'

# analog: a channel needs at least this much black/white contrast (12-bit counts)
# before its readings are trusted for the line position
_MIN_RANGE = const(150)
# analog: normalised readings below this are treated as "no line here"
_NOISE_FLOOR = 0.15


class LineSensor:
    '''
        Base class for every line sensor array.

        All sensors report the same things after update():
            pattern()  - bitmask of sensors seeing the line, bit 0 = S1 (leftmost)
            count()    - how many sensors see the line
            position() - where the line is under the array, -1.0 (under the
                         leftmost sensor) .. +1.0 (rightmost), 0 = centred,
                         None when no sensor sees it
            lost(), cross() - no line at all / line under (almost) every sensor

        Position sign: positive means the line is to the RIGHT of the sensor
        centre, so the robot has to steer right to get back on it.

        The legacy check() keeps the LINE_* states used by older programs;
        there the sign is the robot's side of the line, so LINE_RIGHT means
        the robot is right of the line (line on its left).

        Sensors read 1 for black by default; pass inverted=True for modules
        that output 0 on black.
    '''
    def __init__(self, n_sensors, inverted=False):
        self.n_sensors = n_sensors
        self._inverted = inverted
        # sensor positions across the array, -1 (S1) .. +1 (last)
        if n_sensors > 1:
            self._weights = tuple(-1 + 2*i/(n_sensors - 1) for i in range(n_sensors))
        else:
            self._weights = (0,)
        self._all = (1 << n_sensors) - 1
        self._pattern = 0
        self._count = 0
        self._pos = None
        self._last_pos = 0 # last position while the line was seen

    ######################## To implement in subclasses #####################

    '''
        Reads all sensors in one go and returns a bitmask, bit i = sensor i
        (S1 = bit 0 = leftmost), 1 = on the line, 0 = off. Raw hardware
        polarity; inversion is applied here in the base class.
    '''
    def _read_bits(self):
        return 0

    ######################## Reading #####################

    '''
        Reads the sensors once and caches pattern, count and position.
        Call it once per control loop, then use the getters.

        Returns: the line position, or None when the line is lost
    '''
    def update(self):
        bits = self._read_bits()
        if self._inverted:
            bits = ~bits & self._all
        self._set_pattern(bits)
        return self._pos

    def _set_pattern(self, bits):
        self._pattern = bits
        n = 0
        acc = 0
        for i in range(self.n_sensors):
            if bits & (1 << i):
                acc += self._weights[i]
                n += 1
        self._count = n
        if n == 0:
            self._pos = None
        else:
            self._pos = acc / n
            self._last_pos = self._pos

    '''
        Read status of a specific sensor (0/1), or all of them as a tuple
        (S1 first). Reads the hardware.
    '''
    def read(self, index=None):
        self.update()
        if index is not None:
            return (self._pattern >> index) & 1
        return tuple((self._pattern >> i) & 1 for i in range(self.n_sensors))

    '''
        Line position under the array, -1 (far left) .. +1 (far right),
        None when the line is lost. Reads the hardware.
    '''
    def position(self):
        return self.update()

    '''
        Same as position() as a whole number -100 .. 100, 0 when the line
        is lost. Handy for students' own control loops.
    '''
    def position_percent(self):
        pos = self.update()
        return 0 if pos is None else int(pos * 100)

    # getters on the values cached by the last update(), no I2C traffic
    def pattern(self):
        return self._pattern

    def get_pattern(self): # develop-branch name
        return self._pattern

    def count(self):
        return self._count

    def last_position(self):
        return self._last_pos

    def lost(self):
        return self._count == 0

    '''
        True when the line is under (almost) every sensor: a crossing line
        or the start/finish bar.
    '''
    def cross(self):
        n = self.n_sensors
        if n <= 3:
            return self._count == n
        # both outer eyes on it, at most one inner eye missing (dirt, gap in
        # the print). A sharp corner lights one side only, so it is not a cross
        outer = (self._pattern & 1) and (self._pattern >> (n - 1)) & 1
        return bool(outer) and self._count >= n - 1

    '''
        Line position scaled to -2000..2000 (develop-branch API); when the
        line is lost, the side it was last seen on.
    '''
    def get_error(self):
        pos = self._pos if self._pos is not None else self._last_pos
        return int(pos * 2000)

    ######################## Legacy state #####################

    '''
        Robot position according to the line, in the LINE_* states used by
        older programs. Reads the hardware.
            LINE_LEFT3..LINE_LEFT: robot is left of the line (line on its right)
            LINE_CENTER: on track
            LINE_RIGHT..LINE_RIGHT3: robot is right of the line
            LINE_CROSS: line under every sensor
            LINE_END: no line under any sensor
    '''
    def check(self):
        pos = self.update()
        if pos is None:
            return LINE_END
        if self.cross():
            return LINE_CROSS
        # legacy sign: the robot's side, opposite to the line's side
        p = -pos
        if p < -0.75:
            return LINE_LEFT3
        if p < -0.375:
            return LINE_LEFT2
        if p < -0.125:
            return LINE_LEFT
        if p <= 0.125:
            return LINE_CENTER
        if p <= 0.375:
            return LINE_RIGHT
        if p <= 0.75:
            return LINE_RIGHT2
        return LINE_RIGHT3

    '''
        True when the line is under the inner sensors: the robot is aligned
        with it. Used to end turn_until_line_detected().
    '''
    def centered(self):
        return self._pos is not None and abs(self._pos) <= 0.25


class LineSensor2P(LineSensor):
    '''
        Two sensors straddling the line: both see white while on track, so
        this sensor can never report LINE_END and "position" is only known
        when one sensor has drifted onto the line.
    '''
    def __init__(self, s1, s2, inverted=False):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        super().__init__(2, inverted)

    def _read_bits(self):
        return self._s1.value() | (self._s2.value() << 1)

    def _set_pattern(self, bits):
        self._pattern = bits
        self._count = (bits & 1) + (bits >> 1)
        if bits == 0 or bits == 3:
            self._pos = 0.0 # on track, or on a crossing
        else:
            # S1 sees black: the line is on the left
            self._pos = -1.0 if bits == 1 else 1.0
            self._last_pos = self._pos

    def lost(self):
        return False

    def cross(self):
        return self._pattern == 3

    def check(self):
        self.update()
        if self._pattern == 0:
            return LINE_CENTER
        if self._pattern == 3:
            return LINE_CROSS
        return LINE_RIGHT2 if self._pattern == 1 else LINE_LEFT2

    def centered(self):
        # any black means a sensor has reached the line
        return self._pattern != 0


class LineSensor3P(LineSensor):
    def __init__(self, s1, s2, s3, inverted=False):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        self._s3 = Pin(s3, Pin.IN)
        super().__init__(3, inverted)

    def _read_bits(self):
        return self._s1.value() | (self._s2.value() << 1) | (self._s3.value() << 2)


class LineSensorI2C(LineSensor):
    '''
        OhStem 4-channel line sensor (PCF8574 at 0x23), or the 5-channel
        array when one is found on the bus: LineSensorI2C() with no address
        scans the bus and returns a LineSensor5P_I2C if it sees one, so one
        init block serves both modules.
    '''
    def __new__(cls, address=None, inverted=False):
        if cls is LineSensorI2C and address is None:
            try:
                found = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000).scan()
            except Exception:
                found = []
            if LINE5_ADDR in found:
                return LineSensor5P_I2C()
        return super().__new__(cls)

    def __init__(self, address=None, inverted=False):
        if address is None:
            address = 0x23
        self.address = address
        self.i2c_pcf = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self._retry_at = 0
        super().__init__(4, inverted)
        self.pcf = None
        self._connect()
        if self.pcf is None:
            print('Line sensor not found')

    def _connect(self):
        try:
            self.pcf = pcf8574.PCF8574(self.i2c_pcf, self.address)
        except Exception:
            self.pcf = None

    def _read_bits(self):
        if self.pcf is None:
            # try again now and then, the module may have been plugged in late
            now = time.ticks_ms()
            if time.ticks_diff(now, self._retry_at) >= 0:
                self._retry_at = time.ticks_add(now, 1000)
                self._connect()
            if self.pcf is None:
                return 0
        try:
            return self.pcf.port & 0x0F # one I2C read for all four sensors
        except OSError:
            self.pcf = None
            return 0

    def check(self):
        # the table older programs were written against; the two single
        # inner-sensor cases used to be swapped (S2 alone reported LINE_LEFT
        # although the line is on the left, i.e. the robot is right of it)
        now = self.read()
        if now == (0, 0, 0, 0):
            return LINE_END
        elif now == (1, 1, 1, 1):
            return LINE_CROSS
        elif (now[1], now[2]) == (1, 1) or now == (1, 0, 0, 1):
            return LINE_CENTER
        elif (now[0], now[1]) == (1, 1):
            return LINE_RIGHT2
        elif (now[2], now[3]) == (1, 1):
            return LINE_LEFT2
        elif now == (0, 1, 0, 0):
            return LINE_RIGHT
        elif now == (0, 0, 1, 0):
            return LINE_LEFT
        elif now[1] == 1:
            return LINE_RIGHT2
        elif now[2] == 1:
            return LINE_LEFT2
        elif now[0] == 1:
            return LINE_RIGHT3
        else:
            return LINE_LEFT3


class LineSensor5P_I2C(LineSensor):
    '''
        OhStem "5 Channel Line Finder Array" (STM32G030 I2C slave at 0x24,
        registers in constants.py). S1 is the leftmost eye, S5 the rightmost.

        Two ways to read it:
          digital (default) - the STM32 thresholds each eye itself and
              returns 5 bits in one byte. Position has 9 steps.
          analog - the 5 raw 12-bit readings are normalised with a
              black/white calibration and turned into a continuous position,
              which is what a fast PD line follower needs. Calibration is
              learned while driving (running min/max per eye, with the
              array-wide contrast as fallback for eyes that have not seen
              the line yet), can be forced with a spin (DriveBase.
              line_calibrate) and is saved to flash. Until enough contrast
              has been seen the digital reading is used, so analog mode is
              always safe to switch on.
    '''
    def __init__(self, address=LINE5_ADDR):
        self.address = address
        super().__init__(5)
        self.ok = False
        self.i2c = None
        self._analog = False
        self._raw = (0, 0, 0, 0, 0)
        self._norm = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._cal_min = [4095] * 5
        self._cal_max = [0] * 5
        self._cal_learn = True # keep widening min/max while driving
        self._line_high = None # True: black reads higher than white; None: not known yet
        self._polarity_votes = 0
        self._color = None
        try:
            self.i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
            who = self.i2c.readfrom_mem(address, LINE5_REG_WHO, 1)[0]
            self.ok = (who == address)
            if not self.ok:
                print('5-ch line sensor: bad WHO_AM_I', who)
        except Exception:
            self.ok = False
            print('5-ch line sensor not found')
        self._load_calibration()

    ######################## Registers #####################

    def _read(self, reg, n=1):
        if not self.ok:
            return None
        try:
            return self.i2c.readfrom_mem(self.address, reg, n)
        except OSError:
            return None

    def _write(self, reg, val):
        if not self.ok:
            return
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))
        except OSError:
            pass

    def _read_bits(self):
        # firmware byte: bit4 = S1 .. bit0 = S5, flip so that bit0 = S1
        data = self._read(LINE5_REG_TUPLE, 1)
        b = data[0] if data else 0
        return ((b >> 4) & 1) | ((b >> 2) & 2) | (b & 4) | ((b << 2) & 8) | ((b << 4) & 16)

    '''
        Raw 12-bit reading of one eye, or all five as a tuple (S1 first).
    '''
    def read_raw(self, index=None):
        data = self._read(LINE5_REG_RAW, 10)
        if not data:
            return 0 if index is not None else (0, 0, 0, 0, 0)
        # firmware order is S5..S1
        vals = tuple(data[(4 - i)*2] | (data[(4 - i)*2 + 1] << 8) for i in range(5))
        return vals[index] if index is not None else vals

    ######################## Reading #####################

    '''
        Choose the reading: 'digital' (STM32 thresholds) or 'analog'
        (continuous position from the raw values).
    '''
    def mode(self, mode=None):
        if mode is None:
            return 'analog' if self._analog else 'digital'
        self._analog = (mode in ('analog', 'raw'))

    def update(self):
        if not self._analog:
            return super().update()

        data = self._read(LINE5_REG_RAW, 10)
        if not data:
            self._set_pattern(0)
            return None
        raw = tuple(data[(4 - i)*2] | (data[(4 - i)*2 + 1] << 8) for i in range(5))
        self._raw = raw

        if self._cal_learn:
            for i in range(5):
                v = raw[i]
                if v < self._cal_min[i]:
                    self._cal_min[i] = v
                if v > self._cal_max[i]:
                    self._cal_max[i] = v

        if self._line_high is None:
            # not yet known whether black reads high or low: the STM32's own
            # digital reading tells us, as soon as the line is under some eyes
            bits = self._read_bits()
            self._vote_polarity(raw, bits)
            if self._line_high is None:
                self._set_pattern(bits)
                return self._pos

        # array-wide contrast as fallback for eyes that have not seen the line yet
        lo = min(self._cal_min)
        hi = max(self._cal_max)
        if hi - lo < _MIN_RANGE:
            # nothing learned yet: digital reading until we have seen black and white
            self._set_pattern(self._read_bits())
            return self._pos

        bits = 0
        cnt = 0
        acc = 0.0
        tot = 0.0
        norm = self._norm
        for i in range(5):
            mn = self._cal_min[i]
            mx = self._cal_max[i]
            if mx - mn < _MIN_RANGE:
                mn = lo
                mx = hi
            n = (raw[i] - mn) / (mx - mn)
            if not self._line_high:
                n = 1.0 - n
            if n < _NOISE_FLOOR:
                n = 0.0
            elif n > 1.0:
                n = 1.0
            norm[i] = n
            if n >= 0.5:
                bits |= 1 << i
                cnt += 1
            acc += n * self._weights[i]
            tot += n

        self._pattern = bits
        self._count = cnt
        if tot < 0.4:
            self._pos = None
        else:
            self._pos = acc / tot
            self._last_pos = self._pos
        return self._pos

    def _vote_polarity(self, raw, bits):
        n_on = 0
        n_off = 0
        sum_on = 0
        sum_off = 0
        for i in range(5):
            if bits & (1 << i):
                n_on += 1
                sum_on += raw[i]
            else:
                n_off += 1
                sum_off += raw[i]
        if n_on == 0 or n_off == 0:
            return
        high = (sum_on / n_on) > (sum_off / n_off)
        self._polarity_votes += 1 if high else -1
        if abs(self._polarity_votes) >= 3:
            self._line_high = self._polarity_votes > 0

    '''
        Normalised eye readings 0.0 (white) .. 1.0 (black) from the last
        analog update().
    '''
    def normalized(self):
        return tuple(self._norm)

    ######################## Calibration #####################

    '''
        Forget the analog calibration and start learning it again. Call
        line_calibrate() on the robot afterwards, or just drive.
    '''
    def reset_calibration(self):
        self._cal_min = [4095] * 5
        self._cal_max = [0] * 5
        self._line_high = None
        self._polarity_votes = 0
        try:
            import os
            os.remove(LINE_CALIB_FILE)
        except OSError:
            pass

    def calibrated(self):
        return self._line_high is not None and max(self._cal_max) - min(self._cal_min) >= _MIN_RANGE

    def save_calibration(self):
        try:
            import json
            with open(LINE_CALIB_FILE, 'w') as f:
                json.dump({'min': self._cal_min, 'max': self._cal_max, 'high': self._line_high}, f)
        except Exception as e:
            print('line calibration save failed:', e)

    def _load_calibration(self):
        try:
            import json
            with open(LINE_CALIB_FILE) as f:
                data = json.load(f)
            self._cal_min = [int(v) for v in data['min']]
            self._cal_max = [int(v) for v in data['max']]
            self._line_high = data.get('high')
        except Exception:
            pass

    '''
        Runs the module's own calibration (thresholds for the digital
        reading), and restarts learning the analog one.
    '''
    def calibrate(self):
        self._write(LINE5_REG_CALIB, 1)
        self.reset_calibration()

    ######################## Controls #####################

    def set_white_led(self, on):
        self._write(LINE5_REG_LED, 1 if on else 0)

    ######################## VEML6040 colour sensor on the same board #####################

    def _init_veml(self):
        if not hasattr(self, '_veml'):
            try:
                from veml6040 import VEML6040
                self._veml = VEML6040(i2c=self.i2c)
            except Exception as e:
                print('VEML6040 not found:', e)
                self._veml = None

    def get_lux(self):
        self._init_veml()
        return self._veml.get_lux() if self._veml else 0

    def get_cct(self):
        self._init_veml()
        return self._veml.get_cct() if self._veml else 0

    def get_red(self):
        self._init_veml()
        return self._veml.get_red() if self._veml else 0

    def get_green(self):
        self._init_veml()
        return self._veml.get_green() if self._veml else 0

    def get_blue(self):
        self._init_veml()
        return self._veml.get_blue() if self._veml else 0

    def classify_hue(self):
        self._init_veml()
        return self._veml.classify_hue() if self._veml else None

    '''
        Background task reading the colour sensor; start it once with
        create_task(line_sensor.color_run()) and read color() anywhere.
    '''
    async def color_run(self, interval_ms=30, debug=False):
        self._init_veml()
        while True:
            if self._veml:
                try:
                    self._color = self._veml.classify_hue()
                    if debug:
                        r, g, b, h, s, v, lab = self._veml.hsv_debug()
                        print('COLOR,%d,r=%d,g=%d,b=%d,hue=%.0f,sat=%.3f,val=%.4f,%s' % (
                            time.ticks_ms(), r, g, b, h, s, v, lab))
                except OSError:
                    pass
            await asyncio.sleep_ms(interval_ms)

    def color(self):
        return self._color

    def hsv_debug(self):
        self._init_veml()
        return self._veml.hsv_debug() if self._veml else (0, 0, 0, 0.0, 0.0, 0.0, None)

    def calibrate_color(self, name):
        self._init_veml()
        if self._veml:
            self._veml.calibrate_color(name)
