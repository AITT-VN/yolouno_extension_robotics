from machine import Pin, SoftI2C
from utility import *
from setting import *
from micropython import const
import pcf8574
from constants import *

class LineSensor:
    def __init__(self):
        pass

    '''
        Chheck robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self): 
        return 0

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):    
        return 0


class LineSensor2P(LineSensor):
    def __init__(self, s1, s2):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        super().__init__()

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self):
        state = self.read()
        if state == (0, 0):
            return LINE_CENTER
        elif state == (0, 1):
            return LINE_LEFT2
        elif state == (1, 0):
            return LINE_RIGHT2
        else:
            return LINE_CROSS

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):
        if index == 0:
            return self._s1.value()
        elif index == 1:
            return self._s2.value()
        else:
            return (self._s1.value(), self._s2.value())

class LineSensor3P:
    def __init__(self, s1, s2, s3):
        self._s1 = Pin(s1, Pin.IN)
        self._s2 = Pin(s2, Pin.IN)
        self._s3 = Pin(s3, Pin.IN)
        super().__init__()

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self):
        state = self.read()
        if state == (1, 1, 1):
            return LINE_CROSS
        elif state == (0, 0, 0):
            return LINE_END
        elif state == (1, 1, 0):
            return LINE_RIGHT
        elif state == (0, 1, 1):
            return LINE_LEFT
        elif state == (1, 0, 0):
            return LINE_RIGHT2
        elif state == (0, 0, 1):
            return LINE_LEFT2
        else:
            return LINE_CENTER

    '''
        Read status of a specific sensor
    '''
    def read(self, index=None):
        if index == 0:
            return self._s1.value()
        elif index == 1:
            return self._s2.value()
        else:
            return (self._s1.value(), self._s2.value())


class LineSensorI2C(LineSensor):
    # Tu detect loai cam bien: goi LineSensorI2C() khong truyen dia chi -> quet I2C,
    # neu thay ban 5 mat (LINE5_ADDR=0x24) thi tra ve LineSensor5P_I2C, nguoc lai ban
    # 4 mat (PCF8574 @0x23). => dung CHUNG bien line_sensor = LineSensorI2C() cho ca hai.
    def __new__(cls, address=None):
        if cls is LineSensorI2C and address is None:
            try:
                _i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
                found = _i2c.scan()
            except:
                found = []
            if LINE5_ADDR in found:
                # __init__ cua 5P da chay du; vi 5P khong phai subclass cua
                # LineSensorI2C nen __init__ ban 4 mat se KHONG bi goi lai.
                return LineSensor5P_I2C()
        return super().__new__(cls)

    def __init__(self, address=None):
        if address is None:
            address = 0x23
        scl_pin = Pin(SCL_PIN)
        sda_pin = Pin(SDA_PIN)
        self.i2c_pcf = SoftI2C(scl=scl_pin, sda=sda_pin, freq=100000)
        self.address = address

        # cache cua update() de chay PID centroid digital (giong ban 5 mat)
        self._pos = None        # centroid [-2000,2000] hoac None khi mat line
        self._pattern = 0       # bitmask S1..S4
        self._last_err = 0      # centroid hop le gan nhat (giu huong khi mat line)

        try:
            self.pcf = pcf8574.PCF8574(self.i2c_pcf, self.address)
        except:
            self.pcf = None
            print('Line sensor not found')

    def read(self, index=None):
        # 0 white, 1 black
        if self.pcf is None:
            self._try_reconnect()
        if self.pcf is None:
            return 0 if index is not None else (0, 0, 0, 0)
        try:
            raw = self.pcf.port  # đọc toàn bộ byte trong 1 giao dịch I2C
            if index is not None:
                return (raw >> index) & 1
            return tuple((raw >> i) & 1 for i in range(4))
        except OSError:
            self.pcf = None
            return 0 if index is not None else (0, 0, 0, 0)

    def _try_reconnect(self):
        try:
            self.pcf = pcf8574.PCF8574(self.i2c_pcf, self.address)
        except:
            self.pcf = None

    # ---- centroid tu 4 bit digital: tong trong so / so mat, thang [-2000,2000].
    #      am = line lech trai (S1), duong = line lech phai (S4). None khi mat line.
    def _centroid(self, t):
        cnt = 0
        acc = 0
        for i in range(4):
            if t[i]:
                acc += LINE4_WEIGHTS[i]
                cnt += 1
        if cnt == 0:
            return None
        return acc // cnt

    # ---- doc 1 lan/loop, cache pattern + centroid (cho PID) ----
    def update(self):
        t = self.read()
        if not isinstance(t, tuple):
            t = (0, 0, 0, 0)
        pat = 0
        for i in range(4):
            if t[i]:
                pat |= (1 << i)
        self._pattern = pat
        self._pos = self._centroid(t)
        if self._pos is not None:
            self._last_err = self._pos
        return self

    def get_pattern(self):
        return self._pattern

    def get_error(self):
        # centroid cho PID; khi mat line giu huong cu de lai line
        if self._pos is not None:
            return self._pos
        if self._last_err > 0:
            return 2000
        elif self._last_err < 0:
            return -2000
        return 0

    def position(self):
        return self._centroid(self.read())

    '''
        Check robot position according to line
            -2: too much to the left 
            -1: to the left 
            0: on track 
            1: to the right 
            2: too much to the right
    '''
    def check(self): 
        now = self.read()
        #print(now)
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
        elif now == (0, 0, 1, 0): 
            return LINE_RIGHT
        elif now == (0, 1, 0, 0): 
            return LINE_LEFT
        elif now[1] == 1: 
            return LINE_RIGHT2
        elif now[2] == 1:
            return LINE_LEFT2
        elif now[0] == 1:
            return LINE_RIGHT3
        elif now[3] == 1:
            return LINE_LEFT3


# ============================================================================
#  Module "5 Channel Line Finder Array" (STM32G030, I2C slave @0x24).
#  Khac ban 4-mat (PCF8574): STM32 co ban do thanh ghi rieng (constants.py:LINE5_*).
#
#  CACH DOC: dung TUPLE (thanh ghi 0x06, bit4=S1 .. bit0=S5). STM32 da TU
#  nguong hoa + xu ly reverse -> tra ve 0/1 sach (giong PCF8574 ban 4-mat).
#  Toan bo logic dò line tinh tu pattern so (digital), KHONG phu thuoc calib
#  phia MicroPython -> hoat dong on dinh voi cac khoi lenh dò line san co.
#  RAW analog chi de hien thi/calib (read_raw, calibrate).
# ============================================================================
class LineSensor5P_I2C(LineSensor):
    def __init__(self, address=LINE5_ADDR):
        self.address = address
        # cache cua update() (doc I2C 1 lan/loop)
        self._pos = None        # centroid [-2000,2000] hoac None
        self._pattern = 0       # bitmask S1..S5
        self._count = 0         # so mat tren line
        self._last_err = 0      # centroid hop le gan nhat (giu huong khi mat line)
        # debounce checkpoint
        self._cp_cand = LINE_NORMAL
        self._cp_n = 0
        self._cp_need = 2
        self._checkpoint = LINE_NORMAL
        self._lost_frames = 0

        try:
            self.i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
            who = self.i2c.readfrom_mem(address, LINE5_REG_WHO, 1)[0]
            self.ok = (who == address)
            if not self.ok:
                print('5-ch line sensor: bad WHO_AM_I', who)
        except:
            self.i2c = None
            self.ok = False
            print('5-ch line sensor not found')

    # ---- truy cap thanh ghi ----
    def _read(self, reg, n=1):
        if not self.ok:
            return None
        try:
            return self.i2c.readfrom_mem(self.address, reg, n)
        except:
            return None

    def _write(self, reg, val):
        if not self.ok:
            return
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))
        except:
            pass

    # ---- doc TUPLE 0/1 moi mat (S1=trai nhat .. S5=phai nhat) ----
    def read(self, index=None):
        # 0=nen, 1=tren-line. Firmware byte: bit4=S1 .. bit0=S5 -> dao lai.
        data = self._read(LINE5_REG_TUPLE, 1)
        b = data[0] if data else 0
        if index is None:
            return tuple((b >> (4 - i)) & 1 for i in range(5))
        return (b >> (4 - index)) & 1

    # ---- doc RAW analog 12-bit (firmware tra S5..S1 -> dao de index 0 = S1) ----
    def read_raw(self, index=None):
        data = self._read(LINE5_REG_RAW, 10)
        if not data:
            return 0 if index is not None else (0, 0, 0, 0, 0)
        vals = tuple(data[(4 - i) * 2] | (data[(4 - i) * 2 + 1] << 8) for i in range(5))
        return vals[index] if index is not None else vals

    # ---- centroid tu pattern so: tong trong so / so mat, thang [-2000,2000].
    #      am = line lech trai (S1), duong = line lech phai (S5). None khi mat line.
    def _centroid(self, t):
        cnt = 0
        acc = 0
        for i in range(5):
            if t[i]:
                acc += LINE5_WEIGHTS[i]
                cnt += 1
        if cnt == 0:
            return None
        return acc // cnt

    # ---- doc 1 lan/loop, cache pattern/centroid/checkpoint (debounce) ----
    def update(self):
        t = self.read()
        pat = 0
        cnt = 0
        for i in range(5):
            if t[i]:
                pat |= (1 << i)
                cnt += 1
        self._pattern = pat
        self._count = cnt
        self._pos = self._centroid(t)
        if self._pos is not None:
            self._last_err = self._pos

        cand = self._classify()
        if cand == LINE_LOST:
            if self._lost_frames < 10000:
                self._lost_frames += 1
        else:
            self._lost_frames = 0
        if cand == self._cp_cand:
            if self._cp_n < 10000:
                self._cp_n += 1
        else:
            self._cp_cand = cand
            self._cp_n = 1
        if self._cp_n >= self._cp_need:
            self._checkpoint = cand
        return self

    def _classify(self):
        p = self._pattern
        c = self._count
        if c == 0:                              # mat line
            return LINE_LOST
        if c >= 4:                              # 4-5 mat trum -> vach ngang
            return LINE_CROSS
        edgeL = p & 0b00001; edgeR = p & 0b10000
        center = p & 0b00100
        grpL = p & 0b00011; grpR = p & 0b11000
        if edgeL and edgeR and not center:      # 2 mep ngoai sang -> nga Y
            return LINE_Y
        if c == 3:
            if p == 0b00111:
                return LINE_LEFT_CORNER
            if p == 0b11100:
                return LINE_RIGHT_CORNER
            return LINE_NORMAL
        if edgeL and not grpR:
            return LINE_LEFT_CORNER
        if edgeR and not grpL:
            return LINE_RIGHT_CORNER
        return LINE_NORMAL

    # ---- getter doc cache (khong doc I2C) ----
    def get_pattern(self):
        return self._pattern

    def count(self):
        return self._count

    def get_error(self):
        # centroid cho PID; khi mat line giu huong cu de lai line
        if self._pos is not None:
            return self._pos
        if self._last_err > 0:
            return 2000
        elif self._last_err < 0:
            return -2000
        return 0

    def detect_checkpoint(self):
        return self._checkpoint

    def lost_frames(self):
        return self._lost_frames

    def set_debounce(self, frames):
        self._cp_need = max(1, int(frames))

    # ---- vi tri line cho PID: [-2000,2000], None khi mat line ----
    def position(self):
        return self._centroid(self.read())

    # ---- trang thai roi rac (tuong thich follow_line cu) ----
    def check(self):
        t = self.read()
        cnt = sum(t)
        if cnt == 0:
            return LINE_END
        if cnt >= 4:
            return LINE_CROSS
        c = self._centroid(t)
        # Dao dau de khop convention 4-mat: p duong = robot lech phai (line o trai).
        p = -c
        if   p <= -1500: return LINE_LEFT3
        elif p <=  -750: return LINE_LEFT2
        elif p <   -250: return LINE_LEFT
        elif p <=   250: return LINE_CENTER
        elif p <    750: return LINE_RIGHT
        elif p <   1500: return LINE_RIGHT2
        return LINE_RIGHT3

    # ---- dieu khien / cau hinh ----
    def set_white_led(self, on):
        self._write(LINE5_REG_LED, 1 if on else 0)

    def calibrate(self):
        # Kich calib tren STM32 (tu dong nguong hoa cho TUPLE).
        self._write(LINE5_REG_CALIB, 1)

    # ---- VEML6040 (tich hop san tren board 5-mat) ----
    def _init_veml(self):
        if not hasattr(self, '_veml'):
            try:
                from veml6040 import VEML6040
                self._veml = VEML6040()
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

    def calibrate_white(self):
        self._init_veml()
        if self._veml:
            self._veml.calibrate_white()