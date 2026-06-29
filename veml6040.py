from machine import SoftI2C, Pin
from micropython import const
from setting import SCL_PIN, SDA_PIN

_ADDR   = const(0x10)
_R_CONF = const(0x00)
_R_RED  = const(0x08)
_R_GRN  = const(0x09)
_R_BLU  = const(0x0A)
_R_WHT  = const(0x0B)

# IT config bits (bits 6:4) → G-sensitivity (lux per count)
_IT_GSENS = {0x00: 0.25168, 0x10: 0.12584, 0x20: 0.06292,
             0x30: 0.03146, 0x40: 0.01573, 0x50: 0.007865}

_SAT_MIN = 0.15   # bão hoà tối thiểu (dưới ngưỡng = quá xám)
_VAL_MIN = 0.005  # độ sáng tối thiểu (dưới ngưỡng = quá tối)

def _rgb2hsv(r, g, b):
    mx = max(r, g, b)
    if not mx:
        return {'hue': 0.0, 'sat': 0.0, 'val': 0.0}
    # chuẩn hoá theo max → hue độc lập với độ sáng
    rn, gn, bn = r / mx, g / mx, b / mx
    lo = min(rn, gn, bn)
    d  = 1.0 - lo       # hi luôn = 1.0 sau chuẩn hoá
    s  = d              # sat = (hi - lo) / hi = 1 - lo
    if not d:
        h = 0.0
    elif rn == 1.0:
        h = (gn - bn) / d + (6 if gn < bn else 0)
    elif gn == 1.0:
        h = (bn - rn) / d + 2
    else:
        h = (rn - gn) / d + 4
    return {'hue': h / 6 * 360, 'sat': s, 'val': mx / 65535.0}


class VEML6040:
    def __init__(self, address=_ADDR):
        self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self._addr = address
        self._cfg  = 0x10  # IT=80ms, AF=auto, SD=on
        self._wb   = (1.0, 1.0, 1.0)  # white balance: (kr, kg, kb), mặc định không bù
        if not self._i2c.scan().count(address):
            raise Exception('VEML6040 not found')
        self._i2c.writeto(address, bytes([_R_CONF, self._cfg, 0]))

    def _read16(self, reg):
        d = self._i2c.readfrom_mem(self._addr, reg, 2)
        return d[0] | (d[1] << 8)

    def get_red(self):   return self._read16(_R_RED)
    def get_green(self): return self._read16(_R_GRN)
    def get_blue(self):  return self._read16(_R_BLU)
    def get_white(self): return self._read16(_R_WHT)

    def get_rgb(self):
        # VEML6040 không auto-increment → phải đọc từng kênh riêng
        return (self._read16(_R_RED), self._read16(_R_GRN),
                self._read16(_R_BLU), self._read16(_R_WHT))

    def _get_rgb_balanced(self):
        r, g, b, w = self.get_rgb()
        kr, kg, kb = self._wb
        return int(r * kr), int(g * kg), int(b * kb), w

    def get_lux(self):
        sens = _IT_GSENS.get(self._cfg & 0x70)
        return int(self._read16(_R_GRN) * sens) if sens else -1

    def get_cct(self, offset=0.5):
        r, g, b, _ = self._get_rgb_balanced()
        if g < 1:
            return 0
        return 4278.6 * pow((r - b) / g + offset, -1.2455)

    def classify_hue(self):
        r, g, b, _ = self._get_rgb_balanced()
        hsv = _rgb2hsv(r, g, b)
        if hsv['val'] < _VAL_MIN or hsv['sat'] < _SAT_MIN:
            return None
        hues = (('red',0),('yellow',60),('green',120),('cyan',180),('blue',240),('magenta',300))
        h = hsv['hue']
        return min(hues, key=lambda x: min(abs(h - x[1]), 360 - abs(h - x[1])))[0]
    
    def calibrate_white(self):
        # Đặt cảm biến lên nền trắng rồi gọi hàm này.
        # Tính hệ số bù sao cho R=G=B trên nền trắng → loại bỏ sai lệch kênh.
        r, g, b, _ = self.get_rgb()
        if g < 1 or r < 1 or b < 1:
            return
        self._wb = (g / r, 1.0, g / b)
