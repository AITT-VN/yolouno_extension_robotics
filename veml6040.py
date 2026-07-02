from machine import SoftI2C, Pin
import time
import asyncio
import ujson
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

_SAT_MIN = 0.12   # bão hoà tối thiểu (dưới ngưỡng = quá xám)
_VAL_MIN = 0.002  # độ sáng tối thiểu (dưới ngưỡng = quá tối)

# Tong R+G+B toi thieu: duoi nguong = qua toi (line den/tat) -> tra None.
_SUM_MIN = const(1500)

# Cac nhan "nen" (khong phai mau): reference gan nhat roi vao day -> classify_hue()
# tra None. 'white' = nen trang; 'black' = vach den (calibrate tren sa ban de tranh
# nhan nham line den thanh mau - xem calibrate_color).
_BG_NAMES = ('white', 'black')

# Reference chromaticity (r,g,b chuan hoa theo tong=1) DO TREN CAM BIEN NAY (raw,
# khong white-balance). classify_hue() tra mau co reference gan nhat. Nhan trong
# _BG_NAMES -> tra None. Do lai bang calibrate_color(name) neu doi giay/den.
# LUU Y: cyan & blue tren cam bien nay gan nhu trung nhau (~0.008) -> de lan;
# neu can tach ro thi dung giay khac biet hon, hoac chi dung 1 trong 2.
# 'black' KHONG co gia tri mac dinh: chromaticity vach den phu thuoc muc/den nen,
# nen de nguoi dung tu calibrate tren sa ban (tranh loai nham mau that).
_COLOR_REFS = (
    ('red',     0.435, 0.344, 0.221),
    ('yellow',  0.390, 0.415, 0.195),
    ('green',   0.357, 0.407, 0.236),
    ('cyan',    0.302, 0.349, 0.349),
    ('blue',    0.309, 0.345, 0.346),
    ('magenta', 0.410, 0.333, 0.257),
    ('white',   0.338, 0.387, 0.275),   # nen trang -> classify_hue() tra None
)

# Thu tu hieu chuan hang loat (calibrate_all): (ten tham chieu, nhan hien thi).
_CALIB_ORDER = (
    ('white',   'nen trang'),
    ('black',   'vach den'),
    ('red',     'do'),
    ('yellow',  'vang'),
    ('green',   'xanh la'),
    ('cyan',    'xanh lo (cyan)'),
    ('blue',    'xanh duong'),
    ('magenta', 'magenta'),
)

# Cache đọc RGBW. IT phần cứng = 40ms (đã là min của VEML6040), dữ liệu chỉ mới
# mỗi 40ms nên đọc nhanh hơn là vô ích; đặt cache < IT để bớt staleng chồng thêm.
_CACHE_MS = const(20)

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
    def __init__(self, address=_ADDR, i2c=None):
        if i2c is None:
            self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        else:
            self._i2c = i2c
        self._addr = address
        self._cfg  = 0x00  # IT=40ms, AF=auto, SD=on
        self._last_read = 0
        self._cached_rgb = (0, 0, 0, 0)
        self._refs = list(_COLOR_REFS)   # reference chromaticity (co the calibrate lai)
        self._color = None               # nhan mau hien tai, cache boi task nen color_run()

        if not self._i2c.scan().count(address):
            raise Exception('VEML6040 not found')
        self._i2c.writeto(address, bytes([_R_CONF, self._cfg, 0]))

        # Load calibrated refs from file (if exists)
        self.load_refs()

    def _read16(self, reg):
        d = self._i2c.readfrom_mem(self._addr, reg, 2)
        return d[0] | (d[1] << 8)

    def get_red(self):   return self._read16(_R_RED)
    def get_green(self): return self._read16(_R_GRN)
    def get_blue(self):  return self._read16(_R_BLU)
    def get_white(self): return self._read16(_R_WHT)

    def get_rgb(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_read) >= _CACHE_MS:
            try:
                self._cached_rgb = (self._read16(_R_RED), self._read16(_R_GRN),
                                    self._read16(_R_BLU), self._read16(_R_WHT))
                self._last_read = now
            except OSError:
                # Bus bit-bang dung chung voi line sensor doi khi NACK thoang qua.
                # Giu cache cu, KHONG cap nhat _last_read de lan goi ke tiep thu lai ngay.
                pass
        return self._cached_rgb

    def get_lux(self):
        sens = _IT_GSENS.get(self._cfg & 0x70)
        return int(self._read16(_R_GRN) * sens) if sens else -1

    def get_cct(self, offset=0.5):
        r, g, b, _ = self.get_rgb()
        if g < 1:
            return 0
        return 4278.6 * pow((r - b) / g + offset, -1.2455)

    def classify_hue(self):
        # Phan loai theo reference gan nhat trong khong gian chromaticity (raw,
        # doc lap white-balance -> deterministic). Nhan trong _BG_NAMES -> None.
        r, g, b, _ = self.get_rgb()
        s = r + g + b
        if s < _SUM_MIN:          # qua toi (line den/tat) -> khong phai mau
            return None
        cr, cg, cb = r / s, g / s, b / s
        best = None
        bestd = 1e9
        for name, rr, rg, rb in self._refs:
            d = (cr - rr) * (cr - rr) + (cg - rg) * (cg - rg) + (cb - rb) * (cb - rb)
            if d < bestd:
                bestd = d
                best = name
        return None if best in _BG_NAMES else best

    def calibrate_color(self, name):
        # Dat cam bien len mau 'name' (vd 'green') roi goi -> cap nhat reference.
        # Dung 'white' cho nen trang, 'black' cho vach den (deu tra None khi phan loai).
        # Voi mau that: bo qua neu qua toi. Voi nhan nen (_BG_NAMES): van do duoc khi
        # toi (chi can s > 0) de bat dung chromaticity cua vach den.
        r, g, b, _ = self.get_rgb()
        s = r + g + b
        if s <= 0 or (s < _SUM_MIN and name not in _BG_NAMES):
            return
        ref = (name, r / s, g / s, b / s)
        for i in range(len(self._refs)):
            if self._refs[i][0] == name:
                self._refs[i] = ref
                self.save_refs()  # Luu ngay sau khi cap nhat
                return
        self._refs.append(ref)
        self.save_refs()

    def load_refs(self):
        # Doc tham chieu calib tu file veml6040.json. Neu khong co -> dung default.
        try:
            f = open('veml6040.json', 'r')
            data = ujson.loads(f.read())
            f.close()
            if 'refs' in data:
                self._refs = [tuple(ref) for ref in data['refs']]
                print('VEML6040: loaded refs from file')
                return
        except Exception as e:
            print('VEML6040: no calib file or load error:', e)
        # Fallback: dung default _COLOR_REFS
        self._refs = list(_COLOR_REFS)

    def save_refs(self):
        # Ghi tham chieu calib vao file veml6040.json
        try:
            data = {'refs': self._refs}
            f = open('veml6040.json', 'w')
            f.write(ujson.dumps(data))
            f.close()
            print('VEML6040: saved refs to file')
        except Exception as e:
            print('VEML6040: save error:', e)

    async def calibrate_all(self):
        # Hieu chuan lan luot TAT CA mau trong _CALIB_ORDER bang nut BOOT vat ly:
        # dat cam bien len mau duoc yeu cau -> bam BOOT -> tu dong calib mau do
        # (calibrate_color() da tu luu file), roi chuyen sang mau tiep theo.
        # Het danh sach -> in thong bao va return (khac color_calib.py: KHONG lap lai).
        from abutton import aButton
        try:
            from abutton import BOOT_PIN
        except ImportError:
            from yolo_uno import BOOT_PIN

        btn = aButton(BOOT_PIN)
        pressed = [False]

        async def _on_press():
            pressed[0] = True

        btn.pressed(_on_press)
        try:
            print('== Hieu chuan tat ca mau ==')
            for name, label in _CALIB_ORDER:
                print('>> Dat cam bien len [%s] roi bam BOOT' % label)
                pressed[0] = False
                while not pressed[0]:
                    await asyncio.sleep_ms(50)
                self.calibrate_color(name)
                print('   Da luu [%s]' % label)
        finally:
            btn.deinit()
        print('== Hoan tat hieu chuan tat ca mau (da luu veml6040.json) ==')

    def hsv_debug(self):
        # Chan doan/lay mau: (r,g,b raw, hue, sat, val, label).
        r, g, b, _ = self.get_rgb()
        hsv = _rgb2hsv(r, g, b)
        return (r, g, b, hsv['hue'], hsv['sat'], hsv['val'], self.classify_hue())

    # ---- Task nen doc mau lien tuc (doc lap voi cam bien do line) ----
    #  Khoi dong 1 lan trong setup: create_task(color_sensor.color_run()).
    #  Doc VEML theo nhip interval_ms (<= IT=40ms), cache nhan mau vao self._color.
    #  color() la getter re chi doc cache -> khong chan vong lap chinh.
    async def color_run(self, interval_ms=30, debug=False):
        print('COLOR_RUN start')
        while True:
            try:
                self._color = self.classify_hue()  # get_rgb ben trong da boc OSError
                if debug:
                    r, g, b, h, s, v, lab = self.hsv_debug()
                    print('COLOR,%d,r=%d,g=%d,b=%d,hue=%.0f,sat=%.3f,val=%.4f,%s' % (
                        time.ticks_ms(), r, g, b, h, s, v, lab))
            except OSError:
                pass          # giu gia tri cu, thu lai vong sau
            await asyncio.sleep_ms(interval_ms)

    def color(self):
        # Getter re (khong I2C): nhan mau hien tai da cache boi color_run(), hoac None.
        return self._color
