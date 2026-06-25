from machine import Pin, SoftI2C
from utility import *
from setting import *
from micropython import const
from constants import *
from line_sensor import LineSensor

# ============================================================================
#  Driver cho module "5 Channel Line Finder Array" (STM32G030, I2C slave).
#  Khac voi cam bien 4 mat (dung PCF8574): module nay la STM32 slave co ban do
#  thanh ghi rieng. Doc theo giao thuc register-pointer (readfrom_mem).
#
#  Dia chi I2C mac dinh: 0x24 (khac ban 4 mat = 0x23).
#  2 mode du lieu: TUPLE (1 byte, bit0..bit4 = S1..S5) va RAW (5 x uint16 LE).
#
#  NANG CAP (v2):
#    - update()           : doc 1 lan/loop, cache moi gia tri (realtime, 0 alloc)
#    - get_normalized()   : 5 mat sau calib (0..1000)
#    - get_pattern()      : bit pattern 5 mat (bit0=S1 .. bit4=S5)
#    - get_error()        : sai so centroid [-2000,2000] cho PID (giu huong khi mat line)
#    - detect_checkpoint(): nhan dang giao diem (FSM o drivebase xu ly)
#    - history buffer     : ring buffer pattern de nhan zigzag / Y / T / cua
#    - debounce           : checkpoint chi xac nhan khi on dinh N frame
#  position()/check() giu nguyen hanh vi cu -> tuong thich API.
# ============================================================================

LINE5_DEFAULT_ADDR = const(0x24)

# Ban do thanh ghi (khop firmware src/main.c).
_REG_WHO_AM_I      = const(0x00)
_REG_FW_VERSION    = const(0x01)
_REG_MODE          = const(0x02)
_REG_REVERSE_MODE  = const(0x03)
_REG_CALIB_TRIGGER = const(0x04)
_REG_STATUS        = const(0x05)
_REG_TUPLE         = const(0x06)
_REG_RAW_BASE      = const(0x10)
_REG_LED_WHITE_EN  = const(0x1A)
_REG_LEVEL_MIN_BASE = const(0x20)   # smin (muc tren-line) S1..S5, uint16 LE
_REG_LEVEL_MAX_BASE = const(0x2A)   # smax (muc nen)      S1..S5, uint16 LE

# Trong so truc cho 5 mat (S1..S5) theo thang +-2000 cho PID.
# Am = lech trai, duong = lech phai, 0 = giua.
_WEIGHTS = (-2000, -1000, 0, 1000, 2000)

# Gia tri chuan hoa 0..1000. Nguong quyet dinh "co line" / "vach ngang".
_NORM_MAX  = const(1000)
_FLOOR     = const(250)   # peak < FLOOR  -> coi nhu mat line (None / LINE_LOST)
_HIGH      = const(800)   # value > HIGH  -> mat dang trum line (dem cho LINE_CROSS)
_MIN_SPAN  = const(20)    # (max-min) nho hon -> mat khong dang tin, dung fallback
_ON_THRESH = const(500)   # value > nguong nay -> bit pattern = 1 (mat "tren line")

# ---- Enum checkpoint tra ve boi detect_checkpoint() ------------------------
# Dat trong dai 10..20 de KHONG dung cham voi line state (-3..5) trong constants.py.
# Rieng LINE_CROSS dung lai const(4) co san trong constants -> 1 y nghia duy nhat.
LINE_NORMAL       = const(10)   # dang bam line binh thuong (PID lo)
LINE_LEFT_CORNER  = const(11)   # cua/nhanh trai (line cham mat S1)
LINE_RIGHT_CORNER = const(12)   # cua/nhanh phai (line cham mat S5)
LINE_T            = const(13)   # nga ba chu T (vach ngang day)  -> FSM phan biet voi cross
LINE_Y            = const(15)   # nga re chu Y (2 mep on, giua rong)
LINE_U_TURN       = const(16)   # quay dau (do FSM xac dinh khi mat line lau)
LINE_LOST         = const(17)   # mat line (tat ca mat = nen)
LINE_DASH         = const(18)   # duong dut (mat line ngan, do FSM xac dinh)
LINE_START        = const(19)   # vach xuat phat (app dem vach)
LINE_FINISH       = const(20)   # vach dich (app dem vach)
# LINE_CROSS = const(4)  -> dem tu constants.py, dung cho vach ngang day (+/cross/T/start/finish)

# Do dai history ring buffer (so frame pattern luu lai).
_HIST_LEN = const(16)


class LineArray5Ch(LineSensor):
    def __init__(self, address=LINE5_DEFAULT_ADDR):
        self.address = address
        # Muc calib cache (smin = tren line, smax = nen). Fallback toan thang 12-bit.
        self._lo = [0, 0, 0, 0, 0]
        self._hi = [4095, 4095, 4095, 4095, 4095]
        self._reverse = False
        # Bo loc lam muot gia tri chuan hoa (giam giat PID o vung tuong phan thap).
        self._filt = [0, 0, 0, 0, 0]

        # ---- cache cho update() (tranh alloc/doc I2C nhieu lan trong 1 loop) ----
        self._pos     = None    # centroid [-2000,2000] hoac None
        self._pattern = 0       # bitmask S1..S5
        self._count   = 0       # so mat dang tren line
        self._peak    = 0       # max gia tri chuan hoa
        self._total   = 0       # tong gia tri chuan hoa
        self._last_err = 0      # centroid hop le gan nhat (giu huong khi mat line)
        self._on_thresh = _ON_THRESH

        # ---- history ring buffer (pattern) ----
        self._hist = bytearray(_HIST_LEN)
        self._hist_idx = 0
        self._hist_count = 0

        # ---- debounce checkpoint ----
        self._cp_cand = LINE_NORMAL     # ung vien hien tai
        self._cp_n    = 0               # so frame ung vien da on dinh
        self._cp_need = 2               # so frame can de xac nhan (vd 2 = ~10ms @5ms loop)
        self._checkpoint = LINE_NORMAL  # checkpoint da xac nhan (debounced)
        self._lost_frames = 0           # so frame lien tiep mat line

        try:
            self.i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
            # Kiem tra co dung module khong (doc WHO_AM_I).
            who = self.i2c.readfrom_mem(self.address, _REG_WHO_AM_I, 1)[0]
            self.ok = (who == self.address)
            if not self.ok:
                print('5-channel line sensor: wrong WHO_AM_I', who)
            else:
                self.refresh_calibration()
        except Exception:
            self.i2c = None
            self.ok = False
            print('5-channel line sensor not found')

    # ---- truy cap thanh ghi ------------------------------------------------
    def _read(self, reg, n=1):
        if not self.ok:
            return None
        try:
            return self.i2c.readfrom_mem(self.address, reg, n)
        except Exception:
            return None

    def _write(self, reg, val):
        if not self.ok:
            return
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([val & 0xFF]))
        except Exception:
            pass

    # ---- doc trang thai TUPLE (0/1 moi mat) --------------------------------
    def read(self, index=None):
        # 0 = nen, 1 = thay line
        # Firmware STM32: bit4=S1 (trai) .. bit0=S5 (phai) -> dao nguoc lai.
        data = self._read(_REG_TUPLE, 1)
        b = data[0] if data else 0
        if index is None:
            return tuple((b >> (4 - i)) & 1 for i in range(5))
        return (b >> (4 - index)) & 1

    # ---- doc gia tri RAW analog (12-bit) -----------------------------------
    def read_raw(self, index=None):
        data = self._read(_REG_RAW_BASE, 10)
        if not data:
            return 0 if index is not None else (0, 0, 0, 0, 0)
        # Firmware tra byte theo thu tu S5..S1 -> dao nguoc de index 0 = S1.
        vals = tuple(data[(4 - i) * 2] | (data[(4 - i) * 2 + 1] << 8) for i in range(5))
        if index is None:
            return vals
        return vals[index]

    # ---- cache muc calib (smin/smax) tu firmware ---------------------------
    # Goi luc khoi tao va sau moi lan calibrate(). Doc 2 cum 10 byte tu STM32.
    def refresh_calibration(self):
        if not self.ok:
            return
        lo = self._read(_REG_LEVEL_MIN_BASE, 10)
        hi = self._read(_REG_LEVEL_MAX_BASE, 10)
        rev = self._read(_REG_REVERSE_MODE, 1)
        self._reverse = bool(rev[0]) if rev else False
        if not lo or not hi:
            return
        ok = True
        for i in range(5):
            mn = lo[i * 2] | (lo[i * 2 + 1] << 8)
            mx = hi[i * 2] | (hi[i * 2 + 1] << 8)
            if (mx - mn) < _MIN_SPAN:
                ok = False          # mat nay chua calib dang tin
            self._lo[i] = mn
            self._hi[i] = mx
        if not ok:
            print('5-channel line sensor: chua calib / tuong phan thap, can hieu chuan lai')

    # ---- gia tri chuan hoa 0..1000 tung mat (theo calib) -------------------
    # 1000 = trum line manh nhat, 0 = nen. Ap dung loc IIR nhe lam muot.
    # Ham nay doc I2C (read_raw) -> moi loop chi nen goi 1 lan (qua update()).
    def _normalized(self):
        raw = self.read_raw()
        out = self._filt
        for i in range(5):
            span = self._hi[i] - self._lo[i]
            if span < _MIN_SPAN:
                v = 0
            elif self._reverse:
                # line sang tren nen toi: line = ADC cao
                v = (raw[i] - self._lo[i]) * _NORM_MAX // span
            else:
                # mac dinh: line toi tren nen sang: line = ADC thap
                v = (self._hi[i] - raw[i]) * _NORM_MAX // span
            if v < 0:
                v = 0
            elif v > _NORM_MAX:
                v = _NORM_MAX
            out[i] = (out[i] + v) // 2      # IIR mot nhip (~0.5/0.5)
        return out

    # ========================================================================
    #  API V2 - goi update() 1 lan moi vong dieu khien, roi doc cache.
    # ========================================================================
    def update(self):
        """Doc cam bien 1 lan, cache: normalized, pattern, count, centroid,
        history va checkpoint (co debounce). Goi 1 lan dau moi loop dieu khien."""
        v = self._normalized()
        peak = 0
        total = 0
        acc = 0
        pat = 0
        cnt = 0
        th = self._on_thresh
        for i in range(5):
            x = v[i]
            total += x
            acc += _WEIGHTS[i] * x
            if x > peak:
                peak = x
            if x > th:
                pat |= (1 << i)
                cnt += 1

        self._peak = peak
        self._total = total
        self._pattern = pat
        self._count = cnt

        if total == 0 or peak < _FLOOR:
            self._pos = None
        else:
            p = acc // total
            self._pos = p
            self._last_err = p

        # day pattern vao history ring
        self._hist[self._hist_idx] = pat
        self._hist_idx += 1
        if self._hist_idx >= _HIST_LEN:
            self._hist_idx = 0
        if self._hist_count < _HIST_LEN:
            self._hist_count += 1

        # phan loai + debounce
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
        """Phan loai checkpoint tu 1 frame (chua debounce). Dung cache cua update()."""
        peak = self._peak
        if peak < _FLOOR:
            return LINE_LOST

        p = self._pattern
        c = self._count

        # Vach ngang day: 4-5 mat cung trum -> cross / T / start / finish.
        # 1 hang 5 mat KHONG the phan biet cross vs T vs start/finish ngay tuc thi
        # -> tra LINE_CROSS, FSM o drivebase se phan biet bang continuation/dem vach.
        if c >= 4:
            return LINE_CROSS

        edgeL  = p & 0b00001    # S1 (trai cung)
        edgeR  = p & 0b10000    # S5 (phai cung)
        center = p & 0b00100    # S3
        grpL   = p & 0b00011    # S1|S2
        grpR   = p & 0b11000    # S4|S5

        # Nga re chu Y: 2 mep ngoai sang, giua rong (vd 10001, 11011).
        if edgeL and edgeR and not center:
            return LINE_Y

        if c == 3:
            if p == 0b00111:            # S1 S2 S3 -> line don ve trai
                return LINE_LEFT_CORNER
            if p == 0b11100:            # S3 S4 S5 -> line don ve phai
                return LINE_RIGHT_CORNER
            return LINE_NORMAL

        # c == 1 hoac 2: cua gat khi line cham mat ngoai cung
        if edgeL and not grpR:
            return LINE_LEFT_CORNER
        if edgeR and not grpL:
            return LINE_RIGHT_CORNER
        return LINE_NORMAL

    # ---- cac getter doc cache (KHONG doc I2C, an toan goi nhieu lan/loop) --
    def get_normalized(self):
        """Tra ve list 5 gia tri sau calib (0..1000). CHI doc, dung sua truc tiep.
        Goi update() truoc de co gia tri moi nhat."""
        return self._filt

    def get_pattern(self):
        """Bitmask 5 mat: bit0=S1 .. bit4=S5 (1 = tren line). Vd 0b00100 = chi S3."""
        return self._pattern

    def count(self):
        """So mat dang nam tren line."""
        return self._count

    def get_error(self):
        """Sai so centroid [-2000,2000] cho PID. Khi mat line: giu huong cu (bao hoa
        ve phia da thay line lan cuoi) de PID tiep tuc lai line."""
        if self._pos is not None:
            return self._pos
        if self._last_err > 0:
            return 2000
        elif self._last_err < 0:
            return -2000
        return 0

    def detect_checkpoint(self):
        """Tra ve checkpoint da debounce: LINE_NORMAL / LINE_*_CORNER / LINE_CROSS /
        LINE_Y / LINE_LOST. FSM o drivebase phan biet T/CROSS/START/FINISH/DASH/U_TURN."""
        return self._checkpoint

    def lost_frames(self):
        """So frame lien tiep mat line (de FSM phan biet DASH ngan vs LOST/U-turn lau)."""
        return self._lost_frames

    def history(self, n=1):
        """Lay pattern cua frame thu n tinh nguoc tu hien tai (n=1 la frame moi nhat)."""
        if n < 1 or n > self._hist_count:
            return 0
        idx = self._hist_idx - n
        if idx < 0:
            idx += _HIST_LEN
        return self._hist[idx]

    def set_debounce(self, frames):
        """So frame on dinh can de xac nhan checkpoint (1..). Loop 5ms -> 2 frame ~10ms."""
        self._cp_need = max(1, int(frames))

    def set_on_threshold(self, value):
        """Nguong 0..1000 de tinh bit pattern (mac dinh 500)."""
        self._on_thresh = value

    @staticmethod
    def pattern_str(p):
        """Chuoi 5 ky tu '00100' (S1..S5) - chi dung debug (co cap phat chuoi)."""
        return ''.join('1' if (p >> i) & 1 else '0' for i in range(5))

    # ---- vi tri line (so co dau) cho PID -----------------------------------
    # Centroid CHUAN HOA theo calib (kieu Pololu QTR) -> on dinh du tuong phan thap.
    # Tra ve so trong [-2000, 2000]: am = lech trai, duong = lech phai, 0 = giua.
    # Tra None khi mat line (de nguoi dung / drivebase tu xu ly).
    # GIU NGUYEN: doc I2C doc lap -> tuong thich code cu.
    def position(self):
        v = self._normalized()
        total = sum(v)
        if total == 0 or max(v) < _FLOOR:
            return None
        acc = 0
        for i in range(5):
            acc += _WEIGHTS[i] * v[i]
        return acc // total

    # ---- trang thai roi rac (tuong thich drivebase.follow_line) ------------
    #   -3..-1 trai, 0 giua, 1..3 phai, LINE_CROSS, LINE_END
    # GIU NGUYEN: doc I2C doc lap -> tuong thich code cu.
    def check(self):
        v = self._normalized()
        peak = max(v)
        if peak < _FLOOR:
            return LINE_END
        # nhieu mat cung trum manh -> vach ngang
        if sum(1 for x in v if x > _HIGH) >= 4:
            return LINE_CROSS
        total = sum(v)
        if total == 0:
            return LINE_END
        acc = 0
        for i in range(5):
            acc += _WEIGHTS[i] * v[i]
        p = acc // total                    # thang +-2000
        if p <= -1500:
            return LINE_LEFT3
        elif p <= -750:
            return LINE_LEFT2
        elif p < -250:
            return LINE_LEFT
        elif p <= 250:
            return LINE_CENTER
        elif p < 750:
            return LINE_RIGHT
        elif p < 1500:
            return LINE_RIGHT2
        return LINE_RIGHT3

    # ---- dieu khien / cau hinh --------------------------------------------
    def set_white_led(self, on):
        self._write(_REG_LED_WHITE_EN, 1 if on else 0)

    def set_mode(self, raw):
        # raw=True -> mode RAW, raw=False -> mode TUPLE (luu de tuong thich)
        self._write(_REG_MODE, 1 if raw else 0)

    def set_reverse(self, on):
        # on=True -> line sang tren nen toi
        self._write(_REG_REVERSE_MODE, 1 if on else 0)

    def calibrate(self):
        # Kich calib qua I2C; thoat bang cach nhan nut tren board hoac sau 10s.
        # Sau khi calib xong nho goi refresh_calibration() de cap nhat muc smin/smax.
        self._write(_REG_CALIB_TRIGGER, 1)

    def is_calibrated(self):
        data = self._read(_REG_STATUS, 1)
        return bool(data[0] & 0x01) if data else False
