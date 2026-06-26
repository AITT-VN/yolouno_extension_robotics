# ============================================================================
#  FastLine - do line PID toc do cao cho YoloUNO (port tu STEM kit FastLine5).
#
#  KHAC ban STEM kit: dieu khien dong co qua DriveBase (robot.run_speed) thay vi
#  ghi PWM truc tiep -> chay dung tang dong co MDV cua YoloUNO. Cac ham do line
#  la ASYNC (khong chan vong lap asyncio cua firmware).
#
#  Dung lai cam bien da dang ky cho robot (robot.line_sensor(...)): chay duoc ca
#  cam bien 4 mat (LineSensorI2C) lan 5 mat (LineSensor5P_I2C). Che do 'raw' chi
#  dung duoc voi cam bien co read_raw (5 mat).
#
#  --- DUNG CHUNG ---
#     robot = DriveBase(...)                       # robotics_robot_init
#     line5 = LineSensor5P_I2C(); robot.line_sensor(line5)   # hoac ban 4 mat
#     fast_line = FastLine(robot)
#     fast_line.set_pid(0.5, 0, 0.3)
#     fast_line.set_speed(70)
#     await fast_line.follow_until_cross(15000, BRAKE)
#
#  --- HUONG DAN TINH CHINH PID ---
#  Loi error da chuan hoa ve khoang ~[-2, 2] (0 = giua line).
#     correction = 0  -> di thang ; = 1 -> pivot ; > 1 -> xoay tai cho.
#  => Om cua gat can correction ~1.0-1.5 luc error=2.0 => Kp ~ 0.5-0.75.
#  1) Dat Ki=0. Bat dau Kp ~0.5, Kd ~0.3.
#  2) Cua gat ma robot chay thang ra ngoai -> TANG Kp.
#  3) Duong thang ma robot lac qua lai -> TANG Kd (hoac giam Kp).
#  4) Robot vot qua line khi vao cua -> TANG curve_gain (0.8-0.9).
#  Bat set_debug(True) -> in log CSV roi gui cho AI de goi y Kp/Ki/Kd.
#
#  --- DINH DANG LOG DEBUG (CSV) ---
#  FASTLINE,t_ms,s0,s1,s2,s3,s4,error,P,I,D,correction,m1,m2
# ============================================================================
import time
import asyncio
from constants import *   # STOP, BRAKE, LINE_CROSS, ...

# chu ky vong lap do line (ms). 5ms = 200Hz: du nhanh, dung toc do cam bien I2C
# (STM32G030 khong cap nhat o 1kHz nen loop 1ms chi doc lai data cu + nhieu D-term).
_LOOP_MS = 5

# gioi han chong tich luy qua muc (integral windup) khi dung Ki
_INTEGRAL_LIMIT = 1000.0

# trong so truc S1..S5 cho centroid analog (thang [-2, 2])
_WEIGHTS = (-2.0, -1.0, 0.0, 1.0, 2.0)

# range raw toi thieu cua 1 mat de coi la "tin cay" (duoi nguong = nhieu, bo qua)
_MIN_RANGE = 120
# calib coi la DAT neu mat tot nhat co range >= nguong nay (line/nen tuong phan ro)
_GOOD_RANGE = 300


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class FastLine:
    def __init__(self, robot, mode='digital'):
        # dung lai DriveBase + cam bien da dang ky cho robot
        self.robot = robot

        # che do doc vi tri line:
        #  'digital' -> centroid tu 0/1 (on dinh, khong can calib)
        #  'raw'     -> centroid analog lien tuc (muot hon) NHUNG phai calibrate() truoc
        #               (chi cam bien 5 mat co read_raw)
        self.mode = mode

        # calib cho che do 'raw'
        self._cal_min = [4095, 4095, 4095, 4095, 4095]
        self._cal_max = [0, 0, 0, 0, 0]
        self._line_high = True
        self._calibrated = False
        self._last_aerr = 0.0
        self.lost_threshold = 0.5

        # he so PID mac dinh (thang error ~[-2, 2]).
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.3
        self.invert = 1          # +1 mac dinh; -1 neu robot lai nguoc huong

        # toc do
        self.base_speed = 60
        self.max_speed = 100

        # giam toc tien khi |error| lon (vao cua / mat line). 0 = khong giam;
        # 1 = mat line thi XOAY TAI CHO. Thuong 0.6-0.9.
        self.curve_gain = 0.7

        # vung chet (deadband): |error| <= nguong nay -> coi nhu di thang.
        # Cam bien digital 5 mat: error nho nhat khac 0 la 0.5 (line nam giua 2 mat).
        # deadband=0.3 -> van sua lech 0.5 o cua (tranh bo qua tin hieu truoc cua),
        # nhung bo qua nhieu nho (<0.3). Tang len 0.5 neu muon duong thang muot tuyet doi.
        self.deadband = 0.3

        # trang thai PID
        self.last_error = 0.0
        self.integral = 0.0
        self._d_err = 0.0       # khau D sau khi loc low-pass (IIR)
        self.d_alpha = 0.25     # he so loc cho khau D (time constant ~ 20ms)
        self._lost_start = -1   # timestamp ms khi bat dau mat line (-1 = dang bam)
        self._lost_dir = 1      # huong xoay khi mat line
        self._last_seen_error = 0.0

        # IIR (EMA) lam muot loi do line: loai bo nhieu +/-0.5 khi di thang ma khong lam
        # cham phan ung vao cua. alpha=0.35: lam muot rat tot de tranh giat cuc khi chay digital.
        self._ema_err = 0.0
        self.ema_alpha = 0.35

        # GIOI HAN LAI khi DANG BAM line (khac luc mat line):
        #  - correction_limit: chan correction -> gioi han do ngat lai toi da
        #  - turn_gain: he so lai < 1 -> giam manh lai de khi di thang 2 banh luon tien
        #  base=80, turn_gain=0.8, corr<=0.9 -> turn=57.6 -> banh trong = fwd-turn.
        #  Tai cua (err=1.5): fwd=35, turn=57.6 -> banh trong = -22 (lui nhe = om cua tat).
        self.correction_limit = 0.9
        self.turn_gain = 0.8

        # SAN bu ma sat (stall floor) cho do line. Bu ma sat anh xa [0,100] -> [floor,100],
        # va [-100,0] -> [-100,-floor]: moi gia tri AM du nho deu bi day xuong <= -floor.
        # => floor cao bien banh trong "lui nhe" (arc om cua) thanh "lui manh" (xoay tai cho)
        #    -> robot tich van toc goc nhanh -> vot qua line. Controller cu KHONG co san nay.
        # Mac dinh 0 = passthrough (giong code cu, om cua em). Nang ~10-20 neu banh bi ket
        # o toc do rat thap (motor khong quay duoi nguong PWM). None = dung robot._min_speed.
        self.stall_floor = 0

        # phan tien len khi mat line (0 = xoay tai cho, 0.15 = tien nhe roi xoay).
        # Giam "khung" khi chuyen tu bam line -> mat line. Va giup robot "arc" vao goc
        # thay vi xoay tai cho -> it khi bi lac qua line khi xoay. Mac dinh 0.45.
        self._lost_fwd = 0.45

        # xac nhan vach ngang (debounce): so khung lien tiep de coi la CROSS that.
        # 1 frame = 5ms. Mac dinh 2 = 10ms. Tang neu hay phat hiem sai; giam neu to chuc chap.
        self.cross_confirm = 2

        # debug
        self.debug = False
        self.debug_interval = 100   # ms giua 2 lan in
        self._last_dbg = 0

    # cam bien hien tai (lay lazily -> co the dang ky sensor sau khi tao FastLine)
    @property
    def sensor(self):
        return self.robot._line_sensor

    # ---------------- cau hinh ----------------
    def set_pid(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_speed(self, speed, max_speed=100):
        self.base_speed = speed
        self.max_speed = max_speed

    def set_curve_gain(self, gain):
        self.curve_gain = _clamp(gain, 0, 1)

    def set_deadband(self, db):
        # |error| <= db -> di thang (chong giat khi bam thang). 0 = tat.
        self.deadband = db

    def set_turn_gain(self, gain, correction_limit=0.8):
        # gain < 1 -> khi bam line 2 banh luon tien (muot). gain lon -> be cua manh hon.
        self.turn_gain = gain
        self.correction_limit = correction_limit

    def set_stall_floor(self, v):
        # san bu ma sat (motor % toi thieu khi output khac 0). None = dung robot._min_speed.
        # Ha thap (vd 20-25) neu lai bi nen / recovery giat; nang neu motor bi ket o toc do thap.
        self.stall_floor = None if v is None else max(0, int(v))

    def set_ema_alpha(self, alpha):
        # he so lam muot IIR: 0.5 = muot hon, 1.0 = tat IIR (dung thang). Mac dinh 0.7.
        self.ema_alpha = _clamp(float(alpha), 0.1, 1.0)

    def set_d_alpha(self, alpha):
        # loc khau D: thap (0.2) = D muot/yeu, cao (0.6-0.8) = D nhanh/manh (hai dam hon).
        # Tang neu robot overshoot/lac do thieu dam; giam neu D rung lat ban-bang.
        self.d_alpha = _clamp(float(alpha), 0.1, 1.0)

    def set_lost_fwd(self, ratio):
        # ty le toc tien giu lai khi mat line (arc recovery). 0 = xoay tai cho;
        # 0.4-0.5 = vua tien vua be (lai vao line, tien doc track). Mac dinh 0.45.
        self._lost_fwd = _clamp(float(ratio), 0.0, 1.0)

    def set_cross_debounce(self, n):
        # so khung CROSS lien tiep de xac nhan vach ngang that (tranh phat hien sai o duong cua).
        # 1 frame = 5ms. Tang len 8-10 neu track co nhieu cua gat.
        self.cross_confirm = max(1, int(n))

    def set_invert(self, invert):
        self.invert = 1 if invert >= 0 else -1

    def set_mode(self, mode):
        # 'digital' (mac dinh, on dinh) hoac 'raw' (analog, can calibrate() + read_raw)
        self.mode = mode
        if mode == 'raw':
            s = self.sensor
            if not hasattr(s, 'read_raw'):
                print('FastLine: cam bien nay khong co RAW -> dung digital.')
                self.mode = 'digital'
            elif not self._calibrated:
                print('FastLine: che do raw chua calibrate -> tam dung digital. Goi calibrate() truoc.')

    async def calibrate(self, seconds=3, spin=65):
        # Hoc nguong cho che do 'raw': robot TU XOAY de quet 5 mat qua line + nen.
        # Chi dung duoc voi cam bien co read_raw (5 mat).
        s = self.sensor
        if not hasattr(s, 'read_raw'):
            print('FastLine: cam bien khong ho tro RAW, bo qua calibrate.')
            return
        self._cal_min = [4095, 4095, 4095, 4095, 4095]
        self._cal_max = [0, 0, 0, 0, 0]
        on_total = 0
        on_count = 0
        off_total = 0
        off_count = 0
        duration = int(seconds * 1000)
        half = duration // 2
        start = time.ticks_ms()
        self.robot.run_speed(spin, -spin)        # xoay tai cho
        flipped = False
        while time.ticks_diff(time.ticks_ms(), start) < duration:
            if not flipped and time.ticks_diff(time.ticks_ms(), start) > half:
                self.robot.run_speed(-spin, spin)   # xoay nguoc lai de quet day du
                flipped = True
            raw = s.read_raw()
            dig = s.read()
            for k in range(5):
                v = raw[k]
                if v < self._cal_min[k]:
                    self._cal_min[k] = v
                if v > self._cal_max[k]:
                    self._cal_max[k] = v
                if dig[k]:
                    on_total += v
                    on_count += 1
                else:
                    off_total += v
                    off_count += 1
            await asyncio.sleep_ms(5)
        self.robot.stop()
        # xac dinh chieu: tren-line cho raw cao hay thap (dua vao digital dang tin)
        if on_count > 0 and off_count > 0:
            self._line_high = (on_total / on_count) > (off_total / off_count)
        # kiem tra chat luong calib
        best_range = 0
        for k in range(5):
            r = self._cal_max[k] - self._cal_min[k]
            if r > best_range:
                best_range = r
        if best_range < _GOOD_RANGE:
            self._calibrated = False
            print('FastLine CALIB KEM (range tot nhat %d < %d). Tam dung DIGITAL.' % (best_range, _GOOD_RANGE))
        else:
            self._calibrated = True
            print('FastLine calib OK. min=%s max=%s line_high=%s' % (
                self._cal_min, self._cal_max, self._line_high))

    def set_debug(self, on):
        self.debug = bool(on)
        if self.debug:
            print('FASTLINE,t_ms,s0,s1,s2,s3,s4,error,P,I,D,correction,m1,m2')

    def set_debug_interval(self, ms):
        self.debug_interval = int(ms)

    def reset_pid(self):
        self.last_error = 0.0
        self.integral = 0.0
        self._lost_start = -1
        self._ema_err = 0.0
        self._d_err = 0.0

    # ---------------- doc gia tri ----------------
    def error(self):
        # loi line da chuan hoa ~[-2, 2] (0 = giua line)
        return self._read_error()

    def read(self, index=None):
        # 'raw' -> analog (read_raw); 'digital' -> 0/1 (read). index None = ca mang.
        s = self.sensor
        if self.mode == 'raw' and hasattr(s, 'read_raw'):
            return s.read_raw(index)
        return s.read(index)

    def _read_error(self):
        s = self.sensor
        if self.mode == 'raw' and self._calibrated and hasattr(s, 'read_raw'):
            return self._analog_error()
        # digital: get_error ~[-2000, 2000] cua cam bien -> /1000 -> [-2, 2]
        if hasattr(s, 'get_error'):
            err = s.get_error() / 1000.0
            # pattern==0 duoc xu ly rieng trong step()
            # khong ep thanh error +/-2 o day vi se lam doi huong recovery sai.
            return err
        return 0.0

    def _analog_error(self):
        # centroid analog lien tuc tu read_raw + calib -> muot hon digital
        raw = self.sensor.read_raw()
        acc = 0.0
        tot = 0.0
        for k in range(5):
            rng = self._cal_max[k] - self._cal_min[k]
            if rng < _MIN_RANGE:
                n = 0.0            # mat nay tuong phan qua kem -> bo qua (nhieu)
            else:
                n = (raw[k] - self._cal_min[k]) / rng
                if not self._line_high:
                    n = 1.0 - n
                n = _clamp(n, 0.0, 1.0)
            acc += n * _WEIGHTS[k]
            tot += n
        if tot < self.lost_threshold:
            # mat line -> giu huong cu
            if self._last_aerr > 0:
                return 2.0
            elif self._last_aerr < 0:
                return -2.0
            return 0.0
        err = acc / tot
        self._last_aerr = err
        return err

    # ---------------- 1 vong PID ----------------
    def step(self):
        s = self.sensor
        if hasattr(s, 'update'):
            s.update()                      # refresh cache (centroid/checkpoint)

        # "Mat line" = tat ca mat tat (pattern==0). Dung pattern, KHONG dung error==2,
        # vi error==2 cung xay ra khi chi co 1 mat ria thay line (van dang bam).
        pattern = s.get_pattern() if hasattr(s, 'get_pattern') else 1
        lost = (self.mode != 'raw') and (pattern == 0)

        if lost:
            # === MAT LINE: xoay KHOA theo huong cuoi cung thay line ===
            # Khong doi chieu theo nhieu -> het lat trai/phai bao luc.
            if self._lost_start < 0:
                self._lost_start = time.ticks_ms()
            lost_ms = time.ticks_diff(time.ticks_ms(), self._lost_start)
            # be cang gat dan neu mat line lau hon (luc dau arc nhe, sau moi siet)
            mag = 0.7 if lost_ms < 400 else 0.95

            # dung huong cuoi cung thay line
            correction = self._lost_dir * mag

            # ARC tien (giong controller cu): GIU toc tien that de luon lai vao line
            # va tien doc track, thay vi xoay tai cho -> tranh spin lo qua line.
            recovery_base = min(self.base_speed, 50)
            fwd = recovery_base * self._lost_fwd
            turn = correction * recovery_base
            error = 2.0 * self._lost_dir
            p = i = d = 0.0
            # GIU integral/last_error -> khi bat lai line khong bi D-kick
        else:
            # === DANG BAM LINE ===
            raw_err = self._read_error()       # ~[-2, 2], tu cam bien

            # IIR lam muot error de giam nhieu +/-0.5 khi di thang:
            # - nhieu doi chieu (+-0.5, -0.5, ...) -> EMA giam dan ve 0 -> deadband loc het
            # - cua lien tuc -> EMA tiep can gia tri that sau 3-4 frame (15ms)
            # - nhay dot ngot lon (phat lai line) -> snap ngay, khong cho tich luy
            if abs(raw_err - self._ema_err) > 1.5:
                self._ema_err = raw_err   # snap: khong cho EMA cu keo sai
            else:
                self._ema_err = self.ema_alpha * raw_err + (1.0 - self.ema_alpha) * self._ema_err
            error = self._ema_err

            # cap nhat huong xoay va fwd-speed tu RAW (khong qua IIR, phan ung ngay)
            if raw_err > 0.01:
                self._lost_dir = 1
                self._last_seen_error = raw_err
            elif raw_err < -0.01:
                self._lost_dir = -1
                self._last_seen_error = raw_err

            if self._lost_start >= 0:
                # vua bat lai line sau khi mat -> snap EMA + reset de khong giat
                self._ema_err = raw_err
                error = raw_err
                self.last_error = raw_err
                self.integral = 0.0
                self._d_err = 0.0
                self._lost_start = -1

            # deadband ap dung cho CA P, I, D: trong vung chet correction = 0 tuyet doi
            # -> di THANG TAP, khong giat theo nhieu luong tu hoa (+/-0.5) cua cam bien so.
            e = error
            if -self.deadband <= e <= self.deadband:
                e = 0.0

            # D-term tinh tren error DA qua deadband + loc thong thap (Filtered Derivative)
            d_raw = self.kd * (e - self.last_error)
            self.last_error = e
            self._d_err = self.d_alpha * d_raw + (1.0 - self.d_alpha) * self._d_err
            d = self._d_err

            self.integral = _clamp(self.integral + e, -_INTEGRAL_LIMIT, _INTEGRAL_LIMIT)
            p = self.kp * e
            i = self.ki * self.integral
            correction = _clamp((p + i + d) * self.invert,
                                -self.correction_limit, self.correction_limit)

            # giam toc tien dua tren RAW error (phan ung ngay vao cua, khong cho IIR tre)
            ae = abs(raw_err)
            if ae > 2.0:
                ae = 2.0
            fwd = self.base_speed * (1.0 - self.curve_gain * ae / 2.0)
            # turn_gain < 1 -> 2 banh luon tien khi bam line (muot, khong dao chieu)
            turn = correction * self.base_speed * self.turn_gain

        left_raw = _clamp(fwd + turn, -self.max_speed, self.max_speed)
        right_raw = _clamp(fwd - turn, -self.max_speed, self.max_speed)

        # Bu ma sat (Stall/Deadband Compensation) cho dong co DC/TT tren YoloUNO
        min_start = self.stall_floor if self.stall_floor is not None else getattr(self.robot, '_min_speed', 35)

        if abs(left_raw) < 1.0:
            left = 0.0
        elif left_raw > 0:
            left = min_start + (self.max_speed - min_start) * (left_raw / self.max_speed)
        else:
            left = -min_start + (self.max_speed - min_start) * (left_raw / self.max_speed)

        if abs(right_raw) < 1.0:
            right = 0.0
        elif right_raw > 0:
            right = min_start + (self.max_speed - min_start) * (right_raw / self.max_speed)
        else:
            right = -min_start + (self.max_speed - min_start) * (right_raw / self.max_speed)

        self.robot.run_speed(left, right)

        if self.debug:
            self._print_dbg(error, p, i, d, correction, left, right)
        return error

    def _print_dbg(self, error, p, i, d, correction, m1, m2):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_dbg) < self.debug_interval:
            return
        self._last_dbg = now
        s = self.sensor
        pat = s.get_pattern() if hasattr(s, 'get_pattern') else 0
        print('FASTLINE,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d' % (
            now, pat & 1, (pat >> 1) & 1, (pat >> 2) & 1, (pat >> 3) & 1, (pat >> 4) & 1,
            error, p, i, d, correction, int(m1), int(m2)))

    # ---------------- nhan biet vach ngang ----------------
    def _at_cross(self):
        s = self.sensor
        if hasattr(s, 'detect_checkpoint'):
            return s.detect_checkpoint() == LINE_CROSS
        return s.check() == LINE_CROSS

    # ---------------- cac che do do line (async) ----------------
    async def follow_delay(self, seconds, then=STOP):
        # do line trong N giay roi dung
        self.reset_pid()
        timeout = int(seconds * 1000)
        last_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), last_time) < timeout:
            self.step()
            await asyncio.sleep_ms(_LOOP_MS)
        await self.stop(then)

    async def follow_until_cross(self, timeout=10000, then=STOP):
        # do line den khi gap vach ngang (cross).
        # Bo qua cross luc moi bat dau (status=1 -> 2 sau khi khong con cross).
        # Yeu cau cross_confirm khung lien tiep de tranh phat hien sai tai cua gat.
        self.reset_pid()
        status = 1
        cross_count = 0
        last_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), last_time) < timeout:
            self.step()
            cross = self._at_cross()
            if status == 1:
                if not cross:
                    status = 2
                    cross_count = 0
            elif status == 2:
                if cross:
                    cross_count += 1
                    if cross_count >= self.cross_confirm:
                        break
                else:
                    cross_count = 0
            await asyncio.sleep_ms(_LOOP_MS)
        await self.stop(then)

    async def follow_until(self, condition, timeout=10000, then=STOP):
        # do line den khi condition() tra ve True (debounce 2 lan lien tiep)
        self.reset_pid()
        count = 0
        last_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), last_time) < timeout:
            self.step()
            if condition():
                count += 1
                if count >= 2:
                    break
            else:
                count = 0
            await asyncio.sleep_ms(_LOOP_MS)
        await self.stop(then)

    # ---------------- dung ----------------
    async def stop(self, then=STOP):
        await self.robot.stop_then(then)
