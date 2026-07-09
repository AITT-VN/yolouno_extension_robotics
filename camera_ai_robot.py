"""
camera_ai_robot.py - Thu vien MicroPython cho Camera AI Robot (Yolo UNO, app.ohstem.vn)
Dieu khien robot qua UART ket noi voi Camera AI.
Dong co dieu khien qua doi tuong DriveBase cua thu vien ROBOTICS (Yolo UNO):
  - Tao robot bang block cua thu vien Robotics:  robot = DriveBase(...)
  - Goi block "dung robot Camera AI" -> camera_ai_set_robot(robot)
"""

import time
import machine

# Bien toan cuc UART
_uart = None
_last_recv_time = 0

# Cac nhan goi tu Camera AI (giao thuc "tu mo ta"): "NHAN:du_lieu"
_KNOWN_TAGS = ('OBJ', 'LINE', 'FACE', 'COLOR', 'CLASS', 'TRACK', 'QR')


def _split_tag(text):
    # Tach "NHAN:du_lieu" -> (nhan, du_lieu). Khong co nhan hop le -> ('', text).
    i = text.find(':')
    if i > 0:
        tag = text[:i].strip().upper()
        if tag in _KNOWN_TAGS:
            return tag, text[i + 1:].strip()
    return '', text

# Cua so on dinh khoang cach
_dist_window = []
_DIST_WINDOW_SIZE = 8
_DIST_VAR_MAX = 2.5
_is_dist_stable = False


def camera_ai_init(rx_pin, tx_pin, baudrate=115200):
    # Khoi tao UART noi Camera AI tren Yolo UNO.
    # rx_pin / tx_pin la chan GPIO cua Yolo UNO (vd D13_PIN, D12_PIN do block truyen vao).
    # Gop tat ca tham so vao 1 lan goi -> tranh .init() lam reset chan tren ESP32-S3.
    global _uart, _last_recv_time
    _uart = machine.UART(1, baudrate=baudrate, rx=rx_pin, tx=tx_pin,
                         bits=8, parity=None, stop=1)
    _last_recv_time = time.ticks_ms()
    return _uart


def camera_ai_uart():
    # Tra ve UART thu vien (do 'camera_ai_init' tao theo chan RX/TX da chon).
    # Tren Yolo UNO PHAI goi camera_ai_init(rx, tx) truoc; chua init -> tra None.
    return _uart


# ================================================================
# DIEU KHIEN DONG CO qua DriveBase cua thu vien ROBOTICS (Yolo UNO)
#   1) Tao robot bang block cua thu vien Robotics:  robot = DriveBase(...)
#   2) Goi block "dung robot Camera AI" -> camera_ai_set_robot(robot)
#   Thu vien camera dieu khien dong co bang robot.run_speed(L, R), toc do 0..100.
#   Chua gan robot -> moi lenh di chuyen bo qua (robot dung yen, khong loi).
# ================================================================
_robot = None


def camera_ai_set_robot(robot):
    # Gan doi tuong robot (DriveBase) de thu vien camera dieu khien dong co.
    global _robot
    _robot = robot


# Sàn tốc độ RIENG cho dò line: >0 -> dùng giá trị này (= speed_min của block dò line);
# 0 -> dùng robot._min_speed (mặc định, cho object tracking / lệnh robot khác).
_drive_floor_override = 0


def _eff_min():
    # Toc toi thieu HIEU LUC: uu tien _drive_floor_override (toc do RIENG cua block do line),
    # neu khong co thi dung robot._min_speed. -> do line TACH BIET hoan toan voi block robot.
    if _drive_floor_override > 0:
        return _drive_floor_override
    try:
        return int(getattr(_robot, '_min_speed', 0) or 0)
    except Exception:
        return 0


def _floor(speed):
    # Nang |speed| len it nhat = san toc do hieu luc (_eff_min). 0 van la 0 (dung).
    if speed == 0 or _robot is None:
        return speed
    m = _eff_min()
    if m <= 0:
        return speed
    if 0 < speed < m:
        return m
    if -m < speed < 0:
        return -m
    return speed


def _drive_fw(speed):
    if _robot:
        s = _floor(speed)
        _robot.run_speed(s, s)


def _drive_bw(speed):
    if _robot:
        s = _floor(speed)
        _robot.run_speed(-s, -s)


def _drive_left(speed):
    if _robot:
        s = _floor(speed)
        _robot.run_speed(-s, s)


def _drive_right(speed):
    if _robot:
        s = _floor(speed)
        _robot.run_speed(s, -s)


def _drive_stop():
    if _robot:
        _robot.stop()


def _wheel_floor(v):
    # Kep [-100,100]; nang |v| len >= _min_speed cua robot (dong co hop so can toc toi thieu);
    # |v| rat nho -> 0 (dung banh do -> robot pivot quanh banh con lai khi cua gat).
    iv = int(v)
    if iv > 100:
        iv = 100
    elif iv < -100:
        iv = -100
    m = _eff_min()
    if m <= 0:
        return iv
    if iv >= m or iv <= -m:
        return iv
    dead = m // 3
    if -dead < iv < dead:
        return 0
    return m if iv > 0 else -m


def _drive_run(left, right):
    # Lai vi sai: 2 banh chay toc do rieng (cho phep dung/dao chieu banh trong -> pivot).
    if _robot is None:
        return
    _robot.run_speed(_wheel_floor(left), _wheel_floor(right))


def _drive_curve(left, right):
    # Lai CUA can bang: banh trong duoc phep DUNG HAN (0) khi can re gat -> robot xoay quanh
    # banh trong (pivot nhe). KHONG dao chieu (khong lui) -> khong vot/lo. Banh ngoai cap o 100
    # (line_follow da cap rieng o speed_max truoc khi goi). Duoi nguong stall (>0 nhung < min)
    # -> nang len min de banh chiu quay.
    if _robot is None:
        return
    m = _eff_min()

    def w(v):
        v = int(v)
        if v > 100:
            v = 100
        if v <= 0:
            return 0          # khong lui -> DUNG (banh tru de robot xoay quanh)
        if m > 0 and v < m:
            return m          # >0 nhung duoi nguong quay -> nang len min (chong stall)
        return v

    _robot.run_speed(w(left), w(right))


def _drive_arc(left, right):
    # Lai vi sai GIU CA 2 BANH LUON TIEN (khong dung, khong dao chieu) -> om cua muot,
    # KHONG quay lo. Banh trong cham nhung van tien (>= _min_speed), banh ngoai nhanh hon (<=100).
    if _robot is None:
        return
    m = _eff_min()
    if m < 1:
        m = 1
    l = int(left)
    r = int(right)
    if l > 100:
        l = 100
    if r > 100:
        r = 100
    if l < m:
        l = m
    if r < m:
        r = m
    _robot.run_speed(l, r)


def camera_ai_read_data():
    # Doc goi moi nhat -> (offset, distance, confidence) hoac None
    global _uart, _last_recv_time
    last_data = None
    while _uart and _uart.any():
        last_data = _uart.readline()

    if last_data:
        try:
            text = last_data.decode('utf-8').strip()
            tag, payload = _split_tag(text)
            # Chi nhan du lieu object (OBJ / TRACK cu); bo qua LINE/FACE/COLOR...
            if tag and tag not in ('OBJ', 'TRACK'):
                return None
            _last_recv_time = time.ticks_ms()
            if 'lost' in payload.lower():
                return None
            values = payload.split(',')
            if len(values) >= 3:
                return (float(values[0]), float(values[1]), float(values[2]))
        except Exception:
            pass
    return None


def camera_ai_send_run():
    global _uart
    if _uart:
        _uart.write("RUN\n")


def camera_ai_send_stop():
    global _uart
    if _uart:
        _uart.write("STOP\n")


def camera_ai_robot_follow(speed_turn=35, speed_move=40,
                           dist_far=21, dist_near=18, offset_thresh=35):
    # Theo doi co ban (phan ung tuc thi). Khuyen dung camera_ai_smart_follow de muot hon.
    data = camera_ai_read_data()
    if data is None:
        _drive_stop()
        return

    offset, distance, conf = data
    if conf < 10:
        _drive_stop()
        return

    if offset < -offset_thresh:
        _drive_left(speed_turn)
    elif offset > offset_thresh:
        _drive_right(speed_turn)
    else:
        if distance > dist_far:
            _drive_fw(speed_move)
        elif distance < dist_near:
            _drive_bw(speed_move)
        else:
            _drive_stop()


# Ham noi bo on dinh khoang cach (Sliding Window Variance)
def _push_dist(new_dist):
    global _dist_window, _is_dist_stable
    _dist_window.append(new_dist)
    if len(_dist_window) > _DIST_WINDOW_SIZE:
        _dist_window.pop(0)
    if len(_dist_window) == _DIST_WINDOW_SIZE:
        mean = sum(_dist_window) / _DIST_WINDOW_SIZE
        sq_diff_sum = sum((x - mean) ** 2 for x in _dist_window)
        std_dev = (sq_diff_sum / _DIST_WINDOW_SIZE) ** 0.5
        _is_dist_stable = std_dev < _DIST_VAR_MAX
    else:
        _is_dist_stable = False


def _reset_dist_window():
    global _dist_window, _is_dist_stable
    _dist_window = []
    _is_dist_stable = False


def camera_ai_robot_follow_advanced(speed_turn=35, speed_move=40,
                                    dist_far=21, dist_near=18,
                                    offset_thresh=35, stable_thresh=800):
    data = camera_ai_read_data()
    if data is None:
        _drive_stop()
        _reset_dist_window()
        return {'action': 'stop', 'stable': False}

    offset, distance, conf = data
    if conf < 10:
        _drive_stop()
        _reset_dist_window()
        return {'action': 'stop', 'stable': False}

    _push_dist(distance)

    if offset < -offset_thresh:
        _drive_left(speed_turn)
        return {'action': 'left', 'stable': _is_dist_stable}
    elif offset > offset_thresh:
        _drive_right(speed_turn)
        return {'action': 'right', 'stable': _is_dist_stable}
    else:
        if _is_dist_stable:
            if distance > dist_far:
                _drive_fw(speed_move)
                return {'action': 'forward', 'stable': True}
            elif distance < dist_near:
                _drive_bw(speed_move)
                return {'action': 'backward', 'stable': True}
            else:
                _drive_stop()
                return {'action': 'stop', 'stable': True}
        else:
            _drive_stop()
            return {'action': 'stop', 'stable': False}


# ================================================================
# LOP BO DEM + LAM MUOT + ON DINH
# Goi camera_ai_update() 1 LAN o dau moi vong lap; sau do cac block
# 'muot' / 'on dinh' / getter dem doc tu bo nho (khong rut UART nhieu lan).
# ================================================================
_cur_offset = 0.0
_cur_distance = 0.0
_cur_conf = 0.0
_sm_offset = 0.0
_sm_distance = 0.0
_seen_count = 0          # so frame lien tuc thay vat on dinh
_miss_count = 0          # so frame lien tuc mat (nhan 'lost' hoac tin cay thap)
_ACQUIRE_MISS_RESET = 3  # mat qua 3 goi -> coi nhu mat han, reset bo dem on dinh


def camera_ai_update(min_conf=15, smooth=60):
    # Doc goi MOI NHAT, lam muot offset/khoang cach, cap nhat bo dem on dinh.
    # Goi 1 lan o dau moi vong lap. Tra ve so frame on dinh hien tai.
    # smooth (0-100): do muot. CAO = muot hon/cham hon/it rung; THAP = nhay hon/de rung.
    #   smooth=60 (mac dinh) tai lap dung he so cu: offset 0.6, distance 0.8.
    global _uart, _last_recv_time
    global _cur_offset, _cur_distance, _cur_conf, _sm_offset, _sm_distance
    global _seen_count, _miss_count

    # Ep kieu + kep bien an toan (phong loi goi tay / generator doi)
    try:
        a_off = float(smooth) / 100.0
    except Exception:
        a_off = 0.6
    if a_off < 0.0:
        a_off = 0.0
    elif a_off > 0.95:
        a_off = 0.95          # tran 0.95 -> luon giu >=5% gia tri moi, KHONG dong bang
    # distance luon muot hon offset (diem giua giua a_off va 1.0); tai a_off=0.6 -> 0.8
    a_dist = a_off + (1.0 - a_off) * 0.5

    last_line = None
    while _uart and _uart.any():
        last_line = _uart.readline()

    if last_line is None:
        return _seen_count  # khong co du lieu moi -> giu nguyen

    try:
        text = last_line.decode('utf-8').strip()
    except Exception:
        return _seen_count

    tag, payload = _split_tag(text)
    # Chi xu ly du lieu object (OBJ / TRACK cu); bo qua goi cua mode khac
    if tag and tag not in ('OBJ', 'TRACK'):
        return _seen_count

    _last_recv_time = time.ticks_ms()

    if ',' in payload:
        parts = payload.split(',')
        if len(parts) >= 3:
            try:
                offset = float(parts[0])
                distance = float(parts[1])
                conf = float(parts[2])
            except Exception:
                return _seen_count
            if conf >= min_conf:
                _cur_offset, _cur_distance, _cur_conf = offset, distance, conf
                _miss_count = 0
                if _seen_count == 0:
                    # frame dau thay lai -> nap thang gia tri that
                    _sm_offset = offset
                    _sm_distance = distance
                else:
                    _sm_offset = a_off * _sm_offset + (1.0 - a_off) * offset
                    _sm_distance = a_dist * _sm_distance + (1.0 - a_dist) * distance
                if _seen_count < 1000:
                    _seen_count += 1
                return _seen_count

    # 'lost' hoac tin cay thap
    _cur_conf = 0.0
    _miss_count += 1
    # mat vat -> tra ve 0 NGAY (khong giu gia tri cu) cho block doc offset/khoang cach
    # -> robot ngung xoay/tien ngay, khong xoay them 2-3 nhip thua
    _sm_offset = 0.0
    _sm_distance = 0.0
    if _miss_count >= _ACQUIRE_MISS_RESET:
        _seen_count = 0
    return _seen_count


def camera_ai_is_stable(min_conf=20, min_frames=4):
    # True neu dang thay vat ON DINH: tin cay >= min_conf va lien tuc >= min_frames frame
    return _seen_count >= min_frames and _cur_conf >= min_conf


def camera_ai_offset_smooth():
    # Do lech da lam muot. TU goi update -> dung DOC LAP duoc (khong can khoi
    # "bam theo vat the" chay cung trong vong lap).
    camera_ai_update()
    return _sm_offset


def camera_ai_distance_smooth():
    # Khoang cach (cm) da lam muot. TU goi update -> dung DOC LAP duoc.
    camera_ai_update()
    return _sm_distance


# Alias cho ban generator moi (block "do lech / khoang cach vat the")
def camera_ai_offset_read():
    camera_ai_update()
    return _sm_offset


def camera_ai_distance_read():
    camera_ai_update()
    return _sm_distance


# CHONG QUAN TINH: khi dang TIEN/LUI ma vat lech can quay -> phanh dung
# _OBJ_TURN_DELAY_MS ms cho tat quan tinh roi moi quay -> khong queo lo rong.
# Tang -> phanh lau hon (queo gon hon nhung phan ung cham hon); Giam -> nhanh hon.
_OBJ_TURN_DELAY_MS = 250
_obj_move_mode = 'stop'   # trang thai frame truoc: 'move' (dang tien/lui) | 'turn' | 'stop'
_obj_brake_since = 0      # moc thoi gian bat dau phanh truoc khi quay


def camera_ai_smart_follow(speed_turn=35, speed_move=40, dist_far=21,
                           dist_near=18, pulse_ms=60, offset_thresh=25,
                           min_conf=20, stable_frames=4):
    # Theo doi MUOT: tu goi camera_ai_update, CHI hanh dong khi vat on dinh,
    # dung du lieu da lam muot -> het queo/tien lui bay luc vua thay lai vat.
    global _drive_floor_override, _obj_move_mode, _obj_brake_since
    _drive_floor_override = 0   # object tracking dung robot._min_speed (khong dung san rieng cua line)
    camera_ai_update(min_conf)
    if not camera_ai_is_stable(min_conf, stable_frames):
        _drive_stop()  # chua on dinh / mat vat -> dung yen cho
        _obj_move_mode = 'stop'
        _obj_brake_since = 0
        return
    off = _sm_offset
    dist = _sm_distance
    ao = off if off >= 0 else -off
    if ao > offset_thresh:
        # CHONG QUAN TINH: neu vua chuyen tu 'move' (dang tien/lui) -> phanh delay truoc
        if _obj_move_mode == 'move':
            if _obj_brake_since == 0:
                _obj_brake_since = time.ticks_ms()
            if time.ticks_diff(time.ticks_ms(), _obj_brake_since) < _OBJ_TURN_DELAY_MS:
                _drive_stop()   # phanh, cho quan tinh tien tat het roi moi quay
                return
        _obj_move_mode = 'turn'
        _obj_brake_since = 0
        # ===== XOAY CANH GIUA bang NHIP TI LE (pulse) =====
        # Dong co hop so khong quay cham -> xoay tung NHIP NGAN, do dai nhip ti le do lech:
        #   lech nhieu -> xoay gan nhu lien tuc; gan giua -> xoay rat it roi NGHI.
        # Nho vay banh van quay o toc du manh (khong re re) nhung GOC XOAY moi chu ky nho
        # -> KHONG quay lo, tu can vat ve giua. Nghi giua nhip cho camera doc lai.
        PERIOD = 200          # ms / chu ky nhip
        SPAN = 90.0            # do lech (tren nguong) de nhip dat 100% (xoay lien tuc)
        frac = (ao - offset_thresh) / SPAN
        if frac > 1.0:
            frac = 1.0
        on_ms = int(frac * PERIOD)
        if on_ms < pulse_ms:
            on_ms = pulse_ms   # do rong moi nhip quay (block "nhip quay ms") - lon hon = quay rong hon
        if (time.ticks_ms() % PERIOD) < on_ms:
            if off < 0:
                _drive_left(speed_turn)
            else:
                _drive_right(speed_turn)
        else:
            _drive_stop()      # NGHI -> camera doc lai, chong quay lo
    else:
        # ===== DA CANH GIUA -> giu khoang cach (tien/lui) - phan nay dang on =====
        _obj_brake_since = 0
        if dist > dist_far:
            _drive_fw(speed_move)
            _obj_move_mode = 'move'   # dang tien -> lan sau muon quay phai phanh delay truoc
        elif dist < dist_near:
            _drive_bw(speed_move)
            _obj_move_mode = 'move'   # dang lui -> tuong tu
        else:
            _drive_stop()
            _obj_move_mode = 'stop'   # dung yen -> muon quay thi quay ngay (khong quan tinh)


# ================================================================
# THEO DOI VAT THE tach 2 khoi: cau hinh (dat o 'bat dau') + hanh dong (trong loop)
# ================================================================
_obj_dist_far = 22       # giu khoang cach: xa (cm)
_obj_dist_near = 18      # giu khoang cach: gan (cm)
_obj_pulse_ms = 120      # do rong moi nhip quay canh giua (ms)


def camera_ai_object_config(dist_far, dist_near, pulse_ms):
    # Dat khoang cach giu (xa/gan, cm) + do rong nhip quay (ms) cho 'theo doi vat the'.
    # Chi luu cau hinh -> dat 1 lan o 'bat dau'.
    global _obj_dist_far, _obj_dist_near, _obj_pulse_ms
    _obj_dist_far = dist_far
    _obj_dist_near = dist_near
    _obj_pulse_ms = pulse_ms


def camera_ai_track(speed):
    # THEO DOI VAT THE (non-blocking, bat/tat duoc).
    # TOC DO QUAY = toc do bam +3 (quay khoe hon, du luc vuot go/ma sat, khong re re).
    # Dung khoang cach xa/gan + nhip quay tu block "giu khoang cach vat the" (mac dinh 22/18/120).
    camera_ai_smart_follow(speed + 3, speed, _obj_dist_far, _obj_dist_near, _obj_pulse_ms)


def camera_ai_object_track(speed_turn=35, speed_move=40, dist_far=21, dist_near=18):
    # ALL-IN-ONE theo doi vat the (vong lap VO HAN): cho camera san sang -> gui RUN ->
    # bam vat lien tuc. Dung UART tu camera_ai_init (chua co -> tu dung P13/P14).
    # Gia tri co dinh: STABLE_FRAMES=5, MIN_CONF=60, OFFSET_THRESH=35.
    try:
        from led_onboard import led_onboard
    except ImportError:
        try:
            import led_onboard
        except ImportError:
            class DummyLED:
                def show(self, index, color):
                    pass
            led_onboard = DummyLED()

    uart = camera_ai_uart()
    if uart is None:
        return   # chua khoi tao Camera AI -> dat block "khoi tao camera ai" truoc

    SPEED_TURN = speed_turn
    SPEED_MOVE = speed_move
    DIST_FAR = dist_far
    DIST_NEAR = dist_near
    STABLE_FRAMES = 5
    MIN_CONF = 60
    OFFSET_THRESH = 35
    ACQUIRE_MISS_RESET = 3
    COMM_TIMEOUT = 500
    FRESH_TIMEOUT = 150

    seen_count = 0
    miss_count = 0
    sm_offset = 0.0
    sm_distance = 0.0
    cur_conf = 0.0

    # Cho Camera AI san sang
    valid = 0
    while valid < 8:
        if uart and uart.any():
            d = uart.readline()
            try:
                t = d.decode('utf-8').strip()
                if ',' in t or 'lost' in t.lower():
                    valid += 1
            except Exception:
                pass
        time.sleep_ms(30)

    # Bat dau theo doi NGAY (khong dung nut A)
    uart.write("RUN\n")
    led_onboard.show(0, (0, 255, 0))
    time.sleep_ms(300)
    led_onboard.show(0, (0, 0, 0))
    last_recv_time = time.ticks_ms()
    last_valid_time = time.ticks_ms()

    while True:
        # Doc goi moi nhat
        last_line = None
        while uart.any():
            last_line = uart.readline()
        if last_line is not None:
            try:
                text = last_line.decode('utf-8').strip()
                if ':' in text:
                    text = text.split(':', 1)[1]   # bo nhan "OBJ:" -> con "offset,distance,conf"
                last_recv_time = time.ticks_ms()
                if ',' in text:
                    parts = text.split(',')
                    if len(parts) >= 3:
                        offset = float(parts[0])
                        distance = float(parts[1])
                        conf = float(parts[2])
                        if conf >= MIN_CONF:
                            miss_count = 0
                            cur_conf = conf
                            last_valid_time = time.ticks_ms()
                            if seen_count == 0:
                                sm_offset = offset
                                sm_distance = distance
                            else:
                                sm_offset = 0.5 * sm_offset + 0.5 * offset
                                sm_distance = 0.8 * sm_distance + 0.2 * distance
                            if seen_count < 1000:
                                seen_count += 1
                        else:
                            cur_conf = 0
                            miss_count += 1
                            if miss_count >= ACQUIRE_MISS_RESET:
                                seen_count = 0
                else:
                    cur_conf = 0
                    miss_count += 1
                    if miss_count >= ACQUIRE_MISS_RESET:
                        seen_count = 0
            except Exception:
                pass

        # Chi hanh dong khi vat ON DINH
        stable = (seen_count >= STABLE_FRAMES) and (cur_conf >= MIN_CONF)
        fresh = time.ticks_diff(time.ticks_ms(), last_valid_time) <= FRESH_TIMEOUT
        comm_lost = time.ticks_diff(time.ticks_ms(), last_recv_time) > COMM_TIMEOUT

        if comm_lost:
            _drive_stop()
            led_onboard.show(0, (255, 0, 0))   # mat ket noi camera -> dung an toan
        elif (not stable) or (not fresh):
            _drive_stop()                        # mat vat / du lieu cu / chua on dinh -> DUNG ngay
            led_onboard.show(0, (0, 0, 0))
        else:
            led_onboard.show(0, (0, 0, 0))
            abs_off = abs(sm_offset)
            if abs_off <= OFFSET_THRESH:
                # DA CAN GIUA -> chi chinh khoang cach (tien/lui)
                if sm_distance > DIST_FAR:
                    _drive_fw(SPEED_MOVE)
                elif sm_distance < DIST_NEAR:
                    _drive_bw(SPEED_MOVE)
                else:
                    _drive_stop()
            else:
                # QUAY TY LE voi do lech -> robot tu can bang vao giua
                TURN_MIN = 18
                span = 120.0 - OFFSET_THRESH
                if span < 1.0:
                    span = 1.0
                frac = (abs_off - OFFSET_THRESH) / span
                if frac > 1.0:
                    frac = 1.0
                tspeed = int(TURN_MIN + frac * (SPEED_TURN - TURN_MIN))
                if sm_offset < 0:
                    _drive_left(tspeed)
                else:
                    _drive_right(tspeed)
        time.sleep_ms(20)


# ================================================================
# DO LINE + PHAT HIEN GIAO NHAU (giao lo / nga ba, nga tu)
# Camera gui qua UART: offset "%.1f", "LINE: lost", va "CROSS" khi thay
# vach ngang cat qua. Block camera_ai_line_follow() tu doc UART va bam line PD;
# dung camera_ai_junction_is() / camera_ai_at_junction() de xu ly giao lo.
# ================================================================
_cross_time = 0
_line_offset = 0.0
_line_offset_smooth = 0.0   # do lech line DA LAM MUOT (EMA) - cho quyet dinh re/giu thang
_LINE_SM_ALPHA = 0.6        # do muot 0..0.95: cao hon = muot hon nhung phan ung cham hon
_line_angle = 0.0     # goc nghieng line (do): + = nghieng phai
_line_seen = False
_junction = ''        # 'CROSS4','CROSS3L','CROSS3R','CURVE_L','CURVE_R' hoac ''
_junction_time = 0

# --- He so dieu khien PD: 2 BO (gain scheduling) ---
# Robot tu nhan biet dang CHAY THANG hay VAO CUA roi doi bo so tuong ung:
#   - Thang: Kp nho -> em, khong rung; giu giua line & thoat cua khong lech.
#   - Cua:   Kp lon -> cua dut khoat, qua duoc cua gat (camera xa van kip lai).
_kp_str = 0.45        # Kp khi CHAY THANG
_kd_str = 0.6         # Kd khi CHAY THANG
_kp_cur = 0.9         # Kp khi VAO CUA (manh hon)
_kd_cur = 1.0         # Kd khi VAO CUA
_line_kp = _kp_str    # he so dang dung (tu chon theo vung)
_line_kd = _kd_str
_in_curve = False     # trang thai vung hien tai (co hysteresis chong rung)

# Nguong nhan biet cua theo do lech THO |offset| (co hysteresis):
_CURVE_ON = 20.0      # |offset| > 20 -> coi nhu VAO CUA (nhan SOM de kip giam toc + lai manh, chong van line)
_CURVE_OFF = 12     # |offset| < 15 -> coi nhu da ve CHAY THANG
# Chi ghi nho HUONG LINE khi line lech RO RANG ve 1 ben (|offset| > nguong nay).
# Gan giua offset nho + nhieu -> de doan sai huong; nguong nay loc bo -> tim line dung huong hon.
_DIR_COMMIT = 18.0

# --- Bo nho cho bo dieu khien dt line thong minh (camera_ai_line_follow) ---
_ll_last_dir = 1      # huong cua gan nhat: +1 = phai, -1 = trai (de tim lai khi mat line)
_ll_lost_since = 0    # moc thoi gian bat dau mat line (0 = dang thay line)
_ll_cur_speed = 0.0   # toc do tien hien tai (tang/giam mem -> khong giat)
_ll_cur_turn = 0.0    # do re hien tai (lam muot -> QUAY TU TU, khong giat)
_ll_phase = 0         # dem khung de xen ke tien/quay khi vao cua

# --- DEBUG do line: in trang thai ra console BLE (app.ohstem.vn) de phan tich khi bi vot ---
_LL_DEBUG = False      # True = in log do line; dat False de tat
_LL_DBG_MS = 120      # khoang cach giua 2 lan in (ms) - tranh tran console / lag BLE
_ll_dbg_t = 0         # moc thoi gian lan in gan nhat

# --- Cau hinh TIM LINE (khi mat line) - chinh bang block "tim kiem line" ---
_ll_ref_speed = 0       # toc do do line gan nhat (do camera_ai_line_follow luu) - lam mac dinh cho cac block khac
_search_speed = 0       # toc do quay tim line; 0 = tu dong (= toc do do line = speed_min)
_search_turn_ms = 600   # moi nhip QUAY bao nhieu ms (sweep cang dai cang rong)
_SEARCH_PAUSE_MS = 120  # nhip DUNG sau moi lan quay (ms) - nho = quet lien tuc, lon = xoay tung goc

# >>> THONG SO TIM LINE de TU CHINH (so NHIP quay moi pha) <<<
_SEARCH_FWD_CYCLES = 2  # so nhip quay ve HUONG MAT LINE (last_dir) truoc
_SEARCH_REV_CYCLES = 5  # so nhip quay NGUOC lai neu chua thay; quet het ma khong thay -> DUNG han

# --- Pha CAN GIUA sau khi tim thay line lai (re-acquire) ---
_ll_recenter = False      # dang can giua line truoc khi chay nhanh tro lai
_ll_center_cnt = 0        # so frame lien tuc da gan giua
_ll_recenter_since = 0    # moc bat dau can giua (de gioi han thoi gian)
_CENTER_OK = 18.0         # |offset| <= nguong nay coi nhu da vao giua line
_CENTER_FRAMES = 2        # can >= so frame lien tuc o giua -> coi la on dinh
_RECENTER_MAX = 1200      # can giua toi da (ms) roi chay binh thuong (chong ket)
_prev_line_off = 0.0      # offset frame truoc (tinh bien thien -> ghim lo khi thoat cua)


def camera_ai_set_pd(kp, kd):
    # (Tuong thich cu) Dat CHUNG 1 bo so cho ca chay thang va vao cua.
    global _kp_str, _kd_str, _kp_cur, _kd_cur, _line_kp, _line_kd
    _kp_str = kp; _kd_str = kd
    _kp_cur = kp; _kd_cur = kd
    _line_kp = kp; _line_kd = kd


def camera_ai_set_pd_straight(kp, kd):
    # Bo PID khi CHAY THANG / THOAT CUA (nen Kp nho de em, chong lech).
    global _kp_str, _kd_str
    _kp_str = kp
    _kd_str = kd


def camera_ai_set_pd_curve(kp, kd):
    # Bo PID khi VAO CUA (nen Kp lon de cua dut khoat).
    global _kp_cur, _kd_cur
    _kp_cur = kp
    _kd_cur = kd


def camera_ai_set_search(speed, turn_ms=600, pause=120):
    # Cau hinh TIM LINE (khi mat line): xoay con lac ve huong mat line lan cuoi, mo rong dan.
    #   speed   = toc do quay (0 = tu dong = speed_min + 8). Cao -> xoay nhanh/rong; thap -> em, cham.
    #   turn_ms = do dai moi nhip quay (ms). Dai -> sweep RONG; ngan -> hep.
    #   pause   = nhip DUNG sau moi lan quay (ms). Nho -> quet lien tuc; lon -> xoay tung goc (do nhoe).
    global _search_speed, _search_turn_ms, _SEARCH_PAUSE_MS
    _search_speed = speed
    _search_turn_ms = turn_ms
    _SEARCH_PAUSE_MS = pause


def camera_ai_line_update():
    # Doc goi UART moi nhat cho do line. Camera dong goi 1 dong giong object tracking:
    #   "<offset>,<junc>"  voi junc = S | CROSS4 | CROSS3L | CROSS3R | CURVE_L | CURVE_R
    #   hoac "LINE: lost" khi mat line.
    global _uart, _last_recv_time, _cross_time, _line_offset, _line_seen
    global _line_angle, _junction, _junction_time, _line_offset_smooth
    got = False     # da nhan duoc mau offset moi trong lan goi nay chua
    while _uart and _uart.any():
        ln = _uart.readline()
        if not ln:
            break
        try:
            text = ln.decode('utf-8').strip()
        except Exception:
            continue
        tag, payload = _split_tag(text)
        # Chi xu ly du lieu line (LINE); bo qua OBJ/FACE/COLOR...
        if tag and tag != 'LINE':
            continue
        _last_recv_time = time.ticks_ms()
        if 'lost' in payload.lower():
            _line_seen = False
        elif ',' in payload:
            parts = payload.split(',')
            try:
                _line_offset = float(parts[0])
                _line_seen = True
                got = True
            except Exception:
                continue
            # Goi moi: "offset,angle,junc"; goi cu 4 truong: "offset,angle,far,junc"; goi rat cu: "offset,junc"
            jc = 'S'
            if len(parts) >= 4:        # tuong thich firmware cu (co far o parts[2])
                try:
                    _line_angle = float(parts[1])
                except Exception:
                    pass
                jc = parts[3].strip()
            elif len(parts) >= 3:      # goi MOI: offset,angle,junc
                try:
                    _line_angle = float(parts[1])
                except Exception:
                    pass
                jc = parts[2].strip()
            elif len(parts) >= 2:      # goi rat cu: offset,junc
                jc = parts[1].strip()
            jc = jc.upper()            # firmware gui chu thuong -> chuan hoa HOA de so khop noi bo
            if jc and jc != 'S':
                _junction = jc
                _junction_time = time.ticks_ms()
                if jc.startswith('CROSS'):
                    _cross_time = time.ticks_ms()   # ngã 3/4 = giao nhau that
        else:
            # Tuong thich ban cu (chi gui offset hoac CROSS/JUNC rieng)
            if payload == 'CROSS':
                _cross_time = time.ticks_ms()
                _junction = 'CROSS4'
                _junction_time = time.ticks_ms()
            elif payload.startswith('JUNC:'):
                _junction = payload[5:].strip()
                _junction_time = time.ticks_ms()
                if _junction.startswith('CROSS'):
                    _cross_time = time.ticks_ms()
            else:
                try:
                    _line_offset = float(payload)
                    _line_seen = True
                    got = True
                except Exception:
                    pass
    # Lam muot EMA chi khi co mau moi (khong co data -> giu nguyen gia tri muot)
    if got:
        _line_offset_smooth = (_LINE_SM_ALPHA * _line_offset_smooth
                               + (1.0 - _LINE_SM_ALPHA) * _line_offset)
    return _line_offset


def camera_ai_line_offset_smooth():
    # Tra ve do lech duong line DA LAM MUOT (EMA). Goi camera_ai_line_update()/line_follow truoc.
    # Duong (+) = line lech phai, am (-) = lech trai. Dung de quyet dinh re/giu thang muot hon offset tho.
    return _line_offset_smooth


def camera_ai_line_follow(speed_min=22, speed_max=45, smart_search=True):
    # ================================================================
    # DO LINE THONG MINH (non-blocking: goi 1 lan moi vong lap)
    #   - Bam line PD: steer = Kp*do_lech + Kd*goc_nghieng (don cua som)
    #   - Tu GIAM TOC khi vao cua (cua cang gat chay cang cham -> chong chay lo)
    #   - Tang/giam toc MEM (khong giat)
    #   - Cua RAT gat: xoay tai cho de giu line trong khung (khong mat outline)
    #   - Mat line + smart_search=True -> QUET CON LAC trai-phai CHAM, mo rong dan
    #     (quay tung buoc, co nghi cho camera nhin) de bat lai line ke ca cua gat/hairpin
    # ================================================================
    global _ll_last_dir, _ll_lost_since, _ll_cur_speed, _ll_cur_turn, _ll_phase
    global _in_curve, _line_kp, _line_kd
    global _ll_recenter, _ll_center_cnt, _ll_recenter_since, _prev_line_off
    global _drive_floor_override, _ll_dbg_t, _ll_ref_speed

    # DO LINE dung SAN TOC DO RIENG = speed_min cua block do line (TACH BIET hoan toan):
    # khong bi block "robot toc do toi thieu" ep nua. Tat ca lenh dong co trong line_follow
    # (tien / pivot / arc / tim line) deu lay san = speed_min nay.
    _drive_floor_override = speed_min
    _ll_ref_speed = speed_max   # luu "toc do do line" -> block tai vach ngang / tim line dung lai lam mac dinh

    camera_ai_line_update()

    if _line_seen:
        # Vua TIM THAY LINE sau khi mat that su -> bat pha CAN GIUA truoc khi chay tiep.
        if _ll_lost_since != 0:
            if time.ticks_diff(time.ticks_ms(), _ll_lost_since) > 150:
                _ll_recenter = True
                _ll_center_cnt = 0
                _ll_recenter_since = time.ticks_ms()
            _prev_line_off = _line_offset   # reset bien thien (tranh nhay D khi vua thay lai)
        _ll_lost_since = 0

        # ----- GAIN SCHEDULING + DOAN TRUOC CUA: phan vung theo |offset| VA GOC LINE (mui ten) -----
        # Phan vung co hysteresis (chong nhay qua lai). DOAN TRUOC CUA bang GOC LINE: camera nhin XA
        # thay line BE (goc lon) TRUOC khi offset lon -> vao che do cua SOM, robot quet theo mui ten
        # tu tu (khong doi sat cua moi quay). GIU che do cua cho den khi line THANG that su (offset NHO
        # VA goc NHO) -> khong tang toc som luc line con nghieng -> chong banh ngoai VOT luc thoat cua.
        ao = _line_offset if _line_offset >= 0 else -_line_offset
        aa = _line_angle if _line_angle >= 0 else -_line_angle
        _ANGLE_ON = 18.0       # goc line (do) de coi nhu DANG VAO CUA (doan truoc)
        _ANGLE_OFF = 9.0       # goc line nho hon nay (va offset nho) -> line da THANG -> ra che do cua
        if _in_curve:
            if ao < _CURVE_OFF and aa < _ANGLE_OFF:
                _in_curve = False
        else:
            if ao > _CURVE_ON or aa > _ANGLE_ON:
                _in_curve = True
        if _in_curve:
            _line_kp, _line_kd = _kp_cur, _kd_cur      # VAO CUA: bo so manh
        else:
            _line_kp, _line_kd = _kp_str, _kd_str      # CHAY THANG/THOAT CUA: bo so em

        # PD: steer = Kp*do_lech + Kd*GOC NGHIENG line.
        # Camera GUI goc that (goi "line:offset,angle,junc") -> KET HOP:
        #   - offset = VI TRI line (line dang o ben nao)
        #   - angle  = HUONG line BE toi (+ phai, - trai) - DOC LAP voi vi tri
        # Khi robot lech o cua gat ma offset & huong MAU THUAN (vd offset + nhung line be trai),
        # thanh phan goc keo steer ve dung huong line dang be -> quay tim DUNG huong.
        # steer = Kp*offset + Kd*goc.
        #   - offset = loi VI TRI (robot lech ben nao so voi line) -> phai keo ve.
        #   - goc    = HUONG line phia truoc -> chi de DON CUA.
        # KHI CUA: nhan trong so GOC de don cua som, NHUNG lam MO DAN khi offset LON:
        #   offset nho (<=20)  -> goc x2 (don cua tot, vd line be ngay truoc mat ma offset chua lon).
        #   offset lon (>=45)  -> goc x0 (line da lech han 1 ben -> VI TRI thang, khong de goc lat
        #                          nguoc huong keo ve line -> chong loi thoat cua: off -54 goc +29 be sai).
        #   khoang giua        -> giam tuyen tinh 2 -> 0.
        amul = 1.0
        if _in_curve:
            if ao <= 20.0:
                amul = 2.0
            elif ao >= 45.0:
                amul = 0.0
            else:
                amul = 2.0 * (45.0 - ao) / 25.0
        steer = _line_kp * _line_offset + _line_kd * _line_angle * amul
        amag = steer if steer >= 0 else -steer

        # Ghi nho HUONG tim line theo STEER (da gom offset + goc line).
        if steer > 5:
            _ll_last_dir = 1
        elif steer < -5:
            _ll_last_dir = -1

        # ----- CAN GIUA sau khi tim thay line: chay CHAM + bam manh dua line vao giua -----
        # Tranh vua thay line da lao nhanh -> van line lai. Chi khi |offset| nho lien tuc
        # (hoac het thoi gian) moi cho chay binh thuong.
        if _ll_recenter:
            if ao <= _CENTER_OK:
                _ll_center_cnt += 1
            else:
                _ll_center_cnt = 0
            timeup = time.ticks_diff(time.ticks_ms(), _ll_recenter_since) > _RECENTER_MAX
            if _ll_center_cnt >= _CENTER_FRAMES or timeup:
                _ll_recenter = False           # da on dinh giua -> chay binh thuong
            else:
                # CAN GIUA NHE, KHONG VOT: banh ngoai cap CUNG o speed_max (truoc day _drive_arc khong cap
                # -> speed_min+steer co the vot len 50+ khi vua thay line o mep -> miss). Dung _drive_curve
                # -> banh trong co the ve 0 (pivot nhe can giua) ma banh ngoai van <= speed_max.
                tn = steer
                l = speed_min + tn
                r = speed_min - tn
                if l > speed_max:
                    l = speed_max
                if r > speed_max:
                    r = speed_max
                _ll_cur_speed = speed_min
                _drive_curve(l, r)   # banh ngoai <= speed_max, banh trong tut thap/pivot -> can giua khong vot
                return

        # ----- TOC DO TIEN: CHI tang toc khi line DA ON DINH GIUA khung -----
        # Dung trang thai _in_curve (theo |offset| THAT, hysteresis 30/15) -> KHONG dung steer.
        # Ly do: steer co the NHO (PD can tot, thanh phan goc bu lai) NGAY CA khi line con sat 2 BIEN.
        # Neu tang toc luc do, banh ngoai VOT de keo line ve giua -> OUTLINE ngay khi gan thoat cua.
        #   - Line con o bien (_in_curve = True, dang ra/vao cua): GIU toc cham, chi bam theo line.
        #   - Line DA ve giua (_in_curve = False, on dinh): moi tang toc len MAX + can bang day du.
        if _in_curve:
            target = speed_min     # ra/vao cua, line chua ve giua -> bam cham, KHONG tang toc
        else:
            target = speed_max     # line on dinh giua khung -> chay nhanh
        # Ramp toc KHONG DOI XUNG: GIAM nhanh (vao cua -> an toan), TANG CHAM (thoat cua, chong vot).
        if target < _ll_cur_speed:
            _ll_cur_speed += (target - _ll_cur_speed) * 0.4    # giam toc nhanh
        else:
            _ll_cur_speed += (target - _ll_cur_speed) * 0.12   # tang toc cham -> chong banh ngoai vot cuoi cua
        base = _ll_cur_speed

        # ----- LAI VI SAI: 2 banh luon tien, chenh toc theo do lech -----
        # Luong re = steer (P + D). steer > 0 (line lech phai) -> quay phai
        # -> banh TRAI nhanh hon banh PHAI; banh trong cham lai (van tien).
        turn = steer
        if turn > 100.0:
            turn = 100.0
        elif turn < -100.0:
            turn = -100.0
        # Lam muot KHONG DOI XUNG:
        #   - VAO cua (|do re| TANG) -> lam muot (quay tu tu, khong giat).
        #   - THOAT cua (|do re| GIAM) -> theo NGAY (khong tre) -> robot NGUNG quay dung luc
        #     line da thang, khong bi do re tre keo quay LO ra ngoai (het outline luc thoat cua).
        at = turn if turn >= 0 else -turn
        ac = _ll_cur_turn if _ll_cur_turn >= 0 else -_ll_cur_turn
        if at > ac:
            # VAO cua / do re TANG -> gioi han toc do tang theo BUOC CO DINH (slew rate).
            # Chong VOT khi line lot goc hep luc thoat cua -> steer spike 1-2 frame: chi nhich
            # toi da TURN_SLEW roi tu ve (vi chieu giam la tuc thi). Cua THAT (steer lon keo dai)
            # van tang dan qua vai frame -> van vao duoc cua, nhung KHONG giat/vot vi misread.
            TURN_SLEW = 6.0
            d = turn - _ll_cur_turn
            if d > TURN_SLEW:
                d = TURN_SLEW
            elif d < -TURN_SLEW:
                d = -TURN_SLEW
            _ll_cur_turn += d
        else:
            _ll_cur_turn = turn                            # thoat cua: theo ngay (chong vot/lo)
        turn = _ll_cur_turn

        left = base + turn
        right = base - turn
        # Banh ngoai gioi han o speed_max (khong vot). Banh trong (right) duoc phep tut THAP.
        if left > speed_max:
            left = speed_max
        if right > speed_max:
            right = speed_max
        # _drive_curve: cua VUA (turn nho -> right > 0) banh trong van tien (om cua muot); cua GAT
        # (turn lon -> right <= 0) banh trong DUNG (0) -> pivot, KHONG lui. Vot thoat cua co speed gate lo.
        _drive_curve(left, right)
        _ll_phase = 0

        # ----- DEBUG: in trang thai bam line ra console BLE (throttle _LL_DBG_MS) -----
        # Doc tren app.ohstem.vn. Khi robot VOT/lech o cua -> dung robot, gui dong log nay cho minh:
        #   off = vi tri line (+ phai / - trai) | ang = goc line (mui ten) | C=dang cua, S=thang
        #   base = toc tien | turn = do re (steer da slew) | L/R = toc 2 banh (banh ngoai cao = dang quay)
        if _LL_DEBUG and time.ticks_diff(time.ticks_ms(), _ll_dbg_t) >= _LL_DBG_MS:
            _ll_dbg_t = time.ticks_ms()
            print("LINE off=%d ang=%d %s base=%d turn=%d L=%d R=%d" % (
                int(_line_offset), int(_line_angle), ("C" if _in_curve else "S"),
                int(base), int(turn), int(left), int(right)))
    else:
        # ===== MAT LINE: TIM kieu YOLOBIT - quet CON LAC trai-phai, MO RONG dan =====
        # Pha 0 quay ve HUONG CUA gan nhat (_ll_last_dir, line hay di huong do); sau do DAO CHIEU
        # trai/phai va MO RONG dan (moi pha k gom k+1 buoc). Moi buoc: quay (turn_ms) roi NGHI
        # (pause) cho camera bat khung sach -> xoay tung goc, chong nhoe. Quet ~4 pha roi dung.
        _ll_cur_speed = 0.0   # reset toc tien ve 0 -> khi tim thay line se tang toc TU TU tu dau (khong vot)
        if not smart_search:
            _drive_stop()
            return
        if _ll_lost_since == 0:
            _ll_lost_since = time.ticks_ms()
        lost_ms = time.ticks_diff(time.ticks_ms(), _ll_lost_since)

        # ----- DUNG HAN 1 NHIP truoc khi tim (theo y nguoi dung) -----
        # Dang cua/chay ma DOT NGOT lost (thuong do vot khoi line) -> neu quay tim ngay se mang theo
        # da/momentum -> vot tiep. DUNG HAN _SEARCH_STOP_MS truoc -> giet momentum -> roi moi quet em.
        _SEARCH_STOP_MS = 250
        if lost_ms < _SEARCH_STOP_MS:
            _drive_stop()
            return

        # Thong so quet LAY TU BLOCK "tim kiem line" (camera_ai_set_search) -> nguoi dung tu tinh chinh.
        #   _search_speed  = toc do xoay (0 = tu dong = speed_min + 8). Cao -> xoay nhanh/rong; thap -> em.
        #   _search_turn_ms = do dai moi nhip quay (sweep cang dai cang rong).
        #   _SEARCH_PAUSE_MS = nhip dung sau moi lan quay (nho -> quet lien tuc; lon -> xoay tung goc, do nhoe).
        # camera DO MOI FRAME ke ca khi dang xoay -> thay line la vao pha CAN GIUA (da cap banh ngoai
        # <= speed_max) -> KHONG vot.
        search_sp = speed_max + 2   # toc do quay = toc do do line (+2 du luc quay, khong rit)
        turn_ms = 500               # CO DINH 0.5 giay moi nhip tim line
        local_pause = 200           # nhip dung co dinh 200ms cho camera nhin
        STEP = turn_ms + local_pause          # moi buoc = quay turn_ms + nghi pause
        t = lost_ms - _SEARCH_STOP_MS         # tru pha dung dau -> con lac bat dau sau khi da dung
        c = t // STEP                         # nhip thu may (0,1,2,...) - moi nhip = quay turn_ms + nghi pause
        # ----- HUONG QUET (theo thiet ke) -----
        #   _SEARCH_FWD_CYCLES nhip dau: quay ve last_dir (huong luc mat line) - thu phia nghi ngo.
        #   _SEARCH_REV_CYCLES nhip tiep: quay NGUOC lai (quet phia kia).
        #   Het 2 pha ma chua thay line -> DUNG HAN (khong quay vo ich nua).
        if c < _SEARCH_FWD_CYCLES:
            swing_dir = _ll_last_dir
        elif c < _SEARCH_FWD_CYCLES + _SEARCH_REV_CYCLES:
            swing_dir = -_ll_last_dir
        else:
            _drive_stop()                     # quet ca 2 huong khong thay -> dung
            return
        if swing_dir == 0:
            swing_dir = 1

        rotating = (t % STEP) < turn_ms
        if rotating:                 # ...quay...
            if swing_dir < 0:
                _drive_left(search_sp)
            else:
                _drive_right(search_sp)
        else:                        # ...NGHI cho camera nhin (xoay tung goc)
            _drive_stop()

        # ----- DEBUG: in trang thai TIM LINE ra console BLE (throttle) -----
        # dir = +1 quay phai / -1 quay trai (theo huong mat line lan cuoi) | k = pha con lac (rong dan)
        # spin = dang quay hay nghi cho camera nhin.
        if _LL_DEBUG and time.ticks_diff(time.ticks_ms(), _ll_dbg_t) >= _LL_DBG_MS:
            _ll_dbg_t = time.ticks_ms()
            print("LINE LOST search dir=%d c=%d spin=%d sp=%d" % (
                swing_dir, c, (1 if rotating else 0), int(search_sp)))


def camera_ai_at_junction(action='stop', speed=35, turn_ms=500):
    # ================================================================
    # TAI GIAO LO: tien toi tam giao lo roi thuc hien hanh dong trong turn_ms (ms).
    #   'stop'         = dung lai (turn_ms khong dung)
    #   'left'/'right' = quay trai/phai trong turn_ms
    #   'straight'     = di thang qua giao lo trong turn_ms
    # (Block nay chay XONG moi sang block tiep theo.)
    # ================================================================
    global _ll_cur_speed
    _ll_cur_speed = 0.0

    # speed <= 0 -> dung TOC DO DO LINE + 5 (quay khoe hon); chua co line -> 40
    if speed is None or speed <= 0:
        speed = (_ll_ref_speed + 5) if _ll_ref_speed > 0 else 40

    # 1. Tien toi tam giao lo (camera thay giao lo TU XA -> phai tien them)
    _drive_fw(speed)
    t = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t) < 320:
        camera_ai_line_update()
        time.sleep_ms(5)

    if action == 'stop':
        _drive_stop()
        t = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t) < turn_ms:   # DUNG dung turn_ms roi moi thoat
            time.sleep_ms(10)
        return

    # 2. Thuc hien hanh dong trong turn_ms
    if action == 'left':
        _drive_left(speed)
    elif action == 'right':
        _drive_right(speed)
    else:  # 'straight'
        _drive_fw(speed)
    t = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t) < turn_ms:
        time.sleep_ms(10)
    _drive_stop()


def camera_ai_junction(hold_ms=400):
    # Loai giao nhau gan day: 'CROSS4','CROSS3L','CROSS3R','CURVE_L','CURVE_R' hoac '' neu khong.
    camera_ai_line_update()
    if _junction_time != 0 and time.ticks_diff(time.ticks_ms(), _junction_time) <= hold_ms:
        return _junction
    return ''


def camera_ai_junction_is(code, hold_ms=400):
    # True neu tinh huong hien tai dung bang 'code' trong hold_ms gan day.
    # 'CROSS3' = ngã 3 bat ky; 'CURVE' = dang cong bat ky.
    j = camera_ai_junction(hold_ms)
    code = str(code)
    if code == 'CROSS3':
        return j == 'CROSS3L' or j == 'CROSS3R' or j == 'CROSS3T'
    if code == 'CURVE':
        return j == 'CURVE_L' or j == 'CURVE_R'
    return j == code


# ================================================================
# NHAN DANG KHUON MAT + MAU SAC (doc UART)
# Camera gui: "FACE:<id>" (id>=1 da luu / 0 nguoi la / -1 khong co mat)
#             "COLOR:<TEN>" (RED, GREEN, BLUE, ... mau dang thay o tam)
# Cac block tu goi camera_ai_vision_update() de luon co du lieu moi nhat.
# ================================================================
_face_id = -1
_face_time = 0       # moc thoi gian nhan FACE gan nhat
_color_name = ''
_color_time = 0      # moc thoi gian nhan COLOR gan nhat
_class_name = ''     # ten lop Stream video (Teachable Machine)
_class_conf = 0      # do tin cay (%) cua lop dang nhan
_class_time = 0      # moc thoi gian nhan CLASS gan nhat
_qr_text = ''        # noi dung ma QR doc duoc gan nhat
_qr_time = 0         # thoi diem (ms) nhan QR gan nhat (de biet con dang detect khong)

# Khong nhan du lieu nhan dang moi trong ngan nay (ms) -> coi nhu MAT/RONG (tra rong),
# tranh giu ket qua cu mai khi mat vat / mat ket noi -> dieu khien dung.
_VISION_HOLD_MS = 200


def _vision_fresh(t):
    # True neu moc thoi gian t con moi (trong _VISION_HOLD_MS). t==0 -> chua tung nhan.
    return t != 0 and time.ticks_diff(time.ticks_ms(), t) <= _VISION_HOLD_MS


def camera_ai_vision_update():
    # Doc goi UART moi nhat: cap nhat ket qua FACE / COLOR / CLASS / QR.
    global _uart, _last_recv_time, _face_id, _color_name, _class_name, _class_conf, _qr_text, _qr_time
    global _face_time, _color_time, _class_time
    while _uart and _uart.any():
        ln = _uart.readline()
        if not ln:
            break
        try:
            text = ln.decode('utf-8').strip()
        except Exception:
            continue
        _last_recv_time = time.ticks_ms()
        tag, payload = _split_tag(text)   # tag tu dong chuan hoa HOA -> nhan ca face:/FACE:
        if tag == 'FACE':
            try:
                _face_id = int(payload)
                _face_time = time.ticks_ms()
            except Exception:
                pass
        elif tag == 'COLOR':
            _color_name = payload.strip().upper()   # chuan hoa HOA de camera_ai_is_color so khop dung
            _color_time = time.ticks_ms()
        elif tag == 'CLASS':
            # payload = "<ten lop>,<conf%>"  (vd "meo,87")
            i = payload.rfind(',')
            if i >= 0:
                _class_name = payload[:i].strip()
                try:
                    _class_conf = int(payload[i + 1:])
                except Exception:
                    _class_conf = 0
            else:
                _class_name = payload.strip()
                _class_conf = 0
            _class_time = time.ticks_ms()
        elif tag == 'QR':
            _qr_text = payload   # noi dung ma QR (giu nguyen, khong viet hoa)
            _qr_time = time.ticks_ms()


def camera_ai_face_known():
    # True neu DANG nhan ra nguoi DA LUU (id >= 1). Mat/cu -> False.
    camera_ai_vision_update()
    return _vision_fresh(_face_time) and _face_id >= 1


def camera_ai_face_unknown():
    # True neu DANG thay mat nhung CHUA LUU (id == 0). Mat/cu -> False.
    camera_ai_vision_update()
    return _vision_fresh(_face_time) and _face_id == 0


def camera_ai_color_name():
    # Ten mau dang thay o tam ('RED','GREEN',...). '' neu khong thay / mat ket noi.
    camera_ai_vision_update()
    return _color_name if _vision_fresh(_color_time) else ''


def camera_ai_is_color(name):
    # True neu DANG thay mau dung bang 'name' (vd 'RED'). Mat/cu -> False.
    camera_ai_vision_update()
    return _vision_fresh(_color_time) and _color_name == str(name).upper()


def camera_ai_class():
    # Ten lop nhan dang (Stream video / Teachable Machine). '' neu khong nhan duoc /
    # mat ket noi (khong giu ket qua cu mai) -> dieu khien dung.
    camera_ai_vision_update()
    return _class_name if _vision_fresh(_class_time) else ''


def camera_ai_class_conf():
    # Do tin cay (%) cua lop dang nhan: 0..100. 0 neu khong nhan duoc / mat ket noi.
    camera_ai_vision_update()
    return _class_conf if _vision_fresh(_class_time) else 0


def camera_ai_is_class(name, min_conf=0):
    # True neu DANG nhan lop == 'name' VA tin cay >= min_conf (%). Mat/cu -> False.
    camera_ai_vision_update()
    if not _vision_fresh(_class_time):
        return False
    n = str(name)
    return (_class_name == n or _class_name.upper() == n.upper()) and _class_conf >= min_conf


def camera_ai_qr(hold_ms=1500):
    # Noi dung ma QR khi DANG detect (chuoi). Tra '' (rong) neu khong thay QR
    # trong hold_ms gan day -> chi hien khi co ma trong khung, het thi de trong.
    camera_ai_vision_update()
    if _qr_time != 0 and time.ticks_diff(time.ticks_ms(), _qr_time) <= hold_ms:
        return _qr_text
    return ''
