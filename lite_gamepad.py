# lite_gamepad.py — gamepad ESP32C3 (MAVLink) cho ORC hub (Yolo Uno)
#
# LiteGamepad nhận dữ liệu từ gamepad ESP32C3 (firmware profile ROBOT) qua BLE
# và/hoặc ESP-NOW dưới dạng khung MAVLink, rồi điền vào self.data theo ĐÚNG
# schema của lớp Gamepad cũ (BTN_*, ALX/ALY/AL_DIR/AL_DISTANCE, AR...). Nhờ đó
# robot.run_teleop(lite_gamepad) và các khối Blockly remote-control dùng lại được.
#
# Firmware (profile ROBOT) gửi:
#   MANUAL_CONTROL #69: x=joyRY, y=joyRX, z=joyLY, r=joyLX  (đều -1000..1000)
#                       buttons = bitmask: bit i = btnA[i] (0..11), bit12=joyBtnL, bit13=joyBtnR
#   HEARTBEAT #0      : 1Hz -> dùng cho is_connected()

import math, asyncio
from time import ticks_ms, ticks_diff
from constants import *
from mavlink_lite import MavlinkLite, build_heartbeat

# Bitmask nút trong MANUAL_CONTROL.buttons -> hằng số nút của thư viện.
# Thứ tự bit khớp chỉ số nút trong firmware (btnA[0..11] = DPAD_L_1..DPAD_R_5,
# FUNC_1, FUNC_2; bit12=joystick trái nhấn, bit13=joystick phải nhấn).
# Ánh xạ theo VỊ TRÍ VẬT LÝ trên PCB (2 cụm chữ thập):
#   Cụm trái  = D-pad hướng : L_1=trên, L_2=phải, L_3=dưới, L_4=trái, L_5=giữa
#   Cụm phải  = nút hành động: R_1=trên, R_2=phải, R_3=dưới, R_4=trái, R_5=giữa
_BIT_TO_BTN = {
    0: BTN_UP,        # DPAD_L_1 (trái: trên)
    1: BTN_RIGHT,     # DPAD_L_2 (trái: phải)
    2: BTN_DOWN,      # DPAD_L_3 (trái: dưới)
    3: BTN_LEFT,      # DPAD_L_4 (trái: trái)
    4: BTN_L2,        # DPAD_L_5 (trái: giữa)
    5: BTN_TRIANGLE,  # DPAD_R_1 (phải: trên)
    6: BTN_CIRCLE,    # DPAD_R_2 (phải: phải)
    7: BTN_CROSS,     # DPAD_R_3 (phải: dưới)
    8: BTN_SQUARE,    # DPAD_R_4 (phải: trái)
    9: BTN_R2,        # DPAD_R_5 (phải: giữa)
    10: BTN_L1,       # FUNC_1
    11: BTN_R1,       # FUNC_2
    12: BTN_THUMBL,   # joystick trái nhấn
    13: BTN_THUMBR,   # joystick phải nhấn
}

_CONN_TIMEOUT_MS = 2000  # không có HEARTBEAT/MANUAL_CONTROL quá lâu -> coi như mất kết nối


class LiteGamepad:
    def __init__(self):
        self._verbose = False
        self._last_print = 0

        self.data = {
            BTN_UP: 0,
            BTN_DOWN: 0,
            BTN_LEFT: 0,
            BTN_RIGHT: 0,
            BTN_SQUARE: 0,
            BTN_TRIANGLE: 0,
            BTN_CROSS: 0,
            BTN_CIRCLE: 0,
            BTN_L1: 0,
            BTN_R1: 0,
            BTN_L2: 0,
            BTN_R2: 0,
            BTN_THUMBL: 0,
            BTN_THUMBR: 0,
            BTN_M1: 0,
            BTN_M2: 0,
            AL: 0,
            ALX: 0,
            ALY: 0,
            AL_DIR: -1,
            AL_DISTANCE: 0,
            AR: 0,
            ARX: 0,
            ARY: 0,
            AR_DIR: -1,
            AR_DISTANCE: 0,
        }

        self._cmd_handlers = {}
        self._parser = MavlinkLite()
        self._last_seq = 0
        self._connected = False

        # gửi ngược HEARTBEAT để gamepad bật LED "đã kết nối"
        self._hb = build_heartbeat()
        self._last_hb_send = 0
        self._peer_mac = None       # MAC gamepad học được qua ESP-NOW
        self._peers = set()

        # ---- BLE: chiếm irq để tự route khung MAVLink, chuyển tiếp phần còn lại ----
        self._ble = None
        self._orig_ble_handler = None
        try:
            from ble import ble
            self._ble = ble
            self._orig_ble_handler = ble._message_handler
            ble._ble_uart.irq(self._on_ble_bytes)
        except Exception as e:
            print('LiteGamepad: BLE not available. Ignore it.', e)

        # ---- ESP-NOW: best-effort, cùng kênh với gamepad (kênh 1) ----
        self._esp = None
        try:
            import network, espnow
            sta = network.WLAN(network.STA_IF)
            sta.active(True)
            try:
                sta.config(channel=1)  # phải khớp ESPNOW_CHANNEL của gamepad
            except:
                pass
            self._esp = espnow.ESPNow()
            self._esp.active(True)
        except Exception as e:
            print('LiteGamepad: ESP-NOW not available. Ignore it.', e)
            self._esp = None

    # Nhận byte thô từ BLE UART. Khung MAVLink bắt đầu bằng 0xFD/0xFE -> parser;
    # còn lại (lệnh app OhStem) -> trả về handler gốc của ble.py.
    def _on_ble_bytes(self, data):
        if data and (data[0] == 0xFD or data[0] == 0xFE):
            self._parser.feed(bytes(data))
        elif self._orig_ble_handler:
            self._orig_ble_handler(data)

    def on_button_pressed(self, button, callback):
        self._cmd_handlers[button] = callback

    def is_connected(self):
        return self._connected

    async def run(self):
        prev_btn = {}
        while True:
            # ---- gom gói ESP-NOW (không chặn) ----
            if self._esp:
                try:
                    host, msg = self._esp.recv(0)
                    while msg:
                        self._parser.feed(msg)
                        if host:
                            self._remember_peer(host)
                        host, msg = self._esp.recv(0)
                except:
                    pass

            # ---- áp MANUAL_CONTROL mới nhất ----
            if self._parser.manual_seq != self._last_seq:
                self._last_seq = self._parser.manual_seq
                m = self._parser.manual
                if m:
                    # trục trái = (r, z) ; trục phải = (y, x) ; scale -1000..1000 -> -100..100
                    self.data[ALX] = self._scale(m['r'])
                    self.data[ALY] = self._scale(m['z'])
                    dir, distance = self._calculate_joystick(self.data[ALX], self.data[ALY])
                    self.data[AL_DIR] = dir
                    self.data[AL_DISTANCE] = distance

                    self.data[ARX] = self._scale(m['y'])
                    self.data[ARY] = self._scale(m['x'])
                    dir, distance = self._calculate_joystick(self.data[ARX], self.data[ARY])
                    self.data[AR_DIR] = dir
                    self.data[AR_DISTANCE] = distance

                    # bitmask nút -> data + callback trên cạnh lên
                    buttons = m['buttons']
                    for bit, name in _BIT_TO_BTN.items():
                        val = (buttons >> bit) & 1
                        self.data[name] = val
                        if val and not prev_btn.get(name, 0):
                            cb = self._cmd_handlers.get(name)
                            if cb:
                                asyncio.create_task(cb())
                        prev_btn[name] = val

            # ---- trạng thái kết nối từ HEARTBEAT/MANUAL_CONTROL ----
            now = ticks_ms()
            last = self._parser.last_heartbeat_ms
            if self._parser.last_manual_ms and ticks_diff(self._parser.last_manual_ms, last) > 0:
                last = self._parser.last_manual_ms
            self._connected = last != 0 and ticks_diff(now, last) < _CONN_TIMEOUT_MS

            # ---- gửi ngược HEARTBEAT (~2Hz) để gamepad bật LED "đã kết nối" ----
            if self._connected and ticks_diff(now, self._last_hb_send) > 500:
                self._last_hb_send = now
                if self._ble:
                    try:
                        if self._ble.is_connected():
                            self._ble.send_periph(self._hb)
                    except:
                        pass
                if self._esp and self._peer_mac:
                    try:
                        self._esp.send(self._peer_mac, self._hb)
                    except:
                        pass

            if self._verbose:
                if ticks_diff(now, self._last_print) > 200:
                    print(self.data)
                    self._last_print = now

            await asyncio.sleep_ms(10)

    def _remember_peer(self, mac):
        # nhớ MAC gamepad để gửi HEARTBEAT ngược (ESP-NOW cần add_peer trước khi send)
        key = bytes(mac)
        self._peer_mac = key
        if key not in self._peers:
            self._peers.add(key)
            try:
                self._esp.add_peer(key)
            except:
                pass

    def _scale(self, v):
        v = int(v) // 10
        if v > 100:
            v = 100
        elif v < -100:
            v = -100
        return v

    def _calculate_joystick(self, x, y):
        dir = -1
        distance = int(math.sqrt(x * x + y * y))

        if distance < 15:
            return (-1, 0)
        elif distance > 100:
            distance = 100

        angle = int(math.atan2(y, x) * 180 / math.pi)
        if angle < 0:
            angle += 360

        if 0 <= angle < 10 or angle >= 350:
            dir = DIR_R
        elif 15 <= angle < 75:
            dir = DIR_RF
        elif 80 <= angle < 110:
            dir = DIR_FW
        elif 115 <= angle < 165:
            dir = DIR_LF
        elif 170 <= angle < 190:
            dir = DIR_L
        elif 195 <= angle < 255:
            dir = DIR_LB
        elif 260 <= angle < 280:
            dir = DIR_BW
        elif 285 <= angle < 345:
            dir = DIR_RB

        return (dir, distance)
