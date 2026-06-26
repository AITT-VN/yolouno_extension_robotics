# mavlink_lite.py — bộ giải mã MAVLink tối giản cho ORC hub (Yolo Uno)
#
# Chỉ giải mã 2 bản tin cần cho gamepad ESP32C3 (profile ROBOT):
#   - HEARTBEAT (#0)        : để biết "đã kết nối"
#   - MANUAL_CONTROL (#69)  : 4 trục joystick (x, y, z, r) + bitmask nút (buttons)
#
# Hỗ trợ cả khung MAVLink v2 (0xFD) lẫn v1 (0xFE). Parser hoạt động theo dạng
# nạp byte (feed) nên chịu được khung bị cắt giữa chừng qua nhiều lần nhận
# (BLE notify / ESP-NOW datagram).

import struct
from time import ticks_ms

# Mã bản tin
MSG_HEARTBEAT = 0
MSG_MANUAL_CONTROL = 69

# Hạt giống CRC_EXTRA theo MAVLink common.xml (xác minh khi đổi định nghĩa bản tin)
_CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_MANUAL_CONTROL: 243,
}

# Độ dài payload phần cơ bản (không tính extension) theo thứ tự wire (sắp theo kích thước)
#   HEARTBEAT      : custom_mode(u32), type(u8), autopilot(u8), base_mode(u8),
#                    system_status(u8), mavlink_version(u8)  -> 4 + 5 = 9
#   MANUAL_CONTROL : x(i16), y(i16), z(i16), r(i16), buttons(u16), target(u8) -> 10 + 1 = 11
_BASE_LEN = {
    MSG_HEARTBEAT: 9,
    MSG_MANUAL_CONTROL: 11,
}

_MAX_BUF = 512  # chặn buffer phình to khi gặp rác


def _crc_accumulate(b, crc):
    # Thuật toán X25 (CRC-16/MCRF4XX) — giống mavlink crc_accumulate
    tmp = b ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def build_heartbeat(seq=0):
    # Tạo khung MAVLink v2 HEARTBEAT (#0) kiểu GCS để hub gửi NGƯỢC về gamepad,
    # giúp gamepad bật LED "đã kết nối". custom_mode=0 -> gamepad không vào ALTHOLD.
    #   custom_mode(u32)=0, type(u8)=6(GCS), autopilot(u8)=8(INVALID),
    #   base_mode(u8)=0, system_status(u8)=4(ACTIVE), mavlink_version(u8)=3
    payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3)
    header = bytes([len(payload), 0, 0, seq & 0xFF, 255, 1,
                    MSG_HEARTBEAT & 0xFF, 0, 0])
    crc = 0xFFFF
    for b in header:
        crc = _crc_accumulate(b, crc)
    for b in payload:
        crc = _crc_accumulate(b, crc)
    crc = _crc_accumulate(_CRC_EXTRA[MSG_HEARTBEAT], crc)
    return bytes([0xFD]) + header + payload + struct.pack('<H', crc)


class MavlinkLite:
    def __init__(self):
        self._buf = bytearray()
        # kết quả giải mã
        self.manual = None          # dict {x, y, z, r, buttons} của MANUAL_CONTROL mới nhất
        self.manual_seq = 0         # tăng mỗi khi nhận MANUAL_CONTROL hợp lệ
        self.last_manual_ms = 0
        self.last_heartbeat_ms = 0

    def feed(self, data):
        if not data:
            return
        self._buf += data
        if len(self._buf) > _MAX_BUF:
            # giữ lại đuôi để không mất khung đang dở
            self._buf = self._buf[-_MAX_BUF:]
        self._parse()

    # ---- nội bộ ----
    def _parse(self):
        buf = self._buf
        i = 0
        n = len(buf)
        while i < n:
            stx = buf[i]
            if stx == 0xFD:          # MAVLink v2
                if n - i < 10:
                    break            # chưa đủ header
                plen = buf[i + 1]
                incompat = buf[i + 2]
                total = 10 + plen + 2
                if incompat & 0x01:  # MAVLINK_IFLAG_SIGNED
                    total += 13
                if n - i < total:
                    break            # chưa đủ cả khung
                msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                consumed = self._handle(buf, i, i + 1, i + 10, plen, msgid, total)
                i += consumed
            elif stx == 0xFE:        # MAVLink v1
                if n - i < 6:
                    break
                plen = buf[i + 1]
                total = 6 + plen + 2
                if n - i < total:
                    break
                msgid = buf[i + 5]
                consumed = self._handle(buf, i, i + 1, i + 6, plen, msgid, total)
                i += consumed
            else:
                i += 1               # không phải STX -> trượt tới byte kế

        if i > 0:
            self._buf = buf[i:]

    def _handle(self, buf, frame_start, crc_start, payload_start, plen, msgid, total):
        # Trả về số byte tiêu thụ. crc_start..payload_end là vùng tính CRC.
        extra = _CRC_EXTRA.get(msgid)
        if extra is None:
            # bản tin không quan tâm (vd SERIAL_CONTROL) -> bỏ qua trọn khung
            return total

        payload_end = payload_start + plen
        crc = 0xFFFF
        for k in range(crc_start, payload_end):
            crc = _crc_accumulate(buf[k], crc)
        crc = _crc_accumulate(extra, crc)

        got = buf[payload_end] | (buf[payload_end + 1] << 8)
        if crc != got:
            return 1                 # CRC sai -> trượt 1 byte để tái đồng bộ

        self._dispatch(msgid, buf[payload_start:payload_end])
        return total

    def _dispatch(self, msgid, payload):
        if msgid == MSG_MANUAL_CONTROL:
            need = _BASE_LEN[MSG_MANUAL_CONTROL]
            p = payload
            if len(p) < need:        # v2 cắt zero ở đuôi -> bù lại
                p = p + bytes(need - len(p))
            x, y, z, r, buttons, _target = struct.unpack('<hhhhHB', p[:need])
            self.manual = {'x': x, 'y': y, 'z': z, 'r': r, 'buttons': buttons}
            self.manual_seq = (self.manual_seq + 1) & 0x3FFFFFFF
            self.last_manual_ms = ticks_ms()
        elif msgid == MSG_HEARTBEAT:
            self.last_heartbeat_ms = ticks_ms()
