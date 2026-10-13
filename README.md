# yolouno_extension_robotics
Thư viện để xây dựng các dự án xe robot di chuyển và dò line

## Mạch điều khiển động cơ

Khối "tạo motor ... cổng ..." chọn được: **Control Hub** (ORC Hub), **Motor Driver V3**, **Motor Driver V2**, **Motor Driver V1**. Đối tượng motor và cách tạo robot (`DriveBase`) giống nhau cho mọi mạch.

**Motor Driver V3** (`mdv3.py`, class `MotorDriverV3`) là mạch I2C 4-Ch Motor Stepper Driver (RP2040 + 4 × DRV8870), firmware ở [firmware_motor_driver_4_channel](https://github.com/AITT-VN/firmware_motor_driver_4_channel), địa chỉ 0x30 (+ DIP 0..3):

- Cả 4 cổng M1..M4 đều có encoder. Chọn E1/E2 thì hiểu là M1/M2, nên chương trình viết cho Motor Driver V2 chỉ cần đổi tên mạch. `DriveBase` nhận encoder ở mọi cổng của V3 (hàm `has_encoder()` của driver).
- `md_v3.battery()` đọc điện áp nguồn động cơ (V), robot gửi lên App như V2.
- Firmware 0.2 trở lên có điều khiển vòng kín chạy trong mạch: giữ tốc độ (PID), quay chính xác tới góc, giữ vị trí, dừng êm, tự đo thông số, bảo vệ kẹt. Firmware 0.1 vẫn chạy được các khối cũ; khối mới chạy kiểu không phản hồi.

Khối "Điều khiển chính xác (Motor Driver V3)" (cần khối bật encoder để đổi ra độ và vòng/phút):

| Khối | Hàm `DCMotor` | Trên Motor Driver V3 | Trên ORC Hub, V1, V2, motor 2/3 chân |
|---|---|---|---|
| tự đo thông số, lưu lại | `await m.tune()`, `m.save_settings()` | đo VMAX, deadband, τ; tự sửa encoder đếm ngược; lưu flash | trả về `False` |
| giữ tốc độ … vòng/phút | `m.run_speed(rpm)` | PID trong mạch | `run()` với % = rpm / rpm của encoder |
| quay chính xác … độ/vòng | `await m.run_angle_precise(speed, angle, then)` | profile hình thang, dừng trong ±10 count | `run_angle()` |
| quay tới góc … | `await m.run_target(speed, angle, then)` | tới góc tuyệt đối | `run_angle()` theo phần chênh lệch |
| giữ vị trí hiện tại | `m.hold()` | servo vị trí | phanh |
| giảm tốc dừng êm | `await m.stop_smooth(then)` | giảm tốc rồi phanh / giữ / thả | dừng ngay |
| giới hạn dòng, PID | `m.set_current(mA)`, `m.set_pid(kp, ki, kd)` | ghi vào mạch | bỏ qua |
| đã tới đích / bị kẹt | `m.done()`, `m.is_stalled()` | đọc từ mạch | `True` / `False` |

`then` nhận `BRAKE`, `HOLD` (giữ vị trí, mạch khác hiểu là `BRAKE`) hoặc `STOP`. Các khối cũ (`run`, `run_angle`, `run_time`, …) giữ nguyên cách chạy không phản hồi trên mọi mạch; lệnh cũ trên một motor V3 cũng thoát chế độ vòng kín của motor đó. Kiểm tra trên máy tính: `python3 tools/mdv3/test_mdv3.py`.

## Điều khiển bằng gamepad

`Gamepad` gom hai nguồn vào cùng một bảng `gamepad.data`: màn hình **Tay cầm điều khiển** của OhStem App (BLE) và tay cầm PS4 qua bộ thu 2.4GHz (I2C 0x55). `robot.run_teleop(gamepad)` đọc bảng đó và lái robot.

App gửi `0x15` + `TÊN=giá trị`, mỗi lần ghi BLE một tin:

| Tin | Ý nghĩa |
|---|---|
| `U` `D` `L` `R` `SQ` `TR` `CR` `CI` `L1` `R1` `L2` `R2` `M1` (SHARE; nút START trên màn hình App) `M2` (OPTIONS) `PS` `THUMBL` `THUMBR` `=1/0` | nhấn / nhả nút, cùng tên với tay cầm PS4 |
| `AL=<n>`, `AR=<n>` | joystick trái / phải, `n = x*256 + y`, x, y từ -100 đến 100 (y dương là đẩy lên) |
| `MODE=<0..3>` | chế độ lái (bên dưới) |
| `SPD=<%>` | số tốc độ, % của `robot.speed()` |
| `GEARS=<a>,<b>,<c>` | các mức tốc độ người dùng đặt trên App (% mỗi mức), nút SHARE chuyển qua lần lượt; mặc định 40, 70, 100, đặt bằng `robot.teleop_gears(...)` |

Chế độ lái (`robot.teleop_mode(...)`, block "chế độ lái gamepad"):

| Hằng | | Lái bằng |
|---|---|---|
| `DRIVE_DPAD` (0, mặc định) | phím điều hướng | dpad 8 hướng (hoặc joystick trái đẩy quá nửa), tốc độ tăng dần khi giữ — như trước đây |
| `DRIVE_JOYSTICK` (1) | joystick kiểu xBot | joystick trái 8 hướng, đẩy càng xa càng nhanh |
| `DRIVE_SPLIT` (2) | 2 joystick | joystick trái tiến/lùi, joystick phải rẽ (mecanum: joystick trái đẩy ngang thì đi ngang) |
| `DRIVE_TANK` (3) | xe tăng | joystick trái bánh trái, joystick phải bánh phải |

Dpad lái được ở mọi chế độ khi hai joystick để yên. Đèn trên tay cầm PS4 đổi màu theo chế độ (cùng màu với App). Trên tay cầm, SHARE chuyển số tốc độ (40 / 70 / 100 %, hoặc các mức đặt trên App; rung 1-3 nhịp). OPTIONS để trống cho chương trình dùng; muốn OPTIONS chuyển chế độ lái thì gọi `robot.teleop_buttons(mode_button=BTN_M2)`, `robot.teleop_buttons(None, None)` tắt cả hai. Nút nào có `on_teleop_command()` riêng thì chạy lệnh đó. Chế độ/số đổi trên tay cầm được báo lại App bằng `MODE=` / `SPD=`.

An toàn: App lặp lại nút/joystick đang giữ mỗi 250 ms; sau tin `MODE=` đầu tiên, `Gamepad` nhả nút/joystick nào quá 1 giây không được nhắc lại, và nhả tất cả khi App ngắt kết nối hay tay cầm PS4 mất sóng.

Robot báo lên App (`ble.send_value(tên, giá trị)`), App hiện ở hàng thông tin trên cùng: `run_teleop` tự gửi điện áp pin `VBAT=<V>` mỗi 2 giây khi mạch động cơ đọc được (Motor Driver V2 / ORC Hub); chương trình gửi thêm giá trị riêng bất kỳ (ví dụ `ble.send_value('Nhiệt độ', 30)`).

Lệnh cho nút (`on_teleop_command`) chạy song song với việc lái, lặp lại mỗi 200 ms khi giữ nút. Kiểm tra trên máy tính: `python3 tools/teleop/test_teleop.py`.
