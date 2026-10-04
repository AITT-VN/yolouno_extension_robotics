# yolouno_extension_robotics
Thư viện để xây dựng các dự án xe robot di chuyển và dò line

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
