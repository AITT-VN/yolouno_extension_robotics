# ============================================================================
#  CHUONG TRINH LAY MAU MAU (color sampler)
#  - Dat cam bien len 1 mau (nen trang / do / vang / xanh la / xanh lo / xanh duong)
#  - Bam BOOT -> doc 10 mau cach nhau 60ms, in tung dong + trung binh RGB
#  - KHONG chay motor (de so on dinh, lap lai duoc giua cac lan boot)
#  Gui toan bo log ve cho minh de chinh _COLOR_REFS / _SUM_MIN chuan xac.
# ============================================================================
from line_sensor import *
from yolo_uno import *
from abutton import *

line_sensor = LineSensorI2C()
btn_BOOT = aButton(BOOT_PIN)

def deinit():
  btn_BOOT.deinit()

import yolo_uno
yolo_uno.deinit = deinit

# thu tu de nghi dat cam bien khi bam lan luot (chi de goi y nhan, khong bat buoc)
_ORDER = ['nen trang', 'do', 'vang', 'xanh la', 'xanh lo (cyan)', 'xanh duong', 'magenta']
_n = [0]

async def on_abutton_BOOT_pressed():
  idx = _n[0]
  name = _ORDER[idx] if idx < len(_ORDER) else ('lan %d' % (idx + 1))
  print('===== SET #%d : %s =====' % (idx + 1, name))

  N = 10
  sr = sg = sb = 0
  for i in range(N):
    r, g, b, h, s, v, lab = line_sensor.hsv_debug()
    print('SAMPLE,%d,r=%d,g=%d,b=%d,hue=%.1f,sat=%.3f,val=%.4f,%s' % (i, r, g, b, h, s, v, lab))
    sr += r; sg += g; sb += b
    await asleep_ms(60)   # > IT 40ms -> moi mau la 1 lan tich phan doc lap

  # trung binh RGB (chromaticity on dinh); hue tinh lai tu RGB de tranh loi wrap quanh 0/360
  ar, ag, ab = sr // N, sg // N, sb // N
  print('AVG ,r=%d,g=%d,b=%d' % (ar, ag, ab))
  print('')
  _n[0] = idx + 1

async def setup():
  print('== Color sampler ==')
  print('Dat cam bien len 1 mau roi bam BOOT. Thu tu de nghi:')
  print('  ' + ' -> '.join(_ORDER))
  line_sensor.set_white_led(True)
  await asleep_ms(300)   # cho LED sang on dinh + qua 1 chu ky tich phan 40ms
  btn_BOOT.pressed(on_abutton_BOOT_pressed)

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
