from mdv2 import *
from motor import *
from drivebase import *
from ble import *
from gamepad import *
from abutton import *
from mpu6050 import *
from angle_sensor import *

async def on_abutton_BOOT_pressed():
  await asleep_ms(1000)
  print('left')
  await robot.test_forward(2, 90)

mdv2 = MotorDriverV2(0x56)
mdv2.pid_off()
m1 = DCMotor(mdv2, MDV2_E1, reversed=False )
m2 = DCMotor(mdv2, MDV2_E2, reversed=False )
m1.set_encoder(250, 11, 34)
m2.set_encoder(250, 11, 34)
mdv2.reverse_encoder(MDV2_E1)

robot = DriveBase(MODE_2WD, m1, m2)
gamepad = Gamepad()
btn_BOOT= aButton(BOOT_PIN)
imu = MPU6050()
angle_sensor = AngleSensor(imu)

def deinit():
  robot.stop()

import yolo_uno
yolo_uno.deinit = deinit

async def setup():
  neopix.show(0, hex_to_rgb('#ff0000'))
  print('App started')
  btn_BOOT.pressed(on_abutton_BOOT_pressed)
  create_task(ble.wait_for_msg())
  create_task(gamepad.run())
  #create_task(robot.run_teleop(gamepad, 50, 3))
  create_task(angle_sensor.run())
  robot.angle_sensor(angle_sensor)
  #create_task(print_test())
  neopix.show(0, hex_to_rgb('#00ff00'))

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
