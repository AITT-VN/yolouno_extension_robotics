from mdv1 import *
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
  await angle_sensor.reset()
  await robot.turn_left()
  
  while True:
    #print(angle_sensor.heading)
    if angle_sensor.heading < -90:
        robot.brake()
        print('Done: ', angle_sensor.heading)
        break
    await asleep_ms(10)

  await asleep_ms(2000)
  print('Angle done: ', angle_sensor.heading) 

'''
mdv1 = MotorDriverV1()
m1 = DCMotor(mdv1, MDV1_M1, reversed=True)
m2 = DCMotor(mdv1, MDV1_M4, reversed=True)
'''
mdv2 = MotorDriverV2()
mdv2.pid_off()
m1 = DCMotor(mdv2, MDV2_E2, reversed=False )
m2 = DCMotor(mdv2, MDV2_E1, reversed=False )

robot = DriveBase(MODE_2WD, m1, m2)
#robot.speed(30)
gamepad = Gamepad()
btn_BOOT= aButton(BOOT_PIN)
imu = MPU6050()
angle_sensor = AngleSensor(imu)

def deinit():
  robot.stop()

import yolo_uno
yolo_uno.deinit = deinit

async def print_test():
    while True:
        print(angle_sensor.heading)
        await asleep_ms(500)

async def setup():
  neopix.show(0, hex_to_rgb('#ff0000'))
  print('App started')
  btn_BOOT.pressed(on_abutton_BOOT_pressed)
  #create_task(ble.wait_for_msg())
  gamepad.start()
  #create_task(robot.start_rc_mode(gamepad, 50, 3))
  angle_sensor.start(500)
  robot.angle_sensor(angle_sensor)
  #create_task(print_test())
  neopix.show(0, hex_to_rgb('#00ff00'))

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())


