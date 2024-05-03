from motor import *
from mdv2 import *
from drivebase import *
from servo import *
from mpu6050 import MPU6050
from angle_sensor import AngleSensor
from ble import *
from gamepad import *
from tm1637 import *
from abutton import *

async def on_cmd_BTN_L1():
  await servo1.run_angle(angle=180, speed=100)

async def on_cmd_BTN_L2():
  await servo1.run_angle(angle=0, speed=100)

async def on_cmd_BTN_R1():
  await servo2.run_angle(angle=80, speed=100)

async def on_cmd_BTN_R2():
  await servo2.run_angle(angle=0, speed=100)

async def on_abutton_BOOT_pressed():
  robot.speed(60, min_speed=45)
  robot.mode_auto = True
  angle_sensor.calibrate(500)
  await asleep_ms(500)
  for count in range(3):
    await robot.forward_for(60, unit=CM, then=BRAKE)
    await asleep_ms(1000)
    await robot.backward_for(60, unit=CM, then=BRAKE)
    await asleep_ms(1000)
    await robot.turn_left_for(85, unit=DEGREE, then=BRAKE)
    await asleep_ms(1000)
    await robot.turn_right_for(85, unit=DEGREE, then=BRAKE)
    await asleep_ms(1000)
  robot.mode_auto = False
  robot.speed(100, min_speed=45)

md_v2 = MotorDriverV2()
motor1 = DCMotor(md_v2, M1, reversed=False)
motor2 = DCMotor(md_v2, M2, reversed=False)
motor3 = DCMotor(md_v2, E1, reversed=True)
motor4 = DCMotor(md_v2, E2, reversed=True)
robot = DriveBase(MODE_MECANUM, m1=motor1, m2=motor2, m3=motor3, m4=motor4)
servo1 = Servo(md_v2, S1, 180)
servo2 = Servo(md_v2, S2, 180)
servo3 = Servo(md_v2, S4, 360)
imu = MPU6050()
angle_sensor = AngleSensor(imu)
gamepad = Gamepad()
tm1637 = TM1637(dio=D10_PIN, clk=D9_PIN)
btn_BOOT= aButton(BOOT_PIN)

def deinit():
  robot.stop()
  btn_BOOT.deinit()

import yolo_uno
yolo_uno.deinit = deinit

async def task_U_Q_v_D():
  while True:
    await asleep_ms(75)
    if gamepad.data[BTN_TRIANGLE] == 1:
      await servo1.run_steps(5)
    if gamepad.data[BTN_CROSS] == 1:
      await servo1.run_steps((-5))
    if gamepad.data[BTN_CIRCLE] == 1:
      await servo2.run_steps((-5))
    if gamepad.data[BTN_SQUARE] == 1:
      await servo2.run_steps(5)
    if (gamepad.data[ARY]) > 50:
      servo3.spin(0)
    if (gamepad.data[ARY]) < -50:
      servo3.spin(75)

async def task_s_e_F_v():
  while True:
    await asleep_ms(2000)
    tm1637.number(((md_v2.battery()) * 10))

async def setup():

  print('App started')
  neopix.show(0, hex_to_rgb('#ff0000'))
  motor3.set_encoder(rpm=600, ppr=11, gears=34)
  motor4.set_encoder(rpm=600, ppr=11, gears=34)
  servo1.limit(min=0, max=180)
  servo2.limit(min=0, max=80)
  await servo1.run_angle(angle=45, speed=95)
  await servo2.run_angle(angle=45, speed=95)
  robot.speed(100, min_speed=45)
  robot.pid(Kp=8, Ki=0.15, Kd=0)
  angle_sensor.calibrate(100)
  create_task(angle_sensor.run())
  robot.angle_sensor(angle_sensor)
  neopix.show(0, hex_to_rgb('#00ff00'))

  robot.on_teleop_command(BTN_L1, on_cmd_BTN_L1)
  robot.on_teleop_command(BTN_L2, on_cmd_BTN_L2)
  robot.on_teleop_command(BTN_R1, on_cmd_BTN_R1)
  robot.on_teleop_command(BTN_R2, on_cmd_BTN_R2)
  btn_BOOT.pressed(on_abutton_BOOT_pressed)
  create_task(ble.wait_for_msg())
  create_task(gamepad.run())
  create_task(robot.run_teleop(gamepad, accel_steps=3))
  create_task(task_U_Q_v_D())
  create_task(task_s_e_F_v())

async def main():
  await setup()
  while True:
    await asleep_ms(100)

run_loop(main())
