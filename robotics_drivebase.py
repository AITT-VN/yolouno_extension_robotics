from micropython import const
from time import ticks_ms
import asyncio
from machine import SoftI2C, Pin
from i2c_motors_driver import MotorDriver
from robotics_motor import *
from robotics_line_sensor import *
from robotics_gamepad import GamePadReceiver
from ble import *
from utility import *
from yolo_uno import *
from setting import SCL_PIN, SDA_PIN

# stop method
STOP = const(0)
BRAKE = const(1)

# move unit
SECOND = const(0)
DEGREE = const(1)
CM = const(2)

# remote control
OHSTEM_APP = const(0)
GAMEPAD = const(1)

BTN_FORWARD = '!B516'
BTN_BACKWARD = '!B615'
BTN_LEFT = '!B714'
BTN_RIGHT = '!B814'

BTN_RELEASED = ''

APP_BTN_A = '!B11:'
APP_BTN_B = '!B219'
APP_BTN_C = '!B318'
APP_BTN_D = '!B417'

GAMEPAD_BTN_A = const(0)
GAMEPAD_BTN_B = const(1)
GAMEPAD_BTN_X = const(2)
GAMEPAD_BTN_Y = const(3)
GAMEPAD_BTN_L1 = const(4)
GAMEPAD_BTN_R1 = const(5)
GAMEPAD_BTN_L2 = const(6)
GAMEPAD_BTN_R2 = const(7)


class DriveBase:
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        self._speed = 75
        self._turn_speed = 50
        self._m1 = m1
        self._m2 = m2
        self._m3 = m3
        self._m4 = m4

        self._line_sensor = None
        self._angle_sensor = None

        # speed adjustment rate for 2 wheels when moving straight forward
        self._straight_adjust_rate = 0.3
        self._accel_distance = 0.3 # only run fast after 25% distance
        self._accel_rate = 0.75 # slow down using 85% speed at first
        self._decel_distance = 0.7 # run slow after 75% distance
        self._decel_rate = 0.75 # slow down using 85% at last
        self._turn_error = 0.85 # should stop before reaching end goal

        # remote control
        self._rc_cmd = None
        self._last_rc_cmd = None
        self._rc_speed = 0
        self._rc_cmd_handlers = {}

        self._last_line_state = LINE_CENTER

    ######################## Configuration #####################

    '''
        Config move and turn speed.

        Parameters:
             speed (Number) - Default speed used to move, 0 to 100.
             turn_speed (Number) - Default speed used to turn, 0 to 100.
    '''
    def speed(self, speed=None, turn_speed=None):
        if speed == None and turn_speed == None:
            return (self._speed, self._turn_speed)
        else:
            self._speed = speed
            if turn_speed == None:
                self._turn_speed = round(0.75*self._speed)
            else:
                self._turn_speed = turn_speed
    
    def line_sensor(self, sensor):
        self._line_sensor = sensor
    
    def angle_sensor(self, sensor):
        self._angle_sensor = sensor

    ######################## Driving functions #####################

    async def turn(self, ratio, speed, time=None, then=STOP):
        '''
        if ratio > 0:
            left = speed
            rigth = -2*(speed/100)*ratio + speed 
        elif ratio < 0:
            right = speed
            left = -2*(speed/100)*ratio + speed 
        else:
            self.forward(speed)
        '''
        raise NotImplementedError

    async def turn_angle(self, ratio, speed, angle, then=STOP):
        raise NotImplementedError

    async def forward(self, speed, time=None, then=STOP):
        raise NotImplementedError
    
    async def backward(self, speed, time=None, then=STOP):
        raise NotImplementedError
    
    async def turn_left(self, speed, time=None, then=STOP):
        raise NotImplementedError

    async def turn_right(self, speed, time=None, then=STOP):
        raise NotImplementedError

    async def move_left(self, speed, time=None, then=STOP):
        pass

    async def move_right(self, speed, time=None, then=STOP):
        pass

    def run(self, m1_speed, m2_speed, m3_speed=None, m4_speed=None):
        self._m1.speed(m1_speed)
        self._m2.speed(m2_speed)
        if m3_speed:
            self._m3.speed(m3_speed)
        if m4_speed:
            self._m4.speed(m4_speed)

    ######################## Stop functions #####################
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    #def stop(self):
    #    pass
    
    '''
        Stops the robot by given method.

        Parameters:
            then: STOP or BRAKE or None
    '''
    async def stop_then(self, then):
        if then == BRAKE:
            self.brake()
            await asleep_ms(100)
            self.stop()
        elif then == STOP:
            self.stop()
        else:
            return
    
    '''
        Stops the robot by passively braking the motors.
    '''
    #def brake(self):
    #    pass

    ######################## Remote control #####################
    async def enable_rc_mode(self, by=OHSTEM_APP):
        global ble
        if by == OHSTEM_APP:
            ble.on_receive_msg("string", self.on_ohstem_app_cmd)

        if by == GAMEPAD:
            try:
                self._i2c_gp = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
                self._gamepad = GamePadReceiver(self._i2c_gp)
            except:
                self._gamepad = None
                print('Wireless gamepad receiver not found')
                return

        while True:
            if by == GAMEPAD and self._gamepad:
                # read command from gamepad receiver if connected
                self._gamepad.update()

                if self._gamepad.is_connected == True:
                    if self._gamepad.data['dpad_up']:
                        self._rc_cmd = BTN_FORWARD
                    elif self._gamepad.data['dpad_down']:
                        self._rc_cmd = BTN_BACKWARD
                    elif self._gamepad.data['dpad_left']:
                        self._rc_cmd = BTN_LEFT
                    elif self._gamepad.data['dpad_right']:
                        self._rc_cmd = BTN_RIGHT
                    elif self._gamepad.data['a']:
                        self._rc_cmd = GAMEPAD_BTN_A
                    elif self._gamepad.data['b']:
                        self._rc_cmd = GAMEPAD_BTN_B
                    elif self._gamepad.data['x']:
                        self._rc_cmd = GAMEPAD_BTN_X
                    elif self._gamepad.data['y']:
                        self._rc_cmd = GAMEPAD_BTN_Y
                    elif self._gamepad.data['l1']:
                        self._rc_cmd = GAMEPAD_BTN_L1
                    elif self._gamepad.data['r1']:
                        self._rc_cmd = GAMEPAD_BTN_R1
                    elif self._gamepad.data['l2']:
                        self._rc_cmd = GAMEPAD_BTN_L2
                    elif self._gamepad.data['r2']:
                        self._rc_cmd = GAMEPAD_BTN_R2
                    else:
                        self._rc_cmd = BTN_RELEASED

            if self._rc_cmd != self._last_rc_cmd: # got new command
                self._rc_speed = 25 # reset speed
            else:
                if self._rc_speed < 50:
                    self._rc_speed = self._rc_speed + 1
                else:
                    self._rc_speed = 50

            if self._rc_cmd in self._rc_cmd_handlers:
                self._rc_cmd_handlers[self._rc_cmd]
                if self._rc_cmd_handlers[self._rc_cmd] != None:
                    await self._rc_cmd_handlers[self._rc_cmd]()

            elif self._rc_cmd == BTN_FORWARD:
                await self.forward(self._rc_speed*2)

            elif self._rc_cmd == BTN_BACKWARD:
                await self.backward(self._rc_speed*2)

            elif self._rc_cmd == BTN_LEFT:
                await self.turn_left(self._rc_speed)

            elif self._rc_cmd == BTN_RIGHT:
                await self.turn_right(self._rc_speed)

            else:
                self.stop()
            
            self._last_rc_cmd = self._rc_cmd
            #self._rc_cmd = None
            await asyncio.sleep_ms(10)
    
    async def on_ohstem_app_cmd(self, value):
        self._rc_cmd = value

    def on_rc_command(self, cmd, callback):
        self._rc_cmd_handlers[cmd] = callback


    ######################## Utility functions #####################

    def _calib_speed(self, speed, angle_error_min=0.1, angle_error_max=10):
        if self._angle_sensor == None:
            return (speed, speed)

        z = self._angle_sensor.heading

        if abs(z) >= 360:
            z = (abs(z) - 360) * z / abs(z)
        if abs(z) > 180:
            z = z - 360 * z / abs(z)

        adjust_rate = int(abs(speed)*self._straight_adjust_rate*z)

        if adjust_rate > 25:
            adjust_rate = 25

        if abs(z) > angle_error_max:
            return (speed, speed)

        if abs(z) > angle_error_min:
            left_speed = round(speed - adjust_rate)

            if abs(left_speed) < 20:
                if left_speed < 0:
                    left_speed = -20
                elif left_speed > 0:
                    left_speed = 20
            elif abs(left_speed) > 100:
                if left_speed < 0:
                    left_speed = -100
                else:
                    left_speed = 100

            right_speed = round(speed + adjust_rate)

            if abs(right_speed) < 20:
                if right_speed < 0:
                    right_speed = -20
                elif right_speed > 0:
                    right_speed = 20
            elif abs(right_speed) > 100:
                if right_speed < 0:
                    right_speed = -100
                else:
                    right_speed = 100

            #print(z, adjust_rate, left_speed, right_speed)
            return (left_speed, right_speed)
        else:
            return (speed, speed)
    
    def _calc_steering(self, speed, steering):
        left_speed = 0
        right_speed = 0
        
        if steering > 0:
            left_speed = speed
            right_speed = int(-2*(speed/100)*steering + speed)
        elif steering < 0:
            right_speed = speed
            left_speed = int(-2*(speed/100)*abs(steering) + speed)
        else:
            left_speed = right_speed = speed
        
        return (left_speed, right_speed)
    
    ######################## Line following #####################

    async def follow_line(self, speed, backward=True, line_state=None):
        if self._line_sensor == None:
            return
        
        self.speed_factors = [ 25, 50, 100 ] # 1: light turn, 2: normal turn, 3: heavy turn
        ratio = 0
        speed = abs(max(min(100, speed),-100))

        if line_state == None:
            line_state = self._line_sensor.check()

        if line_state == LINE_END: #no line found
            if backward:
                self.backward(speed)
        else:
            if line_state == LINE_CENTER:
                if self._last_line_state == LINE_CENTER:
                    await self.forward(speed) #if it is running straight before then robot should speed up now           
                else:
                    await self.forward(int(speed * 2/3)) #just turn before, shouldn't set high speed immediately, speed up slowly
            else:
                if line_state == LINE_RIGHT2:
                    ratio = -35 #left normal turn
                elif line_state == LINE_LEFT2:
                    ratio = 35 #right normal turn
                elif line_state == LINE_RIGHT:
                    ratio = 15 # left light turn
                elif line_state == LINE_LEFT:
                    ratio = -15 # right light turn
                elif line_state == LINE_RIGHT3:
                    ratio = -70 # left heavy turn
                elif line_state == LINE_LEFT3:
                    ratio = 70 # right heavy turn
                
                await self.turn(ratio, speed)
        
        self._last_line_state = line_state


    async def follow_line_until_end(self, speed, then=STOP):
        count = 2

        while True:
            line_state = self._line_sensor.check()
            #print(line_state)

            if line_state == LINE_END:
                count = count - 1
                if count == 0:
                    break

            await self.follow_line(abs(speed), False, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def follow_line_until_cross(self, speed, then=STOP):
        status = 1
        count = 0

        while True:
            line_state = self._line_sensor.check()
            #print(line_state)

            if status == 1:
                if line_state != LINE_CROSS:
                    status = 2
            elif status == 2:
                if line_state == LINE_CROSS:
                    count = count + 1
                    if count == 2:
                        break

            await self.follow_line(abs(speed), True, line_state)

            await asleep_ms(10)

        await self.forward(speed, 20)
        await self.stop_then(then)

    async def follow_line_until(self, speed, condition, then=STOP):
        status = 1
        count = 0

        while True:
            line_state = self._line_sensor.check()

            if status == 1:
                if line_state != LINE_CROSS:
                    status = 2
            elif status == 2:
                if condition():
                    count = count + 1
                    if count == 2:
                        break

            await self.follow_line(speed, True, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_line_detected(self, ratio, speed, then=STOP):
        counter = 0
        status = 0

        await self.turn(ratio, speed)

        while True:
            line_state = self._line_sensor.check()

            if status == 0:
                if line_state == LINE_END: # no black line detected
                    # ignore case when robot is still on black line since started turning
                    status = 1
            
            elif status == 1:
                if line_state != LINE_END:
                    self.turn(int(ratio*0.75), speed)
                    counter = counter - 1
                    if counter <= 0:
                        break

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_condition(self, ratio, speed, condition, then=STOP):
        count = 0

        await self.turn(ratio, speed)

        while True:
            if condition():
                count = count + 1
                if count == 3:
                    break
            await asleep_ms(10)

        await self.stop_then(then)


class Robot2WD(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor):
        super().__init__(m1, m2, None, None)
        self._m1.reverse()
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        self.run(0, 0, None, None)
        
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        self._m1.brake()
        self._m2.brake()
    
    async def turn(self, ratio, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), 0))
        left_speed, right_speed = self._calc_steering(speed, ratio)
        self.run(left_speed, right_speed)
        
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def turn_angle(self, ratio, speed, angle, then=STOP):
        if self._angle_sensor == None:
            return

        speed = abs(max(min(100, speed), 0))
        left_speed, right_speed = self._calc_steering(speed, ratio)

        if angle < 0:
            tmp = left_speed
            left_speed = right_speed
            right_speed = tmp

        if abs(angle) > 359:
            angle = 359

        accel_angle = abs(int(angle*self._accel_distance))
        decel_angle = abs(int(angle*self._decel_distance))
        end_angle = abs(angle * self._turn_error) # slow down during last 15% angle
        time_start = ticks_ms()
        await self._angle_sensor.reset()
        
        while True:
            distance = int(abs(self._angle_sensor.heading))
            #print(distance)

            if distance > end_angle:
                break
            elif distance > decel_angle:
                self.run(int(left_speed*self._decel_rate), int(right_speed*self._decel_rate))
            elif distance < accel_angle:
                self.run(int(left_speed*self._accel_rate), int(right_speed*self._accel_rate))        
            else:
                self.run(left_speed, right_speed)

            if ticks_ms() - time_start > 5000:
                print('Turn timeout')
                break
            await asleep_ms(1)

        await self.stop_then(then)
        #await asleep_ms(500)
        #distance = int(abs(self._angle_sensor.heading))
        #print('Stopped at: ', distance)

    async def forward(self, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), -100))
        if self._angle_sensor != None:
            await self._angle_sensor.reset()
        self.run(speed, speed, None, None)
        if time:
            time_start = ticks_ms()
            while ticks_ms() - time_start < time:
                # go straight
                left_speed, right_speed = self._calib_speed(speed)
                #print(self._angle_sensor.heading, left_speed, right_speed)
                self.run(left_speed, right_speed, None, None)
                await asyncio.sleep_ms(1)

            await self.stop_then(then)
    
    async def backward(self, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), -100))
        self.run(-speed, -speed, None, None)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def turn_left(self, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), -100))
        self.run(-speed, speed, None, None)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def turn_right(self, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), -100))
        self.run(speed, -speed, None, None)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

class Robot4WD(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        super().__init__(m1, m2, m3, m4)
        self._m1.reverse()
        self._m3.reverse()
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        self.run(0, 0, 0, 0)
        
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        self._m1.brake()
        self._m2.brake()
        self._m3.brake()
        self._m4.brake()

    async def turn(self, ratio, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), 0))
        left_speed, right_speed = self._calc_steering(speed, ratio)

        self.run(left_speed, right_speed, left_speed, right_speed)
        
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def turn_angle(self, ratio, speed, angle, then=STOP):
        if self._angle_sensor == None:
            return

        speed = abs(max(min(100, speed), 0))
        left_speed, right_speed = self._calc_steering(speed, ratio)

        angle = abs(max(min(359, angle), 0))
        accel_angle = abs(int(angle*self._accel_distance))
        decel_angle = abs(int(angle*self._decel_distance))
        end_angle = abs(angle * self._turn_error) # slow down during last 15% angle
        time_start = ticks_ms()
        await self._angle_sensor.reset()
        
        while True:
            distance = int(abs(self._angle_sensor.heading))

            if distance < accel_angle:
                self.run(int(left_speed*self._accel_rate), int(right_speed*self._accel_rate), 
                         int(left_speed*self._accel_rate), int(right_speed*self._accel_rate))
            elif distance > decel_angle:
                self.run(int(left_speed*self._decel_rate), int(right_speed*self._decel_rate), 
                         int(left_speed*self._decel_rate), int(right_speed*self._decel_rate))
            elif distance > end_angle:
                break
            else:
                self.run(left_speed, right_speed, left_speed, right_speed)

            if ticks_ms() - time_start > 2500:
                print('Turn timeout')
                break
            await asleep_ms(10)

        await self.stop_then(then)
    
    async def forward(self, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed), 0))
        if self._angle_sensor != None:
            await self._angle_sensor.reset()
        self.run(speed, speed, speed, speed)
        if time:
            time_start = ticks_ms()
            while ticks_ms() - time_start < time:
                # go straight
                left_speed, right_speed = self._calib_speed(speed)
                self.run(left_speed, right_speed, left_speed, right_speed)

            await self.stop_then(then)
    
    async def backward(self, speed, time=None, then=STOP):
        self.run(-speed, -speed, -speed, -speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def turn_left(self, speed, time=None, then=STOP):
        self.run(-speed, speed, -speed, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def turn_right(self, speed, time=None, then=STOP):
        self.run(speed, -speed, speed, -speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)


class RobotMecanum(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        super().__init__(m1, m2, m3, m4)
        # Mecanum mode
        # Motor connection
        # // m1 | m4 \\
        # ------| -----
        # \\ m2 | m3 //
        
        #     Dir
        # NW   N   NE
        #  W   .   E
        # SW   S   SE
        
        # SpinR-SpinL
        self._dir = ((1, -1, 1, -1),    # turn right
                     (1, 0, 0, 1),
                     (1, 1, 1, 1),      # forward
                     (0, 1, 1, 0),
                     (-1, 1, -1, 1),    # turn left
                     (-1, 0, 0, -1),    
                     (-1, -1, -1, -1),  # backward
                     (0, -1, -1, 0),
                     (-1, 1, 1, -1),    # move left
                     (1, -1, -1, 1))    # move right
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        self._m1.speed(0)
        self._m2.speed(0)
        self._m3.speed(0)
        self._m4.speed(0)
        
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        self._m1.brake()
        self._m2.brake()
        self._m3.brake()
        self._m4.brake()

    def drive(self, dir, speed):
        # calculate direction based on angle
        #         90(3)
        #   135(4) |  45(2)
        # 180(5)---+----Angle=0(dir=1)
        #   225(6) |  315(8)
        #         270(7)

        speed = abs(max(min(100, speed),-100))

        self._m1.speed(speed*self._dir[dir-1][0])
        self._m4.speed(speed*self._dir[dir-1][1])
        self._m2.speed(speed*self._dir[dir-1][2])
        self._m3.speed(speed*self._dir[dir-1][3])

    async def turn(self, ratio, speed, time=None, then=STOP):
        speed = abs(max(min(100, speed),-100))

        if ratio > 0:
            self.drive(2, speed)
        elif ratio < 0:
            self.drive(4, speed)
        else:
            await self.forward(speed)
        
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def turn_angle(self, ratio, speed, angle, then=STOP):
        if self._angle_sensor == None:
            return

        speed = abs(max(min(100, speed), -100))

        angle = abs(max(min(359, angle), 0))
        end_angle = abs(angle * self._turn_error) # slow down during last 15% angle
        time_start = ticks_ms()
        self._angle_sensor.reset()

        if ratio > 0:
            self.drive(2, speed)
        elif ratio < 0:
            self.drive(4, speed)
        else:
            await self.forward(speed)
        
        while True:
            distance = int(abs(self._angle_sensor.heading))

            if distance > end_angle:
                break

            if ticks_ms() - time_start > 2500:
                print('Turn timeout')
                break
            await asleep_ms(10)

        await self.stop_then(then)
    
    async def forward(self, speed, time=None, then=STOP):
        self.drive(3, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def backward(self, speed, time=None, then=STOP):
        self.drive(7, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def turn_left(self, speed, time=None, then=STOP):
        self.drive(5, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def turn_right(self, speed, time=None, then=STOP):
        self.drive(1, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)
    
    async def move_left(self, speed, time=None, then=STOP):
        self.drive(9, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

    async def move_right(self, speed, time=None, then=STOP):
        self.drive(10, speed)
        if time:
            await asleep_ms(time)
            await self.stop_then(then)

