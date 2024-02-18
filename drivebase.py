from time import ticks_ms
import asyncio, math
from ble import *
from utility import *
from yolo_uno import *
from setting import *
from constants import *
from motor import *
from line_sensor import *
from gamepad import *

class DriveBase:
    def __init__(self, drive_mode, m1, m2, m3=None, m4=None):
        if drive_mode not in (MODE_2WD, MODE_4WD, MODE_MECANUM):
            raise ValueError("Invalid drivebase mode, should be MODE_2WD, MODE_4WD or MODE_MECANUM")
        else:
            self._drive_mode = drive_mode
        
        self._m1 = m1 # front left motor
        self._m2 = m2 # front right motor
        self._m3 = m3 # back left motor
        self._m4 = m4 # back right motor

        if drive_mode == MODE_2WD:
            self._m1.reverse()
        elif drive_mode == MODE_4WD or drive_mode == MODE_MECANUM:
            self._m1.reverse()
            self._m3.reverse()

        self._speed = 75

        self._turn_mode = ENCODER
        self._wheel_diameter = 65 # mm
        self._width = 200 # mm
        self._wheel_circ = math.pi * self._wheel_diameter # mm
        self._ticks_per_rev = 0
        self._ticks_to_m = 0

        self._line_sensor = None
        self._angle_sensor = None
        self._use_gyro = False

        # remote control
        self._gamepad = None
        self._teleop_cmd = None
        self._last_teleop_cmd = None
        self._teleop_speed = 0
        self._teleop_cmd_handlers = {}

        # line following sensor state detected
        self._last_line_state = LINE_CENTER

        # mecanum mode speed setting

        # Motor connection
        # \\ m1 | m2 //
        # ------| -----
        # // m3 | m4 \\

        self._mecanum_speed_factor = (
            (1, 1, 1, 1),      # forward DIR_FW 
            (1, 0, 0, 1),      # right forward DIR_RF
            (1, -1, 1, -1),    # turn right DIR_R
            (0, -1, -1, 0),    # right backward DIR_RB
            (-1, -1, -1, -1),  # backward DIR_BW
            (-1, 0, 0, -1),    # left backward DIR_LB
            (-1, 1, -1, 1),    # turn left DIR_L
            (0, 1, 1, 0),      # left forward DIR_LF
            (-1, 1, 1, -1),    # move side left DIR_SL
            (1, -1, -1, 1)     # move side right DIR_SR
        )


    ######################## Configuration #####################

    '''
        Config moving speed.

        Parameters:
             speed (Number) - Default speed used to move, 0 to 100.
    '''
    def speed(self, speed=None):
        if speed == None:
            return self._speed
        else:
            self._speed = speed
    
    def line_sensor(self, sensor):
        self._line_sensor = sensor
    
    def angle_sensor(self, sensor):
        self._angle_sensor = sensor
    
    '''
        Config robot size and moving mode.

        Parameters:
             turn_mode (Number) - Default turning mode, use motor encoder or angle sensor.
             width (Number, mm) - Width between two wheels.
             wheel (Number, mm) - Wheel diameter
    '''
    def config(self, turn_mode=ENCODER, wheel=65, width=120):
        if width < 0 or wheel < 0 or turn_mode not in (ENCODER, GYRO):
            raise Exception("Invalid robot config value")

        self._turn_mode = turn_mode
        self._wheel_diameter = wheel
        self._width = width
        self._wheel_circ = math.pi * self._wheel_diameter
        self._ticks_per_rev = int((self._m1.ticks_per_rev + self._m2.ticks_per_rev)/2)
        self._ticks_to_m = (self._wheel_circ / self.ticks_per_rev) / 1000
    
    '''
        Config robot PID.

        Parameters:

    '''
    def pid(self, kp, ki, kd, kc):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._kc = kc


    ######################## Driving functions #####################

    async def forward(self, amount=None, unit=SECOND, then=STOP):
        if not amount: # run forever
            self.run(DIR_FW)
        else:
            await self.straight(self._speed, amount, unit, then)
    
    async def backward(self, amount=None, unit=SECOND, then=STOP):
        if not amount: # run forever
            self.run(DIR_BW)
        else:
            await self.straight(-self._speed, amount, unit, then)
    
    async def turn_left(self, amount=None, unit=SECOND, then=STOP):
        if not amount: # run forever
            self.run(DIR_L)
        else:
            await self.turn(-100, amount, unit, then)

    async def turn_right(self, amount=None, unit=SECOND, then=STOP):
        if not amount: # run forever
            self.run(DIR_R)
        else:
            await self.turn(100, amount, unit, then)

    async def move_left(self, amount=None, unit=SECOND, then=STOP):
        if self._drive_mode != MODE_MECANUM:
            await self.turn_left(amount, unit, then)
            return

        if not amount: # run forever
            self.run(DIR_SL)
        else:
            if unit != SECOND:
                return
            # only support SECOND unit
            distance = abs(abs(amount*1000)) # to ms
            driven = 0
            last_driven = 0
            time_start = ticks_ms()

            while True:
                driven = ticks_ms() - time_start                

                if driven > distance:
                    break

                # speed smoothing and go straight
                adjusted_speed = self._calc_speed(abs(self._speed), distance, driven, last_driven)
                self.run(DIR_SL, adjusted_speed)

                last_driven = driven
                await asyncio.sleep_ms(10)

            await self.stop_then(then)

    async def move_right(self, amount=None, unit=SECOND, then=STOP):
        if self._drive_mode != MODE_MECANUM:
            await self.turn_right(amount, unit, then)
            return

        if not amount: # run forever
            self.run(DIR_SR)
        else:
            if unit != SECOND:
                return
            # only support SECOND unit
            distance = abs(abs(amount*1000)) # to ms
            driven = 0
            last_driven = 0
            time_start = ticks_ms()

            while True:
                driven = ticks_ms() - time_start                

                if driven > distance:
                    break

                # speed smoothing and go straight
                adjusted_speed = self._calc_speed(abs(self._speed), distance, driven, last_driven)
                self.run(DIR_SR, adjusted_speed)

                last_driven = driven
                await asyncio.sleep_ms(10)

            await self.stop_then(then)

    '''
        Drives straight for a given distance and then stops.

        Parameters:
            speed (Number, %) - Speed to travel

            amount (Number, cm or inch or seconds) - Amount to travel

            then (STOP | BRAKE) - What to do after coming to a standstill.

            unit - can be CM, INCH, or SECOND
    '''
    async def straight(self, speed, amount, unit=SECOND, then=STOP):
        await self.reset_angle()
        # calculate target 
        distance = 0
        driven = 0
        last_driven = 0
        expected_speed = 0

        if unit == CM:
            distance = abs(int(amount*10)) # to mm
        elif unit == INCH:
            distance = abs(int(amount*25.4)) # to mm
        elif unit == SECOND:
            distance = abs(abs(amount*1000)) # to ms
            time_start = ticks_ms()

        while True:
            if unit == SECOND:
                driven = ticks_ms() - time_start                
            else:
                driven = abs(self.distance())

            if driven >= distance:
                break

            # speed smoothing using accel and deccel technique
            expected_speed = self._calc_speed(abs(speed), distance, driven, last_driven)
            # keep moving straight
            left_speed, right_speed = self._calib_speed(expected_speed)

            self.run_speed(left_speed, right_speed)

            last_driven = driven
            
            await asyncio.sleep_ms(10)

        await self.stop_then(then)

    '''
        Turns in place by a given angle and then stops.

        Drives an arc along a circle of a given radius, by a given angle if radius > 0.

        Parameters:
            amount (Number, deg or second) - Amount of degree or time of the turn.

            radius (Number, mm) - Radius of the arc turn.

            then - What to do after coming to a standstill.

            unit - UNIT_DEGREE or UNIT_SECOND
    '''
    async def turn(self, steering, amount=None, unit=SECOND, then=STOP):
        if not amount:
            speed = self._speed
            left_speed, right_speed = self._calc_steering(speed, steering)
            self.run_speed(left_speed, right_speed)
            return

        # calculate distance
        distance = 0
        driven_distance = 0
        last_driven = 0

        if unit == DEGREE:
            if self._use_gyro:
                if self._angle_sensor == None: # no angle sensor
                    return

                distance = amount

                if abs(distance) > 359:
                    distance = 359
            else: # use encoders
                # use encoders
                # Arc length is computed accordingly.
                # arc_length = (10 * abs(angle) * radius) / 573
                radius = 0 # Fix me
                distance = abs(( math.pi * (radius+self._width/2)*2 ) * (amount / 360 ))
                print('arc length: ', distance)
                # reference link: https://subscription.packtpub.com/book/iot-and-hardware/9781789340747/12/ch12lvl1sec11/making-a-specific-turn
            await self.reset_angle()

        elif unit == SECOND:
            distance = abs(amount*1000) # to ms

        time_start = ticks_ms()

        speed = self._speed
        left_speed, right_speed = self._calc_steering(speed, steering)
        #print(left_speed, right_speed)

        while True:
            if unit == SECOND:
                driven_distance = ticks_ms() - time_start
            elif unit == DEGREE:
                driven_distance = abs(self.angle())

            adjusted_left_speed = self._calc_speed(abs(left_speed), distance, driven_distance, last_driven)
            adjusted_right_speed = self._calc_speed(abs(right_speed), distance, driven_distance, last_driven)

            if left_speed < 0:
                adjusted_left_speed = -adjusted_left_speed
            if right_speed < 0:
                adjusted_right_speed = -adjusted_right_speed

            #print(driven_distance, adjusted_left_speed, adjusted_right_speed)
            
            self.run_speed(adjusted_left_speed, adjusted_right_speed)

            last_driven = driven_distance

            if driven_distance >= distance:
                break

            await asyncio.sleep_ms(10)
        
        await self.stop_then(then)

    ######################## Drive forever #####################

    '''
        Starts driving to the specified direction at given speed. 

        Parameters:
            dir (Number) - One of 8 directions plus 2 sidingg for mecanum mode

            speed (Number, %) - Running speed, from 0 to 100.
            
    '''
    
    def run(self, dir, speed=None):

        # calculate direction based on angle
        #           90(DIR_FW)
        #   135(DIR_LF) |  45(DIR_RF)
        # 180(DIR_L) ---+----Angle=0(dir=DIR_R)
        #   225(DIR_LB) |  315(DIR_RB)
        #         270(DIR_BW)
        #
        # DIR_SL: move side left DIR_SR: move side right only for mecanum

        if speed == None:
            speed = self._speed
        else:
            speed = abs(max(min(100, speed), -100))

        if self._drive_mode == MODE_MECANUM:
            self._m1.run(speed*self._mecanum_speed_factor[dir][0])
            self._m2.run(speed*self._mecanum_speed_factor[dir][1])
            self._m3.run(speed*self._mecanum_speed_factor[dir][2])
            self._m4.run(speed*self._mecanum_speed_factor[dir][3])
            return
        else:
            if dir == DIR_FW:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(speed)
                    self._m2.run(speed)
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(speed)
                    self._m2.run(speed)
                    self._m3.run(speed)
                    self._m4.run(speed)

            elif dir == DIR_BW:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(-speed)
                    self._m2.run(-speed)
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(-speed)
                    self._m2.run(-speed)
                    self._m3.run(-speed)
                    self._m4.run(-speed)

            elif dir == DIR_L:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(-speed)
                    self._m2.run(speed)
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(-speed)
                    self._m2.run(speed)
                    self._m3.run(-speed)
                    self._m4.run(speed)

            elif dir == DIR_R:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(speed)
                    self._m2.run(-speed)
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(speed)
                    self._m2.run(-speed)
                    self._m3.run(speed)
                    self._m4.run(-speed)

            elif dir == DIR_RF:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(speed)
                    self._m2.run(int(speed/2))
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(speed)
                    self._m2.run(int(speed/2))
                    self._m3.run(speed)
                    self._m4.run(int(speed/2))

            elif dir == DIR_LF:
                if self._drive_mode == MODE_2WD:
                    self._m2.run(speed)
                    self._m1.run(int(speed/2))
                elif self._drive_mode == MODE_4WD:
                    self._m2.run(speed)
                    self._m1.run(int(speed/2))
                    self._m4.run(speed)
                    self._m3.run(int(speed/2))
            
            elif dir == DIR_RB:
                if self._drive_mode == MODE_2WD:
                    self._m1.run(-speed)
                    self._m2.run(int(-speed/2))
                elif self._drive_mode == MODE_4WD:
                    self._m1.run(-speed)
                    self._m2.run(int(-speed/2))
                    self._m3.run(-speed)
                    self._m4.run(int(-speed/2))

            elif dir == DIR_LB:
                if self._drive_mode == MODE_2WD:
                    self._m2.run(-speed)
                    self._m1.run(int(-speed/2))
                elif self._drive_mode == MODE_4WD:
                    self._m2.run(-speed)
                    self._m1.run(int(-speed/2))
                    self._m4.run(-speed)
                    self._m3.run(int(-speed/2))

            else:
                self.stop()
    
    '''
        Starts driving with the specified left and right speed. 

        Parameters:
            left_speed (Number, %) - Left motor speed, from 0 to 100.

            right_speed (Number, %) - Right motor speed, from 0 to 100.
            
    '''
    
    def run_speed(self, left_speed, right_speed=None):
        left_speed = max(min(100, left_speed), -100)

        if right_speed == None:
            right_speed = left_speed
        else:
            right_speed = max(min(100, right_speed), -100)

        if self._drive_mode == MODE_MECANUM:
            self._m1.run(left_speed)
            self._m3.run(left_speed)
            self._m2.run(right_speed)
            self._m4.run(right_speed)
            return
        elif self._drive_mode == MODE_2WD:
            self._m1.run(left_speed)
            self._m2.run(right_speed)
        elif self._drive_mode == MODE_4WD:
            self._m1.run(left_speed)
            self._m2.run(right_speed)
            self._m3.run(left_speed)
            self._m4.run(right_speed)
        else:
            self.stop()

    ######################## Stop functions #####################
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        if self._drive_mode == MODE_2WD:
            self._m1.run(0)
            self._m2.run(0)
        elif self._drive_mode == MODE_4WD or self._drive_mode == MODE_MECANUM:
            self._m1.run(0)
            self._m2.run(0)
            self._m3.run(0)
            self._m4.run(0)
    
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        if self._drive_mode == MODE_2WD:
            self._m1.brake()
            self._m2.brake()
        elif self._drive_mode == MODE_4WD or self._drive_mode == MODE_MECANUM:
            self._m1.brake()
            self._m2.brake()
            self._m3.brake()
            self._m4.brake()

    '''
        Stops the robot by given method.

        Parameters:
            then: STOP or BRAKE or None
    '''
    async def stop_then(self, then):
        if then == BRAKE:
            self.brake()
            await asyncio.sleep_ms(500)
            self.stop()
        elif then == STOP:
            self.stop()
        else:
            return

    ######################## Measuring #####################

    '''
        Gets the estimated driven distance.

        Returns:
            Driven distance since last reset (mm).
    '''
    def distance(self):
        angle = (abs(self._m1.angle()) + abs(self._m2.angle()))/2
        distance = (angle * self._wheel_circ) / 360

        return distance
    
    '''
        Gets the estimated driven angle.

        Returns:
            Driven angle since last reset (degree).
    '''
    def angle(self):
        if self._use_gyro:
            if self._angle_sensor:
                return self._angle_sensor.heading
            else:
                return 0
        else:
            return (abs(self._m1.angle()) + abs(self._m2.angle()))/2
    
    '''
        Resets the estimated driven distance and angle to 0.
    '''
    async def reset_angle(self):
        if self._angle_sensor:
            await self._angle_sensor.reset()

        if self._drive_mode == MODE_2WD:
            self._m1.reset_angle()
            self._m2.reset_angle()
        elif self._drive_mode == MODE_4WD or self._drive_mode == MODE_MECANUM:
            self._m1.reset_angle()
            self._m2.reset_angle()
            self._m3.reset_angle()
            self._m4.reset_angle()

    ######################## Remote control #####################

    async def run_teleop(self, gamepad, start_speed=30, accel_step=5):
        self._gamepad = gamepad
        self._teleop_cmd = ''
        speed = start_speed
        turn_speed = start_speed
        while True:
            if self._gamepad.data[BTN_UP]:
                self._teleop_cmd = BTN_UP
            elif self._gamepad.data[BTN_DOWN]:
                self._teleop_cmd = BTN_DOWN
            elif self._gamepad.data[BTN_LEFT]:
                self._teleop_cmd = BTN_LEFT
            elif self._gamepad.data[BTN_RIGHT]:
                self._teleop_cmd = BTN_RIGHT
            elif self._gamepad.data[BTN_L1]:
                self._teleop_cmd = BTN_L1
            elif self._gamepad.data[BTN_R1]:
                self._teleop_cmd = BTN_R1
            elif self._gamepad.data[BTN_TRIANGLE]:
                self._teleop_cmd = BTN_TRIANGLE
            elif self._gamepad.data[BTN_SQUARE]:
                self._teleop_cmd = BTN_SQUARE
            elif self._gamepad.data[BTN_CROSS]:
                self._teleop_cmd = BTN_CROSS
            elif self._gamepad.data[BTN_CIRCLE]:
                self._teleop_cmd = BTN_CIRCLE
            elif self._gamepad.data[BTN_L2]:
                self._teleop_cmd = BTN_L2
            elif self._gamepad.data[BTN_R2]:
                self._teleop_cmd = BTN_R2
            else:
                self._teleop_cmd = ''

            if self._teleop_cmd != self._last_teleop_cmd: # got new command
                speed = start_speed # reset speed
                turn_speed = start_speed
            else:
                if speed < 100:
                    speed = speed + accel_step
                else:
                    speed = 100
                
                if turn_speed < 100:
                    turn_speed = turn_speed + int(accel_step/2)
                else:
                    turn_speed = 100

            if self._teleop_cmd in self._teleop_cmd_handlers:
                self._teleop_cmd_handlers[self._teleop_cmd]
                if self._teleop_cmd_handlers[self._teleop_cmd] != None:
                    await self._teleop_cmd_handlers[self._teleop_cmd]()
            else:
                # moving
                #print(self._teleop_cmd, self._teleop_speed)
                if self._gamepad.data[AL_DISTANCE] > 50:
                    if self._drive_mode == MODE_MECANUM:
                        if self._gamepad.data[AL_DIR] == DIR_L:
                            self.run(DIR_SL, speed)
                        elif self._gamepad.data[AL_DIR] == DIR_R:
                            self.run(DIR_SR, speed)
                        else:
                            self.run(self._gamepad.data[AL_DIR], speed)
                    else:
                        if self._gamepad.data[AL_DIR] == DIR_L:
                            self.run(DIR_L, turn_speed)
                        elif self._gamepad.data[AL_DIR] == DIR_R:
                            self.run(DIR_R, turn_speed)
                        else:
                            self.run(self._gamepad.data[AL_DIR], speed)

                elif self._teleop_cmd == BTN_UP:
                    self.run(DIR_FW, speed)

                elif self._teleop_cmd == BTN_DOWN:
                    self.run(DIR_BW, speed)

                elif self._teleop_cmd == BTN_LEFT:
                    self.run(DIR_L, turn_speed)

                elif self._teleop_cmd == BTN_RIGHT:
                    self.run(DIR_R, turn_speed)

                else:
                    self.stop()
            
            self._last_teleop_cmd = self._teleop_cmd
            await asyncio.sleep_ms(10)
    
    def on_teleop_command(self, cmd, callback):
        self._teleop_cmd_handlers[cmd] = callback


    ######################## Utility functions #####################

    '''
        Used to calculate all the speeds in our programs. Brakes and accelerates

        Parameters:
            speed: The current speed the robot has
            start_speed: Speed the robot starts at. Type: Integer. Default: No default value.
            max_speed: The maximum speed the robot reaches. Type: Integer. Default: No default value.
            end_speed: Speed the robot aims for while braking, minimum speed at the end of the program. Type: Integer. Default: No default value.
            add_speed: Percentage of the distance after which the robot reaches the maximum speed. Type: Integer. Default: No default value.
            brakeStartValue: Percentage of the driven distance after which the robot starts braking. Type: Integer. Default: No default value.
            drivenDistance: Calculation of the driven distance in degrees. Type: Integer. Default: No default value.
    '''
    def _calc_speed(self, speed, distance, driven_distance, last_driven):
        start_speed = round(speed/2.5)
        max_speed = speed
        end_speed = start_speed
        accel_distance = 0.3*distance
        decel_distance = 0.7*distance

        if driven_distance == 0:
            return start_speed
        elif abs(driven_distance) < abs(accel_distance):
            return int(start_speed + (max_speed - start_speed) * driven_distance / accel_distance)
        elif abs(driven_distance) > abs(decel_distance):
            return int(max_speed - (max_speed - end_speed) * (driven_distance-decel_distance) / (distance-decel_distance))
        else:
            return speed
    
    def _calib_speed(self, speed, angle_error_min=0.1, angle_error_max=10):
        # TODO: 
        # Apply PID to calculate speed using current steering value (using encoder ticks or angle sensor)
        '''
        if self._turn_mode == GYRO:
            if self._angle_sensor == None:
                return (speed, speed)

            z = self._angle_sensor.heading

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
        '''
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

    async def follow_line(self, backward=True, line_state=None):
        if self._line_sensor == None:
            return
        
        self.speed_factors = [ 25, 50, 100 ] # 1: light turn, 2: normal turn, 3: heavy turn
        ratio = 0

        if line_state == None:
            line_state = self._line_sensor.check()

        if line_state == LINE_END: #no line found
            if backward:
                self.backward()
        else:
            if line_state == LINE_CENTER:
                if self._last_line_state == LINE_CENTER:
                    await self.forward() #if it is running straight before then robot should speed up now           
                else:
                    self.run(DIR_FORWARD, int(self._speed * 2/3)) #just turn before, shouldn't set high speed immediately, speed up slowly
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
                
                await self.turn(ratio)
        
        self._last_line_state = line_state

    async def follow_line_until_end(self, then=STOP):
        count = 2

        while True:
            line_state = self._line_sensor.check()
            #print(line_state)

            if line_state == LINE_END:
                count = count - 1
                if count == 0:
                    break

            await self.follow_line(False, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def follow_line_until_cross(self, then=STOP):
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

            await self.follow_line(True, line_state)

            await asleep_ms(10)

        await self.forward(speed, 20)
        await self.stop_then(then)

    async def follow_line_until(self, condition, then=STOP):
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

            await self.follow_line(True, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_line_detected(self, ratio, then=STOP):
        counter = 0
        status = 0

        await self.turn(ratio)

        while True:
            line_state = self._line_sensor.check()

            if status == 0:
                if line_state == LINE_END: # no black line detected
                    # ignore case when robot is still on black line since started turning
                    status = 1
            
            elif status == 1:
                if line_state != LINE_END:
                    self.turn(int(ratio*0.75))
                    counter = counter - 1
                    if counter <= 0:
                        break

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_condition(self, ratio, condition, then=STOP):
        count = 0

        await self.turn(ratio)

        while True:
            if condition():
                count = count + 1
                if count == 3:
                    break
            await asleep_ms(10)

        await self.stop_then(then)
'''

from mdv1 import *
from motor import *
from ble import *
from gamepad import *
from abutton import *
from mpu6050 import *
from angle_sensor import *

async def on_abutton_BOOT_pressed():
  await asleep_ms(1000)
  print('left')
  await robot.test_forward(1, 75)

mdv2 = MotorDriverV2()
mdv2.pid_off()
m1 = DCMotor(mdv2, MDV2_E2, reversed=False )
m2 = DCMotor(mdv2, MDV2_E1, reversed=False )
m1.set_encoder(250, 11, 34)
m2.set_encoder(250, 11, 34)
mdv2.reverse_encoder(MDV2_E2)

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

'''