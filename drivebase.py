from time import ticks_ms, ticks_diff
import asyncio, math
from ble import *
from utility import *
from yolo_uno import *
from setting import *
from constants import *
from motor import *
from line_sensor import *
from gamepad import *
from pid import PIDController

# Enum checkpoint module 5 mat (LINE_NORMAL..LINE_FINISH) da nam trong constants.py.

# ---- hang so engine "line PID" (port tu FastLine) ----
_LINE_INTEGRAL_LIMIT = 1000.0          # chong windup khi dung Ki
_LINE_WEIGHTS = (-2.0, -1.0, 0.0, 1.0, 2.0)   # trong so truc S1..S5 (centroid analog ~[-2,2])
_LINE_MIN_RANGE = 120                  # range raw toi thieu de coi 1 mat la tin cay
_LINE_GOOD_RANGE = 300                 # calib DAT neu mat tot nhat co range >= nguong nay


def _line_clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class DriveBase:
    def __init__(self, drive_mode, m1, m2, m3=None, m4=None):
        if drive_mode not in (MODE_2WD, MODE_4WD, MODE_MECANUM):
            raise ValueError("Invalid drive mode, should be MODE_2WD, MODE_4WD or MODE_MECANUM")
        else:
            self._drive_mode = drive_mode
        
        self.left = []
        self.right = []
        self.left_motor_ports = 0
        self.right_motor_ports = 0
        self.m1 = None # front left motor
        self.m2 = None # front right motor
        self.m3 = None # back left motor
        self.m4 = None # back right motor
        self.left_encoder = None
        self.right_encoder = None

        if m1 != None:
            self.m1 = m1 # front left motor
            self.m1.reverse()
            self.left.append(m1)
            if m1.port in (E1, E2):
                self.left_encoder = m1
        
        if m3 != None:
            self.m3 = m3 # back left motor
            self.m3.reverse()
            self.left.append(m3)
            if m3.port in (E1, E2):
                self.left_encoder = m3
        
        if m2 != None:
            self.m2 = m2 # front right motor
            self.right.append(m2)
            if m2.port in (E1, E2):
                self.right_encoder = m2
        
        if m4 != None:
            self.m4 = m4 # back right motor
            self.right.append(m4)
            if m4.port in (E1, E2):
                self.right_encoder = m4

        for m in self.left:
            self.left_motor_ports += m.port

        for m in self.right:
            self.right_motor_ports += m.port

        self._speed = 75
        self._min_speed = 40

        self._wheel_diameter = 80 # mm
        self._width = 300 # mm
        self._wheel_circ = math.pi * self._wheel_diameter # mm
        self._ticks_per_rev = 0
        self._ticks_to_m = 0

        self._line_sensor = None
        self._angle_sensor = None
        self._use_gyro = False

        # remote control
        self.mode_auto = True
        self._teleop_cmd = None
        self._last_teleop_cmd = None
        self._teleop_cmd_handlers = {}
        self.side_move_mode = JOYSTICK

        # line following sensor state detected
        self._last_line_state = LINE_CENTER

        # ---- PID bam line (engine "line PID", port tu FastLine da tinh chinh) ----
        # error da chuan hoa ~[-2, 2] (0 = giua line). correction = 0 -> di thang;
        # = 1 -> pivot; > 1 -> xoay tai cho. follow_line_pid() la 1 buoc PID (sync).
        self._line_kp = 0.5
        self._line_ki = 0.0
        self._line_kd = 0.9
        self._line_invert = 1            # +1 mac dinh; -1 neu robot lai nguoc huong
        self._line_integral = 0.0
        self._line_last_error = 0.0
        # toc do
        self._line_base_speed = 60
        self._line_max_speed = 100
        # giam toc tien khi |error| lon (vao cua / mat line). 0 = khong giam.
        self._line_curve_gain = 0.85
        # vung chet: |error| <= db -> coi nhu di thang (chong giat tin hieu so).
        self._line_deadband = 0.3
        # gioi han lai khi dang bam line (turn_gain<1 -> 2 banh luon tien khi thang).
        self._line_turn_gain = 0.8
        self._line_corr_limit = 0.9
        # san bu ma sat (motor % toi thieu khi output khac 0). 0 = passthrough.
        self._line_stall_floor = 0
        # loc IIR (EMA) sai so dau vao + loc khau D.
        self._line_ema_alpha = 0.35
        self._line_d_alpha = 0.25
        # ty le toc tien giu lai khi mat line (arc recovery). 0 = xoay tai cho.
        self._line_lost_fwd = 0.45
        # so khung lien tiep de xac nhan vach ngang (debounce).
        self._line_cross_confirm = 2
        # trang thai chay PID
        self._line_d_err = 0.0
        self._line_lost_start = -1       # ts ms khi bat dau mat line (-1 = dang bam)
        self._line_lost_dir = 1
        self._line_ema_err = 0.0
        self._line_last_seen_error = 0.0
        # che do raw/analog (chi cam bien 5 mat co read_raw). 'digital' on dinh hon.
        self._line_mode = 'digital'
        self._line_cal_min = [4095, 4095, 4095, 4095, 4095]
        self._line_cal_max = [0, 0, 0, 0, 0]
        self._line_high = True
        self._line_calibrated = False
        self._line_last_aerr = 0.0
        self._line_lost_threshold = 0.5
        # debug CSV
        self._line_debug = False
        self._line_debug_interval = 100
        self._line_last_dbg = 0
        # bat PID khi nguoi dung keo khoi "line PID" (line_pid()/line_mode()).
        # False = giu thuat toan IF/ELSE roi rac (follow_line) cho cac khoi check point.
        self._line_use_pid = False

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
            (-1.2, 1.2, 1.2, -1.2),    # move side left DIR_SL
            (1.2, -1.2, -1.2, 1.2)     # move side right DIR_SR
        )

        # PID related settings
        self._pid = PIDController(5, 0.15, 0.1, setpoint=0, sample_time=None, output_limits=(-10, 10))

        self._speed_ratio = (1, 1)

    ######################## Configuration #####################

    '''
        Config moving speed.

        Parameters:
             speed (Number) - Default speed used to move, 0 to 100.
    '''
    def speed(self, speed=None, min_speed=None):
        if speed == None and min_speed == None:
            return self._speed
        else:
            self._speed = speed
            if min_speed != None:
                self._min_speed = min_speed
            else:
                self._min_speed = int(speed/2)
    
    def line_sensor(self, sensor):
        self._line_sensor = sensor

    def angle_sensor(self, sensor):
        self._angle_sensor = sensor
    
    '''
        Config robot size and moving mode.

        Parameters:
             width (Number, mm) - Width between two wheels.
             wheel (Number, mm) - Wheel diameter
    '''
    def size(self, wheel, width):
        if width < 0 or wheel < 0:
            raise Exception("Invalid robot config value")

        self._wheel_diameter = wheel
        self._width = width
        self._wheel_circ = math.pi * self._wheel_diameter

        if self.left_encoder and self.right_encoder:
            self._ticks_per_rev = int((self.left_encoder.ticks_per_rev + self.right_encoder.ticks_per_rev)/2)
            self._ticks_to_m = (self._wheel_circ / self._ticks_per_rev) / 1000
    
    '''
        Config sensor used to drive and turn precisely.

        Parameters:
             enabled (Boolean) - If True, will use gyroscope, else will use encoder
    '''
    def use_gyro(self, enabled):
        self._use_gyro = enabled

    '''
        Config robot PID.

        Parameters:

    '''
    def pid(self, Kp, Ki, Kd):
        self._pid.tunings = (Kp, Ki, Kd)
    
    '''
        Config robot speed ration to keep it moving straight.

        Parameters:

    '''
    def speed_ratio(self, left, right):
        self._speed_ratio = (left, right)

    ######################## Driving functions #####################

    def forward(self):
        self.run(DIR_FW)

    async def forward_for(self, amount, unit=SECOND, then=STOP):
        await self.straight(self._speed, amount, unit, then)
    
    def backward(self):
        self.run(DIR_BW)

    async def backward_for(self, amount, unit=SECOND, then=STOP):
        await self.straight(-self._speed, amount, unit, then)
    
    def turn_left(self):
        self.run(DIR_L)
    
    async def turn_left_for(self, amount, unit=SECOND, then=STOP):
        await self.turn(-100, amount, unit, then)

    def turn_right(self):
        self.run(DIR_R)

    async def turn_right_for(self, amount, unit=SECOND, then=STOP):
        await self.turn(100, amount, unit, then)

    def move_left(self):
        if self._drive_mode != MODE_MECANUM:
            self.turn_left()
            return
        else:
            self.run(DIR_SL)

    async def move_left_for(self, amount, unit=SECOND, then=STOP):
        if self._drive_mode != MODE_MECANUM:
            await self.turn_left_for(amount, unit, then)
            return

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

    def move_right(self):
        if self._drive_mode != MODE_MECANUM:
            self.turn_right()
            return
        else:
            self.run(DIR_SR)
    
    async def move_right_for(self, amount, unit=SECOND, then=STOP):
        if self._drive_mode != MODE_MECANUM:
            await self.turn_right_for(amount, unit, then)
            return

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

        # apply pid
        self._pid.reset()

        if unit == CM:
            distance = abs(int(amount*10)) # to mm
        elif unit == INCH:
            distance = abs(int(amount*25.4)) # to mm
        elif unit == SECOND:
            distance = abs(abs(amount*1000)) # to ms
            time_start = ticks_ms()

        speed_dir = speed/(abs(speed)) # direction

        while True:
            if unit == SECOND:
                driven = ticks_ms() - time_start                
            else:
                driven = abs(self.distance())

            #print(driven, distance)
            
            if driven >= distance:
                break
            
            if (unit == SECOND and amount < 2) or (unit == CM and amount < 10) or (unit == INCH and amount < 4):
                expected_speed = speed
            else:
                # speed smoothing using accel and deccel technique when distance is long enough
                expected_speed = speed_dir*self._calc_speed(abs(speed), distance, driven, last_driven)

            # adjust left and right speed to go straight
            left_speed, right_speed = self._calib_speed(expected_speed)

            self.run_speed(left_speed, right_speed)

            last_driven = driven
            
            await asyncio.sleep_ms(5)

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
        speed = self._speed

        if not amount:
            left_speed, right_speed = self._calc_steering(speed, steering)
            self.run_speed(left_speed, right_speed)
            return

        # calculate distance
        distance = 0
        driven_distance = 0
        last_driven = 0

        if unit == DEGREE:
            if self._use_gyro: # use angle sensor
                if self._angle_sensor == None: # no angle sensor
                    return

                distance = amount

                if abs(distance) > 359:
                    distance = 359
            else: # use encoders
                # Arc length is computed accordingly.
                # arc_length = (10 * abs(angle) * radius) / 573
                radius = 0 # Fix me
                distance = abs(( math.pi * (radius+self._width/2)*2 ) * (amount / 360 ))
                #print('arc length: ', distance)
                # reference link: https://subscription.packtpub.com/book/iot-and-hardware/9781789340747/12/ch12lvl1sec11/making-a-specific-turn
            await self.reset_angle()

        elif unit == SECOND:
            distance = abs(amount*1000) # to ms
            time_start = ticks_ms()

        #print(left_speed, right_speed)

        wheel_circ_degree = self._wheel_circ/360

        while True:
            driven_distance = 0
            if unit == SECOND:
                driven_distance = ticks_ms() - time_start
            elif unit == DEGREE:
                if self._use_gyro: # use angle sensor
                    if self._angle_sensor != None:
                        driven_distance = abs(self._angle_sensor.heading)
                    else:
                        driven_distance = 0
                else: # use encoder
                    if steering > 0:
                        driven_distance = abs(self.left_encoder.angle())*wheel_circ_degree
                    else:
                        driven_distance = abs(self.right_encoder.angle())*wheel_circ_degree

            #print(driven_distance)
            if (unit == SECOND and amount < 1) or (unit == DEGREE and amount < 45):
                expected_speed = speed
            else:
                # speed smoothing using accel and deccel technique when distance is long enough
                expected_speed = self._calc_speed(speed, distance, driven_distance, last_driven)

            left_speed, right_speed = self._calc_steering(expected_speed, steering)
            #print(expected_speed, left_speed, right_speed)

            self.run_speed(left_speed, right_speed)

            last_driven = driven_distance

            if driven_distance >= distance:
                break

            await asyncio.sleep_ms(5)
        
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
            self.m1.run(speed*self._mecanum_speed_factor[dir][0]*self._speed_ratio[0])
            self.m2.run(speed*self._mecanum_speed_factor[dir][1]*self._speed_ratio[1])
            self.m3.run(speed*self._mecanum_speed_factor[dir][2]*self._speed_ratio[0])
            self.m4.run(speed*self._mecanum_speed_factor[dir][3]*self._speed_ratio[1])
            return
        else:
            if dir == DIR_FW:
                self.run_speed(speed, speed)

            elif dir == DIR_BW:
                self.run_speed(-speed, -speed)

            elif dir == DIR_L:
                self.run_speed(-speed, speed)

            elif dir == DIR_R:
                self.run_speed(speed, -speed)

            elif dir == DIR_RF:
                self.run_speed(speed, int(speed/2))

            elif dir == DIR_LF:
                self.run_speed(int(speed/2), speed)
            
            elif dir == DIR_RB:
                self.run_speed(-speed, int(-speed/2))

            elif dir == DIR_LB:
                self.run_speed(int(-speed/2), -speed)

            else:
                self.stop()
    
    '''
        Starts driving with the specified left and right speed. 

        Parameters:
            left_speed (Number, %) - Left motor speed, from 0 to 100.

            right_speed (Number, %) - Right motor speed, from 0 to 100.
            
    '''
    
    def run_speed(self, left_speed, right_speed=None):
        if right_speed == None:
            right_speed = left_speed

        for i in range(len(self.left)):
            self.left[i].run(int(left_speed*self._speed_ratio[0]))
            self.right[i].run(int(right_speed*self._speed_ratio[1]))


    ######################## Stop functions #####################
    
    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        self.left[0].driver.set_motors(self.left_motor_ports+self.right_motor_ports, 0)
    
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        self.left[0].driver.brake(self.left_motor_ports+self.right_motor_ports)

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
        if self.left_encoder and self.right_encoder:
            #print(self.left_encoder.angle(), self.right_encoder.angle())
            angle = (abs(self.left_encoder.angle()) + abs(self.right_encoder.angle()))/2
            distance = (angle * self._wheel_circ) / 360

            return distance
        else:
            return 0
    
    '''
        Gets the estimated driven angle.

        Returns:
            Driven angle since last reset (degree).
    '''
    def angle(self):
        if self._angle_sensor:
            return self._angle_sensor.heading
        else:
            return 0
    
    '''
        Resets the estimated driven distance and angle to 0.
    '''
    async def reset_angle(self):
        if self._angle_sensor:
            await self._angle_sensor.reset()

        for m in (self.left + self.right):
            m.reset_angle()

    ######################## Remote control #####################

    async def run_teleop(self, gamepad: Gamepad, accel_steps=5):
        self.mode_auto = False
        self._teleop_cmd = ''
        speed = self._min_speed
        turn_speed = self._min_speed
        last_dir = -1
        dir = -1
        while True:
            if self.mode_auto == True: # auto mode is turned on
                await asyncio.sleep_ms(100)
                continue

            dir = -1
            if gamepad.data[AL_DISTANCE] > 50: # left joystick is acted
                dir = gamepad.data[AL_DIR]

                if self._drive_mode == MODE_MECANUM and self.side_move_mode == JOYSTICK:
                    if dir == DIR_L:
                        dir = DIR_SL
                    elif dir == DIR_R:
                        dir = DIR_SR

            elif gamepad.data[BTN_UP] and gamepad.data[BTN_LEFT]:
                self._teleop_cmd = BTN_UP
                dir = DIR_LF
            elif gamepad.data[BTN_UP] and gamepad.data[BTN_RIGHT]:
                self._teleop_cmd = BTN_UP
                dir = DIR_RF
            elif gamepad.data[BTN_DOWN] and gamepad.data[BTN_LEFT]:
                self._teleop_cmd = BTN_DOWN
                dir = DIR_LB
            elif gamepad.data[BTN_DOWN] and gamepad.data[BTN_RIGHT]:
                self._teleop_cmd = BTN_DOWN
                dir = DIR_RB
            elif gamepad.data[BTN_UP]:
                self._teleop_cmd = BTN_UP
                dir = DIR_FW
            elif gamepad.data[BTN_DOWN]:
                self._teleop_cmd = BTN_DOWN
                dir = DIR_BW
            elif gamepad.data[BTN_LEFT]:
                self._teleop_cmd = BTN_LEFT
                if self._drive_mode == MODE_MECANUM and self.side_move_mode == DPAD:
                    dir = DIR_SL
                else:
                    dir = DIR_L
            elif gamepad.data[BTN_RIGHT]:
                self._teleop_cmd = BTN_RIGHT
                if self._drive_mode == MODE_MECANUM and self.side_move_mode == DPAD:
                    dir = DIR_SR
                else:
                    dir = DIR_R
            elif gamepad.data[BTN_L1]:
                self._teleop_cmd = BTN_L1
            elif gamepad.data[BTN_R1]:
                self._teleop_cmd = BTN_R1
            elif gamepad.data[BTN_TRIANGLE]:
                self._teleop_cmd = BTN_TRIANGLE
            elif gamepad.data[BTN_SQUARE]:
                self._teleop_cmd = BTN_SQUARE
            elif gamepad.data[BTN_CROSS]:
                self._teleop_cmd = BTN_CROSS
            elif gamepad.data[BTN_CIRCLE]:
                self._teleop_cmd = BTN_CIRCLE
            elif gamepad.data[BTN_L2]:
                self._teleop_cmd = BTN_L2
            elif gamepad.data[BTN_R2]:
                self._teleop_cmd = BTN_R2
            elif gamepad.data[BTN_M1]:
                self._teleop_cmd = BTN_M1
            elif gamepad.data[BTN_M2]:
                self._teleop_cmd = BTN_M2
            elif gamepad.data[BTN_THUMBL]:
                self._teleop_cmd = BTN_THUMBL
            elif gamepad.data[BTN_THUMBR]:
                self._teleop_cmd = BTN_THUMBR
            else:
                self._teleop_cmd = ''

            if dir != last_dir: # got new direction command
                speed = self._min_speed # reset speed
                turn_speed = self._min_speed
            else:
                speed = speed + accel_steps
                if speed > self._speed:
                    speed = self._speed
                
                turn_speed = turn_speed + int(accel_steps/2)
                if turn_speed > self._speed:
                    turn_speed = self._speed
            
            if self._teleop_cmd in self._teleop_cmd_handlers:
                self._teleop_cmd_handlers[self._teleop_cmd]
                if self._teleop_cmd_handlers[self._teleop_cmd] != None:
                    await self._teleop_cmd_handlers[self._teleop_cmd]()
                    await asyncio.sleep_ms(200) # wait for button released
            else:
                # moving
                if dir in (DIR_FW, DIR_BW, DIR_SL, DIR_SR):
                    self.run(dir, speed)

                elif dir in (DIR_L, DIR_R, DIR_LF, DIR_RF, DIR_LB, DIR_RB):
                    self.run(dir, turn_speed)

                else:
                    self.stop()
            
            last_dir = dir
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
        start_speed = self._min_speed

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
    
    def _calib_speed(self, speed):

        if self._use_gyro:
            if self._angle_sensor != None:
                angle_error = self._angle_sensor.heading
            else:
                return (speed, speed)
        else:
            left_ticks = 0
            right_ticks = 0
            if self.left_encoder:
                left_ticks = abs(self.left_encoder.encoder_ticks())
            if self.right_encoder:
                right_ticks = abs(self.right_encoder.encoder_ticks())

            if speed > 0:
                angle_error = abs(left_ticks) - abs(right_ticks)
            else:
                angle_error = abs(right_ticks) - abs(left_ticks)

        correction = self._pid(angle_error)

        left = speed + correction
        right = speed - correction
        
        #print("e=" + str(angle_error) + "; c=" + str(correction) + "; L=" + str(left) + "; R=" + str(right))   
        return (left, right)

    
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
        steering = 0

        if line_state == None:
            line_state = self._line_sensor.check()

        if line_state == LINE_END: #no line found
            if backward:
                self.run(DIR_BACKWARD, self._min_speed) # slow down
        else:
            if line_state == LINE_CENTER:
                if self._last_line_state == LINE_CENTER:
                    self.forward() #if it is running straight before then robot should speed up now
                else:
                    self.run(DIR_FORWARD, self._min_speed) #just turn before, shouldn't set high speed immediately, speed up slowly

            elif line_state == LINE_CROSS:
                self.run(DIR_FORWARD, self._min_speed) # cross line found, slow down

            else:
                if line_state == LINE_RIGHT:
                    self.run_speed(self._min_speed, int(self._min_speed*1.25)) # left light turn
                elif line_state == LINE_RIGHT2:
                    self.run_speed(0, self._min_speed) # left normal turn
                elif line_state == LINE_RIGHT3:
                    while line_state != LINE_CENTER and line_state != LINE_LEFT:
                        self.run_speed(-self._min_speed, self._min_speed) # left heavy turn
                        line_state = self._line_sensor.check()
                    self._last_line_state = line_state
                    
                    return
                
                elif line_state == LINE_LEFT:
                    self.run_speed(int(self._min_speed*1.25), self._min_speed) # right light turn
                elif line_state == LINE_LEFT2:
                    self.run_speed(self._min_speed, 0) #right normal turn
                elif line_state == LINE_LEFT3:
                    while line_state != LINE_CENTER and line_state != LINE_RIGHT:
                        self.run_speed(self._min_speed, -self._min_speed) # right heavy turn
                        line_state = self._line_sensor.check()

                    self._last_line_state = line_state
                    return
        
        self._last_line_state = line_state

    '''
        1 BUOC bam line dung chung cho cac khoi check point.
            _line_use_pid = True (keo khoi "do line PID") + sensor co get_error
                -> bam line bang PID centroid (4 mat hoac 5 mat).
            nguoc lai -> giu thuat toan IF/ELSE roi rac (follow_line) nhu cu.
    '''
    async def _follow_step(self, line_state=None, backward=True):
        s = self._line_sensor
        if self._line_use_pid and hasattr(s, 'get_error'):
            # follow_line_pid() tu goi s.update() -> khong update lai o day (tranh doc I2C 2 lan).
            self.follow_line_pid()
        else:
            await self.follow_line(backward, line_state)

    async def follow_line_until_end(self, then=STOP):
        s = self._line_sensor
        if s is None:
            return
        if self._line_use_pid:
            self.reset_line_pid()
        count = 2

        while True:
            line_state = s.check()

            if line_state == LINE_END:
                count = count - 1
                if count == 0:
                    break

            await self._follow_step(line_state, backward=False)

            await asleep_ms(5 if self._line_use_pid else 10)

        await self.stop_then(then)

    async def follow_line_until_cross(self, then=STOP):
        s = self._line_sensor
        if s is None:
            return
        if self._line_use_pid:
            self.reset_line_pid()

        status = 1
        count = 0
        while True:
            line_state = s.check()   # nhan biet LINE_CROSS (ca 2 che do)

            if status == 1:
                if line_state != LINE_CROSS:
                    status = 2
            elif status == 2:
                if line_state == LINE_CROSS:
                    count = count + 1
                    if count == 2:
                        break

            await self._follow_step(line_state, backward=True)

            await asleep_ms(5 if self._line_use_pid else 10)

        await self.stop_then(then)

    async def follow_line_by_time(self, timerun, then=STOP):
        if self._line_use_pid:
            self.reset_line_pid()
        start_time = time.ticks_ms()
        duration = timerun * 1000 # convert to ms

        while time.ticks_ms() - start_time < duration:
            await self._follow_step(backward=True)
            await asleep_ms(5 if self._line_use_pid else 10)

        await self.stop_then(then)

    async def follow_line_until(self, condition, then=STOP):
        s = self._line_sensor
        if self._line_use_pid:
            self.reset_line_pid()
        status = 1
        count = 0

        while True:
            line_state = s.check()

            if status == 1:
                if line_state != LINE_CROSS:
                    status = 2
            elif status == 2:
                if condition():
                    count = count + 1
                    if count == 2:
                        break

            await self._follow_step(line_state, backward=True)

            await asleep_ms(5 if self._line_use_pid else 10)

        await self.stop_then(then)

    async def turn_until_line_detected(self, steering, then=STOP):
        counter = 0
        status = 0

        await self.turn(steering)

        while True:
            line_state = self._line_sensor.check()

            if status == 0:
                if line_state == LINE_END: # no black line detected
                    # ignore case when robot is still on black line since started turning
                    status = 1
            
            elif status == 1:
                if line_state != LINE_END:
                    self.turn(int(steering*0.75))
                    counter = counter - 1
                    if counter <= 0:
                        break

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_condition(self, steering, condition, then=STOP):
        count = 0

        await self.turn(steering)

        while True:
            if condition():
                count = count + 1
                if count == 3:
                    break
            await asleep_ms(10)

        await self.stop_then(then)

    ######################## Line following V2 (engine "line PID") #####################
    #  Port tu FastLine da tinh chinh. error chuan hoa ~[-2, 2]. follow_line_pid() la
    #  1 buoc PID (sync) dung chung cho cac khoi check point (qua _follow_step).

    # ---------------- cau hinh ----------------
    def line_pid(self, kp=None, ki=None, kd=None):
        # Keo khoi nay = bat che do PID cho cac khoi check point.
        self._line_use_pid = True
        if kp is not None:
            self._line_kp = kp
        if ki is not None:
            self._line_ki = ki
        if kd is not None:
            self._line_kd = kd

    def line_speed(self, speed, max_speed=100):
        self._line_base_speed = speed
        self._line_max_speed = max_speed

    def line_curve_gain(self, gain):
        # giam toc tien khi |error| lon (vao cua). 0 = khong giam; 1 = mat line -> xoay tai cho.
        self._line_curve_gain = _line_clamp(gain, 0, 1)

    def line_deadband(self, db):
        # |error| <= db -> di thang (chong giat khi bam thang). 0 = tat.
        self._line_deadband = db

    def line_turn_gain(self, gain, correction_limit=0.8):
        # gain < 1 -> khi bam line 2 banh luon tien (muot). gain lon -> be cua manh hon.
        self._line_turn_gain = gain
        self._line_corr_limit = correction_limit

    def line_stall_floor(self, v):
        # san bu ma sat (motor % toi thieu khi output khac 0). None = dung _min_speed.
        self._line_stall_floor = None if v is None else max(0, int(v))

    def line_ema_alpha(self, alpha):
        # he so lam muot IIR sai so dau vao: 0.5 = muot hon, 1.0 = tat IIR.
        self._line_ema_alpha = _line_clamp(float(alpha), 0.1, 1.0)

    def line_d_alpha(self, alpha):
        # loc khau D: thap (0.2) = D muot/yeu, cao (0.6-0.8) = D nhanh/manh.
        self._line_d_alpha = _line_clamp(float(alpha), 0.1, 1.0)

    def line_lost_fwd(self, ratio):
        # ty le toc tien giu lai khi mat line (arc recovery). 0 = xoay tai cho.
        self._line_lost_fwd = _line_clamp(float(ratio), 0.0, 1.0)

    def line_cross_debounce(self, n):
        # so khung CROSS lien tiep de xac nhan vach ngang that.
        self._line_cross_confirm = max(1, int(n))

    def line_invert(self, invert):
        self._line_invert = 1 if invert >= 0 else -1

    def line_mode(self, mode):
        # 'digital' (mac dinh, on dinh) hoac 'raw' (analog, can line_calibrate() + read_raw).
        self._line_use_pid = True
        self._line_mode = mode
        if mode == 'raw':
            s = self._line_sensor
            if not hasattr(s, 'read_raw'):
                print('line PID: cam bien nay khong co RAW -> dung digital.')
                self._line_mode = 'digital'
            elif not self._line_calibrated:
                print('line PID: che do raw chua calibrate -> tam dung digital. Goi line_calibrate() truoc.')

    async def line_calibrate(self, seconds=3, spin=65):
        # Hoc nguong cho che do 'raw': robot TU XOAY de quet 5 mat qua line + nen.
        # Chi dung duoc voi cam bien co read_raw (5 mat).
        s = self._line_sensor
        if not hasattr(s, 'read_raw'):
            print('line PID: cam bien khong ho tro RAW, bo qua calibrate.')
            return
        self._line_cal_min = [4095, 4095, 4095, 4095, 4095]
        self._line_cal_max = [0, 0, 0, 0, 0]
        on_total = 0
        on_count = 0
        off_total = 0
        off_count = 0
        duration = int(seconds * 1000)
        half = duration // 2
        start = ticks_ms()
        self.run_speed(spin, -spin)        # xoay tai cho
        flipped = False
        while ticks_diff(ticks_ms(), start) < duration:
            if not flipped and ticks_diff(ticks_ms(), start) > half:
                self.run_speed(-spin, spin)   # xoay nguoc lai de quet day du
                flipped = True
            raw = s.read_raw()
            dig = s.read()
            for k in range(5):
                v = raw[k]
                if v < self._line_cal_min[k]:
                    self._line_cal_min[k] = v
                if v > self._line_cal_max[k]:
                    self._line_cal_max[k] = v
                if dig[k]:
                    on_total += v
                    on_count += 1
                else:
                    off_total += v
                    off_count += 1
            await asyncio.sleep_ms(5)
        self.stop()
        # xac dinh chieu: tren-line cho raw cao hay thap (dua vao digital dang tin)
        if on_count > 0 and off_count > 0:
            self._line_high = (on_total / on_count) > (off_total / off_count)
        # kiem tra chat luong calib
        best_range = 0
        for k in range(5):
            r = self._line_cal_max[k] - self._line_cal_min[k]
            if r > best_range:
                best_range = r
        if best_range < _LINE_GOOD_RANGE:
            self._line_calibrated = False
            print('line PID CALIB KEM (range tot nhat %d < %d). Tam dung DIGITAL.' % (best_range, _LINE_GOOD_RANGE))
        else:
            self._line_calibrated = True
            print('line PID calib OK. min=%s max=%s line_high=%s' % (
                self._line_cal_min, self._line_cal_max, self._line_high))

    def line_debug(self, on):
        self._line_debug = bool(on)
        if self._line_debug:
            print('FASTLINE,t_ms,s0,s1,s2,s3,s4,error,P,I,D,correction,m1,m2')

    def line_debug_interval(self, ms):
        self._line_debug_interval = int(ms)

    def reset_line_pid(self):
        self._line_last_error = 0.0
        self._line_integral = 0.0
        self._line_lost_start = -1
        self._line_ema_err = 0.0
        self._line_d_err = 0.0

    # ---------------- doc gia tri ----------------
    def line_error(self):
        # loi line da chuan hoa ~[-2, 2] (0 = giua line)
        return self._line_read_error()

    def line_read(self, index=None):
        # 'raw' -> analog (read_raw); 'digital' -> 0/1 (read). index None = ca mang.
        s = self._line_sensor
        if self._line_mode == 'raw' and hasattr(s, 'read_raw'):
            return s.read_raw(index)
        return s.read(index)

    def _line_read_error(self):
        s = self._line_sensor
        if s is None:
            return 0.0
        if self._line_mode == 'raw' and self._line_calibrated and hasattr(s, 'read_raw'):
            return self._line_analog_error()
        # digital: get_error ~[-2000, 2000] cua cam bien -> /1000 -> [-2, 2]
        if hasattr(s, 'get_error'):
            return s.get_error() / 1000.0
        return 0.0

    def _line_analog_error(self):
        # centroid analog lien tuc tu read_raw + calib -> muot hon digital
        raw = self._line_sensor.read_raw()
        acc = 0.0
        tot = 0.0
        for k in range(5):
            rng = self._line_cal_max[k] - self._line_cal_min[k]
            if rng < _LINE_MIN_RANGE:
                n = 0.0            # mat nay tuong phan qua kem -> bo qua (nhieu)
            else:
                n = (raw[k] - self._line_cal_min[k]) / rng
                if not self._line_high:
                    n = 1.0 - n
                n = _line_clamp(n, 0.0, 1.0)
            acc += n * _LINE_WEIGHTS[k]
            tot += n
        if tot < self._line_lost_threshold:
            # mat line -> giu huong cu
            if self._line_last_aerr > 0:
                return 2.0
            elif self._line_last_aerr < 0:
                return -2.0
            return 0.0
        err = acc / tot
        self._line_last_aerr = err
        return err

    '''
        1 BUOC dieu khien PID bam line (khong block). Goi trong vong lap dieu khien.
        Yeu cau sensor.get_error() + get_pattern (LineSensorI2C / LineSensor5P_I2C).
        Tu dong giam toc vao cua, loc EMA/D, xoay khoa huong khi mat line. Tra ve error.
    '''
    def follow_line_pid(self, base=None):
        s = self._line_sensor
        if s is None:
            return 0.0
        if hasattr(s, 'update'):
            s.update()                      # refresh cache (centroid/checkpoint)

        base = self._line_base_speed if base is None else base

        # "Mat line" = tat ca mat tat (pattern==0). Dung pattern, KHONG dung error==2,
        # vi error==2 cung xay ra khi chi co 1 mat ria thay line (van dang bam).
        pattern = s.get_pattern() if hasattr(s, 'get_pattern') else 1
        lost = (self._line_mode != 'raw') and (pattern == 0)

        if lost:
            # === MAT LINE: xoay KHOA theo huong cuoi cung thay line ===
            if self._line_lost_start < 0:
                self._line_lost_start = ticks_ms()
            lost_ms = ticks_diff(ticks_ms(), self._line_lost_start)
            mag = 0.7 if lost_ms < 400 else 0.95
            correction = self._line_lost_dir * mag
            recovery_base = min(base, 50)
            fwd = recovery_base * self._line_lost_fwd
            turn = correction * recovery_base
            error = 2.0 * self._line_lost_dir
            p = i = d = 0.0
        else:
            # === DANG BAM LINE ===
            raw_err = self._line_read_error()       # ~[-2, 2], tu cam bien

            # IIR lam muot error de giam nhieu +/-0.5 khi di thang
            if abs(raw_err - self._line_ema_err) > 1.5:
                self._line_ema_err = raw_err   # snap: khong cho EMA cu keo sai
            else:
                self._line_ema_err = self._line_ema_alpha * raw_err + (1.0 - self._line_ema_alpha) * self._line_ema_err
            error = self._line_ema_err

            # cap nhat huong xoay tu RAW (khong qua IIR, phan ung ngay)
            if raw_err > 0.01:
                self._line_lost_dir = 1
                self._line_last_seen_error = raw_err
            elif raw_err < -0.01:
                self._line_lost_dir = -1
                self._line_last_seen_error = raw_err

            if self._line_lost_start >= 0:
                # vua bat lai line sau khi mat -> snap EMA + reset de khong giat
                self._line_ema_err = raw_err
                error = raw_err
                self._line_last_error = raw_err
                self._line_integral = 0.0
                self._line_d_err = 0.0
                self._line_lost_start = -1

            # deadband ap dung cho CA P, I, D: trong vung chet correction = 0 tuyet doi
            e = error
            if -self._line_deadband <= e <= self._line_deadband:
                e = 0.0

            # D-term tinh tren error DA qua deadband + loc thong thap (Filtered Derivative)
            d_raw = self._line_kd * (e - self._line_last_error)
            self._line_last_error = e
            self._line_d_err = self._line_d_alpha * d_raw + (1.0 - self._line_d_alpha) * self._line_d_err
            d = self._line_d_err

            self._line_integral = _line_clamp(self._line_integral + e, -_LINE_INTEGRAL_LIMIT, _LINE_INTEGRAL_LIMIT)
            p = self._line_kp * e
            i = self._line_ki * self._line_integral
            correction = _line_clamp((p + i + d) * self._line_invert,
                                     -self._line_corr_limit, self._line_corr_limit)

            # giam toc tien dua tren RAW error (phan ung ngay vao cua, khong cho IIR tre)
            ae = abs(raw_err)
            if ae > 2.0:
                ae = 2.0
            fwd = base * (1.0 - self._line_curve_gain * ae / 2.0)
            turn = correction * base * self._line_turn_gain

        left_raw = _line_clamp(fwd + turn, -self._line_max_speed, self._line_max_speed)
        right_raw = _line_clamp(fwd - turn, -self._line_max_speed, self._line_max_speed)

        # Bu ma sat (Stall/Deadband Compensation) cho dong co DC tren YoloUNO
        min_start = self._line_stall_floor if self._line_stall_floor is not None else self._min_speed
        ms = self._line_max_speed

        if abs(left_raw) < 1.0:
            left = 0.0
        elif left_raw > 0:
            left = min_start + (ms - min_start) * (left_raw / ms)
        else:
            left = -min_start + (ms - min_start) * (left_raw / ms)

        if abs(right_raw) < 1.0:
            right = 0.0
        elif right_raw > 0:
            right = min_start + (ms - min_start) * (right_raw / ms)
        else:
            right = -min_start + (ms - min_start) * (right_raw / ms)

        self.run_speed(left, right)

        if self._line_debug:
            self._line_print_dbg(error, p, i, d, correction, left, right)
        return error

    def _line_print_dbg(self, error, p, i, d, correction, m1, m2):
        now = ticks_ms()
        if ticks_diff(now, self._line_last_dbg) < self._line_debug_interval:
            return
        self._line_last_dbg = now
        s = self._line_sensor
        pat = s.get_pattern() if hasattr(s, 'get_pattern') else 0
        print('FASTLINE,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d' % (
            now, pat & 1, (pat >> 1) & 1, (pat >> 2) & 1, (pat >> 3) & 1, (pat >> 4) & 1,
            error, p, i, d, correction, int(m1), int(m2)))

    '''
        Xoay tai cho cho den khi mat giua (S3) bat lai duoc line.
            direction: -1 quay trai, +1 quay phai.
        Dung de xu ly cua gat 90, nhanh re, lai line sau giao diem.
    '''
    async def turn_until_line(self, direction, speed=None, max_ms=2500, then=None):
        s = self._line_sensor
        if s is None:
            return False

        sp = self._min_speed if speed is None else speed
        if direction < 0:
            self.run_speed(-sp, sp)     # pivot trai
        else:
            self.run_speed(sp, -sp)     # pivot phai

        t0 = ticks_ms()
        phase = 0                       # 0: roi line cu, 1: cho line moi vao giua
        found = False
        while ticks_ms() - t0 < max_ms:
            if hasattr(s, 'update'):
                s.update()
                pat = s.get_pattern()
            else:
                pat = 0
            center = pat & 0b00100      # S3
            if phase == 0:
                if not (pat & 0b01110):  # da roi khoi cum giua -> sang pha bat line
                    phase = 1
            else:
                if center:
                    found = True
                    break
            await asyncio.sleep_ms(5)

        self.reset_line_pid()
        await self.stop_then(then)
        return found

    '''
        Xu ly 1 checkpoint (FSM action). Mac dinh:
            CORNER trai/phai -> pivot lai line.
            CROSS / T        -> di thang qua vach.
            Y (nga re)       -> mac dinh giu trai (doi qua branch_policy).
            LOST / U_TURN    -> tim line ve phia thay lan cuoi, leo thang len quay dau.
        Tra ve dong bo, dung trong run_line_follow().
    '''
    async def handle_checkpoint(self, cp, base=None):
        s = self._line_sensor
        base = self._speed if base is None else base

        if cp == LINE_LEFT_CORNER:
            await self.turn_until_line(-1)
        elif cp == LINE_RIGHT_CORNER:
            await self.turn_until_line(1)
        elif cp == LINE_CROSS or cp == LINE_T:
            # vuot qua vach ngang roi bam tiep (di thang la mac dinh tai cross)
            await self.straight(self._min_speed, 0.12, unit=SECOND, then=None)
        elif cp == LINE_Y:
            await self.turn_until_line(-1)      # mac dinh re trai tai nga re
        elif cp == LINE_LOST or cp == LINE_U_TURN:
            # tim line: quay ve phia thay line lan cuoi
            d = 1 if self._line_last_error >= 0 else -1
            ok = await self.turn_until_line(d, max_ms=1500)
            if not ok:
                # khong thay -> quay nguoc lai (gan nhu quay dau)
                await self.turn_until_line(-d, max_ms=3000)
        self.reset_line_pid()

    '''
        Vong lap FSM bam line hoan chinh:
            FOLLOW_LINE  -> PID centroid
            detect checkpoint -> CHECKPOINT_HANDLER -> tim lai line -> FOLLOW_LINE
        on_event(cp): callback tuy chon (vd dem vach START/FINISH, dieu phoi route).
            - tra ve True  -> da tu xu ly, FSM bo qua handler mac dinh.
            - tra ve None/False -> dung handler mac dinh.
        lost_limit: so frame mat line truoc khi coi la LOST that su (DASH thi bo qua).
    '''
    async def run_line_follow(self, base=None, on_event=None, lost_limit=60):
        s = self._line_sensor
        if s is None or not hasattr(s, 'update'):
            # fallback: sensor cu khong ho tro V2 -> dung follow_line cu
            while self.mode_auto:
                await self.follow_line()
                await asyncio.sleep_ms(10)
            return

        self.reset_line_pid()
        while self.mode_auto:
            s.update()
            cp = s.detect_checkpoint()

            if cp == LINE_NORMAL:
                self.follow_line_pid(base)

            elif cp == LINE_LOST:
                if s.lost_frames() > lost_limit:
                    handled = on_event(LINE_LOST) if on_event else False
                    if not handled:
                        await self.handle_checkpoint(LINE_LOST, base)
                else:
                    # mat line ngan (duong dut / khe nho) -> giu PID di thang qua
                    self.follow_line_pid(base)

            else:
                handled = on_event(cp) if on_event else False
                if not handled:
                    await self.handle_checkpoint(cp, base)

            await asyncio.sleep_ms(5)

        self.stop()
