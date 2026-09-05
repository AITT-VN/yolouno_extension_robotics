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
        self._strafe_ratio = 1.0 # lateral mm moved per mm of wheel travel when strafing
        self._stall_timeout = 2000 # ms without encoder progress before a distance move gives up

        # precise moves: how close is close enough, how long to let the robot
        # settle after braking, and the braking distance learned from previous
        # moves (mm for straight/strafe, degrees for turn) so the next one can
        # stop early instead of overshooting
        self._distance_tolerance = 3 # mm
        self._angle_tolerance = 1 # degrees
        self._settle_time = 800 # ms, longest wait for the robot to stand still after a brake
        self._trim_nudges = 4 # at most this many trim pulses per move
        self._nudge_min = 30 # ms, shortest trim pulse
        self._nudge_max = 300 # ms, longest trim pulse
        self._overshoot = {'straight': 0, 'strafe': 0, 'turn': 0}
        # trim pulse length per unit of error (ms per mm, ms per degree),
        # learned from what each pulse actually moved
        self._trim_gain = {'straight': 1.5, 'strafe': 1.5, 'turn': 5}
        self.debug = False

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

    '''
        Config how far the robot actually moves sideways per unit of wheel
        travel (mecanum only). Measure it: strafe 100 cm with the default
        ratio, divide the distance really covered by 100 and pass it here.

        Parameters:
             ratio (Number) - lateral distance / wheel travel, 0 < ratio <= 1
    '''
    def strafe_ratio(self, ratio):
        if ratio <= 0 or ratio > 1.5:
            raise Exception("Invalid strafe ratio")
        self._strafe_ratio = ratio

    '''
        Config how precisely distance and angle moves have to end.

        Parameters:
             distance (Number, mm) - accepted error at the end of straight/side moves
             angle (Number, deg) - accepted error at the end of turns
    '''
    def tolerance(self, distance=3, angle=1):
        if distance <= 0 or angle <= 0:
            raise Exception("Invalid tolerance")
        self._distance_tolerance = distance
        self._angle_tolerance = angle

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

        await self.strafe(-self._speed, amount, unit, then)

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

        await self.strafe(self._speed, amount, unit, then)

    '''
        Moves sideways (mecanum only) for a given amount and then stops.

        Distance is measured with the encoder motors, the same way straight()
        does: every wheel turns by the same amount when strafing, so the
        average wheel travel is the lateral travel, scaled by strafe_ratio()
        to account for roller slip. If an angle sensor is attached the robot
        holds its heading with the same PID used by straight(); the encoders
        cannot see a rotation while strafing (both sides speed up together),
        so without a gyro the strafe runs open loop.

        Parameters:
            speed (Number, %) - Speed to travel, > 0 right, < 0 left

            amount (Number, cm or inch or seconds) - Amount to travel

            unit - can be CM, INCH, or SECOND

            then (STOP | BRAKE) - What to do after coming to a standstill.
    '''
    async def strafe(self, speed, amount, unit=SECOND, then=STOP):
        if self._drive_mode != MODE_MECANUM or speed == 0:
            return

        await self.reset_angle()
        self._pid.reset()

        side = 1 if speed > 0 else -1 # 1: right, -1: left
        max_speed = abs(speed)

        def drive(v):
            # hold heading with the gyro, if any; the encoders cannot see a
            # rotation while strafing
            correction = 0
            if self._angle_sensor != None:
                correction = self._pid(self._angle_sensor.heading)
            self._run_mecanum(0, side*v, correction)

        if unit == SECOND:
            distance = abs(int(amount*1000)) # to ms
            time_start = ticks_ms()
            driven = 0
            last_driven = 0
            while True:
                driven = ticks_diff(ticks_ms(), time_start)
                if driven >= distance:
                    break
                if amount < 2:
                    expected_speed = max_speed
                else:
                    expected_speed = self._calc_speed(max_speed, distance, driven, last_driven)
                drive(expected_speed)
                last_driven = driven
                await asyncio.sleep_ms(5)
            await self.stop_then(then)
            return

        if unit == CM:
            distance = abs(amount*10 / self._strafe_ratio) # to mm of wheel travel
        elif unit == INCH:
            distance = abs(amount*25.4 / self._strafe_ratio) # to mm of wheel travel
        else:
            return

        await self._drive_to(distance, max_speed, lambda: abs(self.distance()), drive,
                             self._distance_tolerance / self._strafe_ratio, 'strafe')
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
        if speed == 0:
            return

        await self.reset_angle()
        self._pid.reset()

        speed_dir = 1 if speed > 0 else -1 # direction
        max_speed = abs(speed)

        def drive(v):
            # adjust left and right speed to go straight
            left_speed, right_speed = self._calib_speed(speed_dir*v)
            self.run_speed(left_speed, right_speed)

        if unit == SECOND:
            distance = abs(amount*1000) # to ms
            time_start = ticks_ms()
            driven = 0
            last_driven = 0
            while True:
                driven = ticks_diff(ticks_ms(), time_start)
                if driven >= distance:
                    break
                if amount < 2:
                    expected_speed = max_speed
                else:
                    # speed smoothing using accel and deccel technique when distance is long enough
                    expected_speed = self._calc_speed(max_speed, distance, driven, last_driven)
                drive(expected_speed)
                last_driven = driven
                await asyncio.sleep_ms(5)
            await self.stop_then(then)
            return

        if unit == CM:
            distance = abs(amount*10) # to mm
        elif unit == INCH:
            distance = abs(amount*25.4) # to mm
        else:
            return

        await self._drive_to(distance, max_speed, lambda: abs(self.distance()), drive,
                             self._distance_tolerance, 'straight')
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

        def drive(v):
            left_speed, right_speed = self._calc_steering(v, steering)
            self.run_speed(left_speed, right_speed)

        if unit == SECOND:
            distance = abs(amount*1000) # to ms
            time_start = ticks_ms()
            driven = 0
            last_driven = 0
            while True:
                driven = ticks_diff(ticks_ms(), time_start)
                if driven >= distance:
                    break
                if amount < 1:
                    expected_speed = speed
                else:
                    expected_speed = self._calc_speed(speed, distance, driven, last_driven)
                drive(expected_speed)
                last_driven = driven
                await asyncio.sleep_ms(5)
            await self.stop_then(then)
            return

        if unit != DEGREE:
            return

        use_gyro = self._use_gyro
        if use_gyro and self._angle_sensor == None:
            print('turn: no angle sensor, using encoders')
            use_gyro = False

        if use_gyro:
            # unwrapped angle since reset_angle(), so turns past 180 work
            measure = lambda: abs(self._angle_sensor.angle)
        else:
            # both wheels travel the same arc when turning in place:
            # arc = pi * width * angle / 360, so angle = travel * 360 / (pi * width)
            measure = lambda: abs(self.distance()) * 360 / (math.pi * self._width)

        await self.reset_angle()
        await self._drive_to(abs(amount), speed, measure, drive, self._angle_tolerance, 'turn')
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

    '''
        Mecanum mixing. Same sign conventions as _mecanum_speed_factor:
        forward > 0 drives ahead, side > 0 strafes right, rotate > 0 turns
        right (clockwise), so forward=1 gives DIR_FW, side=1 gives DIR_SR and
        rotate=1 gives DIR_R.

        Parameters:
            forward, side, rotate (Number, %) - each from -100 to 100
    '''
    def _run_mecanum(self, forward, side, rotate):
        m1 = forward + side + rotate
        m2 = forward - side - rotate
        m3 = forward - side + rotate
        m4 = forward + side - rotate

        # scale down instead of clipping so the mix keeps its direction
        biggest = max(abs(m1), abs(m2), abs(m3), abs(m4), 100)
        scale = 100 / biggest

        self.m1.run(m1*scale*self._speed_ratio[0])
        self.m2.run(m2*scale*self._speed_ratio[1])
        self.m3.run(m3*scale*self._speed_ratio[0])
        self.m4.run(m4*scale*self._speed_ratio[1])


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
    '''
        Trapezoid speed profile over a distance move: ramp up over the first
        30%, cruise, ramp down over the last 30% to min_speed. Unlike
        _calc_speed it is applied to every move, however short - short moves
        are exactly where a full-speed stop overshoots the most.
    '''
    def _profile_speed(self, max_speed, target, driven):
        low = self._min_speed
        if max_speed <= low or target <= 0:
            return max_speed

        accel_end = 0.3*target
        decel_start = 0.7*target

        if driven < accel_end:
            v = low + (max_speed - low) * driven / accel_end
        elif driven > decel_start:
            v = max_speed - (max_speed - low) * (driven - decel_start) / (target - decel_start)
        else:
            v = max_speed

        return max(low, min(max_speed, v))

    '''
        Waits until the robot has actually stopped moving after a brake:
        measure() has to stay put (within still) between two readings 50 ms
        apart. Capped at _settle_time.

        Returns: the settled measure() value
    '''
    async def _settle(self, measure, still):
        time_start = ticks_ms()
        last = measure()
        while True:
            await asyncio.sleep_ms(50)
            now = measure()
            if abs(now - last) <= still or ticks_diff(ticks_ms(), time_start) > self._settle_time:
                return now
            last = now

    '''
        Drives until measure() reaches target, then trims the result.

        1. Ramp along _profile_speed and stop early by the coasting distance
           learned from previous moves of this kind.
        2. Brake, wait until the robot stands still, remember how far it
           coasted.
        3. If still outside tolerance, pulse back or forth at min_speed. The
           pulse length is proportional to the error, with a ms-per-unit
           gain learned from what each pulse really moved, so it adapts to
           the robot's weight and floor.

        Parameters:
            target (Number) - distance (mm) or angle (deg) to reach
            max_speed (Number, %) - cruise speed
            measure () -> Number - progress towards target, >= 0, same unit as target
            drive (Number) -> None - runs the motors, > 0 towards target, < 0 back
            tolerance (Number) - accepted final error, same unit as target
            kind (str) - key into the learned coasting / trim gain tables
    '''
    async def _drive_to(self, target, max_speed, measure, drive, tolerance, kind):
        if target <= 0:
            return

        coast = self._overshoot[kind]
        stop_at = target - min(coast, 0.5*target)
        still = tolerance / 3

        if self.debug:
            print('[drive_to] %s target=%.1f stop_at=%.1f learned coast=%.1f' % (kind, target, stop_at, coast))

        driven = 0
        last_driven = 0
        last_progress = ticks_ms()

        while True:
            driven = measure()
            # encoders not counting (no motor power, wheel blocked, motor not
            # on an E port, angle sensor task not started): give up instead of
            # spinning forever
            if driven != last_driven:
                last_progress = ticks_ms()
            elif ticks_diff(ticks_ms(), last_progress) > self._stall_timeout:
                print('move: no progress, stopping')
                return

            if driven >= stop_at:
                break

            drive(self._profile_speed(max_speed, target, driven))
            last_driven = driven
            await asyncio.sleep_ms(5)

        self.brake()
        settled = await self._settle(measure, still)

        # remember how far we coasted after the stop command, for the next
        # move: take the first measurement as is, then average
        coasted = max(0, settled - stop_at)
        self._overshoot[kind] = coasted if coast == 0 else 0.5*coast + 0.5*coasted

        if self.debug:
            print('[drive_to] braked at %.1f, settled at %.1f, coasted %.1f' % (driven, settled, coasted))

        # trim
        gain = self._trim_gain[kind]
        for _ in range(self._trim_nudges):
            error = measure() - target
            if abs(error) <= tolerance:
                break

            direction = -1 if error > 0 else 1
            pulse = max(self._nudge_min, min(self._nudge_max, abs(error) * gain))
            before = measure()
            time_start = ticks_ms()
            while ticks_diff(ticks_ms(), time_start) < pulse:
                if (target - measure()) * direction <= 0:
                    break
                drive(direction * self._min_speed)
                await asyncio.sleep_ms(5)

            self.brake()
            after = await self._settle(measure, still)

            moved = abs(after - before)
            if moved > tolerance / 2:
                gain = max(0.2, min(50, 0.5*gain + 0.5*pulse/moved))
                self._trim_gain[kind] = gain

            if self.debug:
                print('[drive_to] trim %+d pulse %d ms: %.1f -> %.1f (error %.1f, gain %.2f ms/unit)' % (direction, pulse, before, after, after - target, gain))

        if self.debug:
            print('[drive_to] done: %.1f / %.1f' % (measure(), target))

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
                diff_ticks = abs(left_ticks) - abs(right_ticks)
            else:
                diff_ticks = abs(right_ticks) - abs(left_ticks)

            # ticks -> mm of extra travel on one side -> degrees the robot has
            # yawed, so the PID sees the same unit whether it runs on encoders
            # or on the gyro (raw ticks saturated the +-10 output at 2 ticks)
            ticks_per_rev = self._ticks_per_rev
            if ticks_per_rev <= 0:
                ticks_per_rev = self.left_encoder.ticks_per_rev if self.left_encoder else 0
            if ticks_per_rev <= 0:
                return (speed, speed)
            diff_mm = diff_ticks * self._wheel_circ / ticks_per_rev
            angle_error = math.degrees(diff_mm / self._width)

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
                self.run(DIR_BW, self._min_speed) # back up slowly to find the line again
        else:
            if line_state == LINE_CENTER:
                if self._last_line_state == LINE_CENTER:
                    self.forward() #if it is running straight before then robot should speed up now
                else:
                    self.run(DIR_FW, self._min_speed) #just turn before, shouldn't set high speed immediately, speed up slowly

            elif line_state == LINE_CROSS:
                self.run(DIR_FW, self._min_speed) # cross line found, slow down

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

    async def follow_line_until_end(self, then=STOP):
        count = 2

        while True:
            line_state = self._line_sensor.check()

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

            if status == 1:
                if line_state != LINE_CROSS:
                    status = 2
            elif status == 2:
                if line_state == LINE_CROSS:
                    count = count + 1
                    if count == 2:
                        break

            await self.follow_line(True, line_state)

            if status == 2 and count == 1:
                await asleep_ms(20)
            else:
                await asleep_ms(10)

        #await self.forward_for(0.1, unit=SECOND) # to pass cross line a bit
        await self.stop_then(then)

    async def follow_line_by_time(self, timerun, then=STOP):
        start_time = ticks_ms()
        duration = timerun * 1000 # convert to ms

        while ticks_diff(ticks_ms(), start_time) < duration:
            await self.follow_line(True)
            await asleep_ms(10)

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

    async def turn_until_line_detected(self, steering, then=STOP):
        counter = 3 # consecutive readings on the line before stopping
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
                    await self.turn(int(steering*0.75)) # slow the turn down while confirming
                    counter = counter - 1
                    if counter <= 0:
                        break
                else:
                    counter = 3

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
