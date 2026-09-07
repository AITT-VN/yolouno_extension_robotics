from time import ticks_ms, ticks_diff, ticks_us
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

# learned coasting distances and trim gains survive a power cycle here, so the
# first move of a program does not have to rediscover them
TUNING_FILE = '/robot_tuning.json'

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

        # line following: speeds default to speed()/min_speed(), see line_speed()
        self._line_cruise = None
        self._line_slow = None
        self._line_kp = 1.0
        self._line_ki = 0.0
        self._line_kd = 0.03
        self._line_slowdown = 1.2 # curve speed reached at |position| = 1/1.2
        self._line_sensor_offset = 0 # mm from the sensor to the axle, see line_sensor_offset()
        self._line_turn_offset_ms = 0 # same as a time, see line_turn_offset()
        self._line_lost_timeout = 3000 # ms searching for a lost line before giving up
        self._line_gap_ms = 250 # ms coasting straight when the line vanishes under the middle
        self._line_end_ms = 100 # ms without line (under the middle) that count as its end
        self._line_confirm = 2 # readings in a row to trust a crossing / condition
        self._line_invert = 1
        self._line_debug = False
        self._line_debug_interval = 100
        self._line_debug_ts = 0
        self._line_mark = None # encoder distance when the last crossing was seen
        self._line_reset()

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
        self._turn_offset_left = 0 # kept for old programs, see turn_offset()
        self._turn_offset_right = 0
        self._strafe_ratio = 1.0 # lateral mm moved per mm of wheel travel when strafing
        self._stall_timeout = 2000 # ms without encoder progress before a distance move gives up

        # precise moves: how close is close enough, how long to let the robot
        # settle after braking, and the braking distance learned from previous
        # moves (mm for straight/strafe, degrees for turn) so the next one can
        # stop early instead of overshooting
        self._distance_tolerance = 3 # mm
        self._angle_tolerance = 1 # degrees
        self._settle_time = 800 # ms, longest wait for the robot to stand still after a brake
        self._trim_nudges = 6 # at most this many trim pulses per move
        self._nudge_min = 30 # ms, shortest trim pulse
        self._nudge_max = 150 # ms, longest trim pulse: longer ones build up speed and coast unpredictably
        self._overshoot = {'straight': 0, 'strafe': 0, 'turn': 0}
        # trim pulse length per unit of error (ms per mm, ms per degree),
        # learned from what each pulse actually moved
        self._trim_gain = {'straight': 1.5, 'strafe': 1.5, 'turn': 5}
        self._tuning_saved = self._load_tuning() # {kind: [coast, min_speed it was learned at]}
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
        Kept for programs saved with the old "turn offset" block, which
        subtracted a fixed angle from every turn to make up for coasting.
        Turns are now trimmed to the target automatically, so the values are
        accepted and stored but no longer applied.
    '''
    def turn_offset(self, offset=None, right_offset=None):
        if offset is None:
            return (self._turn_offset_left, self._turn_offset_right)
        self._turn_offset_left = offset
        self._turn_offset_right = right_offset if right_offset is not None else offset
        if offset or right_offset:
            print('turn_offset is no longer needed: turns are trimmed to the target automatically')

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

    '''
        Forget the coasting distances and trim gains learned so far, in RAM
        and on flash. Use after changing wheels, motors or the robot's weight.
    '''
    def reset_tuning(self):
        for kind in self._overshoot:
            self._overshoot[kind] = 0
        self._trim_gain = {'straight': 1.5, 'strafe': 1.5, 'turn': 5}
        self._tuning_saved = {}
        try:
            import os
            os.remove(TUNING_FILE)
        except OSError:
            pass

    def _load_tuning(self):
        try:
            import json
            with open(TUNING_FILE) as f:
                data = json.load(f)
            for kind, gain in data.get('gain', {}).items():
                if kind in self._trim_gain:
                    self._trim_gain[kind] = gain
            return data.get('coast', {})
        except Exception:
            return {}

    def _save_tuning(self):
        try:
            import json
            coast = dict(self._tuning_saved)
            for kind, value in self._overshoot.items():
                if value > 0:
                    coast[kind] = [value, self._min_speed]
            with open(TUNING_FILE, 'w') as f:
                json.dump({'coast': coast, 'gain': self._trim_gain}, f)
        except Exception as e:
            print('tuning save failed:', e)

    '''
        Coasting distance to expect for this kind of move: what was learned in
        this session, else what was saved by an earlier one, scaled by the
        square of the min_speed ratio since coasting grows with speed squared.
    '''
    def _expected_coast(self, kind):
        if self._overshoot[kind] > 0:
            return self._overshoot[kind]
        saved = self._tuning_saved.get(kind)
        if saved:
            value, at_speed = saved
            if at_speed > 0 and self._min_speed != at_speed:
                value = value * (self._min_speed / at_speed) ** 2
            return value
        return 0

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
           gain learned from how far each pulse moved the robot in total, so
           it adapts to the robot's weight and floor. Pulses are kept short
           (150 ms) to stay in the range where movement is proportional to
           pulse length; a pulse that did not move the robot doubles the next.

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

        coast = self._expected_coast(kind)
        stop_at = target - min(coast, 0.5*target)
        still = tolerance / 3
        gain_before = self._trim_gain[kind]

        if self.debug:
            print('[drive_to] %s target=%.1f stop_at=%.1f expected coast=%.1f%s' % (kind, target, stop_at, coast,
                  '' if self._overshoot[kind] > 0 or coast == 0 else ' (from flash)'))

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
        last_pulse = 0
        last_moved = tolerance
        for _ in range(self._trim_nudges):
            error = measure() - target
            if abs(error) <= tolerance:
                break

            direction = -1 if error > 0 else 1
            pulse = max(self._nudge_min, min(self._nudge_max, abs(error) * gain))
            if last_moved < tolerance / 2:
                # the last pulse did not get the robot going: push harder
                pulse = min(self._nudge_max, max(pulse, 2 * last_pulse))
            before = measure()
            time_start = ticks_ms()
            while ticks_diff(ticks_ms(), time_start) < pulse:
                if (target - measure()) * direction <= 0:
                    break
                drive(direction * self._min_speed)
                await asyncio.sleep_ms(5)
            elapsed = ticks_diff(ticks_ms(), time_start)

            self.brake()
            after = await self._settle(measure, still)

            # learn from pulses that moved the robot a meaningful amount: a
            # short pulse is mostly motor start-up time and would inflate the
            # ms-per-unit gain
            moved = abs(after - before)
            if moved > 2 * tolerance:
                gain = max(0.2, min(50, 0.5*gain + 0.5*elapsed/moved))
                self._trim_gain[kind] = gain
            last_pulse = elapsed
            last_moved = moved

            if self.debug:
                print('[drive_to] trim %+d pulse %d ms: %.1f -> %.1f (error %.1f, gain %.2f ms/unit)' % (direction, elapsed, before, after, after - target, gain))

        # keep what was learned for the next program run, if it changed enough
        # to be worth a flash write
        if abs(self._overshoot[kind] - coast) > 0.05 * max(coast, 1) or abs(self._trim_gain[kind] - gain_before) > 0.05 * gain_before:
            self._save_tuning()

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
    '''
        Line following works on the line position reported by the sensor:
        -1 (line under the leftmost eye) .. +1 (rightmost), 0 = centred.

            steer = Kp*pos + Ki*integral(pos) + Kd*d(pos)/dt, clamped to -1..1
            left  = speed + steer*cruise
            right = speed - steer*cruise

        speed is the cruise speed on straights and drops towards the curve
        speed as the line moves away from the centre, so the robot is fast
        where the line is straight and careful where it bends. When the line
        disappears the robot pivots towards the side it was last seen on (a
        sharp corner); if it vanished from under the middle it first coasts
        straight for a moment (a gap or the end of the line), then backs up
        to where it vanished and pivots.

        Crossings, the end of the line and the turn-until-line search are
        detected on top of that, without changing how the robot steers.

        The 5-channel array is best used in analog mode (line_mode('analog')):
        the continuous position lets the controller react to small drifts
        early, instead of waiting for the next eye to light up.
    '''

    ######################## Configuration #####################

    '''
        Config line following speeds.

        Parameters:
             speed (Number, %) - cruise speed on straight line. Default: robot speed
             min_speed (Number, %) - speed in the tightest curves and when
                 searching for a lost line. Default: robot min_speed
    '''
    def line_speed(self, speed=None, min_speed=None, max_speed=None):
        if max_speed is not None: # develop-branch name for the cruise speed
            speed = max_speed
        if speed is not None:
            self._line_cruise = abs(speed)
        if min_speed is not None:
            self._line_slow = abs(min_speed)

    def _line_speeds(self):
        cruise = self._speed if self._line_cruise is None else self._line_cruise
        slow = self._min_speed if self._line_slow is None else self._line_slow
        return cruise, min(slow, cruise)

    '''
        Config the line following controller. Position is -1..1, steer is
        -1..1 (1 = inner wheel stopped at cruise speed).

        Parameters:
             Kp (Number) - steer per unit of position. 1.0: line under the
                 outer eye gives full steering
             Ki (Number) - steer per unit of position*second. Usually 0
             Kd (Number) - steer per unit of position/second. Damps the
                 swing back onto the line; 0.02..0.06 is typical
    '''
    def line_pid(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None:
            self._line_kp = Kp
        if Ki is not None:
            self._line_ki = Ki
        if Kd is not None:
            self._line_kd = Kd

    '''
        How much to slow down in curves: 0 = never slow down, 1 = curve
        speed when the line is under the outer eye, 2 = already at
        half way. Default 1.2.
    '''
    def line_slowdown(self, amount):
        self._line_slowdown = max(0, amount)

    '''
        'digital' or 'analog' (5-channel array only, see line_sensor.py).
    '''
    def line_mode(self, mode):
        s = self._line_sensor
        if mode in ('analog', 'raw'):
            if hasattr(s, 'mode'):
                s.mode('analog')
            else:
                print('line_mode: this sensor has no analog reading, using digital')
        elif hasattr(s, 'mode'):
            s.mode('digital')

    '''
        Distance from the sensor to the wheel axle (mm). When set,
        turn_until_line_detected() first drives forward so that the axle -
        the centre of the turn - ends up where the sensor saw the last
        crossing, and the robot turns on the junction itself.
        Needs encoder motors; else use line_turn_offset() with a time.
    '''
    def line_sensor_offset(self, mm):
        self._line_sensor_offset = max(0, mm)

    '''
        Same as line_sensor_offset() but as a time (seconds) of following
        the line at the curve speed, for robots without encoders.
    '''
    def line_turn_offset(self, seconds):
        self._line_turn_offset_ms = max(0, int(seconds * 1000))

    '''
        How long (ms) to search for a lost line before giving up, and how
        long (ms) to coast straight when the line vanishes from under the
        middle of the sensor before treating it as a corner and searching
        (dashed lines need a longer gap).
    '''
    def line_lost_timeout(self, ms, gap_ms=None):
        self._line_lost_timeout = max(100, int(ms))
        if gap_ms is not None:
            self._line_gap_ms = max(0, int(gap_ms))

    '''
        Prints one CSV line per interval while following:
        t_ms, pattern, position, steer, speed, left, right
    '''
    def line_debug(self, on, interval_ms=None):
        self._line_debug = bool(on)
        if interval_ms is not None:
            self._line_debug_interval = int(interval_ms)
        if self._line_debug:
            print('LINE,t_ms,pattern,pos,steer,speed,left,right')

    def line_invert(self, invert):
        # -1 if the robot steers away from the line: sensor mounted backwards
        self._line_invert = -1 if (invert is False or invert < 0) else 1

    '''
        Learns the analog calibration of the 5-channel array by spinning in
        place, half the time each way, so every eye sees the line and the
        background. Saved to flash. Start with the line under the sensor.
    '''
    async def line_calibrate(self, seconds=2):
        s = self._line_sensor
        if not hasattr(s, 'reset_calibration'):
            print('line_calibrate: only the 5-channel array has an analog calibration')
            return False
        s.mode('analog')
        s.reset_calibration()
        cruise, slow = self._line_speeds()
        duration = int(seconds * 1000)
        start = ticks_ms()
        flipped = False
        self.run_speed(slow, -slow)
        while ticks_diff(ticks_ms(), start) < duration:
            if not flipped and ticks_diff(ticks_ms(), start) > duration // 2:
                self.run_speed(-slow, slow)
                flipped = True
            s.update()
            await asyncio.sleep_ms(5)
        self.stop()
        if s.calibrated():
            s.save_calibration()
            print('line calibration ok: min=%s max=%s black_high=%s' % (s._cal_min, s._cal_max, s._line_high))
            return True
        print('line calibration poor: the sensor did not see enough black/white contrast')
        return False

    # develop-branch tuning names that no longer have an effect
    def line_curve_gain(self, gain):
        self.line_slowdown(gain * 2)

    def line_deadband(self, db):
        pass

    def line_turn_gain(self, gain, correction_limit=1.0):
        pass

    def line_d_alpha(self, alpha):
        pass

    def line_lost_fwd(self, ratio):
        pass

    def line_lost_grace(self, ms):
        self._line_gap_ms = max(0, int(ms))

    def line_accel(self, accel_per_s):
        pass

    def line_end_detect(self, confirm_ms=None, escape_mag=None, escape_trend=None, coast_ratio=None,
                        recover_hold_ms=None):
        if confirm_ms is not None:
            self._line_end_ms = int(confirm_ms)

    def line_debug_interval(self, ms):
        self._line_debug_interval = int(ms)

    def reset_line_pid(self):
        self._line_reset()

    def line_error(self):
        s = self._line_sensor
        if s is None:
            return 0.0
        pos = s.position()
        return 0.0 if pos is None else pos

    def line_read(self, index=None):
        return self._line_sensor.read(index)

    ######################## Control step #####################

    def _line_reset(self):
        # _line_mark is kept: it belongs to the last crossing seen, and the
        # next turn uses it
        self._line_last_pos = None # seeded by the first reading, see below
        self._line_d = 0.0
        self._line_integral = 0.0
        self._line_abs = 0.0
        self._line_speed_state = None # set to the curve speed on the first step
        self._line_last_us = None
        self._line_lost_since = None
        self._line_lost_ms = 0
        self._line_ok_ts = None # last time the robot held the line for a while
        self._line_hold_since = None # start of the current unbroken stretch on it
        self._line_lost_corner = False
        self._line_pivoting = False
        self._line_steer = 0.0
        self._line_seen = 0
        self._line_side = 0
        self._last_line_state = LINE_CENTER

    '''
        One line following step; call it every 5 ms or so. Never blocks.

        Returns:
            False once the line has been lost for longer than the lost
            timeout (motors stopped), True otherwise.
    '''
    def follow_line_step(self):
        s = self._line_sensor
        if s is None:
            return False

        pos = s.update()
        now = ticks_us()
        if self._line_last_us is None:
            dt = 0.01
        else:
            dt = ticks_diff(now, self._line_last_us) / 1000000
            if dt < 0.001:
                dt = 0.001
            elif dt > 0.05:
                dt = 0.05
        self._line_last_us = now

        cruise, slow = self._line_speeds()
        if self._line_speed_state is None:
            self._line_speed_state = slow

        searching = self._line_lost_since is not None
        if pos is not None and searching:
            # while searching, a line at an outer eye is not caught yet: the
            # opposite eye only brushes the end of the line we came from
            # (ignore it), the expected eye means keep pivoting at search
            # speed until the line reaches the inner eyes
            e = pos * self._line_invert
            if abs(e) > 0.6 and self._line_lost_ms < 400:
                # just after losing it, an edge eye is still brushing the line we
                # came from: keep searching. Later on an edge reading is the line
                # itself arriving, so it counts
                if (e > 0) == (self._line_side > 0) or self._line_side == 0:
                    self._line_side = 1 if e > 0 else -1
                pos = None
                self._line_lost_corner = True
            else:
                # a single frame is a flicker (low contrast, a speck): steer on it
                # but only call the line found again on the second frame in a
                # row, so flickers cannot keep resetting the give-up timer
                self._line_seen += 1
                if self._line_seen < 2:
                    self._line_last_pos = e
                    self._line_d = 0.0
                    steer = max(-1.0, min(1.0, self._line_kp * e))
                    turn = steer * slow
                    self.run_speed(slow + turn, slow - turn)
                    self._line_last_us = now
                    return True
        if pos is None:
            self._line_seen = 0

        if pos is None:
            # ---- line lost ----
            now_ms = ticks_ms()
            if self._line_lost_since is None:
                self._line_lost_since = now_ms
                # a sharp corner: the line left under an outer eye. Otherwise it
                # vanished from under the middle: a gap or the end of the line
                last = self._line_last_pos or 0.0
                self._line_lost_corner = abs(last) >= 0.5
                if self._line_lost_corner:
                    self._line_side = 1 if last > 0 else -1
                elif abs(self._line_steer) > 0.2:
                    # the robot was turning when the line went out of sight, so
                    # the sensor swung off it: the line is on the other side
                    self._line_side = -1 if self._line_steer > 0 else 1
                elif self._line_side == 0:
                    # nothing better known: search where it drifted last
                    self._line_side = 1 if last >= 0 else -1
            self._line_lost_ms = ticks_diff(now_ms, self._line_lost_since)
            self._line_hold_since = None
            if self._line_ok_ts is None:
                self._line_ok_ts = now_ms

            # give up on how long it has been since the robot last followed the
            # line, not since the last glimpse of it: while searching, single
            # frames catching the stub of the line we came from would otherwise
            # keep the search alive for ever
            if ticks_diff(now_ms, self._line_ok_ts) > self._line_lost_timeout:
                self.stop()
                if self._line_debug:
                    print('line lost')
                return False

            steer = 0
            if self._line_lost_corner or self._line_lost_ms >= 2 * self._line_gap_ms + 100:
                # sweep: pivot towards the side the line was last seen on, then
                # turn back a little further each time. A wrong guess costs one
                # short sweep instead of a full turn on the spot
                swept = self._line_lost_ms
                if not self._line_lost_corner:
                    swept -= 2 * self._line_gap_ms + 100
                side = self._line_side
                period = 350
                while swept >= period:
                    swept -= period
                    side = -side
                    period *= 2
                steer = side
                left = slow * steer
                right = -slow * steer
                self._line_pivoting = True
            elif self._line_lost_ms < self._line_gap_ms:
                # coast straight over a gap
                left = right = slow
            else:
                # no line after the gap: back up to where it vanished, so the
                # pivot that follows sweeps the sensor over a branch that was
                # already behind it (short sensor arm, 4-eye missing a corner)
                left = right = -slow
            self._line_speed_state = slow
            self._line_d = 0.0
            self._line_integral = 0.0
            speed = slow
        else:
            # ---- on the line ----
            e = pos * self._line_invert
            if self._line_last_pos is None:
                # first reading of this move: no derivative from a made-up past
                self._line_last_pos = e
            if self._line_lost_since is not None:
                # just found it again: no derivative kick, start gently
                self._line_lost_since = None
                self._line_last_pos = e
                self._line_d = 0.0
                self._line_speed_state = slow
                if self._line_pivoting:
                    # the robot is still turning from the search and would swing
                    # straight past the line: stop that rotation first
                    self._line_pivoting = False
                    self.run_speed(-slow * self._line_side, slow * self._line_side)
                    self._line_last_us = now
                    return True
            if abs(e) >= 0.5:
                self._line_side = 1 if e > 0 else -1
            elif abs(e) < 0.2:
                self._line_side = 0

            d_raw = (e - self._line_last_pos) / dt
            self._line_d += 0.5 * (d_raw - self._line_d)
            self._line_last_pos = e
            if self._line_ki:
                self._line_integral += e * dt
                lim = 0.5 / self._line_ki
                self._line_integral = max(-lim, min(lim, self._line_integral))
            else:
                self._line_integral = 0.0

            steer = self._line_kp * e + self._line_ki * self._line_integral + self._line_kd * self._line_d
            steer = max(-1.0, min(1.0, steer))
            self._line_steer = steer
            # "following" means holding the line for a stretch, not brushing it
            # for a frame: at the end of a line the robot keeps catching the
            # stub it came from, and that must not read as progress
            now_ms = ticks_ms()
            if self._line_hold_since is None:
                self._line_hold_since = now_ms
            elif ticks_diff(now_ms, self._line_hold_since) >= 300:
                self._line_ok_ts = now_ms

            # curve estimate: |pos| held for a moment, so the speed does not
            # jump back up between two eyes lighting up
            ae = abs(e)
            if ae > self._line_abs:
                self._line_abs = ae
            else:
                self._line_abs += (ae - self._line_abs) * min(1.0, dt / 0.15)
            target = cruise - (cruise - slow) * min(1.0, self._line_slowdown * self._line_abs)

            # slow down at once, speed up gradually
            if target <= self._line_speed_state:
                self._line_speed_state = target
            else:
                self._line_speed_state = min(target, self._line_speed_state + max(cruise - slow, 40) * dt / 0.25)
            speed = self._line_speed_state

            turn = steer * cruise
            left = speed + turn
            right = speed - turn
            # keep the difference between the wheels when one saturates
            if left > 100:
                right -= left - 100
                left = 100
            elif right > 100:
                left -= right - 100
                right = 100
            if left < -100:
                left = -100
            if right < -100:
                right = -100

        self.run_speed(left, right)

        if self._line_debug:
            now_ms = ticks_ms()
            if ticks_diff(now_ms, self._line_debug_ts) >= self._line_debug_interval:
                self._line_debug_ts = now_ms
                print('LINE,%d,%s,%s,%.2f,%d,%d,%d' % (now_ms, bin(s.pattern()), 'lost' if pos is None else ('%.2f' % pos),
                                                     steer, speed, left, right))
        return True

    # older names
    async def follow_line(self, backward=True, line_state=None):
        return self.follow_line_step()

    def follow_line_pid(self, base=None):
        return self.follow_line_step()

    async def run_line_follow(self, base=None, on_event=None, lost_limit=60):
        self._line_reset()
        while self.mode_auto:
            if not self.follow_line_step():
                break
            await asyncio.sleep_ms(5)
        self.stop()

    '''
        After a crossing or before a turn: bring the wheel axle to where
        the sensor is, if line_sensor_offset()/line_turn_offset() was set.
        Then stops as asked.
    '''
    async def _line_advance(self, then):
        cruise, slow = self._line_speeds()
        if self._line_sensor_offset > 0 and self.left_encoder and self.right_encoder:
            # the offset counts from where the sensor saw the last crossing,
            # not from where the robot came to a halt after it
            remaining = self._line_sensor_offset
            mark = self._line_mark
            self._line_mark = None
            if mark is not None and self.distance() >= mark:
                remaining -= self.distance() - mark
            if remaining > self._distance_tolerance:
                await self.straight(slow, remaining / 10, CM, then)
            else:
                await self.stop_then(then)
            return
        if self._line_turn_offset_ms > 0:
            start = ticks_ms()
            while ticks_diff(ticks_ms(), start) < self._line_turn_offset_ms:
                if not self.follow_line_step():
                    break
                await asyncio.sleep_ms(5)
        await self.stop_then(then)

    ######################## Follow until #####################

    '''
        Follows the line until a crossing line is under the sensor, then
        stops. A crossing is only accepted
        while the robot is centred on the line, so a sharp corner cutting
        across the eyes is not mistaken for one.

        Returns: True on a crossing, False if the line was lost
    '''
    async def follow_line_until_cross(self, then=STOP):
        s = self._line_sensor
        if s is None:
            return False
        self._line_reset()
        off = 0 # frames since we last saw a crossing (leave the one we start on)
        hits = 0
        first_hit = None
        n = s.n_sensors
        seen_at = [None] * n # when each eye last saw the line
        wide_at = None # when three or more eyes last saw it at once
        ok = True
        while True:
            ok = self.follow_line_step()
            if not ok:
                break
            # A bar reached at an angle - right after a curve - sweeps across
            # the array instead of lighting it all at once: first one outer
            # eye, a few frames later the other. Judge it over a short window:
            # both outer eyes lit within 100 ms, and three eyes at once at
            # some point in it. A single line never spans the array that way.
            now_ms = ticks_ms()
            pat = s.pattern()
            for i in range(n):
                if pat & (1 << i):
                    seen_at[i] = now_ms
            if s.count() >= 3:
                wide_at = now_ms
            bar = s.cross() or (
                seen_at[0] is not None and seen_at[n - 1] is not None and wide_at is not None
                and ticks_diff(now_ms, seen_at[0]) <= 100 and ticks_diff(now_ms, seen_at[n - 1]) <= 100
                and ticks_diff(now_ms, wide_at) <= 100)
            if bar:
                if off >= 3:
                    if first_hit is None or ticks_diff(now_ms, first_hit) > 150:
                        first_hit = now_ms
                        hits = 0
                    hits += 1
                    # slow down at once so the stop lands close to the bar, and
                    # remember where it was for the sensor offset move
                    self._line_speed_state = self._line_speeds()[1]
                    if self._line_mark is None:
                        self._line_mark = self.distance()
                    # confirmed by a second sighting within the same 150 ms; the
                    # frames in between may miss it (weak eyes flicker over black)
                    if hits >= self._line_confirm:
                        break
            elif off < 100:
                off += 1
            await asyncio.sleep_ms(5)
        await self.stop_then(then)
        return ok

    '''
        Follows the line until it ends: no eye sees it for a moment while
        it was under the middle of the sensor just before. Losing it under
        an outer eye is a corner instead, and the robot turns to find it.

        Returns: True at the end of the line, False if it was lost in a
        corner and not found again
    '''
    async def follow_line_until_end(self, then=STOP):
        s = self._line_sensor
        if s is None:
            return False
        self._line_reset()
        self._line_ok_ts = ticks_ms()
        while True:
            if not self.follow_line_step():
                break # searched for the line long enough: it has ended
            if s.lost() and not self._line_lost_corner and self._line_lost_ms >= self._line_end_ms:
                break
            if ticks_diff(ticks_ms(), self._line_ok_ts) > max(self._line_end_ms, 1200):
                break # only brushing the line since a while: this is its end
            await asyncio.sleep_ms(5)
        await self.stop_then(then)
        return True

    async def follow_line_by_time(self, timerun, then=STOP):
        if self._line_sensor is None:
            return False
        self._line_reset()
        start_time = ticks_ms()
        duration = timerun * 1000 # convert to ms
        ok = True
        while ticks_diff(ticks_ms(), start_time) < duration:
            ok = self.follow_line_step()
            if not ok:
                break
            await asyncio.sleep_ms(5)
        await self.stop_then(then)
        return ok

    '''
        Follows the line until condition() has been true for a few
        readings in a row.
    '''
    async def follow_line_until(self, condition, then=STOP):
        if self._line_sensor is None:
            return False
        self._line_reset()
        count = 0
        ok = True
        while True:
            ok = self.follow_line_step()
            if not ok:
                break
            if condition():
                count += 1
                if count >= self._line_confirm:
                    break
            else:
                count = 0
            await asyncio.sleep_ms(5)
        await self.stop_then(then)
        return ok

    ######################## Turning onto a line #####################

    '''
        Turns until the line is centred under the sensor, so the robot is
        ready to follow it. Turns fast with the angle sensor (if enabled)
        for the first part, then slowly while looking for the line, and
        trims until the line is centred.

        Parameters:
            steering (Number) - > 0 turn right, < 0 turn left; +-100 pivots
                in place, smaller values arc

        Returns: True when the line was found, False on timeout
    '''
    async def turn_until_line_detected(self, steering, then=STOP):
        s = self._line_sensor
        if s is None or steering == 0:
            return False
        self._line_reset()
        await self._line_advance(None)

        cruise, slow = self._line_speeds()
        sign = 1 if steering > 0 else -1
        use_gyro = self._use_gyro and self._angle_sensor is not None and abs(steering) >= 100
        if use_gyro:
            await self.reset_angle()

        def pivot(v):
            l, r = self._calc_steering(v, steering)
            self.run_speed(l, r)

        # the line sweeps in from the side the robot turns towards; brake as soon
        # as it reaches the inner eye on that side, the rest of the way is
        # covered by the braking itself. Following takes it from there
        centre = 0.3
        entry = sign
        def dbg(what, pos):
            if self._line_debug:
                print('TURN,%d,%s,%s,%s' % (ticks_diff(ticks_ms(), start), what, bin(s.pattern()),
                                           'lost' if pos is None else ('%.2f' % pos)))

        pivot(cruise if use_gyro else slow)
        start = ticks_ms()
        left_at = None
        # the line we started from stays within reach of the sensor for the
        # first few degrees (a crossing bar runs right past it) and can come
        # back into view after a short blank; the next line is the one that
        # shows up after the sensor has seen nothing for a while. With the
        # gyro the first 40 degrees are turned blind, which settles that
        blank_needed = 0 if use_gyro else 120
        blank_since = None
        found = False
        dbg('start', s.update())
        while ticks_diff(ticks_ms(), start) < self._line_lost_timeout * 3:
            if use_gyro:
                turned = abs(self._angle_sensor.angle)
                if turned < 40:
                    # still on or next to the line we started from: keep going
                    await asyncio.sleep_ms(5)
                    continue
                if turned >= 60:
                    pivot(slow)
                    use_gyro = False
            pos = s.update()
            now_ms = ticks_ms()
            if pos is None:
                if blank_since is None:
                    blank_since = now_ms
            if left_at is None:
                if pos is None or abs(pos) > 0.6 or ticks_diff(now_ms, start) > 600:
                    left_at = now_ms
                    dbg('left the line', pos)
            elif pos is not None:
                pivot(slow)
                blank = blank_since is not None and ticks_diff(now_ms, blank_since) >= blank_needed
                if pos * entry <= 0.5 and (blank or blank_needed == 0):
                    found = True
                    dbg('found', pos)
                    break
            if pos is not None and (blank_since is None or ticks_diff(now_ms, blank_since) < blank_needed):
                blank_since = None # too short a blank: still the starting line
            await asyncio.sleep_ms(5)

        self.brake()
        if found:
            # let it settle, then nudge back if the brake overshot: short pulses,
            # shorter every time, so they cannot overshoot again
            # pulses at the straight-line speed, longer each time the robot did
            # not budge: from standstill the shortest pulse often does nothing
            # against the static friction of the drive
            pulse = 40
            last = None
            for _ in range(4):
                await asyncio.sleep_ms(120)
                pos = s.update()
                dbg('settled', pos)
                if pos is None or abs(pos) <= centre:
                    break
                if last is not None and abs(pos - last) < 0.1:
                    pulse *= 2
                last = pos
                v = cruise if pos > 0 else -cruise # line on the right: turn right
                self.run_speed(v, -v)
                await asyncio.sleep_ms(pulse)
                self.brake()
        else:
            dbg('timeout', s.update())
        await self.stop_then(then)
        return found

    # develop-branch name: direction -1 left, +1 right
    async def turn_until_line(self, direction, speed=None, max_ms=2500, then=None):
        return await self.turn_until_line_detected(100 if direction > 0 else -100, then)

    async def turn_until_condition(self, steering, condition, then=STOP):
        count = 0
        await self.turn(steering)
        while True:
            if condition():
                count += 1
                if count >= self._line_confirm:
                    break
            else:
                count = 0
            await asyncio.sleep_ms(10)
        await self.stop_then(then)
