from micropython import const
from i2c_motors_driver import MotorDriver
from robotics_motor import *
from robotics_line_sensor import *
from utility import *
from yolo_uno import *

STOP_COAST = const(0)
STOP_BRAKE = const(1)

class DriveBase:
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        self._speed = 0
        self._turn_speed = 0
        self._m1 = m1
        self._m2 = m2
        self._m3 = m3
        self._m4 = m4

        self._line_sensor = None

        self._last_line_state = LINE_CENTER

    def set_line_sensor(self, sensor):
        self._line_sensor = sensor

    def speed(self, speed, turn_speed):    
        self._speed = speed
        self._turn_speed = turn_speed

    def run(self, m1_speed, m2_speed, m3_speed=None, m4_speed=None):
        self._m1.speed(m1_speed)
        self._m2.speed(m2_speed)
        if m3_speed:
            self._m3.speed(m3_speed)
        if m4_speed:
            self._m4.speed(m4_speed)

    '''
        Stops the robot by letting the motors spin freely.
    '''
    def stop(self):
        pass
    
    '''
        Stops the robot by given method.

        Parameters:
            then: STOP_COAST or STOP_BRAKE or None
    '''
    async def stop_then(self, then):
        if then == STOP_BRAKE:
            self.brake()
            await asleep_ms(200)
            self.stop()
        elif then == STOP_COAST:
            self.stop()
        else:
            return
    
    '''
        Stops the robot by passively braking the motors.
    '''
    def brake(self):
        pass

    async def turn(self, ratio=None, speed=None, time=None, then=STOP_COAST):
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
        pass

    async def forward(self, speed=None, time=None, then=STOP_COAST):
        pass
    
    async def backward(self, speed=None, time=None, then=STOP_COAST):
        pass
    
    async def turn_left(self, speed=None, time=None, then=STOP_COAST):
        pass

    async def turn_right(self, speed=None, time=None, then=STOP_COAST):
        pass

    async def follow_line(self, speed, backward=True, line_state=None):
        if self._line_sensor == None:
            return
        
        self.speed_factors = [ 25, 50, 100 ] # 1: light turn, 2: normal turn, 3: heavy turn
        ratio = 0

        if line_state == None:
            line_state = self._line_sensor.check()

        if line_state == LINE_END: #no line found
            if backward:
                self.backward(speed)
        else:
            if line_state == LINE_CENTER:
                if self._last_line_state == LINE_CENTER:
                    self.forward(speed) #if it is running straight before then robot should speed up now           
                else:
                    self.forward(speed * 2/3, speed * 2/3) #just turn before, shouldn't set high speed immediately, speed up slowly
            else:
                if line_state == LINE_RIGHT2:
                    ratio = -50 #left normal turn
                elif line_state == LINE_LEFT2:
                    ratio = 50 #right normal turn
                elif line_state == LINE_RIGHT:
                    ratio = 25 # left light turn
                elif line_state == LINE_LEFT:
                    ratio = -25 # right light turn
                elif line_state == LINE_RIGHT3:
                    ratio = -100 # left heavy turn
                elif line_state == LINE_LEFT3:
                    ratio = 100 # right heavy turn
                
                await self.turn(ratio, speed)
        
        self._last_line_state = line_state


    async def follow_line_until_end(self, speed, then=STOP_COAST):
        count = 3

        while True:
            line_state = self._line_sensor.check()

            if line_state == LINE_END:
                count = count - 1
                if count == 0:
                    break

            await self.follow_line(abs(speed), False, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def follow_line_until_cross(self, speed, then=STOP_COAST):
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

            await self.follow_line(abs(speed), True, line_state)

            time.sleep_ms(10)

        await self.forward(speed, 0.1)
        await self.stop_then(then)

    async def follow_line_until(self, speed, condition, then=STOP_COAST):
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

            self.follow_line(speed, True, line_state)

            await asleep_ms(10)

        await self.stop_then(then)

    async def turn_until_line_detected(self, ratio, then=STOP_COAST):
        counter = 0
        status = 0

        self.turn(ratio)

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

    def turn_until_condition(self, ratio, condition, then=STOP_COAST):
        count = 0

        self.turn(ratio)

        while True:
            if condition():
                count = count + 1
                if count == 3:
                    break
            await asleep_ms(10)

        await self.stop_then(then)


class DriveBase_2WD(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor):
        super().__init__(m1, m2, None, None)
    
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
    
    async def turn(self, ratio=None, time=None, then=STOP_COAST):
        if ratio > 0:
            self.m1.speed(self._speed)
            self.m2.speed(int(-2*(self._speed/100)*ratio + self._speed))
        elif ratio < 0:
            self.m2.speed(self._speed)
            self.m1.speed(int(-2*(self._speed/100)*ratio + self._speed))
        else:
            self.forward()


    async def forward(self, time=None, then=STOP_COAST):
        self.run(self._speed, self._speed, None, None)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def backward(self, time=None, then=STOP_COAST):
        self.run(-self._speed, -self._speed, None, None)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def turn_left(self, time=None, then=STOP_COAST):
        self.run(-self._turn_speed, self._turn_speed, None, None)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

    async def turn_right(self, time=None, then=STOP_COAST):
        self.run(self._turn_speed, -self._turn_speed, None, None)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

class DriveBase_4WD(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        super().__init__(m1, m2, m3, m4)
    
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

    async def forward(self, time=None, then=STOP_COAST):
        self.run(self._speed, self._speed, self._speed, self._speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def backward(self, time=None, then=STOP_COAST):
        self.run(-self._speed, -self._speed, -self._speed, -self._speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def turn_left(self, time=None, then=STOP_COAST):
        self.run(-self._turn_speed, self._turn_speed, -self._turn_speed, self._turn_speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

    async def turn_right(self, time=None, then=STOP_COAST):
        self.run(self._turn_speed, -self._turn_speed, self._turn_speed, -self._turn_speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

class DriveBase_Mecanum(DriveBase):
    def __init__(self, m1: DCMotor, m2: DCMotor, m3: DCMotor, m4: DCMotor):
        super().__init__(m1, m2, m3, m4)
    
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

    async def forward(self, time=None, then=STOP_COAST):
        self.run(self._speed, self._speed, self._speed, self._speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def backward(self, time=None, then=STOP_COAST):
        self.run(-self._speed, -self._speed, -self._speed, -self._speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)
    
    async def turn_left(self, time=None, then=STOP_COAST):
        self.run(-self._turn_speed, self._turn_speed, -self._turn_speed, self._turn_speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

    async def turn_right(self, time=None, then=STOP_COAST):
        self.run(self._turn_speed, -self._turn_speed, self._turn_speed, -self._turn_speed)
        if time:
            await asleep_ms(time)
            self.stop_then(then)

