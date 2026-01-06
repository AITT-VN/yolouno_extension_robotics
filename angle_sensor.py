# Asynchronous sensor fusion for micropython targets.
# Ported to MicroPython by Peter Hinch, May 2017.
# Released under the MIT License (MIT) See LICENSE
# Copyright (c) 2017-2020 Peter Hinch

import asyncio, time
from math import sqrt, atan2, asin, degrees, radians

class DeltaT():
    def __init__(self, timediff):
        if timediff is None:
            self.expect_ts = False
            self.timediff = lambda start, end : time.ticks_diff(start, end)/1000000
        else:
            self.expect_ts = True
            self.timediff = timediff
        self.start_time = None

    def __call__(self, ts):
        if self.expect_ts:
            if ts is None:
                raise ValueError('Timestamp expected but not supplied.')
        else:
            ts = time.ticks_us()
        # ts is now valid
        if self.start_time is None:  # 1st call: self.start_time is invalid
            self.start_time = ts
            return 0.0001  # 100μs notional delay. 1st reading is invalid in any case

        dt = self.timediff(ts, self.start_time)
        self.start_time = ts
        return dt


class AngleSensor(object):
    '''
    Class provides sensor fusion allowing heading, pitch and roll to be extracted. This uses the Madgwick algorithm.
    The update method runs as a coroutine. Its calculations take 1.6mS on the Pyboard.
    '''
    declination = 0                         # Optional offset for true north. A +ve value adds to heading
    def __init__(self, imu, timediff=None):
        self.imu = imu
        self.magbias = (0, 0, 0)            # local magnetic bias factors: set from calibration
        self.gyro_bias = (0, 0, 0)          # local gyroscope bias factors: set from calibration
        self.expect_ts = timediff is not None
        self.deltat = DeltaT(timediff)      # Time between updates
        self.q = [1.0, 0.0, 0.0, 0.0]       # vector to hold quaternion
        GyroMeasError = radians(40)         # Original code indicates this leads to a 2 sec response time
        self.beta = sqrt(3.0 / 4.0) * GyroMeasError  # compute beta (see README)
        self.pitch = 0
        self.heading = 0
        self.roll = 0
        self._flag_reset = True

    async def reset(self):
        self._flag_reset = True
        while self._flag_reset != False:
            await asyncio.sleep_ms(50)

    def calibrate_mag(self, stopfunc):
        res = self.read_imu()
        mag = res[2]
        magmax = list(mag)                  # Initialise max and min lists with current values
        magmin = magmax[:]
        while not stopfunc():
            res = self.read_imu()
            magxyz = res[2]
            for x in range(3):
                magmax[x] = max(magmax[x], magxyz[x])
                magmin[x] = min(magmin[x], magxyz[x])
        self.magbias = tuple(map(lambda a, b: (a +b)/2, magmin, magmax))

    def calibrate(self, samples=500):
        print('Calibrating angle sensor...')
        res = self.read_imu()
        calib_mag = True

        if len(res) == 2 or (self.expect_ts and len(res) == 3):
            calib_mag = False
        else:
            mag = res[2]
            magmax = list(mag)   # Initialise max and min lists with current values
            magmin = magmax[:]

        gyro_bias_sum = [0, 0, 0]    
        
        for i in range(samples):
            res = self.read_imu()
            # calib gyroscope
            gyroxyz = res[1]
            for x in range(3):
                gyro_bias_sum[x] += gyroxyz[x]

            if calib_mag:
                # also calib mag
                magxyz = res[2]
                for x in range(3):
                    magmax[x] = max(magmax[x], magxyz[x])
                    magmin[x] = min(magmin[x], magxyz[x])
                    
            await asyncio.sleep_ms(5)
    
        self.gyro_bias = (gyro_bias_sum[0]/samples, gyro_bias_sum[1]/samples, gyro_bias_sum[2]/samples)
        print('Calibration and reset done')
        #print('Gyro offset: ', self.gyro_bias)

        if calib_mag:
            self.magbias = tuple(map(lambda a, b: (a +b)/2, magmin, magmax))
            print('Magnetometer bias: ', self.magbias)
        
        # reset everything
        self.pitch = 0
        self.heading = 0
        self.roll = 0
        self.angle_speed = 0.0
        self._flag_reset = True
        
    
    async def run(self, calib_samples=0):
        if calib_samples > 0:
            self.calibrate(calib_samples)

        data = self.read_imu()
        update_task = None
        if len(data) == 2 or (self.expect_ts and len(data) == 3):
            update_task = self._update_nomag
            #asyncio.create_task(self._update_nomag(slow_platform))
        else:
            update_task = self._update_mag
            #asyncio.create_task(self._update_mag(slow_platform))
        
        while True:
            if self._flag_reset:
                self.q = [1.0, 0.0, 0.0, 0.0]
                self.pitch = 0
                self.heading = 0
                self.roll = 0
                self._flag_reset = False
                await asyncio.sleep_ms(50)

            update_task()
            await asyncio.sleep_ms(5)

    def read_imu(self):
        imu_data = self.imu.sensors
        if len(imu_data) == 2 or (self.expect_ts and len(imu_data) == 3):
            return imu_data[0].xyz, imu_data[1].xyz # accelerometer, gyro
        else:
            return imu_data[0].xyz, imu_data[1].xyz, imu_data[2].xyz # accelerometer, gyro, magnetometer
    
    def _update_nomag(self):
        if self.expect_ts:
            accel, gyro, ts = self.read_imu()
        else:
            accel, gyro = self.read_imu()
            ts = None
        ax, ay, az = accel                  # Units G (but later normalised)
        gyro_calib = (gyro[0] - self.gyro_bias[0], gyro[1] - self.gyro_bias[1], gyro[2] - self.gyro_bias[2])
        gx, gy, gz = (radians(x) for x in gyro_calib) # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        
        self.angle_speed = gz
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _4q1 = 4 * q1
        _4q2 = 4 * q2
        _4q3 = 4 * q3
        _8q2 = 8 * q2
        _8q3 = 8 * q3
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm        # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Gradient decent algorithm corrective step
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4 * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4 * q2q2 * q4 - _2q2 * ax + 4 * q3q3 * q4 - _2q3 * ay
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        #self.heading = 0  # Meaningless without a magnetometer
        self.heading = degrees(-atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    async def _update_mag(self):
        if self.expect_ts:
            accel, gyro, mag, ts = self.read_imu()
        else:
            accel, gyro, mag = self.read_imu()
            ts = None
        mx, my, mz = (mag[x] - self.magbias[x] for x in range(3)) # Units irrelevant (normalised)
        ax, ay, az = accel                  # Units irrelevant (normalised)
        gyro_calib = (gyro[0] - self.gyro_bias[0], gyro[1] - self.gyro_bias[1], gyro[2] - self.gyro_bias[2])
        gx, gy, gz = (radians(x) for x in gyro_calib) # Units deg/s
        #gx, gy, gz = (radians(x) for x in gyro)  # Units deg/s
        q1, q2, q3, q4 = (self.q[x] for x in range(4))   # short name local variable for readability
        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return                          # handle NaN
        norm = 1 / norm                     # use reciprocal for division
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
            + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
            + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
            + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
            + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
            + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
            + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)    # normalise step magnitude
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        deltat = self.deltat(ts)
        q1 += qDot1 * deltat
        q2 += qDot2 * deltat
        q3 += qDot3 * deltat
        q4 += qDot4 * deltat
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)    # normalise quaternion
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm
        self.heading = self.declination + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

    def print_data(self):
        fs = 'Heading: {:4.0f} Pitch: {:4.0f} Roll: {:4.0f}'
        return fs.format(self.heading, self.pitch, self.roll)