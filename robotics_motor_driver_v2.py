from micropython import const
from machine import SoftI2C, Pin
from setting import *
from utility import *

# version command
CMD_FIRMWARE_INFO = const(0x00)

DRIVER_SCL_PIN = const(22)
DRIVER_SDA_PIN = const(21)

MOTOR_ALL = const(0)
MOTOR_E1 = const(1)
MOTOR_E2 = const(2)
MOTOR_M1 = const(3)
MOTOR_M2 = const(4)

DRIVE_MODE_E1_E2 = const(0)
DRIVE_MODE_M1_M2 = const(1)
DRIVE_MODE_MECANUM = const(2)

MOTOR_DIR_CLOCKWISE = const(1)
MOTOR_DIR_COUNTERCLOCKWISE = const(-1)

STOP_COAST = const(0)
STOP_BRAKE = const(1)
STOP_HOLD = const(2)

UNIT_CM = const(0)
UNIT_INCH = const(1)
UNIT_SECOND = const(2)
UNIT_DEGREE = const(3)

SERVO_S1 = const(0)
SERVO_S2 = const(1)
SERVO_S3 = const(2)
SERVO_S4 = const(3)

PRMC_DEFAULT_I2C_ADDRESS = 0x54
PRMC_MOTOR_MAX_SPEED = 1000

PRMC_REG_RESET_ENC  = const(0)
PRMC_REG_PID_MODE   = const(1)
PRMC_REG_MAX_SPEED  = const(2)
PRMC_REG_PID_KP     = const(4)
PRMC_REG_PID_TI     = const(6)
PRMC_REG_PID_TD     = const(8)
PRMC_REG_PID_ILIM   = const(10)

PRMC_REG_POWER_E1 = const(12) # motor E1
PRMC_REG_POWER_E2 = const(14) # motor E2
PRMC_REG_POWER_M1 = const(16) # motor M1
PRMC_REG_POWER_M2 = const(18) # motor M2

PRMC_REG_REVERSE    = const(20)
PRMC_REG_MOTOR_BRAKE = const(21)

PRMC_REG_SERVO1 = const(22)
PRMC_REG_SERVO2 = const(24)
PRMC_REG_SERVO3 = const(26)
PRMC_REG_SERVO4 = const(28)
PRMC_REG_SERVOS = [PRMC_REG_SERVO1, PRMC_REG_SERVO2, PRMC_REG_SERVO3, PRMC_REG_SERVO4]

# Read-only registers
PRMC_REG_FW_VERSION     = const(40)
PRMC_REG_WHO_AM_I       = const(42)
PRMC_REG_BATTERY        = const(43)
PRMC_REG_ENCODER1       = const(44)
PRMC_REG_ENCODER2       = const(48)
PRMC_REG_SPEED_E1       = const(52)
PRMC_REG_SPEED_E2       = const(54)

class MotorDriver():
    def __init__(self, i2c=None, address=PRMC_DEFAULT_I2C_ADDRESS):
        if i2c == None:
            self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        else:
            self._i2c = i2c
        self._addr = address
        self._encoders = [0, 0]
        self._speeds = [0, 0]
        self._reverse = 0 # reverse status of motors
        self.max_pps = 2000 # 1:90 6V encoder motor
        
        # check i2c connection
        try:
            who_am_i = self._read_8(PRMC_REG_WHO_AM_I)
        except OSError:
            who_am_i = 0

        if who_am_i != PRMC_DEFAULT_I2C_ADDRESS:
            raise RuntimeError("Motor driver not found. Expected: " + PRMC_DEFAULT_I2C_ADDRESS + ", scanned: " + who_am_i)
        else:
            self.set_motors(0)
            self.config_pid(13500, 0.0002, 0.5, 0.005, 1000)
    

    #################### BASIC  FUNCTIONS ####################

    def fw_version(self):
        minor = self._read_8(PRMC_REG_FW_VERSION)
        major = self._read_8(PRMC_REG_FW_VERSION + 1)
        return("{}.{}".format(major, minor))

    #################### MOTOR CONTROL ####################

    def set_motor(self, motor, power):
        if (motor == MOTOR_E1):
            self._write_16(PRMC_REG_POWER_E1, round(power*10))
        elif (motor == MOTOR_E2):
            self._write_16(PRMC_REG_POWER_E2, round(power*10))
        elif (motor == MOTOR_M1):
            self._write_16(PRMC_REG_POWER_M1, round(power*10))
        elif (motor == MOTOR_M2):
            self._write_16(PRMC_REG_POWER_M2, round(power*10))
        else:
            raise RuntimeError('Invalid motor number')

    def set_encoder_motors(self, e1, e2=None):
        if e2 == None:
            e2 = e1

        # convert speed from -100:100 to -1000:1000
        self._write_16_array(PRMC_REG_POWER_E1, [round(e1*10), round(e2*10)])

    def set_dc_motors(self, m1, m2=None):
        if m2 == None:
            m2 = m1

        # convert speed from -100:100 to -1000:1000
        self._write_16_array(PRMC_REG_POWER_M1, [round(m1*10), round(m2*10)])
    
    def set_motors(self, e1, e2=None, m1=None, m2=None):
        if e2 == None:
            e2 = e1
        if m1 == None:
            m1 = e1
        if m2 == None:
            m2 = m1
        
        # convert speed from -100:100 to -1000:1000
        self._write_16_array(PRMC_REG_POWER_E1, [round(e1*10), round(e2*10), round(m1*10), round(m2*10)])
        
    def brake(self, motor):
        if motor < 0 and motor > 4:
            raise RuntimeError('Driver: Invalid motor index')
            
        for i in range(2):
            self._write_8(PRMC_REG_MOTOR_BRAKE, motor)

    def set_servo(self, index, pulse):
        self._write_16(PRMC_REG_SERVOS[index], pulse)

    '''
        Sets the PID values for position and speed control.

        If no arguments are given, this will return the current values.

        Parameters:
            max_pps (int) - Maximum encoder pulses per second of the motor

            Kp (int) - Proportional position control constant.

            Ti (int) - Integral position control constant.

            Td (int) - Derivative position (or proportional speed) control constant.

            Ilim (int) - Integration Limit. It is upper bound for the contribution of the integral term in the PID loop.

    '''
    def config_pid(self, max_pps, Kp=None, Ti=None, Td=None, Ilim=None):
        self.max_pps = max_pps
        if Kp is None:
            Kp = 0.8/max_pps
            Ti = 0.3
            Td = 0.03
            Ilim = 1000
        data = [round(max_pps), round(Kp*10000000), round(Ti*1000), round (Td*1000), round(Ilim)]
        self._write_16_array(PRMC_REG_MAX_SPEED, data)

    def get_encoders(self):
        self._read_32_array(PRMC_REG_ENCODER1, self._encoders)
        return self._encoders

    def get_encoder(self, motor):
        if motor == MOTOR_E1:
            return(self._read_32(PRMC_REG_ENCODER1))
        elif motor == MOTOR_E2:
            return(self._read_32(PRMC_REG_ENCODER2))
        elif motor == MOTOR_M1 or motor == MOTOR_M2:
            return 0
        else:
            raise RuntimeError('Driver: Invalid motor index')
    
    def pid_on(self):
        self._write_8(PRMC_REG_PID_MODE, 1)

    def pid_off(self):
        self._write_8(PRMC_REG_PID_MODE, 0)
        
    def reset_encoder(self, motor=MOTOR_ALL):
        # 0: both
        # 1: enc_1
        # 2: enc_2
        if motor >= 0 and motor <= 2:
            self._write_8(PRMC_REG_RESET_ENC, motor)
        else:
            raise RuntimeError('Driver: Invalid encoder index')
    
    def reverse_encoder(self, motor):
        if motor == MOTOR_ALL:
            self._reverse = 0b1111
        else:
            self._reverse = self._reverse | (1 << (motor-1))
        self._write_8(PRMC_REG_REVERSE, self._reverse)

    def get_speeds(self):
        self._read_16_array(PRMC_REG_SPEED_E1, self._speeds)
        return self._speeds

    def get_speed(self, motor):
        if motor == MOTOR_E1:
            return(self._read_16(PRMC_REG_SPEED_E1))
        elif motor == MOTOR_E2:
            return(self._read_16(PRMC_REG_SPEED_E2))
        else:
            raise RuntimeError('Driver: Invalid encoder index')

    def get_battery(self):
        return self._read_8(PRMC_REG_BATTERY)

    #################### I2C COMMANDS ####################

    def _write_8(self, register, data):
        # Write 1 byte of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, bytes([data]))

    def _write_16(self, register, data):
        # Write a 16-bit little endian value to the specified register
        # address.
        self._i2c.writeto_mem(self._addr, register, bytes(
            [data & 0xFF, (data >> 8) & 0xFF]))

    def _write_16_array(self, register, data):
        # write an array of littel endian 16-bit values  to specified register address
        l = len(data)
        buffer = bytearray(2*l)
        for i in range(l):
            buffer[2*i] = data[i] & 0xFF
            buffer[2*i+1] = (data[i] >> 8) & 0xFF
        self._i2c.writeto_mem(self._addr, register, buffer)

    def _read_8(self, register):
        # Read and return a byte from  the specified register address.
        self._i2c.writeto(self._addr, bytes([register]))
        result = self._i2c.readfrom(self._addr, 1)
        return result[0]

    def _read_8_array(self, register, result_array):
        # Read and  saves into result_arrray a sequence of bytes
        # starting from the specified  register address.
        l = len(result_array)
        self._i2c.writeto(self._addr, bytes([register]))
        in_buffer = self._i2c.readfrom(self._addr, l)
        for i in range(l):
            result_array[i] = in_buffer[i]

    def _read_16(self, register):
        # Read and return a 16-bit signed little  endian value  from the
        # specified  register address.
        self._i2c.writeto(self._addr, bytes([register]))
        in_buffer = self._i2c.readfrom(self._addr, 2)
        raw = (in_buffer[1] << 8) | in_buffer[0]
        if (raw & (1 << 15)):  # sign bit is set
            return (raw - (1 << 16))
        else:
            return raw

    def _read_16_array(self, register, result_array):
        # Read and  saves into result_arrray a sequence of 16-bit little  endian
        # values  starting from the specified  register address.
        l = len(result_array)
        self._i2c.writeto(self._addr, bytes([register]))
        in_buffer = self._i2c.readfrom(self._addr, 2*l)
        for i in range(l):
            raw = (in_buffer[2*i+1] << 8) | in_buffer[2*i]
            if (raw & (1 << 15)):  # sign bit is set
                result_array[i] = (raw - (1 << 16))
            else:
                result_array[i] = raw

    def _read_32(self, register):
        # Read and return a 32-bit signed little  endian value  from the
        # specified  register address.

        self._i2c.writeto(self._addr, bytes([register]))
        in_buffer = self._i2c.readfrom(self._addr, 4)
        raw = (in_buffer[3] << 24) | (in_buffer[2] << 16) | (
            in_buffer[1] << 8) | in_buffer[0]
        if (raw & (1 << 31)):  # sign bit is set
            return (raw - (1 << 32))
        else:
            return raw

    def _read_32_array(self, register, result_array):
        # Read and  saves into result_arrray a sequence of 32-bit little  endian
        # values  starting from the specified  register address.
        l = len(result_array)
        self._i2c.writeto(self._addr, bytes([register]))
        in_buffer = self._i2c.readfrom(self._addr, 4*l)
        for i in range(l):
            raw = (in_buffer[4*i+3] << 24) | (in_buffer[4*i+2]
                                              << 16) | (in_buffer[4*i+1] << 8) | in_buffer[4*i]
            if (raw & (1 << 31)):  # sign bit is set
                result_array[i] = (raw - (1 << 32))
            else:
                result_array[i] = raw

class Servo:
    def __init__(self, driver, port):
        self._driver = driver
        self._port = port
        self._current_pos = None

    def _angle_to_pulse(self, angle, type=180):
        pulse = 25 + int((angle/type)*100)
        return pulse

    async def position(self, angle, speed=100, type=180):
        if angle < 0:
           angle = 0
        
        if angle > type:
           angle = type

        if speed < 0 or speed > 100:
           raise('Servo: invalid speed')

        if speed == 100 or self._current_pos == None:
            pulse = self._angle_to_pulse(angle, type)
            self._driver.set_servo(self._port, pulse)
        else:
            delay = translate(speed, 0, 100, 50, 0)
            if self._current_pos > angle:
                for i in range(self._current_pos, angle, -1):
                    pulse = self._angle_to_pulse(i, type)
                    self._driver.set_servo(self._port, pulse)
                    await wait(delay)
            else:
                for i in range(self._current_pos, angle, 1):
                    pulse = self._angle_to_pulse(i, type)
                    self._driver.set_servo(self._port, pulse)
                    await wait(delay)
        
        self._current_pos = angle

    def spin(self, speed):
        if speed < -100 or speed > 100:
            raise('Servo: invalid spin speed')

        angle = 90 - (speed/100)*90
        pulse = self._angle_to_pulse(angle)
        self._driver.set_servo(self._port, pulse)
        self._current_pos = angle

