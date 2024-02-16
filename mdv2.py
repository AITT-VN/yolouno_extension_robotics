import struct
from micropython import const
from machine import SoftI2C, Pin
from setting import *
from utility import *

MDV2_ALL = const(15)
MDV2_E1 = const(1)
MDV2_E2 = const(2)
MDV2_M1 = const(4)
MDV2_M2 = const(8)

MDV2_STEPPER1 = const(0)
MDV2_STEPPER2 = const(1)

DIR_CW = const(1)
DIR_CCW = const(-1)

MDV2_S1 = const(0)
MDV2_S2 = const(1)
MDV2_S3 = const(2)
MDV2_S4 = const(3)

MDV2_DEFAULT_I2C_ADDRESS = 0x54

# version command
CMD_FIRMWARE_INFO = const(0x00)

MDV2_REG_RESET_ENC  = const(0)
MDV2_REG_PID_MODE_E1   = const(2)
MDV2_REG_PID_MODE_E2   = const(3)
MDV2_REG_MAX_SPEED_E1  = const(4)
MDV2_REG_MAX_SPEED_E2  = const(6)
MDV2_REG_PID_KP     = const(8)
MDV2_REG_PID_KI     = const(10)
MDV2_REG_PID_KD     = const(12)
MDV2_REG_PID_KC     = const(14)

MDV2_REG_MOTOR_INDEX = const(16) # set motor speed - motor index
MDV2_REG_MOTOR_SPEED = const(18) # set motor speed - speed
MDV2_REG_MOTOR_TICKS = const(20) # set motor speed - target ticks (for encoder motors)

MDV2_REG_MOTOR_BRAKE = const(24)
MDV2_REG_REVERSE    = const(25)

MDV2_REG_SERVO1 = const(26)
MDV2_REG_SERVO2 = const(28)
MDV2_REG_SERVO3 = const(30)
MDV2_REG_SERVO4 = const(32)
MDV2_REG_SERVOS = [MDV2_REG_SERVO1, MDV2_REG_SERVO2, MDV2_REG_SERVO3, MDV2_REG_SERVO4]

# Read-only registers
MDV2_REG_FW_VERSION     = const(40)
MDV2_REG_WHO_AM_I       = const(42)
MDV2_REG_BATTERY        = const(43)
MDV2_REG_ENCODER1       = const(44)
MDV2_REG_ENCODER2       = const(48)
MDV2_REG_SPEED_E1       = const(52)
MDV2_REG_SPEED_E2       = const(54)
MDV2_REG_E1_DONE        = const(56)
MDV2_REG_E2_DONE        = const(56)

class MotorDriverV2():
    def __init__(self, address=MDV2_DEFAULT_I2C_ADDRESS):
        self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self._addr = address
        self._encoders = [0, 0]
        self._speeds = [0, 0] # E1 & E2 encoders
        self._reverse = [0, 0] # reverse status of encoders
        self._motors_done = [1, 1]
        self.max_pps = [6500, 6500] # 250rpm 1:34 12V encoder motor
        
        # check i2c connection
        try:
            who_am_i = self._read_8(MDV2_REG_WHO_AM_I)
        except OSError:
            who_am_i = 0

        if who_am_i != MDV2_DEFAULT_I2C_ADDRESS:
            raise RuntimeError("Motor driver not found. Expected: " + str(address) + ", scanned: " + str(who_am_i))
        else:
            self.set_motors(MDV2_ALL, 0)

    #################### BASIC  FUNCTIONS ####################

    def fw_version(self):
        minor = self._read_8(MDV2_REG_FW_VERSION)
        major = self._read_8(MDV2_REG_FW_VERSION + 1)
        return("{}.{}".format(major, minor))

    #################### MOTOR CONTROL ####################

    def set_motors(self, motors, speed, ticks=0):
        #buffer = struct.pack('>BBhI', motors, 0, speed, ticks)
        if speed == 0:
            ticks = 0

        buffer = bytearray(8)
        buffer[0] = motors
        buffer[1] = 0
        speed = round(speed * 10)
        speed_bytes = speed.to_bytes(2, 'little')
        buffer[2] = speed_bytes[0]
        buffer[3] = speed_bytes[1] 
        ticks_bytes = ticks.to_bytes(4, 'little')
        buffer[4] = ticks_bytes[0]
        buffer[5] = ticks_bytes[1]
        buffer[6] = ticks_bytes[2]
        buffer[7] = ticks_bytes[3]

        self._write_8_array(MDV2_REG_MOTOR_INDEX, bytes(buffer))
        
    def stop(self, motors):
        self.set_motors(motors, 0)

    def brake(self, motors):
        self._write_8(MDV2_REG_MOTOR_BRAKE, motors)

    def set_servo(self, index, angle, max=180):
        angle = int(angle*180/max)
        self._write_16(MDV2_REG_SERVOS[index], angle)

    '''
        Sets the PID values for position and speed control.

        If no arguments are given, this will return the current values.

        Parameters:

            kp (int) - Proportional position control constant.

            ki (int) - Integral position control constant.

            kd (int) - Derivative position (or proportional speed) control constant.

            kc (int) - Encoder counts difference between 2 motors

    '''
    def set_pid(self, kp, ki, kd, kc=0):
        data = [round(kp*1000), round(ki*1000), round (kd*1000), round(kc*1000)]
        self._write_16_array(MDV2_REG_PID_KP, data)

    def pid_on(self, motors=MDV2_E1|MDV2_E2):
        if motors & MDV2_E1:
            self._write_8(MDV2_REG_PID_MODE_E1, 1)
        if motors & MDV2_E2:
            self._write_8(MDV2_REG_PID_MODE_E2, 1)

    def pid_off(self, motors=MDV2_E1|MDV2_E2):
        if motors & MDV2_E1:
            self._write_8(MDV2_REG_PID_MODE_E1, 0)
        if motors & MDV2_E2:
            self._write_8(MDV2_REG_PID_MODE_E2, 0)
    
    '''
        Parameters:
            rpm (int) - Motor revolution per minute
            cpr (int) - Encoder count pulse per revolution
            gear (int) - Motor gear reduction ration, for ex 1:34 or 1:90
    '''
    def set_max_speed(self, motors, rpm, ppr, gears):
        max_speed = int(rpm * ppr * gears * 4 / 60) # convert to max count pulse per second

        if (motors & MDV2_E1 and motors & MDV2_E2):
            self._write_16_array(MDV2_REG_MAX_SPEED_E1, [max_speed, max_speed])
        
        elif (motors & MDV2_E1):
            self._write_16(MDV2_REG_MAX_SPEED_E1, max_speed)

        elif (motors & MDV2_E2):
            self._write_16(MDV2_REG_MAX_SPEED_E2, max_speed)

    def get_encoder(self, motors):
        self._read_32_array(MDV2_REG_ENCODER1, self._encoders)
        
        if (motors & MDV2_E1 and motors & MDV2_E2):
            return self._encoders
        elif motors & MDV2_E1:
            return self._encoders[0]
        elif motors & MDV2_E2:
            return self._encoders[1]
        else:
            return 0
        
    def reset_encoder(self, motors=MDV2_E1|MDV2_E2):
        self._write_8(MDV2_REG_RESET_ENC, motors)
    
    def reverse_encoder(self, motors=MDV2_E1|MDV2_E2):
        if motors & MDV2_E1:
            self._reverse[0] = 1

        if motors & MDV2_E2:
            self._reverse[1] = 1
            
        config = self._reverse[0] | (self._reverse[1] << 1)
        self._write_8(MDV2_REG_REVERSE, config)

    def get_speed(self, motor=MDV2_E1|MDV2_E2):
        self._read_16_array(MDV2_REG_SPEED_E1, self._speeds)

        if motor & MDV2_E1 and motor & MDV2_E2:
            return self._speeds
        elif motor & MDV2_E1:
            return self._speeds[0]
        elif motor & MDV2_E2:
            return self._speeds[1]
        else:
            return 0
    
    def get_done(self, motor=MDV2_E1|MDV2_E2):
        self._read_8_array(MDV2_REG_E1_DONE, self._motors_done)
        
        if motor & MDV2_E1 and motor & MDV2_E2:
            return self._motors_done
        elif motor & MDV2_E1:
            return self._motors_done[0]
        elif motor & MDV2_E2:
            return self._motors_done[1]
        else:
            return 0

    def stepper_speed(self, stepper, steps_per_rev, speed):
        if stepper not in (MDV2_STEPPER1, MDV2_STEPPER2):
            raise RuntimeError('Invalid stepper motor port')
        # To be implemented
    

    def stepper_step(self, stepper, steps):
        if stepper not in (MDV2_STEPPER1, MDV2_STEPPER2):
            raise RuntimeError('Invalid stepper motor port')
        # To be implemented
    
    def get_battery(self):
        return self._read_8(MDV2_REG_BATTERY)

    #################### I2C COMMANDS ####################

    def _write_8(self, register, data):
        # Write 1 byte of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, bytes([data]))

    def _write_8_array(self, register, data):
        # Write multiple bytes of data to the specified  register address.
        self._i2c.writeto_mem(self._addr, register, data)

    def _write_16(self, register, data):
        # Write a 16-bit little endian value to the specified register
        # address.
        self._i2c.writeto_mem(self._addr, register, bytes(
            [data & 0xFF, (data >> 8) & 0xFF]))

    def _write_16_array(self, register, data):
        # write an array of litte endian 16-bit values  to specified register address
        l = len(data)
        buffer = bytearray(2*l)
        for i in range(l):
            buffer[2*i] = data[i] & 0xFF
            buffer[2*i+1] = (data[i] >> 8) & 0xFF
        self._i2c.writeto_mem(self._addr, register, buffer)

    def _write_32(self, register, data):
        # Write a 32-bit little endian value to the specified register
        # address.
        self._i2c.writeto_mem(self._addr, register, bytes(
            [data & 0xFF, (data >> 8) & 0xFF, (data >> 16) & 0xFF, (data >> 24) & 0xFF]))

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

#md = MotorDriverV2()