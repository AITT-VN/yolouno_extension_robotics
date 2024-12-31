from micropython import const
from machine import SoftI2C, Pin, PWM
from setting import *
from utility import *
from constants import *

MDV2_STEPPER1 = const(0)
MDV2_STEPPER2 = const(1)

MDV2_DEFAULT_I2C_ADDRESS = 0x54

# version command
CMD_FIRMWARE_INFO = const(0x00)

MDV2_REG_RESET_ENC  = const(0)

MDV2_REG_MOTOR_INDEX = const(16) # set motor speed - motor index
MDV2_REG_MOTOR_SPEED = const(18) # set motor speed - speed

MDV2_REG_MOTOR_BRAKE = const(22)
MDV2_REG_REVERSE     = const(23)

MDV2_REG_SERVO1 = const(24)
MDV2_REG_SERVO2 = const(26)
MDV2_REG_SERVO3 = const(28)
MDV2_REG_SERVO4 = const(30)
MDV2_REG_SERVOS = [MDV2_REG_SERVO1, MDV2_REG_SERVO2, MDV2_REG_SERVO3, MDV2_REG_SERVO4]

# Read-only registers
MDV2_REG_FW_VERSION     = const(40)
MDV2_REG_WHO_AM_I       = const(42)
MDV2_REG_BATTERY        = const(43)
MDV2_REG_ENCODER1       = const(44)
MDV2_REG_ENCODER2       = const(48)
MDV2_REG_SPEED_E1       = const(52)
MDV2_REG_SPEED_E2       = const(54)

MOTOR_FREQ = const(1000)
M3_IN1_PIN = D2_PIN
M3_IN2_PIN = A3_PIN
M4_IN1_PIN = A7_PIN
M4_IN2_PIN = A6_PIN

class MotorDriverV2():
    def __init__(self, address=MDV2_DEFAULT_I2C_ADDRESS):
        self._i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self._addr = address
        self._encoders = [0, 0]
        self._speeds = [0, 0]
        self._reverse = [0, 0] # reverse status of encoders
        
        # ESP32-S3 motor pins. Stop using D2, A3, A6, A7 pin on YoloUNO
        self.m3_1_pin = Pin(M3_IN1_PIN, Pin.OUT)
        self.m3_2_pin = Pin(M3_IN2_PIN, Pin.OUT)
        self.m4_1_pin = Pin(M4_IN1_PIN, Pin.OUT)
        self.m4_2_pin = Pin(M4_IN2_PIN, Pin.OUT)
            
        self.m3_1 = PWM(Pin(M3_IN1_PIN), freq=MOTOR_FREQ, duty=0)
        self.m3_1.deinit()
        self.m3_2 = PWM(Pin(M3_IN2_PIN), freq=MOTOR_FREQ, duty=0)
        self.m3_2.deinit()
        self.m4_1 = PWM(Pin(M4_IN1_PIN), freq=MOTOR_FREQ, duty=0)
        self.m4_1.deinit()
        self.m4_2 = PWM(Pin(M4_IN2_PIN), freq=MOTOR_FREQ, duty=0)
        self.m4_2.deinit()
        
        self.m3_speed = False
        self.m4_speed = False
        
        # check i2c connection
        try:
            who_am_i = self._read_8(MDV2_REG_WHO_AM_I)
        except OSError:
            who_am_i = 0

        if who_am_i != MDV2_DEFAULT_I2C_ADDRESS:
            raise RuntimeError("Motor driver not found. Expected: " + str(address) + ", scanned: " + str(who_am_i))
        else:
            self.stop(ALL)

    #################### BASIC  FUNCTIONS ####################

    def fw_version(self):
        minor = self._read_8(MDV2_REG_FW_VERSION)
        major = self._read_8(MDV2_REG_FW_VERSION + 1)
        return("{}.{}".format(major, minor))

    def battery(self):
        battery = self._read_8(MDV2_REG_BATTERY)
        return round(battery/10, 1)
    
    #################### ESP32 MOTOR CONTROL ##################
    def _set_motors_esp(self, index, value=0):
        value = max(min(100, value), -100)

        if index == M3 and self.m3_speed == False:
            self.m3_1_pin.value(0)
            self.m3_2_pin.value(0)
        
        if index == M4 and self.m4_speed == False:
            self.m4_1_pin.value(0)
            self.m4_2_pin.value(0)

        if value >= 0:
            # Forward
            duty = int(translate(abs(value), 0, 100, 0, 1023))
            if index == M3:
                if self.m3_speed == True:
                    try:
                        self.m3_1.duty(duty)
                    except RuntimeError:
                        self.m3_2.deinit()
                        self.m3_1.init(freq=MOTOR_FREQ, duty=duty)
                else: # need to reset PWM
                    self.m3_2.deinit()
                    self.m3_1.init(freq=MOTOR_FREQ, duty=duty)
                self.m3_speed = True                
            if index == M4:
                if self.m4_speed == True:
                    try:
                        self.m4_1.duty(duty)
                    except RuntimeError:
                        self.m4_2.deinit()
                        self.m4_1.init(freq=MOTOR_FREQ, duty=duty)
                else: # need to reset PWM
                    self.m4_2.deinit()
                    self.m4_1.init(freq=MOTOR_FREQ, duty=duty)
                self.m4_speed = True
        else:
            # Backward
            duty = int(translate(abs(value), 0, 100, 0, 1023))
            if index == M3:
                if self.m3_speed == False:
                    try:
                        self.m3_2.duty(duty)
                    except RuntimeError:
                        self.m3_1.deinit()
                        self.m3_2.init(freq=MOTOR_FREQ, duty=duty)
                else: # need to reset PWM
                    self.m3_1.deinit()
                    self.m3_2.init(freq=MOTOR_FREQ, duty=duty)
                self.m3_speed = False
            if index == M4:
                if self.m4_speed == False:
                    try:
                        self.m4_2.duty(duty)
                    except RuntimeError:
                        self.m4_1.deinit()
                        self.m4_2.init(freq=MOTOR_FREQ, duty=duty)
                else: # need to reset PWM
                    self.m4_1.deinit()
                    self.m4_2.init(freq=MOTOR_FREQ, duty=duty)
                self.m4_speed = False
            
    def _brake_motors_esp(self, index):
        if index == M3:
            self.m3_2.deinit()
            self.m3_1.deinit()
            Pin(M3_IN1_PIN, Pin.OUT).value(1)
            Pin(M3_IN2_PIN, Pin.OUT).value(1)
            self.m3_speed = False
        if index == M4:
            self.m4_2.deinit()
            self.m4_1.deinit()
            Pin(M4_IN1_PIN, Pin.OUT).value(1)
            Pin(M4_IN2_PIN, Pin.OUT).value(1)
            self.m4_speed = False

    #################### MOTOR CONTROL ####################

    def set_motors(self, motors, speed):
        self._write_16_array(MDV2_REG_MOTOR_INDEX, [motors, speed*10])
        for i in [M3, M4]:
            if motors&i:
                self._set_motors_esp(i ,speed)
        
    def stop(self, motors=ALL):
        self._set_motors_esp(motors ,0)
        self.set_motors(motors, 0)

    def brake(self, motors=ALL):
        self._write_8(MDV2_REG_MOTOR_BRAKE, motors)
        for i in [M3, M4]:
            if motors&i:
                self._brake_motors_esp(i)

    def set_servo(self, index, angle, max=180):
        angle = int(angle*180/max)
        self._write_16(MDV2_REG_SERVOS[index], angle)

    def get_encoder(self, motors=ALL):
        self._read_32_array(MDV2_REG_ENCODER1, self._encoders)
        
        if (motors == ALL):
            return self._encoders
        elif motors & E1:
            return self._encoders[0]
        elif motors & E2:
            return self._encoders[1]
        else:
            return 0
        
    def reset_encoder(self, motors=ALL):
        self._write_8(MDV2_REG_RESET_ENC, motors)
    
    def reverse_encoder(self, motors):
        self._write_8(MDV2_REG_REVERSE, motors)

    def get_speed(self, motor=ALL):
        self._read_16_array(MDV2_REG_SPEED_E1, self._speeds)

        if motor == ALL:
            return self._speeds
        elif motor & E1:
            return self._speeds[0]
        elif motor & E2:
            return self._speeds[1]
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




