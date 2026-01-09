
'''
icm426709.py MicroPython driver for the high performance 6-axis MEMS MotionTracking device.
Datasheet: https://invensense.tdk.com/wp-content/uploads/2021/07/ds-000451_icm-42670-p-datasheet.pdf
Authors: OhStem Education, VietNam
V1.0 - 9th Jan 2025
'''

import struct
from time import sleep, sleep_ms, ticks_ms, ticks_diff
import time
from machine import SoftI2C, Pin
from setting import SDA_PIN, SCL_PIN
from vector3d import Vector3d

# Default I2C address of ICM42670P
# Can be 0x68 or 0x69 depending on AD0/LSB pin state
ICM42670P_DEFAULT_ADDRESS = 0x69 # Updated based on your scan result

# Register addresses (Bank 0)
REG_SIGNAL_PATH_RESET = 0x02
REG_PWR_MGMT0 = 0x1F
REG_INTF_CONFIG1 = 0x54

# Accel data registers
REG_ACCEL_XOUT_H = 0x0B
REG_ACCEL_XOUT_L = 0x0C
REG_ACCEL_YOUT_H = 0x0D
REG_ACCEL_YOUT_L = 0x0E
REG_ACCEL_ZOUT_H = 0x0F
REG_ACCEL_ZOUT_L = 0x10

# Gyro data registers
REG_GYRO_XOUT_H = 0x11
REG_GYRO_XOUT_L = 0x12
REG_GYRO_YOUT_H = 0x13
REG_GYRO_YOUT_L = 0x14
REG_GYRO_ZOUT_H = 0x15
REG_GYRO_ZOUT_L = 0x16

# Config registers for ranges
REG_GYRO_CONFIG0 = 0x20
REG_ACCEL_CONFIG0 = 0x21

# WHO_AM_I register (this helps confirm connection)
ICM42670P_WHO_AM_I_REG = 0x75
ICM42670P_WHO_AM_I_VALUE = 0x67 # Expected WHO_AM_I value based on actual reading (0x68)

# Configuration and data registers (refer to ICM42670P datasheet for exact values)
# NEED TO REFER TO ICM42670P DATASHEET TO CONFIRM THESE REGISTER ADDRESSES.
# These addresses and bitmasks are assumed based on other MPU/ICM examples and similar C++ code.
ICM44670P_REG_BANK_SEL = 0x76 # Bank selection register (BANK_SEL)

# Initialization registers from C++ code and ICM42670P datasheet
ICM42670P_INTF_CONFIG1 = 0x4D # Interface Configuration 1 (Reg 0x4D)
INTF_CONFIG1_I2C_ONLY_EN = 0x01 # Bit 0 (I3C_DISABLE) of INTF_CONFIG1 to enable I2C-only (assumed)

# Fix: Soft Reset is performed using SIGNAL_PATH_RESET, not DEVICE_CONFIG
ICM42670P_SIGNAL_PATH_RESET = 0x04 # Signal Path Reset Register (Reg 0x04) (DATASHEET PAGE 46)
SOFT_RESET_DEVICE_CONFIG_EN = 0x01 # Bit 0 (SOFT_RESET) of SIGNAL_PATH_RESET to perform Soft Reset (according to C++ code)

# Ignore INT_STATUS_RESET_DONE_INT as it does not return expected value.
# ICM42670P_INT_STATUS = 0x2D

ICM42670P_PWR_MGMT_0 = 0x1F  # Power Management 0 (Reg 0x1F - DATASHEET PAGE 41, 55)
ICM42670P_ACCEL_CONFIG0 = 0x21 # Accelerometer Config 0 (Reg 0x21 - DATASHEET PAGE 41, 57)
ICM42670P_GYRO_CONFIG0 = 0x20  # Gyroscope Config 0 (Reg 0x20 - DATASHEET PAGE 41, 56)

# Constants
GRAVITY_MS2 = 9.80665
RAD_PER_DEG = 0.017453293 # Degrees to Radians

class ICM42670P(object):
    """
    Module for InvenSense IMUs. Base class implements ICM62670P 6DOF sensor.
    """

    _I2Cerror = "I2C failure when communicating with IMU"

    def __init__(self, i2c=None, device_addr=ICM42670P_DEFAULT_ADDRESS, transposition=(0, 1, 2), scaling=(1, 1, 1)):

        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)

        sleep_ms(100)  # Ensure PSU and device have settled
        if i2c:
            self.i2c = i2c
        else:
            self.i2c = SoftI2C(scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=100000)
        self.addr = device_addr
        self.address = device_addr
        self._accel_fsr_g = 16 # Default FSR value for accelerometer (as in Polling_I2C.ino)
        self._gyro_fsr_dps = 2000 # Default FSR value for gyroscope (as in Polling_I2C.ino)
        self._accel_data_rate = 100 # Default ODR value
        self._gyro_data_rate = 100 # Default ODR value

        self._init_device()
        
        self.accel_range = 0
        self.gyro_range = 0

    def _write_reg(self, reg, val):
        self.i2c.writeto_mem(self.addr, reg, bytes([val]))

    def _read_reg(self, reg, n=1, retries=3):
        for attempt in range(retries):
            try:
                return self.i2c.readfrom_mem(self.addr, reg, n)
            except OSError as e:
                if e.args[0] == 19 and attempt < retries - 1:
                    time.sleep(0.002)
                    continue
                raise
    
    def _init_device(self):
        # Soft reset
        self._write_reg(REG_SIGNAL_PATH_RESET, 0x10)
        time.sleep(0.02)

        # Disable I3C, force I²C
        self._write_reg(REG_INTF_CONFIG1, 0x00)

        # Enable accel + gyro in LN mode
        self._write_reg(REG_PWR_MGMT0, 0x0F)
        time.sleep(0.05)

    def _read16(self, regH, regL):
        high = self._read_reg(regH)[0]
        low = self._read_reg(regL)[0]
        val = (high << 8) | low
        if val & 0x8000:
            val -= 65536
        return val
    
    @property
    def sensors(self):
        """
        returns sensor objects accel, gyro
        """
        return self._accel, self._gyro

    
    def _check_who_am_i(self):
        """Read WHO_AM_I register to confirm sensor."""
        try:
            # Ensure we are in bank 0 to read WHO_AM_I
            self._set_bank(0)
            reg_val = self.i2c.readfrom_mem(self.address, ICM42670P_WHO_AM_I_REG, 1)[0]
            if reg_val != ICM42670P_WHO_AM_I_VALUE:
                # Print read value for easy debugging
                error_msg = f"ICM42670P not found. Read WHO_AM_I: 0x{reg_val:02X}, expected: 0x{ICM42670P_WHO_AM_I_VALUE:02X}. Check wiring and I2C address or expected WHO_AM_I value."
                raise RuntimeError(error_msg)
            print(f"ICM42670P found at address 0x{self.address:02X}")
        except OSError as e:
            raise RuntimeError(f"I2C communication error when reading WHO_AM_I: {e}. Check connection.")

    @property
    def chip_id(self):
        try:
            return self._read_reg(0x75)[0]
        except:
            return 0x00

    def wake(self):
        self._init_device()
        return "awake"

    def sleep(self):
        self._write_reg(REG_PWR_MGMT0, 0x00)
        return "asleep"
    
    # --- Properties for Temperature ---
    @property
    def temperature(self):
        """Returns temperature in degrees C."""
        try:
            high = self._read_reg(0x09)[0]
            low = self._read_reg(0x0A)[0]
            val = (high << 8) | low
            if val & 0x8000:
                val -= 65536
            temp_c = (val / 132.48) + 25
            return temp_c
        except:
            return 0.0

    # --- Properties for Accelerometer ---
    @property
    def accel_range(self):
        try:
            val = self._read_reg(REG_ACCEL_CONFIG0)[0]
            return (val >> 5) & 0x03
        except:
            return 0

    @accel_range.setter
    def accel_range(self, rng):
        if rng not in (0, 1, 2, 3):
            raise ValueError("Accel range must be 0..3")
        # Set FS_SEL (bits 6:5) and ODR (bits 3:0) to 100Hz (0x06)
        val = ((rng & 0x03) << 5) | 0x06
        self._write_reg(REG_ACCEL_CONFIG0, val)
        
        if rng == 0: self._accel_fsr_g = 16
        elif rng == 1: self._accel_fsr_g = 8
        elif rng == 2: self._accel_fsr_g = 4
        elif rng == 3: self._accel_fsr_g = 2

    @property
    def acceleration(self):
        """
        Returns acceleration data (x, y, z) in m/s^2.
        Uses configured FSR to convert raw data.
        """
        try:
            # Read 6 bytes at once to reduce I2C latency (Burst Read)
            data = self._read_reg(REG_ACCEL_XOUT_H, 6)
            x = (data[0] << 8) | data[1]
            y = (data[2] << 8) | data[3]
            z = (data[4] << 8) | data[5]

            if x & 0x8000: x -= 65536
            if y & 0x8000: y -= 65536
            if z & 0x8000: z -= 65536

            return (x, y, z)

        except OSError as e:
            print(f"Warning: Error reading acceleration: {e}. Returning (0,0,0).")
            return (0.0, 0.0, 0.0)

    # --- Properties for Gyroscope ---
    @property
    def gyro_range(self):
        try:
            val = self._read_reg(REG_GYRO_CONFIG0)[0]
            return (val >> 5) & 0x03
        except:
            return 0

    @gyro_range.setter
    def gyro_range(self, rng):
        if rng not in (0, 1, 2, 3):
            raise ValueError("Gyro range must be 0..3")
        # Set FS_SEL (bits 6:5) and ODR (bits 3:0) to 100Hz (0x06)
        val = ((rng & 0x03) << 5) | 0x06
        self._write_reg(REG_GYRO_CONFIG0, val)
        
        if rng == 0: self._gyro_fsr_dps = 2000
        elif rng == 1: self._gyro_fsr_dps = 1000
        elif rng == 2: self._gyro_fsr_dps = 500
        elif rng == 3: self._gyro_fsr_dps = 250

    @property
    def gyroscope(self):
        """
        Returns angular velocity (x, y, z) in rad/s.
        Uses configured FSR to convert raw data.
        """
        try:
            # Read 6 bytes at once to reduce I2C latency (Burst Read)
            data = self._read_reg(REG_GYRO_XOUT_H, 6)
            x = (data[0] << 8) | data[1]
            y = (data[2] << 8) | data[3]
            z = (data[4] << 8) | data[5]

            if x & 0x8000: x -= 65536
            if y & 0x8000: y -= 65536
            if z & 0x8000: z -= 65536

            return (x, y, z)
        except OSError as e:
            print(f"Warning: Error reading gyroscope: {e}. Returning (0,0,0).")
            return (0.0, 0.0, 0.0)

    # Accelerometer
    @property
    def accel(self):
        """
        Acceleremoter object
        """
        return self._accel

    def _accel_callback(self):
        """
        Update accelerometer Vector3d object
        """
        try:
            self._accel._ivector = self.acceleration
        except OSError:
            #raise MPUException(self._I2Cerror)
            #print(self._I2Cerror)
            pass

        if self._accel_fsr_g == 2:
            scale_factor = 16384.0
        elif self._accel_fsr_g == 4:
            scale_factor = 8192.0
        elif self._accel_fsr_g == 8:
            scale_factor = 4096.0
        elif self._accel_fsr_g == 16:
            scale_factor = 2048.0
        else:
            scale_factor = 2048.0 # For 16g default
        self._accel._vector[0] = self._accel._ivector[0] / scale_factor
        self._accel._vector[1] = self._accel._ivector[1] / scale_factor
        self._accel._vector[2] = self._accel._ivector[2] / scale_factor

    def get_accel_irq(self):
        """
        For use in interrupt handlers. Sets self._accel._ivector[] to signed
        unscaled integer accelerometer values
        """
        #self._read(self.buf6, 0x3B, self.mpu_addr)
        #self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        #self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        #self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        pass

    # Gyro
    @property
    def gyro(self):
        """
        Gyroscope object
        """
        return self._gyro

    def _gyro_callback(self):
        """
        Update gyroscope Vector3d object
        """
        try:
            self._gyro._ivector = self.gyroscope
            #print(self._gyro._ivector)
        except OSError:
            #raise MPUException(self._I2Cerror)
            #print(self._I2Cerror)
            pass

        #scale = (131, 65.5, 32.8, 16.4)
        if self._gyro_fsr_dps == 250:
            scale_factor = 131.0
        elif self._gyro_fsr_dps == 500:
            scale_factor = 65.5
        elif self._gyro_fsr_dps == 1000:
            scale_factor = 32.8
        elif self._gyro_fsr_dps == 2000:
            scale_factor = 16.4
        else:
            scale_factor = 16.4 # For 2000dps default

        self._gyro._vector[0] = self._gyro._ivector[0] / scale_factor
        self._gyro._vector[1] = self._gyro._ivector[1] / scale_factor
        self._gyro._vector[2] = self._gyro._ivector[2] / scale_factor

    def get_gyro_irq(self):
        """
        For use in interrupt handlers. Sets self._gyro._ivector[] to signed
        unscaled integer gyro values. Error trapping disallowed.
        """
        pass
