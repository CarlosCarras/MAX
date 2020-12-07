#!/usr/bin/env python3

"""
filename: icm20948_imu.py
author:   Carlos Carrasquillo
created:  October 24, 2020
modified: October 25, 2020
project:  MAX

purpose:  This file reads accelerometer, gyroscope, and magnetometer data from the Invensense ICM20948 IMU over I2C.

datasheet: Invensense DS-000189, Rev. 1.3

discloure: This code was modeled after Gadgetoid's icm20948_imu.py file, which can be found at,
https://github.com/pimoroni/icm20948-python/blob/master/library/icm20948/__init__.py
"""

from smbus2 import SMBus
import time
import struct


C_GRAVITY = 9.80665

ICM20948_ADDR = 0x68
ICM20948_CHIP_ID = 0xEA
ICM20948_BANK_SEL = 0x7F

#------------- User Bank 0 -------------#
# NOTE: low-byte of register is located in the address immediately after the high-byte
ICM20948_USR0_WHO_AM_I                = 0x00
ICM20948_USR0_USER_CTRL               = 0x03
ICM20948_USR0_LP_CONFIG               = 0x05
ICM20948_USR0_PWR_MGMT_1              = 0x06
ICM20948_USR0_PWR_MGMT_2              = 0x07
ICM20948_USR0_INT_PIN_CFG             = 0x0F
ICM20948_USR0_INT_ENABLE              = 0x10
ICM20948_USR0_INT_ENABLE_1            = 0x11
ICM20948_USR0_INT_ENABLE_2            = 0x12
ICM20948_USR0_INT_ENABLE_3            = 0x13
ICM20948_USR0_I2C_MST_STATUS          = 0x17
ICM20948_USR0_INT_STATUS              = 0x19
ICM20948_USR0_INT_STATUS_1            = 0x1A
ICM20948_USR0_INT_STATUS_2            = 0x1B
ICM20948_USR0_INT_STATUS_3            = 0x1C
ICM20948_USR0_DELAY_TIMEH             = 0x28

ICM20948_USR0_ACCEL_XOUT_H            = 0x2D
ICM20948_USR0_ACCEL_YOUT_H            = 0x2F
ICM20948_USR0_ACCEL_ZOUT_H            = 0x31

ICM20948_USR0_GYRO_XOUT_H             = 0x33
ICM20948_USR0_GYRO_YOUT_H             = 0x35
ICM20948_USR0_GYRO_ZOUT_H             = 0x37

ICM20948_USR0_TEMP_OUT_H              = 0x39

ICM20948_USR0_EXT_SLV_SENS_DATA_00    = 0x3B   # ... EXT_SLV_SENS_DATA_23 = 0x52  (incrememnt by 1)

ICM20948_USR0_FIFO_EN_1               = 0x66
ICM20948_USR0_FIFO_EN_2               = 0x67
ICM20948_USR0_RST                     = 0x68
ICM20948_USR0_FIFO_MODE               = 0x69
ICM20948_USR0_FIFO_COUNTH             = 0x70
ICM20948_USR0_FIFO_COUNTL             = 0x71
ICM20948_USR0_FIFO_R_W                = 0x72
ICM20948_USR0_DATA_RDY_STATUS         = 0x74
ICM20948_USR0_FIFO_CFG                = 0x76

#------------- User Bank 1 -------------#

ICM20948_USR1_SELF_TEST_X_GYRO        = 0x02
ICM20948_USR1_SELF_TEST_Y_GYRO        = 0x03
ICM20948_USR1_SELF_TEST_Z_GYRO        = 0x04
ICM20948_USR1_SELF_TEST_X_ACCEL       = 0x0E
ICM20948_USR1_SELF_TEST_Y_ACCEL       = 0x0F
ICM20948_USR1_SELF_TEST_Z_ACCEL       = 0x10

ICM20948_USR1_XA_OFFS_H               = 0x14
ICM20948_USR1_YA_OFFS_H               = 0x17
ICM20948_USR1_ZA_OFFS_H               = 0x1A
ICM20948_USR1_TIMEBASE_CORRECTION_PLL = 0x28

#------------- User Bank 2 -------------#

ICM20948_USR2_GYRO_SMPLRT_DIV         = 0x00
ICM20948_USR2_GYRO_CONFIG_1           = 0x01
ICM20948_USR2_GYRO_CONFIG_2           = 0x02
ICM20948_USR2_XG_OFFS_USRH            = 0x03
ICM20948_USR2_YG_OFFS_USRH            = 0x05
ICM20948_USR2_ZG_OFFS_USRH            = 0x07
ICM20948_USR2_ODR_ALIGN_EN            = 0x09
ICM20948_USR2_ACCEL_SMPLRT_DIV_1      = 0x10
ICM20948_USR2_ACCEL_SMPLRT_DIV_2      = 0x11
ICM20948_USR2_ACCEL_INTEL_CTRL        = 0x12
ICM20948_USR2_ACCEL_WOM_THR           = 0x13
ICM20948_USR2_ACCEL_CONFIG            = 0x14
ICM20948_USR2_ACCEL_CONFIG2           = 0x15
ICM20948_USR2_FSYNC_CONFIG            = 0x52
ICM20948_USR2_TEMP_CONFIG             = 0x53
ICM20948_USR2_MOD_CTRL_USR            = 0x54

#------------- User Bank 3 -------------#

ICM20948_USR3_I2C_MST_ODR_CONFIG      = 0x00
ICM20948_USR3_I2C_MST_CTRL            = 0x01
ICM20948_USR3_I2C_MST_DELAY_CTRL      = 0x02
ICM20948_USR3_I2C_SLV0_ADDR           = 0x03
ICM20948_USR3_I2C_SLV0_REG            = 0x04
ICM20948_USR3_I2C_SLV0_CTRL           = 0x05
ICM20948_USR3_I2C_SLV0_DO             = 0x06
ICM20948_USR3_I2C_SLV1_ADDR           = 0x07
ICM20948_USR3_I2C_SLV1_REG            = 0x08
ICM20948_USR3_I2C_SLV1_CTRL           = 0x09
ICM20948_USR3_I2C_SLV1_DO             = 0x0A
ICM20948_USR3_I2C_SLV2_ADDR           = 0x0B
ICM20948_USR3_I2C_SLV2_REG            = 0x0C
ICM20948_USR3_I2C_SLV2_CTRL           = 0x0D
ICM20948_USR3_I2C_SLV2_DO             = 0x0E
ICM20948_USR3_I2C_SLV3_ADDR           = 0x0F
ICM20948_USR3_I2C_SLV3_REG            = 0x10
ICM20948_USR3_I2C_SLV3_CTRL           = 0x11
ICM20948_USR3_I2C_SLV3_DO             = 0x12
ICM20948_USR3_I2C_SLV4_ADDR           = 0x13
ICM20948_USR3_I2C_SLV4_REG            = 0x14
ICM20948_USR3_I2C_SLV4_CTRL           = 0x15
ICM20948_USR3_I2C_SLV4_DO             = 0x16
ICM20948_USR3_I2C_SLV4_DI             = 0x17

#------------ Magnetometer -------------#

AK09916_SENSITIVITY_FACTOR  = 0.15              # sensitivity scale factor, pp. 13
AK09916_RESOLUTION = (2**16)/2 - 1              # 16-bit output resolution, pp. 13

AK09916_I2C_ADDR            = 0x0C
AK09916_CHIP_ID             = 0x09
AK09916_WIA                 = 0x01
AK09916_ST1                 = 0x10
AK09916_ST1_DOR             = 1 << 1            # data overflow bit
AK09916_ST1_DRDY            = 1 << 0            # data self.ready bit
AK09916_HXL                 = 0x11
AK09916_ST2                 = 0x18
AK09916_ST2_HOFL            = 1 << 3            # magnetic sensor overflow bit
AK09916_CNTL2               = 0x31
AK09916_CNTL2_MODE          = 0x0F
AK09916_CNTL2_MODE_OFF      = 0
AK09916_CNTL2_MODE_SINGLE   = 1
AK09916_CNTL2_MODE_CONT1    = 2
AK09916_CNTL2_MODE_CONT2    = 4
AK09916_CNTL2_MODE_CONT3    = 6
AK09916_CNTL2_MODE_CONT4    = 8
AK09916_CNTL2_MODE_TEST     = 16
AK09916_CNTL3               = 0x32

#------------- Temperature -------------#
ICM20948_TEMPERATURE_SENSITIVITY    = 333.87    # defined in the electrical characteristics, pp. 14
ICM20948_TEMPERATURE_DEGREES_OFFSET = 21        # according to the formula on pp. 45
ICM20948_ROOM_TEMP_OFFSET           = 21        # room temperature, defined to be 21 deg Celsius, pp. 45

#--------------- Bitmasks --------------#
ICM20948_I2C_MST_EN_bm        = 1 <<  5         # USR0 USER_CTRL: i2c master enable, pp. 36
I2C_SLV0_RNW_bm               = 1 <<  7         # USR3 I2C_SLV0_ADDR: transfer is a read (not write), pp. 69


class ICM20948:

    def __init__(self, bus=1, address=ICM20948_ADDR, use_mag=False):
        self._address = address
        self._bus = SMBus(bus)
        self.use_mag = use_mag
        
        self.wake()
        if not self.read(ICM20948_USR0_WHO_AM_I) == ICM20948_CHIP_ID:
            raise RuntimeError("Unable to find ICM20948")

        self.set_gyro_sample_rate(100)
        self.set_gyro_low_pass(enabled=True, mode=5)
        self.set_gyro_full_scale(250)
        self.set_accel_sample_rate(125)
        self.set_accel_low_pass(enabled=True, mode=5)
        self.set_accel_scale(16)


        self.bank(0)
        self.write(ICM20948_USR0_INT_PIN_CFG, 0x00)             # interrupts disabled
        self.write(ICM20948_USR0_PWR_MGMT_2, 0x00)              # all devices on


        if self.use_mag:
            self.bank(3)
            self.write(ICM20948_USR3_I2C_MST_CTRL, 0x4D)
            self.write(ICM20948_USR3_I2C_MST_DELAY_CTRL, 0x01)

            if not self.mag_read(AK09916_WIA) == AK09916_CHIP_ID:
                raise RuntimeError("Unable to find AK09916")

            self.mag_write(AK09916_CNTL3, 0x01)                 # reset the magnetometer
            while self.mag_read(AK09916_CNTL3) == 0x01:
                time.sleep(0.0001)


    #----------------------------- I2C  Utilites -----------------------------#

    def write(self, register, data):
        self._bus.write_byte_data(self._address, register, data)
        time.sleep(0.0001)

    def read(self, register):
        return self._bus.read_byte_data(self._address, register)

    def read_bytes(self, register, num_bytes):
        return self._bus.read_i2c_block_data(self._address, register, num_bytes)

    def bank(self, bank_num):
        """ selects the user bank [0-3] """
        if not self.bank == bank_num:
            self.write(ICM20948_BANK_SEL, bank_num << 4)
            self._bank = bank_num
            time.sleep(0.01)

    def wake(self):
        self.bank(0)
        self.write(ICM20948_USR0_PWR_MGMT_1, 0x80)
        time.sleep(0.01)
        #self.write(ICM20948_USR0_PWR_MGMT_1, 0x01)
        self.write(ICM20948_USR0_PWR_MGMT_2, 0x00)

    def trigger_mag_io(self):
        """ allows magnetometer to become master on the auxiliary i2c bus to execute commands """
        user_ctrl = self.read(ICM20948_USR0_USER_CTRL)
        self.write(ICM20948_USR0_USER_CTRL, user_ctrl | ICM20948_I2C_MST_EN_bm)
        time.sleep(0.005)
        self.write(ICM20948_USR0_USER_CTRL, user_ctrl)

    def mag_write(self, register, data):
        self.bank(3)
        self.write(ICM20948_USR3_I2C_SLV0_ADDR, AK09916_I2C_ADDR)
        self.write(ICM20948_USR3_I2C_SLV0_REG, register)
        self.write(ICM20948_USR3_I2C_SLV0_DO, data)

        self.bank(0)
        self.trigger_mag_io()

    def mag_read(self, register):
        self.bank(3)
        self.write(ICM20948_USR3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_SLV0_RNW_bm)
        self.write(ICM20948_USR3_I2C_SLV0_REG, register)
        self.write(ICM20948_USR3_I2C_SLV0_DO, 0xFF)
        self.write(ICM20948_USR3_I2C_SLV0_CTRL, 0x81)

        self.bank(0)
        self.trigger_mag_io()
        return self.read(ICM20948_USR0_EXT_SLV_SENS_DATA_00)

    def mag_read_bytes(self, register, num_bytes):
        """ reads up to 24 bytes from the slave magnetometer """
        self.bank(3)
        self.write(ICM20948_USR3_I2C_SLV0_CTRL, 0x88 | num_bytes)
        self.write(ICM20948_USR3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_SLV0_RNW_bm)
        self.write(ICM20948_USR3_I2C_SLV0_REG, register)
        self.write(ICM20948_USR3_I2C_SLV0_DO, 0xFF)

        self.bank(0)
        self.trigger_mag_io()
        return self.read_bytes(ICM20948_USR0_EXT_SLV_SENS_DATA_00, num_bytes)


    #------------------------------ Magnetometer -----------------------------#

    def mag_ready(self):
        """ checks the magnetometer status self.ready bit """
        return self.mag_read(AK09916_ST1) & 0x01 > 0

    def get_mag_data(self, timeout=1.0):
        self.mag_write(AK09916_CNTL2, AK09916_CNTL2_MODE_SINGLE)

        t_start = time.time()
        while not self.mag_ready():
            if time.time() - t_start > timeout:
                raise RuntimeError("ERROR: Timeout waiting for Magnetometer Ready")
            time.sleep(0.00001)

        data = self.mag_read_bytes(AK09916_HXL, 6)

        mx, my, mz = struct.unpack("<hhh", bytearray(data))

        mx *= AK09916_SENSITIVITY_FACTOR
        my *= AK09916_SENSITIVITY_FACTOR
        mz *= AK09916_SENSITIVITY_FACTOR

        return mx, my, mz


    #----------------------------- Accelerometer -----------------------------#

    def set_accel_sample_rate(self, rate=125):
        """ sets the accelerometer sample rate in Hz """
        self.bank(2)
        rate = int((1125.0 / rate) - 1)             # 125Hz - 1.125 kHz / (1 + rate), pp. 63
        self.write(ICM20948_USR2_ACCEL_SMPLRT_DIV_1, (rate >> 8) & 0xff)
        self.write(ICM20948_USR2_ACCEL_SMPLRT_DIV_2, rate & 0xff)

    def set_accel_scale(self, scale=16):
        """ sets the accelerometer fulls cale range. the larger the scale, the lower the resolution. pp. 64 """
        self.bank(2)
        accel_config = self.read(ICM20948_USR2_ACCEL_CONFIG) & 0b11111001
        accel_config |= {2: 0b00, 4: 0b01, 8: 0b10, 16: 0b11}[scale] << 1
        self.write(ICM20948_USR2_ACCEL_CONFIG, accel_config)

    def set_accel_low_pass(self, enabled=True, mode=5):
        """ configures the accelerometer low pass filter """
        self.bank(2)
        accel_config = self.read(ICM20948_USR2_ACCEL_CONFIG) & 0b10001110
        if enabled:
            accel_config |= 0b1
        accel_config |= (mode & 0x07) << 4
        self.write(ICM20948_USR2_ACCEL_CONFIG, accel_config)

    def get_accel_data(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_USR0_ACCEL_XOUT_H, 6)
        ax, ay, az = struct.unpack(">hhh", bytearray(data))

        self.bank(2)
        scale = (self.read(ICM20948_USR2_ACCEL_CONFIG) & 0x06) >> 1     # read accelerometer full scale range
        gs = [16384.0, 8192.0, 4096.0, 2048.0][scale]                   # pp. 12

        ax /= gs
        ay /= gs
        az /= gs

        return ax, ay, az


    # ------------------------------- Gyroscope ------------------------------#

    def set_gyro_sample_rate(self, rate=100):
        """ sets the gyro sample rate in Hz """
        self.bank(2)
        rate = int((1100.0 / rate) - 1)             # 100Hz - 1.1 kHz / (1 + rate), pp.59
        self.write(ICM20948_USR2_GYRO_SMPLRT_DIV, rate)

    def set_gyro_full_scale(self, scale=250):
        """ sets the gyro full scale range. the larger the scale, the lower the resolution. 59 """
        self.bank(2)
        value = self.read(ICM20948_USR2_GYRO_CONFIG_1) & 0b11111001
        value |= {250: 0b00, 500: 0b01, 1000: 0b10, 2000: 0b11}[scale] << 1
        self.write(ICM20948_USR2_GYRO_CONFIG_1, value)

    def set_gyro_low_pass(self, enabled=True, mode=5):
        """configures the gyroscope low pass filter """
        self.bank(2)
        value = self.read(ICM20948_USR2_GYRO_CONFIG_1) & 0b10001110
        if enabled:
            value |= 0b1
        value |= (mode & 0x07) << 4
        self.write(ICM20948_USR2_GYRO_CONFIG_1, value)

    def get_gyro_data(self):
        self.bank(0)
        data = self.read_bytes(ICM20948_USR0_GYRO_XOUT_H, 6)
        gx, gy, gz = struct.unpack(">hhh", bytearray(data))

        self.bank(2)
        scale = (self.read(ICM20948_USR2_GYRO_CONFIG_1) & 0x06) >> 1    # read accelerometer full scale range
        dps = [131, 65.5, 32.8, 16.4][scale]                            # pp. 11

        gx /= dps
        gy /= dps
        gz /= dps

        return gx, gy, gz


    # ------------------------------ Temperature -----------------------------#

    def get_temperature(self):
        self.bank(0)
        raw_temperature_bytes = self.read_bytes(ICM20948_USR0_TEMP_OUT_H, 2)
        raw_temperature = struct.unpack('>h', bytearray(raw_temperature_bytes))[0]
        temperature = ((raw_temperature - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_TEMPERATURE_SENSITIVITY) + \
                      ICM20948_TEMPERATURE_DEGREES_OFFSET       # pp. 45
        return temperature


    # --------------------------------- Other --------------------------------#

    def get_all_data(self):
        ax, ay, az = self.get_accel_data()
        gx, gy, gz = self.get_gyro_data()
        temperature = self.get_temperature()

        if self.use_mag:
            mx, my, mz = self.get_mag_data()
            return ax, ay, az, gx, gy, gz, mx, my, mz, temperature
        else:
            return ax, ay, az, gx, gy, gz, temperature

    def get_power_mgmt(self, num):
        self.bank(0)
        register = 0
        if num == 1:
            register = ICM20948_USR0_PWR_MGMT_1
        elif num == 2:
            register = ICM20948_USR0_PWR_MGMT_2
        return self.read(register)