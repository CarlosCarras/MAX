#!/usr/bin/env python3

"""
filename: mpu6050.py
author:   Carlos Carrasquillo
created:  October 23, 2020
modified: October 24, 2020
project:  MAX

purpose:  This file reads accelerometer and gyroscope data from the Invensense MPU6050 IMU over I2C.

datasheet: Invensense RM-MPU-6000A-00, Rev. 4

discloure: This code was modeled after Robocraze's mpu6050.py file, which can be found at,
https://github.com/Robocraze/MPU6050/blob/master/mpu6050.py
"""

from smbus2 import SMBus


class MPU6050:

    MPU6050_ADDR = 0b1101000
    C_GRAVITY = 9.80665

    #--------------------------- MPU6050 Registers ---------------------------#
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F
    TEMP_OUT0 = 0x41
    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    #----------------------------- Sensor Ranges -----------------------------#
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18
    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    #---------------------------- Scale Modifiers ----------------------------#

    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4


    def __init__(self, bus=1, address=MPU6050_ADDR):
        self._address = address
        self._bus = SMBus(bus)

        self.accel_range = self.ACCEL_RANGE_8G                          # setting a default accelerometer range of 8G
        self.set_accel_range(self.accel_range)
        self.gyro_range = self.GYRO_RANGE_250DEG                        # setting a default gyroscope range of 250DEG
        self.set_gyro_range(self.gyro_range)

        self._bus.write_byte_data(self._address, self.PWR_MGMT_1, 0x00) # wakes up device from sleep mode (pp. 9)


    #----------------------------- I2C  Utilites -----------------------------#

    def read_i2c_word(self, register):
        # reads the high and low bytes from consecutive registers
        high = self._bus.read_byte_data(self._address, register)
        low  = self._bus.read_byte_data(self._address, register+1)
        word = (high << 8) + low

        if word >= 0x8000:                                              # accounts for negative numbers
            return -((65535 - word) + 1)
        return word


    #----------------------------- Accelerometer -----------------------------#

    def set_accel_range(self, accel_range):
        if accel_range == 2:
            set_accel_range = self.ACCEL_RANGE_2G
        elif accel_range == 4:
            set_accel_range = self.ACCEL_RANGE_4G
        elif accel_range == 8:
            set_accel_range = self.ACCEL_RANGE_8G
        elif accel_range == 16:
            set_accel_range = self.ACCEL_RANGE_16G
        else:
            return -1

        self.accel_range = accel_range
        self._bus.write_byte_data(self._address, self.ACCEL_CONFIG, set_accel_range)

    def get_accel_range(self, read=False):
        if not read:
            return self.accel_range

        reading = self._bus.read_byte_data(self._address, self.ACCEL_CONFIG)

        if reading == self.ACCEL_RANGE_2G:
            return 2
        elif reading == self.ACCEL_RANGE_4G:
            return 4
        elif reading == self.ACCEL_RANGE_8G:
            return 8
        elif reading == self.ACCEL_RANGE_16G:
            return 16
        else:
            return -1

    def get_accel_scale_mod(self):
        accel_range = self.get_accel_range()

        if accel_range == 2:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == 4:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == 8:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == 16:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            return -1
        return accel_scale_modifier

    def get_accel_data(self):
        ax = self.read_i2c_word(self.ACCEL_XOUT0)
        ay = self.read_i2c_word(self.ACCEL_YOUT0)
        az = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier  = self.get_accel_scale_mod()

        ax = ax * self.C_GRAVITY / accel_scale_modifier
        ay = ay * self.C_GRAVITY / accel_scale_modifier
        az = az * self.C_GRAVITY / accel_scale_modifier

        return {'x': ax, 'y': ay, 'z': az}


    # ------------------------------- Gyroscope ------------------------------#

    def set_gyro_range(self, gyro_range):
        if gyro_range == 2:
            set_gyro_range = self.GYRO_RANGE_250DEG
        elif gyro_range == 4:
            set_gyro_range = self.GYRO_RANGE_500DEG
        elif gyro_range == 8:
            set_gyro_range = self.GYRO_RANGE_1000DEG
        elif gyro_range == 16:
            set_gyro_range = self.GYRO_RANGE_2000DEG
        else:
            return -1

        self.gyro_range = gyro_range
        self._bus.write_byte_data(self._address, self.ACCEL_CONFIG, set_gyro_range)

    def get_gyro_range(self, read=False):
        if not read:
           return self.gyro_range

        reading = self._bus.read_byte_data(self._address, self.GYRO_CONFIG)

        if reading == self.GYRO_RANGE_250DEG:
            return 250
        elif reading == self.GYRO_RANGE_500DEG:
            return 500
        elif reading == self.GYRO_RANGE_1000DEG:
            return 1000
        elif reading == self.GYRO_RANGE_2000DEG:
            return 2000
        else:
            return -1

    def get_gyro_scale_mod(self):
        gyro_range = self.get_gyro_range()

        if gyro_range == 250:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == 500:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == 1000:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == 2000:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            return -1
        return gyro_scale_modifier

    def get_gyro_data(self):
        gx = self.read_i2c_word(self.GYRO_XOUT0)
        gy = self.read_i2c_word(self.GYRO_YOUT0)
        gz = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier  = self.get_gyro_scale_mod()

        gx = gx / gyro_scale_modifier
        gy = gy / gyro_scale_modifier
        gz = gz / gyro_scale_modifier

        return {'x': gx, 'y': gy, 'z': gz}


    # ------------------------------ Temperature -----------------------------#

    def get_temp(self):
        temperature = self.read_i2c_word(self.TEMP_OUT0)
        return temperature / 340.0 + 36.53


    # --------------------------------- Other --------------------------------#

    def get_all_data(self):
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        temp = self.get_temp()

        return [accel, gyro, temp]
