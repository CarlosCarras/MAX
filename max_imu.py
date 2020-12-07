#!/usr/bin/env python3

"""
filename: max_imu.py
author:   Carlos Carrasquillo
created:  October 23, 2020
modified: October 24, 2020
project:  MAX

purpose:  This file stores the IMU class, which can be used to perform useful operations with the MPU6050 IMU.
discloure: This code was modeled after Maurice Rahme's max_imu.py file, which can be found at,
https://github.com/OpenQuadruped/spot_mini_mini/blob/spot/spot_real/Control/RPi/lib/max_imu.py
"""

import math
import numpy as np
import time
import icm20948_imu


class IMU:

    def __init__(self, bus=1):
        self._bus = bus
        self.imu = icm20948_imu.ICM20948(bus)

        self.imu_dt = 0
        self.roll_int = 0
        self.pitch_int = 0
        self.yaw_int = 0
        self.prev_time = 0
        self.comp_filter = 0.02                # complementary filter coefficient

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.gx_calibration = 0
        self.gy_calibration = 0
        self.gz_calibration = 0
        self.roll_calibration = 0
        self.pitch_calibration = 0
        self.yaw_calibration = 0

        self.scale_x = 1
        self.scale_y = 1
        self.scale_z = 1
        self.mag_x_bias = 0
        self.mag_y_bias = 0
        self.mag_z_bias = 0
        self.yaw_bias = 0

        self.calibrate_imu()
        #self.calibrate_magnetometer()

        # Software In the Loop:
        self.roll_cntr = 0
        self.pitch_cntr = 0


    def get_temperature(self):
        return self.imu.get_temperature()

    def calibrate_imu(self):
        self.gx_calibration = 0
        self.gy_calibration = 0
        self.gz_calibration = 0
        self.roll_calibration = 0
        self.pitch_calibration = 0
        gx_sum = gy_sum = gz_sum = 0
        roll_sum = pitch_sum = 0                # not using yaw for now

        num_calibrations = 100
        for i in range(num_calibrations):
            ax, ay, az = self.imu.get_accel_data()
            gx, gy, gz = self.imu.get_gyro_data()

            gx_sum += gx
            gy_sum += gy
            gz_sum += gz

            # accelerometer data can be used to compute the roll, pitch, and yaw.
            roll_sum += 180.0 * math.atan2(ay, az) / np.pi      # Y and Z
            pitch_sum += 180.0 * math.atan2(ax, az) / np.pi     # X and Z

        self.gx_calibration = gx_sum / float(num_calibrations)
        self.gy_calibration = gy_sum / float(num_calibrations)
        self.gz_calibration = gy_sum / float(num_calibrations)
        self.roll_calibration = roll_sum / float(num_calibrations)
        self.pitch_calibration = pitch_sum / float(num_calibrations)
        self.pitch_calibration = pitch_sum / float(num_calibrations)

        print("Gyroscope and Accelerometer calibration complete.\n")

    def calibrate_magnetometer(self):
        collection_time = 5.0               # get 10 seconds of magnemometer data
        mag_x_max = mag_y_max = mag_z_max = -icm20948_imu.AK09916_RESOLUTION
        mag_x_min = mag_y_min = mag_z_min = icm20948_imu.AK09916_RESOLUTION

        input("Starting magnetometer calibration for " + str(collection_time) + " seconds.")
        start_time = time.time()
        while time.time() - start_time < collection_time:
            mx, my, mz = self.imu.get_mag_data()

            if mx > mag_x_max:
                mag_x_max = mx              # setting the maximum mx
            if mx < mag_x_min:
                mag_x_min = mx              # setting the minimum mx

            if my > mag_y_max:
                mag_y_max = my              # setting the maximum my
            if my < mag_y_min:
                mag_y_min = my              # setting the minimum my

            if mz > mag_z_max:
                mag_z_max = mz              # setting the maximum mz
            if mz < mag_z_min:
                mag_z_min = mz              # setting the minimum mz

        self.mag_x_bias = (mag_x_max + mag_x_min) / 2.0     # computing the bias
        self.mag_y_bias = (mag_y_max + mag_y_min) / 2.0
        self.mag_z_bias = (mag_z_max + mag_z_min) / 2.0

        # soft iron distortion - scale biases method   (https://appelsiini.net/2018/calibrate-magnetometer)
        mag_x_delta = (mag_x_max - mag_x_min) / 2.0
        mag_y_delta = (mag_y_max - mag_y_min) / 2.0
        mag_z_delta = (mag_z_max - mag_z_min) / 2.0
        avg_delta = (mag_x_delta + mag_y_delta + mag_z_delta) / 3.0

        self.scale_x = avg_delta / mag_x_delta
        self.scale_y = avg_delta / mag_y_delta
        self.scale_y = avg_delta / mag_z_delta

        # for each axis: corrected_reading = (reading - bias) * scale
        print("Magnetometer calibration complete.\n")

    def get_rpy(self):
        ax, ay, az = self.imu.get_accel_data()
        gx, gy, gz = self.imu.get_gyro_data()
        #mx, my, mz = self.imu.get_mag_data()

        current_time = time.time()                          # get current time (s)
        self.imu_dt = current_time - self.prev_time         # compute dt
        self.prev_time = current_time

        if self.imu_dt < 0:                                 # catch rollovers
            self.imu_dt= 0

        roll_gyro_dt = gx * self.imu_dt
        pitch_gyro_dt = gy * self.imu_dt
        #yaw_gyro_dt = gz * self.imu_dt

        #self.roll_int += roll_gyro_dt
        #self.pitch_int += pitch_gyro_dt
        #self.yaw_int += yaw_gyro_dt

        roll_accel  = 180.0 * math.atan2(ay, ax) / np.pi - self.roll_calibration        # acclerometer Y, Z
        pitch_accel = 180.0 * math.atan2(ax, az) / np.pi - self.pitch_calibration       # acclerometer X, Z
        #yaw_mag = 180.0 * (math.atan2(my, mx) / np.pi)                                 # magnetometer Y, X

        # calculate filtered Roll & Pitch (RP) data
        self.roll = roll_accel*self.comp_filter + (1-self.comp_filter)*(roll_gyro_dt + self.roll)
        self.pitch = pitch_accel*self.comp_filter + (1-self.comp_filter)*(pitch_gyro_dt + self.pitch)
        #self.yaw = yaw_mag - self.yaw_bias
        #self.yaw -= 360.0 * math.floor((self.yaw + 180) / 360.0)

        return self.roll, self.pitch, self.yaw

    def get_power_status(self):
        data = self.imu.get_power_mgmt(1)
        print("USER0 Power Management 1 Data: " + hex(data))

        clksel = data & 0b111
        temp_dis = data & (1 <<  4)
        sleep = data & (1 << 6)
        reset = data & (1 << 7)

        print("\tCLKSEL: ", end='')
        if clksel> 0 and clksel < 6: print("Auto.")
        if clksel == 7: print("The clock is stopped.")
        else: print("Internal 20 MHz oscillator.")

        print("\tTEMP: ", end='')
        if temp_dis: print("Disabled.")
        else: print("Enabled.")

        print("\tSLEEP: ", end='')
        if sleep: print("On.")
        else: print("Off.")

        print("\tRESET: ", end='')
        if reset: print("Resetting.")
        else: print("False.")


    def get_device_status(self):
        data = self.imu.get_power_mgmt(2)

        bm = 0b111
        gyro = data & bm
        accel = (data >> 3) & bm

        if accel == bm: print("Accelerometer (all axes) disabled.")
        elif accel == 0b000: print("Accelerometer (all axes) enabled.")
        else: print("ERROR with the accelerometer configuration: 0b" + "{0:b}".format(accel))

        if gyro == bm: print("Gyroscope (all axes) disabled.")
        elif gyro == 0b000: print("Gyroscope (all axes) enabled.")
        else: print("ERROR with the gyroscope configuration: 0b" + "{0:b}".format(gyro))

        print('\n')


    def test(self, dur=30):
        start_time = time.time()

        while time.time() - start_time < dur:
            ax, ay, az = self.imu.get_accel_data()
            gx, gy, gz = self.imu.get_gyro_data()
            print('AX: ' + str(ax) + ', AY: ' + str(ay) + ', AZ: ' + str(az))
            print('GX: ' + str(gx) + ', GY: ' + str(gy) + ', GZ: ' + str(gz))
            self.get_power_status()
            self.get_device_status()
            time.sleep(1)

    def test_get_roll_data(self):
        self.roll_cntr += 0.001
        return np.sin(self.roll_cntr)

    def test_get_pitch_data(self):
        self.pitch_cntr += 0.001
        return np.sin(self.pitch_cntr)

    def test_software_in_loop_roll(self, dur=20):
        start_time = time.time()
        while time.time() - start_time < dur/2:
            roll, pitch, yaw = [self.test_get_roll_data(), 0, 0]
        return roll, pitch, yaw

    def test_software_in_loop_pitch(self, dur=20):
        start_time = time.time()
        while time.time() - start_time < dur/2:
            roll, pitch, yaw = [0, self.test_get_pitch_data(), 0]
        return roll, pitch, yaw







