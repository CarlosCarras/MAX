#!/usr/bin/env python3

"""
filename: max_health.py
author:   Carlos Carrasquillo
created:  Dececember 3, 2020
modified: Dececember 3, 2020
project:  MAX
purpose:  This file acts as a failsafe for overheating conditions.

hardware: AMG8833, ICM20948
datasheets: SPECIFICATIONS FOR Infrared Array Sensor, Invensense DS-000189, Rev. 1.3
"""

IMU_MAX_TEMP = 85
CAM_MAX_TEMP = 80

class HealthCheck():
    def __init__(self, imu, camera):
        self.imu = imu
        self.camera = camera

    def status(self):
        imu_temp = self.imu.get_temperature()
        camera_temp = self.camera.get_temperature()

        print('IMU Temperature:' + str(imu_temp))
        print('Camera Temperature:' + str(camera_temp))

        if imu_temp > IMU_MAX_TEMP:
            print('Initiating Shutdown Procedure:  The ICM20948 Temperature Exceeded the Allowable Limit.')
            self.send_distress()
        if camera_temp  > CAM_MAX_TEMP:
            print('Initiating Shutdown Procedure:  The AMG8833 Temperature Exceeded the Allowable Limit.')
            self.send_distress()

    def send_distress(self):
        print('Sending an SOS Signal.')


