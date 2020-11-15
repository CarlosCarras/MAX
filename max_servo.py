#!/usr/bin/env python3

"""
filename: max_servo.py
author:   Carlos Carrasquillo
created:  October 23, 2020
modified: November 14, 2020
project:  MAX

purpose:  This file reads accelerometer and gyroscope data from the Invensense MPU6050 IMU over I2C.

datasheet: https://components101.com/motors/mg996r-servo-motor-datasheet
"""

import pca9685

MG996R_MIN_PULSE = 1500
MG996R_MAX_PULSE = 2500
MG996R_MIN_ANGLE = 0.0
MG996R_MAX_ANGLE = 180.0


class MAXServo:

    def __init__(self, channel, type, stand_angle=60.0, rest_angle=60.0):
        self.channel = channel
        self.stand_angle = stand_angle
        self.rest_angle = rest_angle
        self.leg_type = type.split(' ')[0]
        self.joint_type = type.split(' ')[1]

        self.min_pulse = MG996R_MIN_PULSE
        self.max_pulse = MG996R_MAX_PULSE
        self.min_angle = MG996R_MIN_ANGLE
        self.max_angle = MG996R_MAX_ANGLE

        self.allowable_error = 1                # deg
        self.goal_pose = self.stand_angle       # deg
        self.current_pose = self.stand_angle    # deg
        self.desired_speed = 0.0                # deg/sec
        self.delay = 0.001                      # seconds

        self.controller = pca9685.PCA9685()
        self.set_pwm()

    def get_channel(self):
        return self.channel

    def get_legtype(self):
        return self.leg_type

    def get_jointtype(self):
        return self.joint_type

    def goal_reached(self):
        return abs(self.current_pose - self.goal_pose) < self.allowable_error

    def get_direction(self):
        if self.goal_pose - self.current_pose > 0:
            return 1.0
        else:
            return -1.0

    def compute_pwm(self):
        slope = (self.max_pulse - self.min_pulse) / (self.max_angle - self.min_angle)
        intercept = self.max_pulse - slope * self.max_angle
        return round(self.current_pose*slope + intercept)

    def set_pwm(self, pwm = None):
        if not pwm:
            pwm = self.compute_pwm()
        self.controller.set_servo_pwm(self.channel, pwm)

    def set_goal(self, goal):
        self.goal_pose = goal

    def get_goal(self):
        return self.goal_pose

    def update(self):
        if not self.goal_reached():
            self.current_pose += self.get_direction() * self.delay * self.desired_speed
        else:
            self.current_pose = self.goal_pose
        self.set_pwm()

