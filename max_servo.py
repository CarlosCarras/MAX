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

MG996R_MIN_PWM = 90
MG996R_MAX_PWM = 670
MG996R_MIN_PULSE = 1999.5
MG996R_MAX_PULSE = 2001.5
MG996R_MIN_ANGLE = 0.0
MG996R_MAX_ANGLE = 225

class MAXServo:

    def __init__(self, controller, channel, motor_type, stand_angle, rest_angle):
        self.channel = channel
        self.stand_angle = stand_angle
        self.rest_angle = rest_angle
        self.leg_type, self.joint_type = motor_type.split(' ')

        self.min_pwm = MG996R_MIN_PWM
        self.max_pwm = MG996R_MAX_PWM
        self.min_pulse = MG996R_MIN_PULSE
        self.max_pulse = MG996R_MAX_PULSE
        self.min_angle = MG996R_MIN_ANGLE
        self.max_angle = MG996R_MAX_ANGLE

        self.allowable_error = 1                # deg
        self.goal_pose = self.stand_angle       # deg
        self.current_pose = self.stand_angle    # deg
        self.joint_speed = 90.0                 # deg/sec
        self.speed_scalar = 0.01

        self.controller = controller
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
        slope = (self.max_pwm - self.min_pwm) / (self.max_angle - self.min_angle)
        intercept = self.max_pwm - slope * self.max_angle
        return round(self.current_pose*slope + intercept)

    def set_pwm(self, pwm=None):
        if not pwm:
            pwm = self.compute_pwm()
        self.controller.set_servo_pwm(self.channel, pwm)

    def set_goal(self, goal):
        self.goal_pose = goal

    def get_goal(self):
        return self.goal_pose

    def increment_goal(self, increment):
        self.goal_pose += increment

    def set_speed(self, speed):
        self.speed_scalar = speed

    def get_speed(self):
        return self.speed_scalar

    def stand(self):
        self.set_goal(self.stand_angle)

    def rest(self):
        self.set_goal(self.rest_angle)

    def clk(self, speed=None):
        if not speed:
            speed = self.speed_scalar

        if not self.goal_reached():
            print(speed)
            self.current_pose += self.get_direction() * self.joint_speed * speed
        else:
            self.current_pose = self.goal_pose

        self.set_pwm()

    def update(self):
        while not self.goal_reached():
            self.clk()
        self.clk()
