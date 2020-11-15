#!/usr/bin/env python3

"""
filename: calibrate_servos.py
author:   Carlos Carrasquillo
created:  November 9, 2020
modified: November 14, 2020
project:  MAX

purpose:  This file allows the user to test different angles for each of the servo channels.
"""

import max_servo
import pca9685

motors = ["FR Knee", "FR Hip", "FR AB/AD",
          "FL Knee", "FL Hip", "FL AB/AD",
          "RR Knee", "RR Hip", "RR AB/AD",
          "RL Knee", "RL Hip", "RL AB/AD"]

controller = pca9685.PCA9685()

def set_angle(channel):
    print("---------------- Tuning the", motors[channel - 1], "angle. -----------------")

    motor_type = motors[channel]
    servo = max_servo.MAXServo(channel, motor_type)

    while True:
        command = input("Select an angle [0 -180]: ")
        if command == "q":
            print('\n')
            return

        angle = float(command)
        if angle < max_servo.MG996R_MIN_ANGLE or angle > max_servo.MG996R_MAX_ANGLE:
            print("Error: Invalid Servo Angle.")
        else:
            servo.set_goal(angle)
            servo.update()
            print("Test Worked!")


def calibrate():
    print('-------------------------------------------------------------')
    print('                  Entering Calibration Mode                  ')
    print('-------------------------------------------------------------\n')

    while True:
        command = input("Enter Servo Channel [1-16]: ")
        if command == "q":
            break

        channel = int(command)
        if channel < 1 or channel > 16:
            print("Error: Invalid Servo Channel.")
        else:
            set_angle(channel)


