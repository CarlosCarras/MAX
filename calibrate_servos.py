#!/usr/bin/env python3

"""
filename: calibrate_servos.py
author:   Carlos Carrasquillo
created:  November 9, 2020
modified: November 14, 2020
project:  MAX

purpose:  This file allows the user to test different angles for each of the servo channels.
"""

import pca9685
import max_servo

motors = ["FR Knee", "FR Hip", "FR AB/AD",
          "FL Knee", "FL Hip", "FL AB/AD",
          "RR Knee", "RR Hip", "RR AB/AD",
          "RL Knee", "RL Hip", "RL AB/AD"]

def set_pulse(channel):
    print("---------------- Tuning the", motors[channel - 1], "angle. -----------------")

    motor_type = motors[channel]
    controller = pca9685.PCA9685()

    while True:
        out_str = "Select a pulse [" + str(max_servo.MG996R_MIN_PULSE) + ", " + str(max_servo.MG996R_MAX_PULSE) + "]: "
        command = input(out_str)
        if command == "q":
            print('\n')
            return

        pulse = float(command)
        if pulse < max_servo.MG996R_MIN_PULSE or pulse > max_servo.MG996R_MAX_PULSE:
            print("Error: Invalid Servo PWM.")
        else:
            #servo.set_goal(angle)
            controller.set_servo_pulse(channel, pulse)


def calibrate():
    print('-------------------------------------------------------------')
    print('                  Entering Calibration Mode                  ')
    print('-------------------------------------------------------------\n')

    while True:
        command = input("Enter Servo Channel [0-15]: ")
        if command == "q":
            break

        channel = int(command)
        if channel < 0 or channel > 15:
            print("Error: Invalid Servo Channel.")
        else:
            set_pulse(channel)


