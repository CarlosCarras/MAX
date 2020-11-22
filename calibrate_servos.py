#!/usr/bin/env python3

"""
filename: calibrate_servos.py
author:   Carlos Carrasquillo
created:  November 9, 2020
modified: November 22, 2020
project:  MAX

purpose:  This file allows the user to test different angles for each of the servo channels.
"""

import max_servo


def set_pulse(controller, channel):
    print("---------------- Tuning the", controller.motors[channel], "angle. -----------------")

    while True:
        out_str = "Select a pulse [" + str(max_servo.MG996R_MIN_ANGLE) + ", " + str(max_servo.MG996R_MAX_ANGLE) + "]: "
        command = input(out_str)
        if command == "q":
            print('\n')
            return

        angle = float(command)
        if angle < max_servo.MG996R_MIN_ANGLE or angle > max_servo.MG996R_MAX_ANGLE:
            print("Error: Invalid Servo Angle.")
        else:
            controller.servos[channel].set_goal(angle)
            controller.servos[channel].update()


def calibrate(controller):
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
            set_pulse(controller, channel)


