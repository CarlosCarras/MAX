#!/usr/bin/env python3

"""
filename: calibrate_servos.py
author:   Carlos Carrasquillo
created:  November 9, 2020
modified: November 22, 2020
project:  MAX

purpose:  This file allows the user to test different angles for each of the servo channels.
"""

import motor_control
import max_servo


def set_pulse(controller, channel):
    print("---------------- Tuning the", controller.motors[channel], "angle. -----------------")

    motor_type = controller.motors[channel]

    while True:
        out_str = "Select a pulse [" + str(max_servo.MG996R_MIN_PWM) + ", " + str(max_servo.MG996R_MAX_PWM) + "]: "
        command = input(out_str)
        if command == "q":
            print('\n')
            return

        pulse = float(command)
        if pulse < max_servo.MG996R_MIN_PWM or pulse > max_servo.MG996R_MAX_PWM:
            print("Error: Invalid Servo PWM.")
        else:
            #servo.set_goal(angle)
            controller.pwm.set_servo_pwm(channel, pwm)


def calibrate():
    print('-------------------------------------------------------------')
    print('                  Entering Calibration Mode                  ')
    print('-------------------------------------------------------------\n')

    controller = motor_control.Controller()

    while True:
        command = input("Enter Servo Channel [0-15]: ")
        if command == "q":
            break

        channel = int(command)
        if channel < 0 or channel > 15:
            print("Error: Invalid Servo Channel.")
        else:
            set_pulse(controller=controller, channel=channel)


