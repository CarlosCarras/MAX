#!/usr/bin/env python3

"""
filename: motor_control.py
author:   Carlos Carrasquillo
created:  November 22, 2020
modified: November 22, 2020
project:  MAX

purpose:  This file oversees the operation of all of the servo motors in MAX.
"""

import pca9685
from max_servo import MAXServo


class Controller:
    motors = ["FR Knee", "FR Hip", "FR AB/AD",
              "FL Knee", "FL Hip", "FL AB/AD",
              "RR Knee", "RR Hip", "RR AB/AD",
              "RL Knee", "RL Hip", "RL AB/AD"]

    stand_angles = [120.0, 120.0, 120.0,
                    120.0, 120.0, 120.0,
                    120.0, 120.0, 120.0,
                    120.0, 120.0, 120.0]

    rest_angles = [225.0, 225.0, 225.0,
                   225.0, 225.0, 225.0,
                   225.0, 225.0, 225.0,
                   225.0, 225.0, 225.0]

    def __init__(self):
        self.pwm = pca9685.PCA9685()
        self.servos = []
        self.num_servos = 0

    def attach_servo(self, stand_angle=120, rest_angle=120, channel=None):
        if not channel:
            channel = self.num_servos
        elif channel > self.num_servos:
            print("Error: The servo channel exceeds the current number of servos.")
            return

        self.servos.insert(channel, MAXServo(self.pwm, channel, self.motors[channel], stand_angle, rest_angle))
        self.num_servos += 1
        out_str = "Succesfully added the " + self.motors[channel] + " servo!"
        print(out_str)
        return self.servos[channel]

    def get_num_servos(self):
        return self.num_servos

    def actuate(self, channel, pulse):
        self.pwm.set_pwm(channel, 0, pulse)

    def clk(self):
        for i in range(self.num_servos):
            self.servos[i].clk()

    def update(self):
        for i in range(self.num_servos):
            self.servos[i].update()

    def stand(self):
        for i in range(self.num_servos):
            self.servos[i].stand()
        self.update()

    def rest(self):
        for i in range(self.num_servos):
            self.servos[i].rest()
        self.update()