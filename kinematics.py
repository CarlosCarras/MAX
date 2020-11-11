#!/usr/bin/env python3

"""
filename: kinematics.py
author:   Carlos Carrasquillo
created:  October 24, 2020
modified: October 24, 2020
project:  MAX

purpose:  Solves the inverse kinematics for each of the robot's legs.

reference paper: https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Rob

nomenclature:
    - L[1]: length of side swing joint (i.e. length of shoulder)
    - L[2]: length of hip joint (i.e. length of elbow/upper leg)
    - L[3]: length of knee joint (i.e. length of wrist/lower leg)
    - [x1, y1, z1]: coordinate system of the side swing joint
    - [x2, y2, z2]: coordinate system of the hip joint
    - [x3, y3, z3]: coordinate system of the knee joint
    - [x4, y4, z4]: coordinate system of the endpoint of leg
    - euler[1]: roll of robot
    - euler[2]: pitch of robot
    - euler[3]: yaw of robot
    - theta[1]: angle of side swing joint
    - theta[2]: angle of hip joint
    - theta[3]: angle of knee joint

NOTE: the coordinate system used in the attached article is different. the ground coordinate system for this project is:
    - Z: up
    - Y: into the page
    - X: Y cross Z
"""

import numpy as np


class Kinematics:

    def __init__(self, sideswing_length, hip_length, knee_length):
        self.L = [0, sideswing_length, hip_length, knee_length]
        self.theta = [0.0, 0.0, 0.0, 0.0]

    def get_domain(self, x4, y4, z4):
        D = (x4 ** 2 + y4 ** 2 - self.L[1] ** 2 + z4 ** 2 - self.L[2] ** 2 - self.L[3] ** 2) / (
                    2 * self.L[2] * self.L[3])
        return np.clip(D, -1, 1)

    def right_IK(self, x4, y4, z4):
        D = self.get_domain(x4, y4, z4)
        self.theta[3] = np.arctan2(-np.sqrt(1.0 - D ** 2), D)
        self.theta[2] = np.arctan2(-x4, np.sqrt(y4 ** 2 - z4 ** 2 - self.L[1] ** 2)) - \
                        np.arctan2(self.L[3] * np.sin(self.theta[3]), self.L[2] + self.L[3] * np.cos(self.theta[3]))
        self.theta[1] = -np.arctan2(z4, y4) - np.arctan2(np.sqrt(y4 ** 2 - z4 ** 2 - self.L[1] ** 2), -self.L[1])

    def left_IK(self, x4, y4, z4):
        D = self.get_domain(x4, y4, z4)
        self.theta[3] = np.arctan2(-np.sqrt(1.0 - D ** 2), D)
        self.theta[2] = np.arctan2(-x4, np.sqrt(y4 ** 2 - z4 ** 2 - self.L[1] ** 2)) - \
                        np.arctan2(self.L[3] * np.sin(self.theta[3]), self.L[2] + self.L[3] * np.cos(self.theta[3]))
        self.theta[1] = -np.arctan2(z4, y4) - np.arctan2(np.sqrt(y4 ** 2 - z4 ** 2 - self.L[1] ** 2), self.L[1])

    def solve_IK(self, x4, y4, z4, leg):
        if leg == "FR" or leg == "RR":
            self.right_IK(x4, y4, z4)
        elif leg == "FL" or leg == "RL":
            self.left_IK(x4, y4, z4)

        # correcting the direction of the angle
        self.theta[2] = -self.theta[2]
        self.theta[3] = -self.theta[3]

        return [self.theta[1], self.theta[2], self.theta[3]]
