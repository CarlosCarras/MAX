from __future__ import division
import time
from threading import Timer


GAIT_TROT = 0x00
STEP_FORWARD_RIGHT = 0
STEP_FORWARD_LEFT = 1

LEG_FR = 0
LEG_FL = 1
LEG_RR = 2
LEG_RL = 3

LIFT_LEGS = [0, 0, 0, -20, 20, 0, 0, 0, 0, 0, 0, 0]
SWING_LEGS = [0, 0, 0, 0, -50, 0, 0, 0, 0, 0, 0, 0]

BOW = [0, 0, 0, -20, 20, 0, 0, 0, 0, 0, 0, 0]
BODY_ROLL = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class GaitPlanner():
    def __init__(self, controller, imu=None, gait=GAIT_TROT, wait_time=0.5):
        self.controller = controller
        self.imu = imu
        self.gait = gait
        self.last_step = None
        self.step_time = 0
        self.wait_time = wait_time      # in seconds

        self.kp = 0.01


    def set_gait(self, gait):
        if gait is GAIT_TROT:
            self.gait = GAIT_TROT


    def update_legs(self, change, legs):
        for i in range(len(change)):
            if not (i//3 in legs):
                print(change[i])
                change[i] = 0

        self.controller.change_pose(change)
        self.controller.update()


    def raise_legs(self, legs):
        self.update_legs(LIFT_LEGS, legs)


    def swing_legs(self, legs):
        self.update_legs(SWING_LEGS, legs)


    def lower_legs(self, legs):
        change = [LIFT_LEGS[i]*-1 for i in LIFT_LEGS]
        self.update_legs(change, legs)
        self.step_time = time.time()


    def restore_legs(self, legs):
        change = [SWING_LEGS[i] * -1 for i in SWING_LEGS]
        self.update_legs(change, legs)


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return
        self.last_step = not self.last_step

        if self.gait is GAIT_TROT:
            if self.last_step == STEP_FORWARD_RIGHT:
                legs = [LEG_FL, LEG_RR]
            else:
                legs = [LEG_FR, LEG_RL]
        else:
            legs = []

        self.raise_legs(legs)
        time.sleep(0.2)
        self.swing_legs(legs)
        time.sleep(0.2)
        self.lower_legs(legs)
        time.sleep(1)
        self.restore_legs(legs)
        #Timer(0.5, self.lower_legs, legs).start()























    def dance(self, dur):
        start = time.time()
        while time.time() - start < dur:
            self.step()
            time.sleep(0.5)

    def stretch(self, hold, speed=None):
        if speed:
            original_speed = self.controller.get_speed()
            self.controller.set_speed(speed)

        original_pose = self.controller.get_pose()
        self.controller.set_pose(BOW)
        self.controller.update()
        time.sleep(hold)
        self.controller.set_pose(original_pose)
        self.controller.update()
        time.sleep(1.25)

        if speed:
            self.controller.set_speed(original_speed)

    def correct(self):
        roll, pitch, yaw = self.imu.get_rpy()

        roll_correction = [roll*self.kp*-1*i for i in BODY_ROLL]
        self.controller.change_pose(roll_correction)


