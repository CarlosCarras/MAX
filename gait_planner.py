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

TROT = [[155.0,  35.0, 110.0, 45.0, 200.0,  55.0, 160.0,  25.0, 145.0, 120.0, 160.0,  20.0],
        [155.0,  35.0, 110.0, 45.0, 150.0,  55.0, 160.0,  75.0, 145.0, 120.0, 160.0,  20.0],
        [155.0,  35.0, 110.0, 75.0, 150.0,  55.0, 130.0,  75.0, 145.0, 120.0, 160.0,  20.0],
        [175.0,  15.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 100.0, 180.0,  20.0],
        [175.0,  65.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 100.0, 130.0,  20.0],
        [145.0,  65.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 130.0, 130.0,  20.0],
        [155.0,  35.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 120.0, 160.0,  20.0]]

TROT_SPEED = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

BOW = [0, 0, 0, -20, 20, 0, 0, 0, 0, 0, 0, 0]
BODY_ROLL = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class GaitPlanner:
    def __init__(self, controller, imu=None, gait=GAIT_TROT, wait_time=0.5):
        self.controller = controller
        self.imu = imu
        self.gait = gait
        self.step_time = 0
        self.wait_time = wait_time      # in seconds

        self.kp = 0.01


    def set_gait(self, gait):
        if gait is GAIT_TROT:
            self.gait = GAIT_TROT


    def trot(self):
        for i in range(len(TROT)):
            self.controller.set_pose(TROT[i])
            self.controller.set_speed(TROT_SPEED[i])
            self.controller.update()
            time.sleep(0.15)


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.gait is GAIT_TROT:
            self.trot()
        else:
            return
        #Timer(0.5, self.lower_legs, legs).start()

    def walk_forward(self, dur):
        self.stand()
        time.sleep(1)

        start = time.time()
        while time.time() - start < dur:
            self.step()

    def stand(self):
        self.controller.stand()
        self.controller.update_simultaneously()

    def rest(self):
        self.controller.rest()
        self.controller.update_simultaneously()




















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


