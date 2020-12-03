from __future__ import division
import time
from threading import Timer


GAIT_TROT = 0x00

LEG_FR = 0
LEG_FL = 1
LEG_RR = 2
LEG_RL = 3

TROT = [[155.0,  35.0, 110.0, 45.0, 200.0,  55.0, 160.0,  25.0, 145.0, 120.0, 160.0,  20.0],
        [155.0,  35.0, 110.0, 45.0, 150.0,  55.0, 160.0,  75.0, 145.0, 120.0, 160.0,  20.0],
        [155.0,  35.0, 110.0, 75.0, 150.0,  55.0, 130.0,  75.0, 145.0, 120.0, 160.0,  20.0],
        [175.0,  15.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 100.0, 180.0,  20.0],
        [175.0,  65.0, 110.0, 65.0, 180.0,  55.0, 140.0,  45.0, 145.0, 100.0, 130.0,  20.0],
        [155.0,  65.0, 110.0, 45.0, 200.0,  55.0, 160.0,  25.0, 145.0, 120.0, 130.0,  20.0]]

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


    def execute_step(self, steps, speed=None, increment=False, sleep_dur=0):
        if increment:
            self.controller.change_pose(steps)
        else:
            self.controller.set_pose(steps)
        if speed: self.controller.set_speed(speed)
        self.controller.update()
        time.sleep(sleep_dur)


    def trot(self):
        for row in TROT:
            self.execute_step(row, TROT_SPEED)


    def raise_leg(self, leg, angle=20, speed=None, sleep_dur=0):
        step = [0] * 12
        step[leg*3] = angle*(1 - 2*(leg%2))
        step[leg*3+1] = -1 * step[leg*3]
        self.execute_step(steps=step, speed=speed, increment=True, sleep_dur=sleep_dur)


    def swing_out(self, leg, angle=20, speed=None, sleep_dur=0):
        step = [0] * 12
        if leg == 1 or leg == 2:
            step[leg*3+2] = angle
        elif leg == 0 or leg == 3:
            step[leg*3+2] = -angle
        self.execute_step(steps=step, speed=speed, increment=True, sleep_dur=sleep_dur)


    def swing_in(self, leg, angle=20, speed=None, sleep_dur=0):
        self.swing_out(leg, -angle, speed, sleep_dur)


    def lower_leg(self, leg, angle=20, speed=None, sleep_dur=0):
        self.raise_leg(leg, -angle, speed, sleep_dur)


    def sidestep_out(self, leg, angle=20, speed=None, sleep_dur=0):
        self.raise_leg(leg, angle, speed, sleep_dur)
        self.swing_out(leg, angle, speed, sleep_dur)
        self.lower_leg(leg, angle, speed, sleep_dur)


    def sidestep_in(self, leg, angle=20, speed=None, sleep_dur=0):
        self.raise_leg(leg, angle, speed, sleep_dur)
        self.swing_in(leg, angle, speed, sleep_dur)
        self.lower_leg(leg, angle, speed, sleep_dur)


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.gait is GAIT_TROT:
            self.trot()
        else:
            return
        #Timer(0.5, self.lower_legs, legs).start()

    def step_right(self):
        self.stand()
        self.sidestep_out(LEG_FR)
        self.sidestep_out(LEG_RR)
        self.sidestep_in(LEG_RL)
        self.stand()
        self.sidestep_in(LEG_FL)
        self.controller.servos[5].stand()
        self.controller.servos[5].update()

    def step_left(self):
        self.stand()
        self.sidestep_out(LEG_FL)
        self.sidestep_out(LEG_RL)
        self.sidestep_in(LEG_RR)
        self.stand()
        self.sidestep_in(LEG_FR)
        self.controller.servos[2].stand()
        self.controller.servos[2].update()

    def walk_forward(self, dur):
        self.stand()
        time.sleep(1)

        start = time.time()
        while time.time() - start < dur:
            self.step()
        self.stand()

    def walk_right(self, dur):
        start = time.time()
        while time.time() - start < dur:
            self.step_right()

    def walk_left(self, dur):
        start = time.time()
        while time.time() - start < dur:
            self.step_left()


    def stand(self):
        self.controller.stand()
        self.controller.update()


    def rest(self):
        self.controller.rest()
        self.controller.update()


    def set_pitch(self, pitch, speed=None):
        if speed:
            original_speed = self.controller.get_speed()
            self.controller.set_speed(speed)

        motors = [1, -1, 0, -1, 1, 0, -1, 1, 0, 1, -1, 0]    # motors and directions to actuate
        motors = [i*pitch for i in motors]
        self.controller.change_pose(motors)
        self.controller.update()

        if speed:
            self.controller.set_speed(original_speed)


    def set_height(self, height, speed=None):
        if speed:
            original_speed = self.controller.get_speed()
            self.controller.set_speed(speed)

        motors = [1, -1, 0, -1, 1, 0, 1, -1, 0, -1, 1, 0]    # motors to actuate
        motors = [i*height for i in motors]
        self.controller.change_pose(motors)
        self.controller.update()

        if speed:
            self.controller.set_speed(original_speed)


    def stretch(self, dur=16, deg=20, speed=None):
        sleep_time = dur/8

        time.sleep(sleep_time)
        self.stand()
        time.sleep(sleep_time)
        self.set_height(deg)
        time.sleep(sleep_time)
        self.stand()
        time.sleep(sleep_time)
        self.set_height(-deg)
        time.sleep(sleep_time)
        self.set_pitch(deg)
        time.sleep(sleep_time)
        self.set_pitch(-deg)
        time.sleep(sleep_time)
        self.stand()
        time.sleep(sleep_time)


    def point(self, motor=0, angle=20):
        self.controller.servos[motor].increment_goal(angle)
        self.controller.update()


    def intimidate(self, angle=20):
        for leg in range(4):
            self.sidestep_out(leg, angle)


    def cower(self, angle=20):
        for leg in range(4):
            self.sidestep_in(leg, angle)








    def correct(self):
        roll, pitch, yaw = self.imu.get_rpy()

        roll_correction = [roll*self.kp*-1*i for i in BODY_ROLL]
        self.controller.change_pose(roll_correction)


