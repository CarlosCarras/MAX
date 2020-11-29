import time
from threading import Timer


GAIT_TROT = 0x00
STEP_RIGHT = "R"
STEP_LEFT = "L"
STEP_UP = "U"
STEP_DOWN = "D"


TROT_STEP_RIGHT_POSE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
TROT_STEP_LEFT_POSE = [0, 0, 0, -20, 20, 0, 0, 0, 0, 0, 0, 0]
TROT_STEP_LEFT_SWING = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0]
TROT_STEP_RIGHT_SWING = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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


    def step_trot(self, step_dir):
        if self.last_step == STEP_RIGHT:
            change = TROT_STEP_LEFT_POSE
            swing = TROT_STEP_LEFT_SWING
        else:
            change = TROT_STEP_RIGHT_POSE
            swing = TROT_STEP_RIGHT_SWING

        if step_dir == STEP_DOWN:
            change =  [i*-1 for i in change]

        self.controller.change_pose(change)
        self.controller.update()
        self.step_time = time.time()
        time.sleep(0.2)

        self.controller.change_pose(swing)
        self.controller.update()


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.last_step == STEP_RIGHT:
            self.last_step = STEP_LEFT
        else:
            self.last_step = STEP_RIGHT

        if self.gait is GAIT_TROT:
            self.step_trot(STEP_UP)

            step_down = Timer(0.5, self.step_trot, STEP_DOWN)
            step_down.start()





















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


