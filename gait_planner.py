import time
from threading import Timer


GAIT_TROT = 0x00
STEP_RIGHT = "R"
STEP_LEFT = "L"
STEP_UP = "U"
STEP_DOWN = "D"


TROT_STEP_RIGHT_POSE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
TROT_STEP_LEFT_POSE = [0, 0, 0, -10, -10, 0, 0, 0, 0, 0, 0, 0]


class GaitPlanner():
    def __init__(self, controller, gait=GAIT_TROT, wait_time=2):
        self.controller = controller
        self.gait = gait
        self.last_step = None
        self.step_time = 0
        self.wait_time = wait_time      # in seconds


    def set_gait(self, gait):
        if gait is GAIT_TROT:
            self.gait = GAIT_TROT


    def step_trot(self, step_dir):
        if self.last_step == STEP_RIGHT:
            change = TROT_STEP_LEFT_POSE
            self.last_step = STEP_LEFT
        else:
            change = TROT_STEP_RIGHT_POSE
            self.last_step = STEP_RIGHT

        if step_dir == STEP_DOWN:
            change =  [i*-1 for i in change]

        self.controller.change_pose(change)
        self.step_time = time.time()
        print(change)


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.gait is GAIT_TROT:
            self.step_trot(STEP_UP)

            step_down = Timer(1.0, self.step_trot, STEP_DOWN)
            step_down.start()
