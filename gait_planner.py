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
        self.last_move = STEP_DOWN
        self.step_time = 0
        self.wait_time = wait_time      # in seconds


    def set_gait(self, gait):
        if gait is GAIT_TROT:
            self.gait = GAIT_TROT


    def step_trot(self):
        if self.last_step == STEP_RIGHT:
            change = TROT_STEP_LEFT_POSE
            self.last_step = STEP_LEFT
        else:
            change = TROT_STEP_RIGHT_POSE
            self.last_step = STEP_RIGHT

        if self.last_move == STEP_UP:
            change = change * -1
            self.last_move = STEP_DOWN
        else:
            self.last_move = STEP_UP

        self.controller.change_pose(change)
        self.step_time = time.time()


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.gait is GAIT_TROT:
            self.step_trot()

            step_down = Timer(1.0, self.step_trot)

            step_down.start()
