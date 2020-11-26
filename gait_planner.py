import time


GAIT_TROT = 0x00
STEP_RIGHT = "R"
STEP_LEFT = "L"


TROT_STEP_RIGHT_POSE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
TROT_STEP_LEFT_POSE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class gaitPlanner():
    def __init__(self, controller, gait=GAIT_TROT, wait_time=2):
        self.controller = controller
        self.gait = gait
        self.last_step = None
        self.step_time = 0
        self.wait_time = wait_time      # in seconds


    def set_gait(self, gait):
        if gait is GAIT_TROT:
            self.gait = GAIT_TROT


    def step_trot(self):
        if self.last_step == STEP_RIGHT:
            pose = TROT_STEP_LEFT_POSE
            self.last_step = STEP_LEFT
        else:
            pose = TROT_STEP_RIGHT_POSE
            self.last_step = STEP_RIGHT

        self.controller.set_pose(pose)
        self.step_time = time.time()


    def step(self):
        if time.time() - self.step_time < self.wait_time:
            return

        if self.gait is GAIT_TROT:
            self.step_trot()
