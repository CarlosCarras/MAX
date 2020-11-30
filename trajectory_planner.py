import numpy as np
import time


class TrajectoryPlanner():

    def __init__(self, camera, controller):
        self.camera = camera
        self.controller = controller

    def get_hotspot_dir(self):         # 0 is left, 1 is right

        pixels = np.array(self.camera.get_pixels())
        pixels = pixels.transpose()
        if np.argmax(pixels) < self.camera.num_pixels / 2:
            return 0
        else:
            return 1

    def point_hotspot(self, dur=None):
        if not dur: dur = 0.05

        last_dir = None
        start_time = time.time()
        while time.time() - start_time < dur:
            if not self.get_hotspot_dir():
                dir = 0
            else:
                dir = 1

            if last_dir is not dir:
                self.controller.stand()
                self.controller.point(dir)
            last_dir = dir


