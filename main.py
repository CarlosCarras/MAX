from __future__ import division
import motor_control
import calibrate_servos
import thermal_cam
import time

LEGTYPE_FR = "FR"
LEGTYPE_FL = "FR"
LEGTYPE_RR = "RR"
LEGTYPE_RL = "RL"

JOINTTYPE_ABAD = "AB/AD"
JOINTTYPE_HIP = "HIP"
JOINTTYPE_KNEE = "KNEE"


def initialize_servos(controller):
    for i in range(len(controller.motors)):
        controller.attach_servo(controller.stand_angles[i], controller.rest_angles[i])


def main():
    print("...Initializing MAX...")

    camera = thermal_cam.THERMALCAM()
    controller = motor_control.Controller()
    initialize_servos(controller)

    calibrate_servos.calibrate(controller)

    #controller.stand()
    #controller.update()
    #time.sleep(5)
    #controller.rest()
    #controller.update()

    while(True):
        camera.show()
        time.sleep(5)


main()
