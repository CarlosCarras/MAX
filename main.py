from __future__ import division
import motor_control
import calibrate_servos
import time

LEGTYPE_FR = "FR"
LEGTYPE_FL = "FR"
LEGTYPE_RR = "RR"
LEGTYPE_RL = "RL"

JOINTTYPE_ABAD = "AB/AD"
JOINTTYPE_HIP = "HIP"
JOINTTYPE_KNEE = "KNEE"


def initialize_servos(controller):
    for i in range(12):
        controller.attach_servo(stand_angle=145, rest_angle=225)


def main():
    print("...Initializing MAX...")

    controller = motor_control.Controller()
    initialize_servos(controller)

    calibrate_servos.calibrate(controller)

    controller.stand()
    controller.update()

    time.sleep(5)

    controller.rest()
    controller.update()


main()
