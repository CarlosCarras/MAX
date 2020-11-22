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

def initalize_servos(controller):
    for i in range(12):
        controller.attach_servo()


def main():
    print("...Initializing MAX...")

    controller = motor_control.Controller()
    initalize_servos(controller)

    calibrate_servos.calibrate(controller)

    controller.stand()
    controller.update()

    time.sleep(10)

    controller.rest()
    controller.update()

main()
