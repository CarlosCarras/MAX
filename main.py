from __future__ import division
import max_servo
import calibrate_servos

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
    calibrate_servos.calibrate()


main()
