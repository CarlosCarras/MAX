
from __future__ import division
import max_servo
import calibrate_servos

LEGTYPE_FR = "FR"
LEGTYPE_FL = "FR"
LEGTYPE_RR = "RR"
LEGTYPE_RL = "RL"

JOINTTYPE_ABAD = "AB/AD"
JOINTTYPE_HIP  = "HIP"
JOINTTYPE_KNEE = "KNEE"

def add_servo(channel, stand_angle, rest_angle) -> object:
    if channel > 15:       # check to make sure the PCA9685 can support another servo
        print("ERROR: Unable to add a new servo.")
        return -1

    new_servo = max_servo.MAXServo(channel, calibrate_servos.motors(channel), stand_angle, rest_angle)
    print("Added a new", new_servo.get_legtype(), new_servo.get_jointtype(), "servo.")
    return new_servo


def main():
    print("...Initializing MAX...")
    calibrate_servos.calibrate()

main()