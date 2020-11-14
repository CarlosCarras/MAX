
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
    print("hello world!")

    FR_knee = add_servo(1, 120, 80)
    RR_knee = add_servo(2, 120, 80)
    FL_knee = add_servo(3, 120, 80)
    RL_knee = add_servo(4, 120, 80)




main()