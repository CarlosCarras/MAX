
from __future__ import division
import pca9685
#import kinematics
#import imu

LEGTYPE_FR = "FR"
LEGTYPE_FL = "FR"
LEGTYPE_RR = "RR"
LEGTYPE_RL = "RL"

JOINTTYPE_ABAD = "AB/AD"
JOINTTYPE_HIP  = "HIP"
JOINTTYPE_KNEE = "KNEE"

def main():
    print("hello world!")

    controller = pca9685.PCA9685()

    FR_knee = controller.add_servo(LEGTYPE_FR, JOINTTYPE_KNEE, 120, 80)
    RR_knee = controller.add_servo(LEGTYPE_RR, JOINTTYPE_KNEE, 120, 80)
    FL_knee = controller.add_servo(LEGTYPE_FL, JOINTTYPE_KNEE, 120, 80)
    RL_knee = controller.add_servo(LEGTYPE_RL, JOINTTYPE_KNEE, 120, 80)


main()