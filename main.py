from __future__ import division
import motor_control
import calibrate_servos
import thermal_cam
import gait_planner
import time


stand_angles = [155.0,  35.0, 110.0,
                 65.0, 180.0,  55.0,
                135.0,  45.0, 145.0,
                120.0, 160.0,  20.0]

rest_angles = [225.0,   0.0, 110.0,
                20.0, 220.0,  55.0,
               185.0,  10.0, 145.0,
                40.0, 205.0,  20.0]


def initialize_servos(controller):
    for i in range(len(controller.motors)):
        controller.attach_servo(stand_angles[i], rest_angles[i])


def main():
    print("...Initializing MAX...")

    camera = thermal_cam.ThermalCam()
    controller = motor_control.Controller()
    initialize_servos(controller)

    calibrate_servos.calibrate(controller)

    controller.stand()
    time.sleep(3)
    #controller.rest()

    move = gait_planner.GaitPlanner(controller)

    while True:
        move.step()
        time.sleep(5)

main()
