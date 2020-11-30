from __future__ import division
import motor_control
import calibrate_servos
import thermal_cam
import gait_planner
import trajectory_planner
import max_imu
import time


stand_angles = [155.0,  35.0, 110.0,
                 65.0, 180.0,  55.0,
                140.0,  45.0, 145.0,
                120.0, 160.0,  20.0]

rest_angles = [225.0,   0.0, 110.0,
                20.0, 220.0,  55.0,
               200.0,  10.0, 145.0,
                40.0, 205.0,  20.0]


def initialize_servos(controller):
    for i in range(len(controller.motors)):
        controller.attach_servo(stand_angles[i], rest_angles[i])


def main():
    print('\n...Initializing MAX...\n')

    #imu = max_imu.IMU()
    camera = thermal_cam.ThermalCam()
    controller = motor_control.Controller()
    initialize_servos(controller)

    move = gait_planner.GaitPlanner(controller)
    perception = trajectory_planner.TrajectoryPlanner(camera, move)
    calibrate_servos.calibrate(controller)

    #perception.point_hotspot(60)
    move.walk_forward(30)

main()