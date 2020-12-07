from __future__ import division
import motor_control
import calibrate_servos
import thermal_cam
import gait_planner
import trajectory_planner
import max_imu
import max_health
import xbox_controller
import time


def initialize_servos(controller):
    for i in range(len(controller.motors)):
        controller.attach_servo(gait_planner.STAND_ANGLES[i], gait_planner.REST_ANGLES[i])


def main():
    print('\n...Initializing MAX...\n')

    imu = max_imu.IMU()
    camera = thermal_cam.ThermalCam()
    health = max_health.HealthCheck(imu, camera)
    controller = motor_control.Controller()
    initialize_servos(controller)

    move = gait_planner.GaitPlanner(controller, imu)
    perception = trajectory_planner.TrajectoryPlanner(camera, move)
    xbox = xbox_controller.Controller(move)
    calibrate_servos.calibrate(controller)

    move.stand()
    time.sleep(5)
    move.rest()

    input('Press [Enter] to start walking.')

    move.walk_forward(30)
    health.status()

    input('Press [Enter] to test Xbox.')
    xbox.test(30)
    health.status()

    input('Press [Enter] to test Thermal.')
    perception.point_hotspot()



main()