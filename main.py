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

    #move.imu_test()
    # joy = xbox.Joystick()

    while xbox.gamepad.isConnected():
        # Wait for the next event
        eventType, control, value = xbox.gamepad.getNextEvent()
        print(eventType)
        print(control)
        print(value)
        # Determine the type
        if eventType == 'BUTTON':
            # Button changed
            if control == xbox.buttonStand:
                # Happy button (event on press and release)
                if value:
                    print(':)')


main()