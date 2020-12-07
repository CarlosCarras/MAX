#!/usr/bin/env python
# coding: utf-8

import time
from xbox import Gamepad


class Controller():

    buttonStand = 'X'
    buttonRest = 'RB'
    buttonExit = 'XBOX'
    joystickSpeed = 'LEFT-Y'
    joystickSteering = 'RIGHT-X'

    def __init__(self, controller):
        self.move = controller

        self.gamepadType = Gamepad.XboxOne
        self.pollInterval = 0.1

        if not Gamepad.available():
            print('\nPlease Connect Your Xbox One Controller...')
            while not Gamepad.available():
                time.sleep(1.0)
        self.gamepad = self.gamepadType()
        print('\nXbox One Controller Connected! ')

        self.running = True
        self.speed = 0.0
        self.steering = 0.0

        self.gamepad.startBackgroundUpdates()
        self.gamepad.addButtonPressedHandler(self.buttonStand, self.stand)
        self.gamepad.addButtonPressedHandler(self.buttonRest, self.rest)
        self.gamepad.addButtonPressedHandler(self.buttonExit, self.exitButtonPressed)

    def stand(self):
        self.move.stand()
        print('MAX is standing.')

    def rest(self):
        self.move.rest()
        print('MAX is resting.')

    def exitButtonPressed(self):
        self.running = False
        print(':)')
        exit()

    def test(self):
        try:
            while self.running and self.gamepad.isConnected():
                # update the joystick positions
                #self.speed = -self.gamepad.axis(self.joystickSpeed)       # speed control (inverted)
                #self.steering = self.gamepad.axis(self.joystickSteering)  # steering control (not inverted)
                #print('%+.1f %% speed, %+.1f %% steering' % (self.speed * 100, self.steering * 100))
                time.sleep(self.pollInterval)
        finally:
            self.gamepad.disconnect()                     # ensure the background thread is always terminated when done
