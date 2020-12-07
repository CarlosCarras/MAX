#!/usr/bin/env python
# coding: utf-8

import time
from xbox import Gamepad


class Controller:

    buttonExit = 'XBOX'
    #joystickSpeed = 'LEFT-Y'
    joystickSteering = 'RIGHT-X'

    def __init__(self):
        self.gamepad = Gamepad.Gamepad()
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

    def happyButtonPressed(self):
        print(':)')

    def happyButtonReleased(self):
        print(':(')

    def exitButtonPressed(self):
        print('EXIT')
        self.running = False

        self.gamepad.startBackgroundUpdates()

        self.gamepad.addButtonPressedHandler(self.buttonExit, self.exitButtonPressed)

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
