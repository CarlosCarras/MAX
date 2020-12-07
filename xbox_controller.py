#!/usr/bin/env python
# coding: utf-8

import time
from xbox import Gamepad


class Controller():

    buttonStand = 'A'
    buttonRest = 'B'
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

        #self.gamepad.startBackgroundUpdates()
        #self.gamepad.addButtonPressedHandler(self.buttonStand, self.stand)
        #self.gamepad.addButtonPressedHandler(self.buttonRest, self.rest)
        #self.gamepad.addButtonPressedHandler(self.buttonExit, self.exitButtonPressed)

    def stand(self):
        self.move.stand()
        print('MAX is standing.')

    def rest(self):
        self.move.rest()
        print('MAX is resting.')

    def exitButtonPressed(self):
        self.running = False
        print(":)")
        exit()

    def test(self, dur=None):
        if dur is None: dur = 60
        start_time = time.time()

        while self.gamepad.isConnected() and time.time()-start_time < dur:
            eventType, control, value = self.gamepad.getNextEvent()
            if eventType == 'BUTTON':
                if control == self.buttonStand:
                    if value:
                        self.stand()
                if control == self.buttonRest:
                    if value:
                        self.rest()