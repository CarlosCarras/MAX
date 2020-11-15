#!/usr/bin/env python3

"""
filename: pca9685.py
author:   Carlos Carrasquillo
created:  October 24, 2020
modified: November 14, 2020
project:  MAX

purpose:  This file writes to the PCA9685 PWM controller.

datasheet: NXP Semiconductors, PCA9685 Datasheet, Rev. 4

discloure: This code was modeled after Tony DiCola (Adafruit Industries) PCA9685.py file, which can be found at,
https://github.com/adafruit/Adafruit_Python_PCA9685/blob/master/Adafruit_PCA9685/PCA9685.py
"""

from smbus2 import SMBus
import time
import math


class PCA9685:

    PCA9685_ADDR = 0x40
    PCA9685_RESOLUTION = 12                                 # 12-bit resolution (4096 steps)
    PCA9685_OSCILLATOR_FREQ = 25000000.0                    # the internal oscillator of the PCA9685 is 25MHz (pp. 1)
    sleep_dur = 0.005                                       # general sleep duration (pp. 14)

    # --------------------------- PCA9685 Registers ---------------------------#
    MODE1 = 0x00
    MODE2 = 0x01
    SUBADR1 = 0x02
    SUBADR2 = 0x03
    SUBADR3 = 0x04
    PRESCALE = 0xFE
    LED0_ON0 = 0x06
    LED0_OFF0 = 0x08
    ALL_LED_ON0 = 0xFA
    ALL_LED_OFF0 = 0xFC

    # mode register bitmasks, pp. 14
    MODE1_RESTART = 1 << 7
    MODE1_SLEEP   = 1 << 4
    MODE1_ALLCALL = 1 << 0
    MODE2_INVRT   = 1 << 4
    MODE2_OUTDRV  = 1 << 2


    def __init__(self, bus=1, address=PCA9685_ADDR):
        self.address = address
        self.bus = SMBus(bus)

        self.set_all_pwm(0, 0)
        self.bus.write_byte_data(self.address, self.MODE2, self.MODE2_OUTDRV)  # totem pole output (pp. 16)
        time.sleep(self.sleep_dur)                                             #  waiting 500 us for oscillator

        mode1 = self.bus.read_byte_data(self.address, self.MODE1)
        mode1 = mode1 & ~self.MODE1_SLEEP                           # bitmask to disable sleep mode
        self.bus.write_byte_data(self.address, self.MODE1, mode1)   # wakes up the 16-channel, 12-bit PWM controller (pp. 15)
        time.sleep(self.sleep_dur)                                  #  waiting 500 us for oscillator

    #----------------------------- I2C  Utilites -----------------------------#

    def read_i2c_word(self, register):
        # reads the high and low bytes from consecutive registers
        high = self.bus.read_byte_data(self.address, register)
        low  = self.bus.read_byte_data(self.address, register+1)
        word = (high << 8) + low

        if word >= 0x8000:                                              # accounts for negative numbers
            return -((65535 - word) + 1)
        return word

    def write_i2c_word(self, register, data):
        low = data & 0xFF
        high = (data >> 8) & 0xFF

        # writes the high and low bytes from consecutive registers
        self.bus.write_byte_data(self.address, register, low)
        self.bus.write_byte_data(self.address, register+1, high)

    # ----------------------------- PWM Control ------------------------------#

    def set_pwm_freq(self, freq_hz):
        # computing the appropriate register value for a Hz input (pp. 14)
        prescale_val = self.PCA9685_OSCILLATOR_FREQ
        prescale_val /= self.PCA9685_RESOLUTION
        prescale_val /= float(freq_hz)
        prescale_val -= 1.0
        prescaler = int(math.floor(prescale_val + 0.5))

        # sets the prescaler value
        mode1 = self.bus.read_byte_data(self.address, self.MODE1)
        mode1_temp = (mode1 & 0x7F) | self.MODE1_SLEEP  # temporary bitmask to set the oscillator to sleep mode
        self.bus.write_byte_data(self.address, self.MODE1, mode1_temp)

        self.bus.write_byte_data(self.address, self.PRESCALE, prescaler)     # writes out the prescaler value

        self.bus.write_byte_data(self.address, self.MODE1, mode1)
        time.sleep(self.sleep_dur)
        self.bus.write_byte_data(self.address, self.MODE1, mode1 | self.MODE1_RESTART)

    def set_pwm(self, channel, on, off):
        self.write_i2c_word(self.LED0_ON0 + 4*channel, on)
        self.write_i2c_word(self.LED0_OFF0 + 4*channel, off)

    def set_all_pwm(self, on, off):
        self.write_i2c_word(self.ALL_LED_ON0, on)
        self.write_i2c_word(self.ALL_LED_OFF0, off)

    # ---------------------------- Servo Control -----------------------------#
    # sets the pulse length in seconds
    # e.g. set_servo_pulse(0, 0.001) is a ~1 millisecond pulse width

    def set_servo_pulse(self, channel, pulse):
        pulse_length = 1000000      # 1,000,000 us per second
        pulse_length //= 60         # 60 Hz
        pulse_length //= 4096       # 12 bits of resolution
        pulse *= 1000
        pulse //= pulse_length
        self.set_pwm(channel, 0, pulse)

    def set_servo_pwm(self, channel, off):
        self.set_pwm(channel, 0, off)
