#!/usr/bin/env python3

"""
filename: amg88xx.py
author:   Carlos Carrasquillo
created:  November 22, 2020
modified: November 22, 2020
project:  MAX
purpose:  This file reads infrared data from the 8x8 AMG8833 sensor.

hardware: AMG8833
datasheet: SPECIFICATIONS FOR Infrared Array Sensor

discloure: This code was modeled after deanm1278's and Limor Fried's (Adafruit Industries) Adafruit_AMG99xx.cpp file,
which can be found at,
https://github.com/adafruit/Adafruit_AMG88xx/blob/master/Adafruit_AMG88xx.cpp
"""

from smbus2 import SMBus
import time

AMG88XX_PCTL = 0x00
AMG88XX_RST = 0x01
AMG88XX_FPSC = 0x02         # fps control register
AMG88XX_INTC = 0x03         # interrupt control register
AMG88XX_STAT = 0x04
AMG88XX_SCLR = 0x05
AMG88XX_AVE = 0x07
AMG88XX_INTHL = 0x08
AMG88XX_INTHH = 0x09
AMG88XX_INTLL = 0x0A
AMG88XX_INTLH = 0x0B
AMG88XX_IHYSL = 0x0C
AMG88XX_IHYSH = 0x0D
AMG88XX_TTHL = 0x0E
AMG88XX_TTHH = 0x0F
AMG88XX_INT_OFFSET = 0x010
AMG88XX_PIXEL_OFFSET = 0x80

# power/operating modes
AMG88XX_NORMAL_MODE = 0x00
AMG88XX_SLEEP_MODE = 0x10
AMG88XX_STAND_BY_60 = 0x20
AMG88XX_STAND_BY_10 = 0x21

# software resets
AMG88XX_FLAG_RESET = 0x30
AMG88XX_INITIAL_RESET = 0x3F

# frame rates
AMG88XX_FPS_10 = 0x00
AMG88XX_FPS_1 = 0x01

# interrupt enables
AMG88XX_INT_DISABLED = 0x00
AMG88XX_INT_ENABLED = 0x01

# interrupt modes
AMG88XX_DIFFERENCE = 0x00
AMG88XX_ABSOLUTE_VALUE = 0x01

# important bits
AMG88XX_INTEN_bit = 1 << 0
AMG88XX_INTMOD_bit = 1 << 1
AMG88XX_FPS_bit = 1 << 0

def _signed_12bit_to_float(val):
    # take first 11 bits as absolute val
    abs_val = val & 0x7FF
    if val & 0x800:
        return 0 - float(abs_val)
    return float(abs_val)


def _twos_comp_to_float(val):
    val &= 0xFFF
    if val & 0x800:
        val -= 0x1000
    return float(val)

class AMG88XX:
    # AMG8833 specs
    AMG8833_PIXEL_ARRAY_WIDTH = 8
    AMG8833_PIXEL_ARRAY_HEIGHT = 8
    AMG8833_PIXEL_TEMP_CONVERSION = 0.25
    AMG8833_THERMISTOR_CONVERSION = 0.0625
    AMG8833_MIN_TEMP = 26.0
    AMG8833_MAX_TEMP = 32.0

    def __init__(self, bus=1, address=0x69):
        self._address = address
        self._bus = SMBus(bus)

        self.write(AMG88XX_PCTL, AMG88XX_NORMAL_MODE)
        self.write(AMG88XX_RST, AMG88XX_INITIAL_RESET)
        self.write_bit(AMG88XX_INTC, AMG88XX_INTEN_bit, AMG88XX_INT_DISABLED)
        self.write_bit(AMG88XX_FPSC, AMG88XX_FPS_bit, AMG88XX_FPS_10)

    # ----------------------------- I2C  Utilites -----------------------------#
    def write(self, register, data):
        self._bus.write_byte_data(self._address, register, data)
        time.sleep(0.0001)

    def write_bit(self, register, bit, val):
        bitmask = (val & 0b1) << bit
        data = self._bus.read_byte_data(self._address, register)
        data |= bitmask
        self._bus.write_byte_data(self._address, register, data)

    def read(self, register):
        return self._bus.read_byte_data(self._address, register)

    def read_bytes(self, register, num_bytes):
        return self._bus.read_i2c_block_data(self._address, register, num_bytes)

    # ------------------------------- AMG8833  --------------------------------#
    def get_temperature(self):
        """Temperature of the sensor in Celsius"""
        raw = (self.read(AMG88XX_TTHH << 8)) | self.read(AMG88XX_TTHL)
        return _signed_12bit_to_float(raw) * self.AMG8833_THERMISTOR_CONVERSION

    def get_pixels(self):
        """Temperature of each pixel across the sensor in Celsius.
           Temperatures are stored in a two dimensional list where the first index is the row and
           the second is the column. The first row is on the side closest to the writing on the
           sensor."""
        pixels = [[0] * self.AMG8833_PIXEL_ARRAY_WIDTH for _ in range(self.AMG8833_PIXEL_ARRAY_HEIGHT)]

        for row in range(self.AMG8833_PIXEL_ARRAY_HEIGHT):
            for col in range(self.AMG8833_PIXEL_ARRAY_WIDTH):
                reg = AMG88XX_PIXEL_OFFSET + ((row * self.AMG8833_PIXEL_ARRAY_HEIGHT + col) << 1)
                raw = self.read_bytes(reg, 2)
                print(str(raw[1]) + ' , ' + str(raw[0]))
                reading = raw[1] << 8 | raw[0]
                print(reading)
                pixels[row][col] = _twos_comp_to_float(reading) * self.AMG8833_PIXEL_TEMP_CONVERSION

        return pixels
