#!/usr/bin/env python3

"""
filename: thermal_cam.py
author:   Carlos Carrasquillo
created:  November 24, 2020
modified: November 24, 2020
project:  MAX
purpose:  This file controls the operation of the infrared data from the 8x8 AMG8833 sensor.

hardware: AMG8833
datasheet: SPECIFICATIONS FOR Infrared Array Sensor

discloure: This code was modeled after Alexander Chernenko's and Jeff Epler's (Adafruit Industries)
amg88xx_rpi_thermal_cam_console.py file, which can be found at,
https://github.com/adafruit/Adafruit_AMG88xx/blob/master/Adafruit_AMG88xx.cpp
"""

import sys
import math
from colour import Color
import amg88xx


MIN_TEMP = 26.0
MAX_TEMP = 32.0
COLORDEPTH = 1024

COLORS = list(Color("indigo").range_to(Color("red"), COLORDEPTH))
COLORS = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in COLORS]
CONSOLE_COLORS = [17, 18, 19, 20, 21, 57, 93, 129, 165, 201, 200, 199, 198, 197, 196, 202, 208, 214, 220]

class THERMALCAM:

    def __init__(self):
        self.amg8833 = amg88xx.AMG88XX()
        self.width = self.amg8833.AMG8833_PIXEL_ARRAY_WIDTH
        self.height = self.amg8833.AMG8833_PIXEL_ARRAY_HEIGHT
        self.min_pixel_temp = MIN_TEMP
        self.max_pixel_temp = MAX_TEMP
        self.current_min_pixel_temp = 0
        self.current_max_pixel_temp = 0
        self.points = [(math.floor(i / self.width), (i % self.height)) for i in range(0, self.width * self.height)]

    def map_temp(self, val):
        return (val - self.min_pixel_temp) * (COLORDEPTH - 1) / (self.max_pixel_temp - self.min_pixel_temp)


    def print_temps(self, console_x, console_y, text, color):
        """ outputs a colored text to console at coordinates """
        sys.stdout.write("\x1b7\x1b[48;5;%dm" % (color))
        sys.stdout.write("\x1b7\x1b[%d;%df%s\x1b8" % (console_x, console_y, text))

    def show(self):
        pixels = self.amg8833.get_pixels()

        self.current_max_pixel_temp = self.map_temp(pixels[1][1])
        self.current_min_pixel_temp = self.current_max_pixel_temp

        y_console = 2
        for ix in range(self.height):
            x_console = 2
            for jx in range(self.width):
                pixel = self.map_temp(pixels[ix][jx])
                color_index = int(round((pixel - self.min_pixel_temp)))

                if color_index < 0:
                    color_index = 0
                if color_index > len(CONSOLE_COLORS) - 1:
                    color_index = len(CONSOLE_COLORS) - 1

                self.print_temps(x_console, y_console * 2 - 2, "  ", CONSOLE_COLORS[color_index])

                if pixel > self.current_max_pixel_temp:
                    self.current_min_pixel_temp = pixel
                    if pixel > self.max_pixel_temp:
                        self.max_pixel_temp = pixel
                if pixel < self.current_min_pixel_temp:
                    self.current_min_pixel_temp = pixel
                    if pixel < self.min_pixel_temp:
                        self.min_pixel_temp = pixel
                x_console += 1
            y_console += 1

        sys.stdout.flush()

    def get_heat_range(self):
        return self.max_pixel_temp - self.min_pixel_temp

    def get_current_heat_range(self):
        return self.current_max_pixel_temp - self.current_min_pixel_temp

    def get_color_range(self):
        return self.get_heat_range() / len(CONSOLE_COLORS)

    def get_current_color_range(self):
        return self.get_current_heat_range() / len(CONSOLE_COLORS)
