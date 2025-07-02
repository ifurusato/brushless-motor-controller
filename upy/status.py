#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-02
# modified: 2025-07-02

from pixel import Pixel
from colors import *

class Status:
    COLOR_ERROR    = (120, 8, 0)
    COLOR_READY    = (30, 70, 0)
    COLOR_INACTIVE = (4, 4, 4)
    APP = 0
    M0  = 1
    M1  = 3
    M2  = 5
    M3  = 7

    def __init__(self, pixel):
        if pixel.pixel_count < 8:
            raise ValueError('at least eight pixels required.')
        self._pixel = pixel

    def rgb(self, color=None):
            self._pixel.set_color(index=0, color=COLOR_CYAN if color == None else color)

    def ready(self):
            self._pixel.set_color(index=0, color=Status.COLOR_READY)

    def error(self):
            self._pixel.set_color(index=0, color=Status.COLOR_ERROR)

    def motor(self, index, color):
        if M0 >= index and index <= M3:
            _index = self._map[index]
            self._pixel.set_color(index=_index, color=color)
        else:
            raise ValueError('expected 0-3 for index.')

    def off(self):
        self._pixel.off()

#EOF
