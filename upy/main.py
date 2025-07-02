#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-23
# modified: 2025-07-02
#
# An overly-complicated main app. An alternative to the UartSlaveApp.

import time
from colorama import Fore, Style

from logger import Logger, Level
from pixel import Pixel
from colors import *

#import cwd
#import free

class MainApp:
    def __init__(self):
        self._log = Logger('main', Level.INFO)
        self._pixel = Pixel(brightness=0.1)
        self._log.info('ready.')

    def start(self):
        self._wait_a_bit()
        self.pixel_on(color=(20, 64, 0))
        self._log.info('started.')

    def pixel_on(self, color=None):
        self._pixel.set_color(color=COLOR_CYAN if color == None else color)

    def pixel_off(self):
        self._pixel.set_color(color=None)

    def _wait_a_bit(self):
        from pyb import LED
        _led = LED(1)
        for _ in range(3):
            self.pixel_on()
            _led.on()
            time.sleep_ms(50)
            self.pixel_off()
            _led.off()
            time.sleep_ms(950)
        self.pixel_off()
        _led.off()

    def close(self):
        self.pixel_off()

# for REPL usage or testing
def exec():
    app = MainApp()
    app.start()

if __name__ == "__main__":
    app = MainApp()
    app.start()

#EOF
