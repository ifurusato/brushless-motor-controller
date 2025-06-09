#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-09-19
# modified: 2025-06-09
#
# A simplified, asynchronous version of the DigitalPotentiometer class, used
# for testing.
#

import time
import asyncio
import traceback
import threading
import colorsys
from colorama import init, Fore, Style
init()

import ioexpander as io
from core.logger import Logger, Level

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DigitalPotentiometer:
    I2C_ADDR   = 0x0C
    PIN_RED    = 1
    PIN_GREEN  = 7
    PIN_BLUE   = 2
    POT_ENC_A  = 12
    POT_ENC_B  = 3
    POT_ENC_C  = 11
    BRIGHTNESS = 0.5
    PERIOD     = int(255 / BRIGHTNESS)

    def __init__(self, fps=30, level=Level.INFO):
        try:
            self._log = Logger('pot', level)
            self._max = 3.3
            self.ioe = io.IOE(i2c_addr=self.I2C_ADDR)
            self.ioe.set_mode(self.POT_ENC_A, io.PIN_MODE_PP)
            self.ioe.set_mode(self.POT_ENC_B, io.PIN_MODE_PP)
            self.ioe.set_mode(self.POT_ENC_C, io.ADC)
            self.ioe.output(self.POT_ENC_A, 1)
            self.ioe.output(self.POT_ENC_B, 0)
            self.ioe.set_pwm_period(self.PERIOD)
            self.ioe.set_pwm_control(divider=2)
            self.ioe.set_mode(self.PIN_RED, io.PWM, invert=True)
            self.ioe.set_mode(self.PIN_GREEN, io.PWM, invert=True)
            self.ioe.set_mode(self.PIN_BLUE, io.PWM, invert=True)
            self._log.info("running LED with {} brightness steps.".format(int(self.PERIOD * self.BRIGHTNESS)))
            self._fps  = fps
            self._task = None
            self._loop = None
            self._loop_thread = None
            self._stop_event = threading.Event()
            self._log.info('ready.')
        except Exception as e:
            self._log.info('{} raised: {}'.format(type(e), e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def analog(self):
        return self.ioe.input(self.POT_ENC_C)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        val = max(0.0, min(self.analog, self._max))
        return 1.0 - (val / self._max)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _update(self):
        h = max(0.0, min(self.analog / self._max, 1.0))
        r, g, b = [int(c * self.PERIOD * self.BRIGHTNESS)
                   for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
        self.ioe.output(self.PIN_RED, r)
        self.ioe.output(self.PIN_GREEN, g)
        self.ioe.output(self.PIN_BLUE, b)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def start(self):
        if self._task is not None:
            return # already running
        def run_loop():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            async def runner():
                self._task = asyncio.create_task(self._run())
                await self._task  # Wait for _run() to complete
            try:
                self._loop.run_until_complete(runner())
            except Exception as e:
                self._log.error('{} raised in run loop: {}\n{}'.format(type(e), e, traceback.format_exc()))
            finally:
                self._loop.close()
        self._stop_event.clear()
        self._loop_thread = threading.Thread(target=run_loop, daemon=True)
        self._loop_thread.start()

    async def _run(self):
        delay = 1.0 / self._fps
        while not self._stop_event.is_set():
            self._update()
            await asyncio.sleep(delay)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def off(self):
        self.ioe.output(self.PIN_RED, 0)
        self.ioe.output(self.PIN_GREEN, 0)
        self.ioe.output(self.PIN_BLUE, 0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def stop(self):
        if self._task is None:
            return # not running
        self._stop_event.set()
        # wait for the loop thread to finish gracefully
        if self._loop_thread:
            self._loop_thread.join()
        self._task = None
        self._loop = None
        self._loop_thread = None

#EOF
