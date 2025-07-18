#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2025-06-05
# modified: 2025-07-18

import utime
from colorama import Fore, Style
from logger import Logger, Level

class SlewLimiter:
    '''
    A slew limiter for the BrushlessMotor, used to limit the rate of change
    of the motor's target speed, preventing sudden, abrupt changes that could
    be harmful to the motor and ensuring the changes to the motor's speed are
    gradual, more elegant, rather than instantaneous.
    '''
    def __init__(self, name=None, max_delta_per_sec=None, safe_threshold=10):
        '''
        Args:
            name:                name for this slew limiter
            max_delta_per_sec:   Maximum allowed change per second (e.g. RPM/sec or %/sec)
            safe_threshold:      Minimum magnitude below which direction changes are allowed
        '''
        self._log = Logger('slew-{}'.format(name), Level.INFO)
        self.max_delta_per_sec = float(max_delta_per_sec)
        self.safe_threshold = float(safe_threshold)
        self._last_value = None
        self._last_time_ms = None # Renamed for MicroPython ticks_ms
        self.reset()
        self._log.info(Fore.MAGENTA + 'ready.')

    def limit(self, value):
        '''
        Returns the limited value based on the time passed since the last call.
        '''
        now_ms = utime.ticks_ms()
        value = float(value)
        if self._last_value is None:
            self._last_value = value
            self._last_time_ms = now_ms
            return value
        elapsed_ms = utime.ticks_diff(now_ms, self._last_time_ms)
        if elapsed_ms <= 0:
            return self._last_value
        elapsed_seconds = elapsed_ms / 1000.0
        if (value * self._last_value < 0) and (abs(self._last_value) > self.safe_threshold):
            return self._last_value  # Block reversal if not below safe threshold
        max_delta = self.max_delta_per_sec * elapsed_seconds
        delta = value - self._last_value
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        new_value = self._last_value + delta
        self._last_value = new_value
        self._last_time_ms = now_ms
        return new_value

    def reset(self, value=None):
        '''
        Resets the limiter to a specific value (or clears it).
        '''
        self._last_value = float(value) if value is not None else None
        self._last_time_ms = utime.ticks_ms()

#EOF
