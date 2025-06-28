#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2025-06-05
# modified: 2025-06-09
#

from datetime import datetime as dt

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class SlewLimiter:
    '''
    A slew limiter for the BrushlessMotor, used to limit the rate of change
    of the motor's target speed, preventing sudden, abrupt changes that could
    be harmful to the motor and ensuring the changes to the motor's speed are
    gradual, more elegant, rather than instantaneous.
    '''
    def __init__(self, max_delta_per_sec, safe_threshold=10):
        '''
        :param max_delta_per_sec:   Maximum allowed change per second (e.g. RPM/sec or %/sec)
        :param safe_threshold:      Minimum magnitude below which direction changes are allowed
        '''
        self.max_delta_per_sec = float(max_delta_per_sec)
        self.safe_threshold = float(safe_threshold)
        self._last_value = None
        self._last_time = None
        self.reset()

    def limit(self, value):
        '''
        Returns the limited value based on the time passed since the last call,
        '''
        now = dt.now()
        value = float(value)
        if self._last_value is None:
            self._last_value = value
            self._last_time = now
            return value
        elapsed = (now - self._last_time).total_seconds()
        if elapsed <= 0.0:
            return self._last_value  # no time has passed
        # prevent direction change unless speed is low
        if ( value * self._last_value < 0 and abs(self._last_value) > self.safe_threshold ):
            return self._last_value  # block reversal
        # calculate allowed delta
        max_delta = self.max_delta_per_sec * elapsed
        delta = value - self._last_value
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
            value = self._last_value + delta
        self._last_value = value
        self._last_time = now
        return value

    def reset(self, value=None):
        '''
        Resets the limiter to a specific value (or clears it).
        '''
        self._last_value = float(value) if value is not None else None
        self._last_time = dt.now()

#EOF
