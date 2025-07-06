#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-04
# modified: 2025-07-06

import utime
import itertools
from colorama import Fore, Style
from logger import Logger, Level

class PID:
    def __init__(self, id, config, level=Level.INFO):
        self._log = Logger('pid-{}'.format(id), level=level)
        self._log.info('initialising PID controller…')
        _cfg = config['kros']['pid']
        self._kp         = _cfg['kp']
        self._ki         = _cfg['ki']
        self._kd         = _cfg['kd']
        self._setpoint   = _cfg['setpoint']
        self._log.info(Fore.MAGENTA + "kp={:>4.2f}; ki={:>4.2f}; kd={:>4.2f}; setpoint: {}".format(self._kp, self._ki, self._kd, self._setpoint))
        self._output_min = _cfg['min'] 
        self._output_max = _cfg['max']
        self._last_error = 0.0
        self._integral_error = 0.0
        self._last_time  = utime.ticks_us()
        self._counter    = itertools.count()
        self._log.info('ready.')

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        self._setpoint = float(value)

    def update(self, current_value): # Removed dt_seconds from arguments
        current_time = utime.ticks_us() # New variable to store current time
        dt_us = utime.ticks_diff(current_time, self._last_time) # Calculate dt
        self._last_time = current_time # Update last_time for the next iteration
        # convert microseconds to seconds, handle potential zero dt
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - current_value
        # Proportional term
        p_term = self._kp * error
        # Integral term
        self._integral_error += error * dt_seconds
        # anti-windup: clamp integral error to prevent over-integration when output is saturated
        if self._ki != 0:
            integral_limit = (self._output_max - self._output_min) / self._ki
            self._integral_error = max(min(self._integral_error, integral_limit), -integral_limit)
        i_term = self._ki * self._integral_error
        # Derivative term: based on the change in error over time
        d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        output = p_term + i_term + d_term
        self._last_error = error
        limited = max(self._output_min, min(output, self._output_max))
        if next(self._counter) % 50 == 0:
            self._log.info("p={:.2f}, i={:.2f}, d={:.2f}, output={:.2f} ({:.2f})".format(p_term, i_term, d_term, output, limited))
        return limited

    def reset(self):
        self._last_error = 0.0
        self._integral_error = 0.0
        self._last_time = utime.ticks_us() # Reset time for next calculation to ensure accurate dt

#EOF
