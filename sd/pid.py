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
from math import isclose
from colorama import Fore, Style
from logger import Logger, Level
from util import Util

class PID:
    def __init__(self, id, config, level=Level.INFO):
        self._log = Logger('pid-{}'.format(id), level=level)
        self._log.info('initialising PID controller…')
        _cfg = config['kros']['pid']
        self._verbose    = _cfg['verbose']
        self._kp         = _cfg['kp']
        self._ki         = _cfg['ki']
        self._kd         = _cfg['kd']
        self._setpoint   = _cfg['setpoint']
        max_motor_speed  = config['kros']['motor_controller']['max_motor_speed']
        self._output_min = -max_motor_speed
        self._output_max = max_motor_speed
        self._log_frequency = 100 
        self._log.info(Fore.MAGENTA + "kp={:>5.3f}; ki={:>5.3f}; kd={:>5.3f}; setpoint: {}; limits: {}🠊 {}".format(
                self._kp, self._ki, self._kd, self._setpoint, self._output_min, self._output_max))
        self._last_error = 0.0
        self._int_error  = 0.0
        self._p_term     = 0.0
        self._i_term     = 0.0
        self._d_term     = 0.0
        self._output     = 0.0
        self._awup       = False # anti-windup
        self._counter    = itertools.count()
        self._epsilon_rpm_for_stop = 2.0  # stop threshold (e.g., from 1.0 to 5.0 RPM)
        self._log.info('ready.')

    @property
    def info(self):
        '''
        Returns a tuple containing: p_term, i_term, d_term, setpoint, output
        '''
        return ( self._p_term, self._i_term, self._d_term, self._setpoint, self._output )

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        self._setpoint = float(value)

    def update(self, value, dt_us):
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - value
        # Proportional term
        self._p_term = self._kp * error
        # Derivative term
        self._d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        if self._setpoint == 0.0:
            if isclose(value, 0.0, abs_tol=self._epsilon_rpm_for_stop):
                self._int_error = 0.0
            elif value > 0: # Motor moving forward (positive RPM)
                self._int_error = min(0.0, self._int_error + error * dt_seconds)
            else:
                self._int_error = max(0.0, self._int_error + error * dt_seconds)
        else:
            self._int_error += error * dt_seconds
        if self._awup: # anti-windup
            if self._ki != 0:
                int_max = (self._output_max - self._p_term - self._d_term) / self._ki
                int_min = (self._output_min - self._p_term - self._d_term) / self._ki
                self._int_error = Util.clip(self._int_error, int_min, int_max)
        # Integral term
        self._i_term = self._ki * self._int_error
        self._output = self._p_term + self._i_term + self._d_term
        self._last_error = error
        self._output = Util.clip(self._output, self._output_min, self._output_max)
        if self._verbose:
#           if abs(self._setpoint) > 0 and next(self._counter) % self._log_frequency == 0:
            if next(self._counter) % self._log_frequency == 0:
                self._log.info("p={:.2f}, i={:.2f}, d={:.2f}, setpoint={:.2f}; output={:.2f}".format(
                        self._p_term, self._i_term, self._d_term, self._setpoint, self._output))
        return self._output

    def x_update(self, value, dt_us):
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - value
        self._p_term = self._kp * error
        self._d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        if self._setpoint == 0.0:
            if isclose(value, 0.0, abs_tol=self._epsilon_rpm_for_stop):
                self._int_error = 0.0
            elif value > 0:
                self._int_error = min(0.0, self._int_error + error * dt_seconds)
            else:
                self._int_error = max(0.0, self._int_error + error * dt_seconds)
        else:    
            self._int_error += error * dt_seconds
        self._i_term = self._ki * self._int_error
        self._output = self._p_term + self._i_term + self._d_term
        # --- MODIFIED: Direct Integral Anti-Windup (Conditional Accumulation) ---
        # Prevent integral accumulation if output exceeds limits and error pushes it further
        if self._ki != 0:
            if (self._output > self._output_max and error > 0) or \
               (self._output < self._output_min and error < 0):
                # If output is saturating and error is trying to increase integral further,
                # revert the integral accumulation from this cycle.
                self._int_error -= error * dt_seconds
        # Re-calculate self._i_term after potential integral modification by anti-windup
        self._i_term = self._ki * self._int_error
        self._output = self._p_term + self._i_term + self._d_term # Re-calculate total output
        self._last_error = error  
        limited = max(self._output_min, min(self._output, self._output_max))
        if self._verbose:
            if next(self._counter) % self._log_frequency == 0:
                self._log.info(Style.DIM + "dt={}µs; p={:.2f}, i={:.2f}, d={:.2f}, setpoint={:.2f}; output={:.2f} ({:.2f})".format(
                        dt_us, self._p_term, self._i_term, self._d_term, self._setpoint, self._output, limited))
        return limited

    def reset(self):
        self._last_error = 0.0
        self._int_error = 0.0

#EOF
