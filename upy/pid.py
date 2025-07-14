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
        self._output_min = _cfg['min'] 
        self._output_max = _cfg['max']
        self._log_frequency = 100 
        self._log.info(Fore.MAGENTA + "kp={:>5.3f}; ki={:>5.3f}; kd={:>5.3f}; setpoint: {}; limits: {}🠊 {}".format(
                self._kp, self._ki, self._kd, self._setpoint, self._output_min, self._output_max))
        self._last_error = 0.0
        self._integral_error = 0.0
        self._counter    = itertools.count()
        self._epsilon_rpm_for_stop = 2.0  # stop threshold (e.g., from 1.0 to 5.0 RPM)
        self._log.info('ready.')

    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, value):
        self._setpoint = float(value)

    def update(self, value, dt_us):
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - value
        p_term = self._kp * error
        d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        if self._setpoint == 0.0:
            if isclose(value, 0.0, abs_tol=self._epsilon_rpm_for_stop):
                self._integral_error = 0.0
            elif value > 0:
                self._integral_error = min(0.0, self._integral_error + error * dt_seconds)
            else:
                self._integral_error = max(0.0, self._integral_error + error * dt_seconds)
        else:    
            self._integral_error += error * dt_seconds
        i_term = self._ki * self._integral_error
        output = p_term + i_term + d_term
        # --- MODIFIED: Direct Integral Anti-Windup (Conditional Accumulation) ---
        # Prevent integral accumulation if output exceeds limits and error pushes it further
        if self._ki != 0:
            if (output > self._output_max and error > 0) or \
               (output < self._output_min and error < 0):
                # If output is saturating and error is trying to increase integral further,
                # revert the integral accumulation from this cycle.
                self._integral_error -= error * dt_seconds
        # Re-calculate i_term after potential integral modification by anti-windup
        i_term = self._ki * self._integral_error
        output = p_term + i_term + d_term # Re-calculate total output
        self._last_error = error  
        limited = max(self._output_min, min(output, self._output_max))
        if self._verbose:
            if next(self._counter) % self._log_frequency == 0:
                self._log.info(Style.DIM + "dt={}µs; p={:.2f}, i={:.2f}, d={:.2f}, setpoint={:.2f}; output={:.2f} ({:.2f})".format(
                        dt_us, p_term, i_term, d_term, self._setpoint, output, limited))
        return limited

    def x_update(self, value, dt_us):
        dt_seconds = dt_us / 1_000_000.0 if dt_us > 0 else 0.000001
        error = self._setpoint - value
        # Proportional term
        p_term = self._kp * error
        # Derivative term
        d_term = self._kd * ((error - self._last_error) / dt_seconds) if dt_seconds > 0 else 0.0
        if self._setpoint == 0.0:
            if isclose(value, 0.0, abs_tol=self._epsilon_rpm_for_stop):
                self._integral_error = 0.0
            elif value > 0: # Motor moving forward (positive RPM)
                self._integral_error = min(0.0, self._integral_error + error * dt_seconds)
            else:
                self._integral_error = max(0.0, self._integral_error + error * dt_seconds)
        else:
            self._integral_error += error * dt_seconds
        # anti-windup
        if self._ki != 0:
            integral_component_for_max_output = (self._output_max - p_term - d_term) / self._ki
            integral_component_for_min_output = (self._output_min - p_term - d_term) / self._ki
            self._integral_error = max(min(self._integral_error, integral_component_for_max_output), integral_component_for_min_output)
        # Integral term
        i_term = self._ki * self._integral_error
        output = p_term + i_term + d_term
        self._last_error = error
        limited = max(self._output_min, min(output, self._output_max))
#       if abs(self._setpoint) > 0 and next(self._counter) % self._log_frequency == 0:
        if next(self._counter) % self._log_frequency == 0:
            self._log.info("dt={}µs; p={:.2f}, i={:.2f}, d={:.2f}, setpoint={:.2f}; output={:.2f} ({:.2f})".format(
                    dt_us, p_term, i_term, d_term, self._setpoint, output, limited))
        return limited

    def reset(self):
        self._last_error = 0.0
        self._integral_error = 0.0

#EOF
