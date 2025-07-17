#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2025-06-05
# modified: 2025-07-09

import utime

class SlewLimiter:
    '''
    A slew limiter for a brushless motor, used to limit the rate of change of
    the motor's target speed, preventing sudden, abrupt changes that could
    be harmful to the motor and ensuring the changes to the motor's speed are
    gradual, more elegant, rather than instantaneous.
    '''
    def __init__(self, max_delta_per_sec):
        '''
        :param max_delta_per_sec:   Maximum allowed change per second (e.g. RPM/sec or %/sec)
        '''
        self._max_delta_per_sec = float(max_delta_per_sec)
        self._last_value    = 0.0
        self._last_time_ms  = utime.ticks_ms()
        self._target_value  = 0.0
        self._enabled       = False
        self.reset()

    def enable(self, initial_value=0.0):
        """
        Enables the slew limiter and resets its last value to the provided initial_value.
        This provides a smooth start from the current motor state.

        Args:
            initial_value (float): The value to initialize the limiter's current position to.
        """
        self._enabled = True
        self.reset(initial_value)
        
    def disable(self):
        """
        Disables the slew limiter. Its get_current_target() method will then
        immediately return the set target_value, bypassing limiting.
        """
        self._enabled = False

    @property
    def enabled(self):
        """
        Returns True if the slew limiter is currently enabled.
        """
        return self._enabled

    def set_target(self, value):
        """
        Sets the ultimate target value for the slew limiter.
        The actual output will be slewed towards this target via get_current_target().

        Args:
            value (float): The target value of the slew limiter (can be speed or RPM depending on mode).
        """
        if not self.enabled:
            raise RuntimeError('slew limited disabled.')
        self._target_value = float(value)

    def get_current_target(self):
        """
        Returns the current slew-limited value. This is what the PID should use.
        This method will also perform the limiting step based on elapsed time.
        """
        if not self.enabled:
            raise RuntimeError('slew limited disabled.')
        now_ms = utime.ticks_ms()
        if self._last_value is None or self._last_time_ms is None:
            self._last_value = self._target_value
            self._last_time_ms = now_ms
            return self._last_value
        elapsed_ms = utime.ticks_diff(now_ms, self._last_time_ms)
        if elapsed_ms <= 0: # avoid division by zero or negative time
            return self._last_value
        elapsed_seconds = elapsed_ms / 1000.0
        max_delta = self._max_delta_per_sec * elapsed_seconds
        delta = self._target_value - self._last_value
        if abs(delta) > max_delta:
            limited_delta = max_delta if delta > 0 else -max_delta
        else:
            limited_delta = delta
        new_value = self._last_value + limited_delta
        if (delta > 0 and new_value > self._target_value) or \
           (delta < 0 and new_value < self._target_value):
            new_value = self._target_value
        self._last_value = new_value
        self._last_time_ms = now_ms
        return new_value

    def reset(self, value=None):
        '''
        Resets the limiter to a specific value.
        This also resets the internal target value to this value.

        Args:
            value (float): The value to reset the limiter's internal state to.
                           If None, resets to 0.0.
        '''
        initial_value = float(value) if value is not None else 0.0
        self._last_value = initial_value
        self._last_time_ms = utime.ticks_ms()
        self._target_value = initial_value

#EOF
