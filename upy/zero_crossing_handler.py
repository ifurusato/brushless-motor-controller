#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-09
# modified: 2025-07-09

import utime
import uasyncio as asyncio
from colorama import Fore, Style

from logger import Logger, Level
from motor import Motor
from zc_state import ZeroCrossingState
from fsm import FiniteStateMachine

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class ZeroCrossingHandler(FiniteStateMachine):
    '''
    The Zero Crossing Handler is a state machine for a single motor, triggered
    whenever there is a difference in direction between the current and the 
    target direction. As an interruptable ballistic behaviour, the target speed
    will need to decelerate to zero and then accelerate to the target speed in 
    the other direction, smoothly managing this zero-crossing transition.

    This class must work in both open- and closed-loop mode, and work in
    cooperation with the SlewLimiter if it is active. Also, it must dynamically
    react to the motor's constantly-changing target speed.

    Args:
        motor_id:  the identifier for the motor (used for logging)
        config:    the application configuration
        motor:     the instance of the Motor class
        level:     the logging level
    '''
class ZeroCrossingHandler(FiniteStateMachine):
    _ZC_TRANSITION_MAP = {
        ZeroCrossingState.IDLE:            {ZeroCrossingState.DECELERATING, ZeroCrossingState.IDLE},
        ZeroCrossingState.DECELERATING:    {ZeroCrossingState.CONFIRMING_STOP, ZeroCrossingState.IDLE},
        ZeroCrossingState.CONFIRMING_STOP: {ZeroCrossingState.STOPPED, ZeroCrossingState.IDLE},
        ZeroCrossingState.STOPPED:         {ZeroCrossingState.ACCELERATING, ZeroCrossingState.IDLE},
        ZeroCrossingState.ACCELERATING:    {ZeroCrossingState.COMPLETE, ZeroCrossingState.IDLE},
        ZeroCrossingState.COMPLETE:        {ZeroCrossingState.IDLE},
        ZeroCrossingState.ERROR:           {ZeroCrossingState.IDLE}
    }

    def __init__(self, motor, pid, slew_limiter=None):
        self._log = Logger('zch', level=Level.INFO)
        super().__init__(
            logger=self._log,
            task_name='zc-fsm-{}'.format(motor.id),
            state_class=ZeroCrossingState,
            initial_state=ZeroCrossingState.IDLE,
            transition_map=self._ZC_TRANSITION_MAP
        )
        self.motor = motor
        self.pid = pid
        self.slew_limiter = slew_limiter
        self.target_rpm = 0

    def update(self, target_rpm):
        """Call this on each control loop tick."""
        self.target_rpm = target_rpm
        current_rpm = self.motor.rpm
        target_direction = 1 if target_rpm > 0 else -1 if target_rpm < 0 else 0
        current_direction = 1 if current_rpm > 0 else -1 if current_rpm < 0 else 0
        state = self.state

        # FSM logic for state transitions
        if state == ZeroCrossingState.IDLE:
            if current_direction != target_direction and target_direction != 0:
                self.transition(ZeroCrossingState.DECELERATING)

        elif state == ZeroCrossingState.DECELERATING:
            # Decelerate to zero (open/closed loop, slew limiter if present)
            if abs(current_rpm) <= self.pid.deadband:
                self.transition(ZeroCrossingState.CONFIRMING_STOP)

        elif state == ZeroCrossingState.CONFIRMING_STOP:
            # Confirm physical stop (PID stop threshold)
            if abs(current_rpm) <= self.pid._stop_threshold:
                self.transition(ZeroCrossingState.STOPPED)

        elif state == ZeroCrossingState.STOPPED:
            if target_direction != 0:
                self.transition(ZeroCrossingState.ACCELERATING)

        elif state == ZeroCrossingState.ACCELERATING:
            # Accelerate to final target RPM in new direction
            if (target_direction == current_direction and
                abs(current_rpm - target_rpm) <= self.pid.deadband):
                self.transition(ZeroCrossingState.COMPLETE)

        elif state == ZeroCrossingState.COMPLETE:
            self.transition(ZeroCrossingState.IDLE)

        elif state == ZeroCrossingState.ERROR:
            # Error/abort handling; can expand as needed
            pass

        # If target direction changes abruptly during crossing, restart
        if (self.state != ZeroCrossingState.IDLE and
            current_direction != target_direction and
            target_direction != 0):
            self.transition(ZeroCrossingState.DECELERATING)

    def get_state(self):
        return self.state

#EOF
