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
        motor:     the instance of the Motor class
        pid:       the instance of the PID controller
        slew_limiter: optional, instance of SlewLimiter for rate limiting
    '''
    _ZC_TRANSITION_MAP = {
        ZeroCrossingState.IDLE:            {ZeroCrossingState.DECELERATING, ZeroCrossingState.IDLE},
        ZeroCrossingState.DECELERATING:    {ZeroCrossingState.CONFIRMING_STOP, ZeroCrossingState.IDLE},
        ZeroCrossingState.CONFIRMING_STOP: {ZeroCrossingState.STOPPED, ZeroCrossingState.IDLE},
        ZeroCrossingState.STOPPED:         {ZeroCrossingState.ACCELERATING, ZeroCrossingState.IDLE},
        ZeroCrossingState.ACCELERATING:    {ZeroCrossingState.COMPLETE, ZeroCrossingState.IDLE},
        ZeroCrossingState.COMPLETE:        {ZeroCrossingState.IDLE},
        ZeroCrossingState.ERROR:           {ZeroCrossingState.IDLE}
    }

    def __init__(self, motor, pid, slew_limiter=None, level=Level.INFO):
        self._log = Logger('zch', level=level)
        super().__init__(
            logger         = self._log,
            task_name      = 'zc-fsm-{}'.format(motor.id),
            state_class    = ZeroCrossingState,
            initial_state  = ZeroCrossingState.IDLE,
            transition_map = self._ZC_TRANSITION_MAP
        )
        self.motor        = motor
        self.pid          = pid
        self.slew_limiter = slew_limiter
        self.target_rpm   = 0
        self._log.info('ready.')

    def update(self, target_rpm):
        '''
        Called on each control loop tick.
        '''
        self.target_rpm = target_rpm
        current_rpm = self.motor.rpm
        target_direction = 1 if target_rpm > 0 else -1 if target_rpm < 0 else 0
        current_direction = 1 if current_rpm > 0 else -1 if current_rpm < 0 else 0
        state = self.state

        # FSM logic for state transitions
        if state == ZeroCrossingState.IDLE:
            self._log.info(Fore.YELLOW + 'ZCH idle.')
            if current_direction != target_direction and target_direction != 0:
                self.transition(ZeroCrossingState.DECELERATING)

        elif state == ZeroCrossingState.DECELERATING:
            self._log.info(Fore.YELLOW + 'ZCH decelerating.')
            # decelerate to zero (open/closed loop, slew limiter if present)
            if abs(current_rpm) <= self.pid.deadband:
                self.transition(ZeroCrossingState.CONFIRMING_STOP)

        elif state == ZeroCrossingState.CONFIRMING_STOP:
            self._log.info(Fore.YELLOW + 'ZCH confirming stop.')
            # confirm physical stop (PID stop threshold)
            if abs(current_rpm) <= self.pid._stop_threshold:
                self.transition(ZeroCrossingState.STOPPED)

        elif state == ZeroCrossingState.STOPPED:
            self._log.info(Fore.YELLOW + 'ZCH stopped.')
            if target_direction != 0:
                self.transition(ZeroCrossingState.ACCELERATING)

        elif state == ZeroCrossingState.ACCELERATING:
            self._log.info(Fore.YELLOW + 'ZCH accelerating.')
            # accelerate to final target RPM in new direction
            if (target_direction == current_direction
                    and abs(current_rpm - target_rpm) <= self.pid.deadband):
                self.transition(ZeroCrossingState.COMPLETE)

        elif state == ZeroCrossingState.COMPLETE:
            self._log.info(Fore.YELLOW + 'ZCH complete.')
            self.transition(ZeroCrossingState.IDLE)

        elif state == ZeroCrossingState.ERROR:
            self._log.info(Fore.YELLOW + 'ZCH error.')
            # error/abort handling; can expand as needed
            pass

        # if target direction changes abruptly during crossing, restart
        if (self.state != ZeroCrossingState.IDLE
                and current_direction != target_direction
                and target_direction != 0):
            self.transition(ZeroCrossingState.DECELERATING)

    def get_state(self):
        return self.state

    def handle_new_command(self, target_rpm, current_rpm, pid):
        '''
        Update handler on new command. Used to reset state machine or set new targets.
        NOTE: This method is called externally by MotorController.
        '''
        self.target_rpm = target_rpm
        self.update(target_rpm)

    @property
    def get_effective_pid_target_rpm(self):
        '''
        Returns the RPM setpoint for the PID controller, based on current FSM state.
        Uses SlewLimiter if present.
        '''
        state = self.state

        if self.slew_limiter:
            # When stopped or confirming stop, slew to zero
            if state in (ZeroCrossingState.STOPPED, ZeroCrossingState.CONFIRMING_STOP):
                return self.slew_limiter.limit(0.0)
            # Otherwise, slew to target RPM
            else:
                return self.slew_limiter.limit(self.target_rpm)
        else:
            if state in (ZeroCrossingState.STOPPED, ZeroCrossingState.CONFIRMING_STOP):
                return 0.0
            else:
                return self.target_rpm

    @property
    def is_active(self):
        '''
        Returns True if ZCH is active (not idle or complete).
        '''
        return self.state not in (ZeroCrossingState.IDLE, ZeroCrossingState.COMPLETE)

#EOF
