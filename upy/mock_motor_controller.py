#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-24
# modified: 2025-06-24

from colorama import Fore, Style
from core.logger import Logger, Level

__VERBOSE = True

class MockMotor:
    STOPPED           = 0
    FULL_SPEED        = 100
    def __init__(self, index, verbose):
        self._log = Logger('motor-{}'.format(index), Level.INFO)
        self._index   = index
        self._verbose = verbose
        self._speed   = MockMotor.STOPPED # speed as 0-100%
        self._log.info('motor {} ready.'.format(index))

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        self._speed = value
        if self._verbose:
            self._log.debug('motor {} speed set to {}%'.format(self._index, value))

    def stop(self):
        self.speed = MockMotor.STOPPED
        if self._verbose:
            self._log.info('stopped.')

class MockMotorController:
    def __init__(self):
        self._log = Logger('motor-ctrl', Level.INFO)
        self.enabled     = False
        self._verbose    = __VERBOSE
        self._motors     = {}
        self._motor_list = []
        for index in range(4):
            motor = MockMotor(index, self._verbose)
            self._motors[index] = motor
            self._motor_list.append(motor)

        self._log.info('ready.')

    def enable(self):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'enable.')
        self.enabled = True

    def disable(self):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'disable.')
        self.enabled = False

    def stop(self):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'stop.')

    def rotate(self, values):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'rotate: {}'.format(values))

    def crab(self, values):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'crab: {}'.format(values))

    def close(self):
        if self._verbose:
            self._log.info(Fore.MAGENTA + 'close.')

    def set_motor_speed(self, motor_nums, speed):
        '''     
        Set speed for one or more motors.
        :param motor_nums: int or list/tuple of ints (motor numbers 0–3)
        :param speed: speed percentage (0–100)
        '''
        if self._verbose:
            self._log.info('set motor(s) {} speed to {}'.format(motor_nums, speed))
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        for motor in self.iter_valid_motors(motor_nums):
            if self._verbose:
                self._log.debug(Fore.MAGENTA + 'set motor {} speed to {}.'.format(motor, speed))
            motor.speed = speed
            
    def set_motor_direction(self, motor_nums, direction):
        '''
        Set direction for one or more motors.
        :param motor_nums: int, list/tuple of ints (motor numbers 0–3), or 'all'
        :param direction: 0 or 1
        '''
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if isinstance(motor_nums, str) and motor_nums == 'all':
            motor_nums = self.motor_ids
        elif isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        for motor in self.iter_valid_motors(motor_nums):
            if self._verbose:
                self._log.info(Fore.MAGENTA + 'set motor {} direction to {}.'.format(motor, direction))
            motor.direction = direction

    def iter_valid_motors(self, motor_nums):
        if isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        return list(self._motors[m] for m in motor_nums if m in self._motors)

#EOF
