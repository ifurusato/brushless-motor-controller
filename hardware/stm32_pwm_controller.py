#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-16
# modified: 2025-06-16
#

import time
import spidev
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from hardware.pwm_controller import PWMController
from hardware.tlc59711 import TLC59711
from hardware.controller_channel import ControllerChannel

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class STM32PWMController(PWMController):
    '''
    Implements the PWMController API using an STM32F405 Pyboard for motor control.
    '''
    def __init__(self, pi, config: dict, level=Level.INFO):
        '''
        Initialize the STM32PWMController.

        Note that the PWM signal for the motor is inverted: 0% duty cycle is
        full speed, 100% is stopped.

        :param pi:               pigpio pi instance
        :param level:            logging level (default INFO)
        '''
        self._log = Logger('tlc-pwm-ctrl', level=level)
        self._pi         = pi
        self._config     = config
        _cfg = config['kros'].get('hardware').get('motor_controller')
        self._verbose    = _cfg.get('verbose', True)
        self._stopped    = 65535 # 100% duty (motor stopped)
        self._full_speed = 0     # 0% duty (full speed)
        self._log.info('STM32 PWM controller ready.')

    def set_pwm(self, speed):
        '''
        Set the motor speed by adjusting the PWM duty cycle. Note that
        direction is not indicated by a negative number but changing
        the value of the direction pin, therefore it's not handled here.

        :param speed:   target speed as a percentage (0-100)
        '''
        if not (0 <= speed <= 100):
            raise ValueError('speed must be between 0 and 100.')
        pwm_value = int(self._full_speed + (self._stopped - self._full_speed) * (speed / 100))
        if self._verbose:
            self._log.info('set speed:' + Fore.GREEN + ' {:>6.2f}% '.format(speed) + Fore.CYAN + Style.DIM + ' -> PWM value: {}'.format(pwm_value))
        pass # TODO

    def stop_pwm(self):
        '''
        Stop the PWM output by setting all RGB values to zero (off).
        '''
        self._log.debug('stopping PWM…')
        pass # TODO

    def stop(self):
        '''
        Stop the motor by turning off the PWM signal.
        '''
        self.stop_pwm()
        self._log.info('motor stopped.')

#EOF
