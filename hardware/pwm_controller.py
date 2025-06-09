#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-08
# modified: 2025-06-09
#

from abc import ABC, abstractmethod
import pigpio
from core.logger import Logger, Level

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class PWMController(ABC):
    '''
    Abstract base class for motor PWM controllers.
    All PWM controllers must implement set_pwm and stop_pwm.
    '''
    STOPPED    = 1_000_000  # 100% duty (inverted logic)
    FULL_SPEED = 0          # 0% duty (inverted logic)

    @abstractmethod
    def set_pwm(self, speed_percent):
        pass

    @abstractmethod
    def stop_pwm(self):
        pass

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class HardwarePWMController(PWMController):
    '''
    A hardware PWM controller, using one of the Raspberry Pi specific hardware PWM pins.
    '''
    def __init__(self, pi, pwm_pin, pwm_freq, level=Level.INFO):
        self._log = Logger('hw-pwm-ctrl', level)
        self._pi = pi
        self._pwm_pin = pwm_pin
        self._pwm_freq = pwm_freq
        self._pi.set_mode(self._pwm_pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(self._pwm_pin, self._pwm_freq)
        self._log.info('ready.')

    def set_pwm(self, speed_percent):
        speed_percent = max(0, min(speed_percent, 100))
        duty_cycle = int((100 - speed_percent) * 10_000)  # inverted logic
        self._pi.hardware_PWM(self._pwm_pin, self._pwm_freq, duty_cycle)

    def stop_pwm(self):
        self._pi.hardware_PWM(self._pwm_pin, self._pwm_freq, self.STOPPED)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class SoftwarePWMController(PWMController):
    '''
    A software PWM controller, using one of the Raspberry Pi GPIO pins.
    '''
    def __init__(self, pi, pwm_pin, pwm_freq, level=Level.INFO):
        self._log = Logger('sw-pwm-ctrl', level)
        self._pi = pi
        self._pwm_pin = pwm_pin
        self._pwm_freq = pwm_freq
        self._pi.set_mode(self._pwm_pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(self._pwm_pin, self._pwm_freq)
        self._pi.set_PWM_range(self._pwm_pin, 255)  # match pigpio default range
        self._log.info('ready.')

    def set_pwm(self, speed_percent):
        speed_percent = max(0, min(speed_percent, 100))
        duty_cycle = int((100 - speed_percent) * 255 / 100)  # inverted logic
        self._pi.set_PWM_dutycycle(self._pwm_pin, duty_cycle)

    def stop_pwm(self):
        self._pi.set_PWM_dutycycle(self._pwm_pin, 255)  # full stop (100% duty)

#EOF
