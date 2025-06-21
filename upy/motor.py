#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-06-21
#

import uasyncio as asyncio
import time
from pyb import Pin, Timer
from core.logger import Logger, Level
from colorama import Fore, Style

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Motor:
    STOPPED           = 0
    FULL_SPEED        = 100
    DIRECTION_FORWARD = 1
    DIRECTION_REVERSE = 0
    def __init__(self, name, pwm_timer, pwm_channel,
            pwm_pin, pwm_pin_name=None,
            direction_pin=None, direction_pin_name=None,
            encoder_pin=None, encoder_pin_name=None,
            enc_timer=None, enc_channel=None,
            reverse=False, log_level=Level.INFO):
        try:
            self._enabled = False
            self._name = name
            self._log = Logger('motor-{}'.format(name), level=log_level)
            self._log.info('initialising motor {}'.format(name))
            self._tick_count      = 0
            self._last_capture    = None
            self._prev_capture    = None
            self._last_interval   = None
            self._interval        = None
            self._pulse_intervals = []
            self._rpm             = 0
            self._reverse         = reverse
            self._update_interval = 0.1
            self._pulses_per_output_rev = 270 # 569 # self._pulses_per_motor_rev * self._gear_ratio
            # setup PWM channel with pin
            self._pwm_pin = Pin(pwm_pin)
            self._pwm_pin_name = pwm_pin_name or str(self._pwm_pin)
            self._pwm_channel = pwm_timer.channel(pwm_channel, Timer.PWM_INVERTED, pin=self._pwm_pin)
            self._speed = self.STOPPED # speed as 0-100%
            # setup direction GPIO pin
            self._direction_pin = Pin(direction_pin, Pin.OUT)
            self._direction_pin.value(1)
            self._direction_pin_name = direction_pin_name or str(self._direction_pin)
            # setup input capture and callback
            self._encoder_pin = Pin(encoder_pin)
            self._encoder_pin_name = encoder_pin_name or str(self._encoder_pin)
            self._encoder_channel  = enc_timer.channel(enc_channel, Timer.IC, pin=self._encoder_pin, polarity=Timer.RISING) # or Timer.BOTH
            self._timer_freq = enc_timer.freq()
            # info
            self._log.info('Motor {}: PWM timer:  '.format(name) + Fore.GREEN + '{}'.format(pwm_timer))
            self._log.info('Motor {} channel:     '.format(name) + Fore.GREEN + '{}'.format(self._pwm_channel))
            self._log.info('Motor {} pins: PWM:   '.format(name) + Fore.GREEN + '{}'.format(pwm_pin) 
                    + Fore.CYAN + '; direction:   ' + Fore.GREEN + '{}'.format(direction_pin)
                    + Fore.CYAN + '; encoder:     ' + Fore.GREEN + '{}'.format(encoder_pin))
            self._log.info('Motor {} enc channel: '.format(name) + Fore.GREEN + '{}'.format(self._encoder_channel) 
                    + Fore.CYAN + '; frequency:   ' + Fore.GREEN + '{}Hz.'.format(self._timer_freq))
            self._log.info('motor {} ready.'.format(name))
        except Exception as e:
            self._log.error('{} raised by motor: {}'.format(type(e), e))
            raise

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        return self._name

    @property
    def enabled(self):
        return self._enabled

    @property
    def pwm_pin(self):
        return self._pwm_pin

    @property
    def pwm_pin_name(self):
        return self._pwm_pin_name

    @property
    def direction_pin(self):
        return self._direction_pin

    @property
    def direction_pin_name(self):
        return self._direction_pin_name

    @property
    def encoder_pin(self):
        return self._encoder_pin

    @property
    def encoder_pin_name(self):
        return self._encoder_pin_name

    @property
    def tick_count(self):
        return self._tick_count

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        '''
        Set the motor speed as a percentage between 0 and 100.
        '''
        if not 0 <= value <= 100:
            raise ValueError('speed must be between 0 and 100')
        self._speed = value
        duty_percent = value # inverted PWM
#       duty_percent = 100 - value # inverted PWM
        self._pwm_channel.pulse_width_percent(duty_percent)
        self._log.debug('motor {} speed set to {}% (PWM duty {}%)'.format(self._name, value, duty_percent))

    @property
    def direction(self):
        return self._direction_pin.value()

    @direction.setter
    def direction(self, value):
        self._log.debug('motor {} set to {} direction.'.format(self._name, 'forward' if value == Motor.DIRECTION_FORWARD else 'reverse'))
        if self._reverse:
            value = value ^ 1
        if value == Motor.DIRECTION_FORWARD:
            self._direction_pin.value(Motor.DIRECTION_FORWARD)
        elif value == Motor.DIRECTION_REVERSE:
            self._direction_pin.value(Motor.DIRECTION_REVERSE)
        else:
            raise ValueError("invalid value for direction: '{}'",format(value))

    @property
    def rpm(self):
        if self._interval is None:
            return 0.0
        interval_sec = self._interval / self._timer_freq
        rps = 1.0 / (interval_sec * self._pulses_per_output_rev)
        return rps * 60.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _encoder_callback(self, arg):
        current_capture = self._encoder_channel.capture()
        if current_capture is None:
            return
        if self.direction == Motor.DIRECTION_FORWARD:
            self._tick_count += 1
        else:
            self._tick_count -= 1
        self._last_capture = current_capture  # keep last capture for reference if needed

    def enable(self):
        if self.enabled:
            self._log.warning("motor already enabled.")
        else:
            self._log.info("adding encoder callback…")
            self._encoder_channel.callback(self._encoder_callback)
            self._log.info("motor enabled.")

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _calculate_rpm(self):
        if self._last_capture is None or self._prev_capture is None:
            self._prev_capture = self._last_capture
            self._interval = None
            return
        interval = self._last_capture - self._prev_capture
        if interval < 0:
            interval += 0x10000      # assuming 16-bit timer
#           interval += 0x100000000  # assuming 32-bit timer
        if interval == 0:
            self._interval = None
        else:
            self._interval = interval
        self._prev_capture = self._last_capture

    def measure_ticks_per_second(self, test_duration=1.0):
        '''
        Run the motor at full speed for `test_duration` seconds,
        then return the number of pulses counted during that period.
        '''
        self._tick_count = 0
        self.speed = self.FULL_SPEED  # full speed (0 if your scale is inverted)
        time.sleep(test_duration)    # wait for pulses to accumulate
        self.speed = self.STOPPED    # stop the motor
        ticks = self._tick_count
        ticks_per_sec = ticks / test_duration
        self._log.info(Fore.WHITE + Style.BRIGHT + "measured {} pulses in {}s -> {} ticks/sec".format(ticks, test_duration, ticks_per_sec))
        return ticks_per_sec

    def stop(self):
        self.speed = Motor.STOPPED
        self._log.info('stopped.')

#EOF
