#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-07-05

import utime
from pyb import Pin, ExtInt, Timer
from colorama import Fore, Style

from logger import Logger, Level

class Motor:
    STOPPED           = 0
    FULL_SPEED        = 100
    DIRECTION_FORWARD = 1
    DIRECTION_REVERSE = 0

    def __init__(self, id, name, pwm_timer, pwm_channel,
                 pwm_pin, pwm_pin_name=None,
                 direction_pin=None, direction_pin_name=None,
                 encoder_pin=None, encoder_pin_name=None,
                 reverse=False, log_level=Level.INFO):
        try:
            self._enabled = False
            self._id      = id
            self._name    = name
            self._log = Logger('motor-{}'.format(self._name), level=log_level)
            self._log.info('initialising motor {}'.format(self._name))
            self._tick_count       = 0
            self._pulse_intervals  = []
            self._rpm              = 0
            self._reverse          = reverse
            self._update_interval  = 0.1
            self._pulses_per_output_rev = 270  # pulses_per_motor_rev * gear_ratio
            self._no_tick_timeout_us = 70_000 # how much time we wait before declaring motor stopped (at 6rpm this would be 37,037)
            # setup PWM channel with pin
            self._pwm_pin = Pin(pwm_pin)
            self._pwm_pin_name = pwm_pin_name
            self._pwm_channel = pwm_timer.channel(pwm_channel, Timer.PWM_INVERTED, pin=self._pwm_pin)
            self._speed = self.STOPPED # speed as 0-100%
            self._pwm_channel.pulse_width_percent(0) # Ensure motor starts off
            # setup direction GPIO pin
            self._direction_pin = Pin(direction_pin, Pin.OUT)
            self._direction_pin.value(1)
            self._direction_pin_name = direction_pin_name
            # setup encoder pin
            self._encoder_pin = Pin(encoder_pin, Pin.IN, Pin.PULL_UP)
            self._encoder_pin_name = encoder_pin_name
            # variables for utime-based interval and debouncing (NEW)
            self._last_encoder_pulse_us = utime.ticks_us() # Initialize with current time for first pulse
            self._debounce_period_us = 500  # 0.5 milliseconds debounce period. Adjust if needed.
            self._last_calculated_interval_us = 0 # Stores the interval for RPM calculation in main loop
#           self._encoder_pin.irq(trigger=Pin.IRQ_FALLING, handler=self._encoder_callback)
            try:
                self._encoder_irq = ExtInt(self._encoder_pin, ExtInt.IRQ_FALLING, Pin.PULL_UP, self._encoder_callback)
            except ValueError as e: # e.g., ExtInt vector 6 is already in use
                self._log.error('motor IRQ already in use; ' + Style.BRIGHT + 'hard reset required.')
                raise Exception('motor IRQ already in use; hard reset required.')
            self._encoder_irq.disable()
            # info
            self._log.info('motor {}: PWM timer: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_timer))
            self._log.info('motor {} channel: '.format(self._name) + Fore.GREEN + '{}'.format(self._pwm_channel))
            self._log.info('motor {} pins: PWM: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_pin)
                                         + Fore.CYAN + '; direction:     ' + Fore.GREEN + '{}'.format(direction_pin)
                                         + Fore.CYAN + '; encoder:       ' + Fore.GREEN + '{}'.format(encoder_pin))
            self._log.info('motor {} encoder IRQ set on pin: '.format(self._name) + Fore.GREEN + '{}'.format(self._encoder_pin))

            self._log.info('motor {} ready.'.format(self._name))
        except Exception as e:
            self._log.error('{} raised by motor: {}'.format(type(e), e))
            raise

    def _pin_irq_callback(self, arg):
        self._irq_count += 1
        print(self._irq_count)

    @property
    def id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def enabled(self):
        return self._enabled

    @property
    def tick_count(self):
        return self._tick_count

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
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        '''
        Set the motor speed as a percentage between 0 and 100.
        We also set the direction pin depending on the value.
        '''
        if not self.enabled:
            raise Exception('cannot set speed: motor {} not enabled.'.format(self._name))
        if value < 0:
            self.direction = Motor.DIRECTION_REVERSE
            value = abs(value)
        else:
            self.direction = Motor.DIRECTION_FORWARD
        if not 0 <= value <= 100:
            raise ValueError('speed must be between 0 and 100')
        self._speed = value
        duty_percent = value
#       duty_percent = 100 - value # inverted PWM
        self._pwm_channel.pulse_width_percent(duty_percent)
        self._log.debug('motor {} speed set to {}% (PWM duty {}%)'.format(self._name, value, duty_percent))

    @property
    def rpm(self):
        '''
        Calculate the motor RPM and return the value as a property.
        '''
        if self._last_calculated_interval_us == 0 or \
                    utime.ticks_diff(utime.ticks_us(), self._last_encoder_pulse_us) > self._no_tick_timeout_us:
            self._last_calculated_interval_us = 0
            self._rpm = 0.0
            return self._rpm
        pulses_per_second = 1_000_000 / self._last_calculated_interval_us
        revolutions_per_second = pulses_per_second / self._pulses_per_output_rev
        calculated_rpm = revolutions_per_second * 60.0
#       if self.direction == Motor.DIRECTION_REVERSE:
#           calculated_rpm = -calculated_rpm
        self._rpm = calculated_rpm
        return self._rpm

    def _encoder_callback(self, pin):
        current_time_us = utime.ticks_us()
        last_pulse_time = self._last_encoder_pulse_us
        debounce_period = self._debounce_period_us
        raw_interval_us = utime.ticks_diff(current_time_us, last_pulse_time)
        if raw_interval_us >= debounce_period:
            self._last_encoder_pulse_us = current_time_us
            if self.direction == Motor.DIRECTION_FORWARD:
                self._tick_count += 1
            else:
#               self._tick_count += 1
                self._tick_count -= 1
            self._last_calculated_interval_us = raw_interval_us

    def stop(self):
        '''
        Stop the motor.
        '''
        self.speed = Motor.STOPPED
        self._log.info('stopped.')

    def enable(self):
        '''
        Enable the motor and its IRQ.
        '''
        if self.enabled:
            self._log.warning("motor already enabled.")
        else:
            self._enabled = True
            self._encoder_irq.enable()
            # anything?
            self._log.info("motor enabled.")

    def disable(self):
        '''
        Disable the motor and its IRQ.
        '''
        if self.enabled:
            self._enabled = False
            self._encoder_irq.disable()
            self._log.info("motor disabled.")
        else:
            self._log.warning("motor already disabled.")

    def close(self):
        '''
        Disable and close the motor, disable the callback.
        '''
        if self.enabled:
            self.disable()
        self._encoder_irq = None
        self._log.info('disabled.')

#EOF
