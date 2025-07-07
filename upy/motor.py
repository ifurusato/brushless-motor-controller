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
                 max_speed=None, reverse=False, log_level=Level.INFO):
        try:
            self._enabled = False
            self._id      = id
            self._name    = name
            self._log = Logger('motor-{}'.format(self._name), level=log_level)
            self._log.info('initialising motor {}'.format(self._name))
            self._tick_count       = 0
            self._pulse_intervals  = []
            self._rpm              = 0
            self._max_speed        = max_speed
            self._reverse          = reverse
            self._update_interval  = 0.1
            self._pulses_per_output_rev = 270  # pulses_per_motor_rev * gear_ratio
            self._no_tick_timeout_us = 70_000 # how much time we wait before declaring motor stopped (at 6rpm this would be 37,037)

            # Soft stop for direction change variables
            self._pending_direction_change = False
            self._commanded_pwm_value = 0
            self._commanded_direction_value = Motor.DIRECTION_FORWARD
            self._soft_stop_threshold_rpm = 10.0 # RPM below which motor is considered 'stopped enough' for direction change

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
                raise ChannelUnavailableError('motor IRQ already in use; hard reset required.')
            self._encoder_irq.disable()
            # info
            self._log.info('motor {} PWM timer: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_timer))
            self._log.info('motor {} channel: '.format(self._name) + Fore.GREEN + '{}'.format(self._pwm_channel))
            self._log.info('motor {} pins: PWM: '.format(self._name) + Fore.GREEN + '{}'.format(pwm_pin)
                                         + Fore.CYAN + '; direction:     ' + Fore.GREEN + '{}'.format(direction_pin)
                                         + Fore.CYAN + '; encoder:       ' + Fore.GREEN + '{}'.format(encoder_pin))
            self._log.info('motor {} encoder IRQ set on pin: '.format(self._name) + Fore.GREEN + '{}'.format(self._encoder_pin))
            self._log.info('maximum motor speed:  ' + Fore.GREEN + '{}'.format(max_speed))

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
    def x_direction(self, value):
        self._log.debug('motor {} set to {} direction.'.format(self._name, 'forward' if value == Motor.DIRECTION_FORWARD else 'reverse'))
        if self._reverse:
            value = value ^ 1
        if value == Motor.DIRECTION_FORWARD:
            self._direction_pin.value(Motor.DIRECTION_FORWARD)
        elif value == Motor.DIRECTION_REVERSE:
            self._direction_pin.value(Motor.DIRECTION_REVERSE)
        else:
            raise ValueError("invalid value for direction: '{}'",format(value))

    @direction.setter
    def direction(self, value):
        # store the current physical state of the pin *before* any potential change is applied.
        current_physical_pin_state = self.direction
        # determines the 'intended_physical_state' from the input 'value'
        # by applying the motor's '_reverse' configuration.
        if self._reverse:
            value = value ^ 1 # 'value' is now effectively the 'intended_physical_pin_state'
        if value == current_physical_pin_state:
            return
        self._log.info(Style.BRIGHT + '🍏 direction changed from {} to {}'.format(current_physical_pin_state, value))
        # Only proceed to change the pin and log if the intended physical state ('value')
        # is different from the current physical state of the pin.

        # Log the change. We use (value ^ self._reverse) to convert the physical 'value'
        # back to its logical representation for the log message, as the user understands it.
        self._log.info('Motor {} direction changed to {}. Physical pin {} -> {}.'.format(
            self._name, 'FORWARD' if (value ^ self._reverse) == Motor.DIRECTION_FORWARD else 'REVERSE',
            current_physical_pin_state, value
        ))
        if value == Motor.DIRECTION_FORWARD:
            self._direction_pin.value(Motor.DIRECTION_FORWARD)
        elif value == Motor.DIRECTION_REVERSE:
            self._direction_pin.value(Motor.DIRECTION_REVERSE)
        else:
            raise ValueError("invalid value for direction: '{}'".format(value))

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def z_speed(self, value):
        '''
        Set the motor speed using a PID output (can be negative for reverse).
        Implements soft stop for direction changes with a single-pin encoder.
        '''
        if not self.enabled:
            raise Exception('cannot set speed: motor {} not enabled.'.format(self._name))
        # Determine target magnitude and direction from the 'value' (PID output)
        target_magnitude = abs(value)
        target_direction_logic = Motor.DIRECTION_FORWARD if value >= 0 else Motor.DIRECTION_REVERSE
        # Ensure magnitude is within bounds of max_speed
        if not 0 <= target_magnitude <= self._max_speed:
            self._log.warning('Speed magnitude {} for motor {} clipped to max_speed {}.'.format(target_magnitude, self._name, self._max_speed))
            target_magnitude = min(target_magnitude, self._max_speed)
            # raise ValueError('speed magnitude must be between 0 and {}. Got: {}'.format(self._max_speed, target_magnitude))
        # Get current actual direction pin state for comparison
        current_actual_direction_pin_state = self._direction_pin.value()
        # Account for potential wiring reversal in the motor class setup
        current_logical_direction = current_actual_direction_pin_state
        if self._reverse:
            current_logical_direction = current_logical_direction ^ 1
        # Check if a direction change is requested (i.e., target direction differs from current *logical* direction)
        direction_mismatch = (current_logical_direction != target_direction_logic)
        # --- Soft Stop Logic ---
        if direction_mismatch and not self._pending_direction_change:
            # Stage 1: Initiate a soft stop
            self._pending_direction_change = True
            self._commanded_direction_value = target_direction_logic # Store the intended new direction
            self._commanded_pwm_value = target_magnitude # Store the intended new magnitude
            self._pwm_channel.pulse_width_percent(0) # Command immediate stop (0 PWM)
            self._log.info('Motor {} initiating soft stop for direction change. Target direction: {}.'.format(
                    self._name, 'FORWARD' if self._commanded_direction_value == Motor.DIRECTION_FORWARD else 'REVERSE'))
            return # exit, no further action in this cycle, wait for motor to stop

        elif self._pending_direction_change:
            # Stage 2: Continue soft stopping until RPM is below threshold
            current_rpm_magnitude = abs(self.rpm) # Get current measured RPM magnitude (always positive from .rpm)
            if current_rpm_magnitude < self._soft_stop_threshold_rpm:
                # Motor has stopped or is slow enough, complete the direction change
                self.direction = self._commanded_direction_value # Apply the new direction (this uses the setter logic)
                self._pending_direction_change = False # Clear pending flag
                self._log.info('Motor {} direction change completed to {}. Applying initial speed {:.1f}.'.format(
                        self._name, 'FORWARD' if self._commanded_direction_value == Motor.DIRECTION_FORWARD else 'REVERSE', self._commanded_pwm_value))
                # The PID loop will immediately take over and set the correct speed.
                # No need to apply _commanded_pwm_value as a direct PWM % here,
                # as it is an RPM target, not a PWM duty cycle.
                return # exit, direction change handled
            else:
                # Still stopping, keep commanding zero power
                self._pwm_channel.pulse_width_percent(0)
                self._log.debug('Motor {} still stopping (Current RPM: {:.1f}).'.format(self._name, current_rpm_magnitude))
                return # exit, still stopping

        # --- Normal Operation ---
        # Not in a direction change sequence, apply commanded speed and direction directly
        self._pending_direction_change = False # Ensure this is false if it somehow got stuck
        self.direction = target_direction_logic # Apply current direction (this uses the setter logic)
        self._speed = target_magnitude # Store the new speed value
        self._pwm_channel.pulse_width_percent(target_magnitude)
        self._log.debug('Motor {} speed set to {}% (PWM duty {}%)'.format(self._name, target_magnitude, target_magnitude))

    @speed.setter
    def speed(self, value):
        '''
        Set the motor speed as a percentage between 0 and 100,
        or when in closed loop mode the maximum motor speed in RPM.
        We also set the direction pin depending on the value.
        '''
        if not self.enabled:
            raise Exception('cannot set speed: motor {} not enabled.'.format(self._name))
        if value < 0:
            self.direction = Motor.DIRECTION_REVERSE
            value = abs(value)
        else:
            self.direction = Motor.DIRECTION_FORWARD
        if not 0 <= value <= self._max_speed:
            raise ValueError('speed must be between 0 and 100, not {}'.format(value))
        self._speed = value
        duty_percent = value
        # duty_percent = 100 - value # inverted PWM when not configured as Timer.PWM_INVERTED
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
        calculated_rpm_magnitude = revolutions_per_second * 60.0

        # Determine if the sign will be flipped based on the current direction pin state
        should_be_negative_due_to_pin = (self._direction_pin.value() == Motor.DIRECTION_REVERSE) ^ self._reverse
    
        if calculated_rpm_magnitude > 0 and should_be_negative_due_to_pin:
            if self.direction == Motor.DIRECTION_FORWARD: # Check if the *commanded* direction was FORWARD
                self._log.warning('ANOMALY: Motor {} RPM sign flip detected! Pin value {} (logical {}). Expected FORWARD. Raw magnitude={:.2f}'.format(
                    self._name, self._direction_pin.value(), (self._direction_pin.value() ^ self._reverse), calculated_rpm_magnitude
                ))
 
        if (self._direction_pin.value() == Motor.DIRECTION_REVERSE) ^ self._reverse:
            calculated_rpm_magnitude = -calculated_rpm_magnitude
        self._rpm = calculated_rpm_magnitude
        return self._rpm

    @property
    def x_rpm(self):
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
        calculated_rpm_magnitude = revolutions_per_second * 60.0
        if (self._direction_pin.value() == Motor.DIRECTION_REVERSE) ^ self._reverse:
            calculated_rpm_magnitude = -calculated_rpm_magnitude
        self._rpm = calculated_rpm_magnitude
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

class ChannelUnavailableError(Exception):
    '''
    Thrown when a required Timer channel is not available.
    '''
    def __init__(self, message):
        super().__init__(message)

#EOF
