#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-06-26

import sys
import uasyncio as asyncio
import time
from pyb import Timer
from logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor
from mode import Mode

class MotorController:
    '''
    A controller for four brushless motors.

    Args:
        config:          The application-level configuration.
        status:          The Status indicator.
        motors_enabled:  Four flags enabling or disabling individual motors.
        level:           The log level.
    '''
    def __init__(self, config=None, status=None, motors_enabled=(True, True, True, True), level=Level.INFO):
        self._log = Logger('motor-ctrl', level=level)
        self._log.info('initialising Motor Controller…')
        if config is None:
            raise ValueError('no configuration provided.')
        self._enabled    = False
        self._status     = status
        self._motors     = {}
        self._motor_list = []
        self._motor_numbers = [0, 1, 2, 3]
        _cfg             = config["kros"]["motor_controller"]
        _app_cfg         = config["kros"]["application"]
        _motor_cfg       = config["kros"]["motors"]
        _pwm_frequency   = _cfg['pwm_frequency']

        _pid_timer_number = _cfg['pid_timer_number']
        _pid_timer_freq   = _cfg['pid_timer_frequency']
        self._pid_timer = Timer(_pid_timer_number, freq=_pid_timer_freq)
        self._log.info(Fore.MAGENTA + '🍆 PID timer {} configured with frequency of {}Hz; timer: {}'.format(_pid_timer_number, _pid_timer_freq, self._pid_timer))

        self._verbose    = True # _app_cfg["verbose"]
        self._log.info(Fore.MAGENTA + 'verbose: {}'.format(self._verbose))
        try:
            self._log.debug('configuring timers…')
            # RPM timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _rpm_timer_number  = _cfg['rpm_timer_number']
            _rpm_timer_freq    = _cfg['rpm_timer_frequency']
            self._rpm_timer    = Timer(_rpm_timer_number, freq=_rpm_timer_freq)
            self._log.info(Fore.MAGENTA + 'configured Timer {} for RPM calculation at {}Hz'.format(_rpm_timer_number, _rpm_timer_freq))
            # Logging timer ┈┈┈┈┈┈┈┈┈┈┈┈
            _log_timer_number  = _cfg['log_timer_number']
            _log_timer_freq    = _cfg['log_timer_frequency']
            self._logging_task = None # store the logging task to manage it
            self._logging_enabled = False
            self._loop         = None # asyncio loop instance
            self._needs_pid_update = False
            self._pid_task     = None
            # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            for index in range(4):
                if not motors_enabled[index]:
                    continue
                motor_key = "motor{}".format(index)
                self._log.info('configuring {}…'.format(motor_key))
                _m_cfg    = _motor_cfg[motor_key]
                pwm_timer = Timer(_m_cfg["pwm_timer"], freq=_pwm_frequency)
                motor = Motor(
                    id = _m_cfg["id"],
                    name = _m_cfg["name"],
                    pwm_timer = pwm_timer,
                    pwm_channel = _m_cfg["pwm_channel"],
                    pwm_pin = _m_cfg["pwm_pin"],
                    pwm_pin_name = _m_cfg["pwm_pin_name"],
                    direction_pin = _m_cfg["direction_pin"],
                    direction_pin_name = _m_cfg["direction_pin_name"],
                    encoder_pin = _m_cfg["encoder_pin"],
                    encoder_pin_name = _m_cfg["encoder_pin_name"],
                    reverse = _m_cfg["reverse"]
                )
                self._motors[index] = motor
                self._motor_list.append(motor)
            self._log.info('ready.')
        except Exception as e:
            self._log.error('{} raised by motor controller constructor: {}'.format(type(e), e))
            sys.print_exception(e)

    def _rpm_timer_callback(self, timer):
        '''
        This callback's primary role is to signal when PID updates are needed. The motor's
        RPM is continuously updated by the encoder's ISR and can be directly accessed via
        the rpm() property when needed by the PID controller.
        '''
        for motor in self.motors:
            motor._pid_needs_update = True   # set flag for PID update in an asynchronous task
        self._needs_pid_update = True        # set a global flag for the MotorController to manage PID

    @property
    def motor_ids(self):
        '''
        Return a list of motor IDs (keys) for all instantiated motors.
        '''
        return list(self._motors.keys())

    @property
    def motors(self):
        '''
        Return a list of all instantiated motors.
        '''
        return self._motor_list

    @property
    def enabled(self):
        return self._enabled

    def _pid_control_callback(self, arg):
        '''
        Callback to run PID control for all motors that need it.
        '''
        if self._needs_pid_update:
            for motor in self.motors:
                if motor._pid_needs_update:
                    # do PID logic here
                    motor._pid_needs_update = False
            self._needs_pid_update = False  # Reset controller flag

    def enable_rpm_logger(self, interval_ms: int = 1000):
        '''
        Starts the asynchronous RPM logger.
        '''
        if self._loop is None:
            self._loop = asyncio.get_event_loop()
        if not self._logging_enabled:
            self._log.info(Fore.MAGENTA + "starting RPM logger with interval: {}ms.".format(interval_ms))
            self._logging_task = self._loop.create_task(self._rpm_logger_coro(interval_ms))
            self._logging_enabled = True
        else:
            self._log.info("RPM logger already running.")

    def disable_rpm_logger(self):
        '''
        Cancels the RPM logger task if running.
        '''
        if self._logging_enabled and self._logging_task is not None:
            self._logging_task.cancel()
            self._logging_task = None
            self._logging_enabled = False
            self._log.info("RPM logger stopped.")
        else:
            self._log.info("RPM logger was not running.")

    async def _rpm_logger_coro(self, interval_ms: int):
        '''
        Periodically logs the current RPM and tick count for all motors.

        Args:
            interval_ms: The interval between calls to the logger.
        '''
        if self._verbose:
            self._log.info(Fore.MAGENTA + "called RPM logger coroa with interval: {}ms.".format(interval_ms))
        try:
            while self._logging_enabled:
                if self._motor_list:
                    rpm_values = ", ".join(
                        "{}: {:.2f} RPM; {} ticks".format(motor.name, motor.rpm, motor.tick_count)
                        for motor in self._motor_list
                    )
                    if self._verbose:
                        self._log.info(Fore.MAGENTA + "current RPM: {}".format(rpm_values))
                else:
                    self._log.warning("no motors configured for RPM logging.")
                await asyncio.sleep_ms(interval_ms)
        except asyncio.CancelledError:
            self._log.info("RPM logger task cancelled.")
        finally:
            self._log.info(Fore.MAGENTA + "rpm_logger_coro done.")

    def enable(self):
        '''
        Enables the motor controller.
        '''
        if self.enabled:
            self._log.warning(Style.DIM + "motor controller already enabled.")
        else:
            for motor in self.motors:
                motor.enable()
            self._enabled = True
            self._rpm_timer.callback(self._rpm_timer_callback)
            self._pid_timer.callback(self._pid_control_callback)
            self.enable_rpm_logger() # already starts timer & logging
            self._log.info("motor controller enabled.")

    def get_motor(self, index):
        '''
        Returns the corresponding motor.

        Args:
            index (int):  The motor number (0-3).
        '''
        if isinstance(index, int):
            return self._motors[index]
        raise ValueError('expected an int, not a {}'.format(type(index)))

    @staticmethod
    def _apply_mode(base_power, mode):
        return tuple(p * m for p, m in zip(base_power, mode.speeds))

    def go(self, mode=Mode.STOP, speeds=None):
        '''
        Set the navigation mode and speeds for all motors.

        Args:
            mode:  the enumeated Mode, which includes a tuple multiplier against the speeds.
            speeds:  the four speeds of the motors, in order: pfwd, sfwd, paft, saft
        '''
        transform = MotorController._apply_mode(speeds, mode)
        self._log.debug('go: {}; speeds: {}; type: {}; transform: {}'.format(mode, speeds, type(transform), transform))
        self._status.motors(transform)
        self._set_motor_speed(transform)

    def _set_motor_speed(self, speeds):
        '''
        An internal call to set the speeds for all motors.

        Args:
            speed: Sets motor speed as a percentage (0–100).
        '''
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if self._verbose:
            self._log.info('set speeds to {}'.format(speeds))
        for motor, speed in zip(self._motor_list, list(speeds)):
            motor.speed = speed

    def set_motor_direction(self, direction):
        '''
        Set the direction for all motors. This is generally not used during
        movement, as individual motor direction is set by the polarity of
        the speed value sent to each motor.

        Args:
            direction (int): Sets the direction of all motors as 0 (reverse) or 1 (forward).
        '''
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        for motor in self._motor_list:
            motor.direction = direction

    async def accelerate(self, target_speed, step=1, delay_ms=50):
        '''
        Gradually change speed of one or more motors toward a target
        speed, starting from each motor's current speed.

        Args:
            target_speed:  The target speed (0–100).
            step:          The speed increment per step.
            delay_ms:      The delay in milliseconds between steps.
        '''
        done = False
        while not done:
            done = True
            for motor in self._motor_list:
                current = motor.speed
                if current == target_speed:
                    continue
                done = False
                delta = target_speed - current
                direction = 1 if delta > 0 else -1
                new_speed = current + direction * min(step, abs(delta))
                motor.speed = new_speed
            await asyncio.sleep_ms(delay_ms)

    async def decelerate_to_stop(self, step=1, delay_ms=50):
        '''
        Gradually slow one or more motors to a stop (speed = 100).

        Args:
            step:          The speed increment per step.
            delay_ms:      The delay in milliseconds between steps.
        '''
        if self._verbose:
            self._log.info("decelerating motors to stop…")
        await self.accelerate(target_speed=Motor.STOPPED, step=step, delay_ms=delay_ms)

    def log_pin_configuration(self):
        '''
        Print pin mappings for each motor using clean, labeled pin names.
        '''
        self._log.info("Motor        PWM           Dir           Enc")
        col_widths = [12, 18, 18, 18]
        for num in sorted(self._motors):
            m = self._motors[num]
            line = "{}{:<{}} {:<{}} {:<{}} {:<{}}{}".format(
                Fore.CYAN, "Motor {}".format(num), col_widths[0],
                Fore.GREEN + m.pwm_pin_name, col_widths[1],
                Fore.GREEN + m.direction_pin_name, col_widths[2],
                Fore.GREEN + (m.encoder_pin_name or "N/A"), col_widths[3],
                Style.RESET_ALL
            )
            self._log.info(line)

    def stop(self):
        '''
        Stop all motors.
        '''
        for motor in self.motors:
            motor.stop()
        if self._verbose:
            self._log.info("all motors stopped.")
        return True

    def disable(self):
        '''
        Stop and disable all motors, then disable the motor controller.
        '''
        if self.enabled:
            self._log.warning("motor controller already disabled.")
        else:
            self._enabled = False
            _ = self.stop()
            for motor in self.motors:
                motor.disable()
            self._rpm_timer.callback(None)
            self._pid_timer.callback(None)
            self._encoder_channel.callback(None)
            if self._pid_task is not None:
                self._pid_task.cancel()
                self._pid_task = None

    def close(self):
        '''
        Stop and disable motors, their respective callbacks, and close the motor controller.
        '''
        self.disable()
        for motor in self.motors:
            motor.close()
        self._log.info("closed.")

#EOF
