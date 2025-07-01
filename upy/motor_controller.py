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
from core.logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor

class MotorController:
    def __init__(self, config=None, motors_enabled=(True, True, True, True), level=Level.INFO):
        self._log = Logger('motor-ctrl', level=level)
        self._log.info('initialising Motor Controller…')
        if config is None:
            raise ValueError('no configuration provided.')
        self._enabled    = False
        self._motors     = {}
        self._motor_list = []
        _cfg             = config["kros"]["motor_controller"]
        _app_cfg         = config["kros"]["application"]
        _motor_cfg       = config["kros"]["motors"]
        _pwm_frequency   = _cfg['pwm_frequency']
        _enc_frequency   = _cfg['encoder_frequency']
        self._verbose    = _app_cfg["verbose"]
        self._log.info(Fore.MAGENTA + 'verbose: {}'.format(self._verbose))
        try:
            # RPM timer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _rpm_timer_number  = _cfg['rpm_timer_number']
            _rpm_timer_freq    = _cfg['rpm_timer_frequency']
            self._rpm_timer    = Timer(_rpm_timer_number, freq=_rpm_timer_freq)
            # RPM timer callback enabled by enable()
#           self._rpm_timer.callback(self._rpm_timer_callback)
            self._log.info(Fore.MAGENTA + 'configured Timer {} for RPM calculation at {}Hz'.format(_rpm_timer_number, _rpm_timer_freq))
            # Logging timer ┈┈┈┈┈┈┈┈┈┈┈┈
            _log_timer_number  = _cfg['log_timer_number']
            _log_timer_freq    = _cfg['log_timer_frequency']
            self._logging_task = None # store the logging task to manage it
            self._logging_enabled = False
            self._loop         = None # asyncio loop instance
            self._timer        = None # timer for logging
            self._asyncio_event_from_isr = asyncio.Event() # Event to signal asyncio from ISR
            self._needs_pid_update = False
            self._pid_task     = None
            # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            for index in range(4):
                if not motors_enabled[index]:
                    continue
                motor_key = "motor{}".format(index)
                self._log.info('configuring {}…'.format(motor_key))
                mcfg      = _motor_cfg[motor_key]
                pwm_timer = Timer(mcfg["pwm_timer"], freq=_pwm_frequency)
                enc_timer = Timer(mcfg["enc_timer"], freq=_enc_frequency)
                _id = mcfg["id"],
                motor = Motor(
                    id=_id,
                    pwm_timer=pwm_timer,
                    pwm_channel=mcfg["pwm_channel"],
                    pwm_pin=mcfg["pwm_pin"],
                    pwm_pin_name=mcfg["pwm_pin_name"],
                    direction_pin=mcfg["direction_pin"],
                    direction_pin_name=mcfg["direction_pin_name"],
                    encoder_pin=mcfg["encoder_pin"],
                    encoder_pin_name=mcfg["encoder_pin_name"],
                    enc_timer=enc_timer,
                    enc_channel=mcfg["enc_channel"],
                    reverse=mcfg["reverse"]
                )
                motor.speed = Motor.STOPPED
                self._motors[index] = motor
                self._motor_list.append(motor)
            self._log.debug('configuring additional timers…')
            # immediately stop all motors
            self.stop()
            self._log.info('ready.')

        except Exception as e:
            self._log.error('{} raised by motor controller constructor: {}'.format(type(e), e))
            sys.print_exception(e)

    def _rpm_timer_callback(self, timer):
        for motor in self.motors:
            motor._calculate_rpm()
            motor._pid_needs_update = True # flag for PID update in async task
        self._needs_pid_update = True

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

    async def _pid_control_coro(self, interval_ms: int = 10):
        """
        Coroutine to run PID control for all motors that need it.
        """
        self._log.info("PID control coroutine started, running every {}ms".format(interval_ms))
        try:
            while True:
                if self._needs_pid_update:
                    for motor in self.motors:
                        if motor._pid_needs_update:
                            # Do PID logic here
                            motor._pid_needs_update = False
                    self._needs_pid_update = False  # Reset controller flag
                await asyncio.sleep_ms(interval_ms)
        except asyncio.CancelledError:
            self._log.info("PID control coroutine cancelled.")

    def enable_rpm_logger(self, interval_ms: int = 1000):
        """
        Starts the asynchronous RPM logger.
        """
        if self._loop is None:
            self._loop = asyncio.get_event_loop()
        if not self._logging_enabled:
            self._log.info(Fore.MAGENTA + "starting RPM logger with interval: {}ms.".format(interval_ms))
            self._logging_task = self._loop.create_task(self._rpm_logger_coro(interval_ms))
            self._logging_enabled = True
        else:
            self._log.info("RPM logger already running.")

    def disable_rpm_logger(self):
        """
        Cancels the RPM logger task if running.
        """
        if self._logging_enabled and self._logging_task is not None:
            self._logging_task.cancel()
            self._logging_task = None
            self._logging_enabled = False
            self._log.info("RPM logger stopped.")
        else:
            self._log.info("RPM logger was not running.")

    async def _rpm_logger_coro(self, interval_ms: int):
        """
        Periodically logs the current RPM and tick count for all motors.
        """
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
                        self._log.info(Fore.MAGENTA + "🍆 current RPM: {}".format(rpm_values))
                else:
                    self._log.warning("no motors configured for RPM logging.")
                await asyncio.sleep_ms(interval_ms)
        except asyncio.CancelledError:
            self._log.info("RPM logger task cancelled.")
        finally:
            self._log.info(Fore.MAGENTA + "🍆 rpm_logger_coro done.")

    def enable(self):
        if self.enabled:
            self._log.warning("motor controller already enabled.")
        else:
            for motor in self.motors:
                motor.enable()
            self._enabled = True
            self._rpm_timer.callback(self._rpm_timer_callback)
            self.enable_rpm_logger() # already starts timer & logging
            if self._pid_task is None:
                self._pid_task = self._loop.create_task(self._pid_control_coro())
            self._log.info("motor controller enabled.")

    def disable(self):
        if self.enabled:
            self._log.warning("motor controller already disabled.")
        else:
            for motor in self.motors:
                motor.disable()
            self._rpm_timer.callback(None)
            self._encoder_channel.callback(None)
            if self._pid_task is not None:
                self._pid_task.cancel()
                self._pid_task = None
            self._enabled = False

    def get_motor(self, motor_num):
        if isinstance(motor_num, int):
            return self._motors[motor_num]
        raise ValueError('expected an int.')

    def set_motor_speed(self, motor_nums, speeds):
        '''
        Set speed for the motors.
        :param motor_nums: int or list/tuple of ints (motor numbers 0–3)
        :param speed: speed percentage (0–100)
        '''
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if self._verbose:
            self._log.info(Fore.WHITE + Style.BRIGHT + 'set motor(s) {} speed to {}'.format(motor_nums, speed))
        motor_nums = [motor_nums] if isinstance(motor_nums, int) else list(motor_nums)
        speeds = list(speeds)
#       if self._verbose:
        self._log.info(Fore.WHITE + 'set motor(s) {} speed to {}'.format(motor_nums, speeds))
        for motor, speed in zip(self.iter_valid_motors(motor_nums), speeds):
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
            motor.direction = direction

    def iter_valid_motors(self, motor_nums):
        if isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        return list(self._motors[m] for m in motor_nums if m in self._motors)

    async def accelerate(self, motor_nums, target_speed, step=1, delay_ms=50):
        '''
        Gradually change speed of one or more motors toward a target
        speed, starting from each motor's current speed.

        :param motor_nums:    int or list/tuple of ints (motor numbers 0–3)
        :param target_speed:  target speed (0–100)
        :param step:          speed increment per step
        :param delay_ms:      delay in milliseconds between steps
        '''
        if isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        motors = self.iter_valid_motors(motor_nums)
        done = False
        while not done:
            done = True
            for motor in motors:
                current = motor.speed
                if current == target_speed:
                    continue
                done = False
                delta = target_speed - current
                direction = 1 if delta > 0 else -1
                new_speed = current + direction * min(step, abs(delta))
                motor.speed = new_speed
            await asyncio.sleep_ms(delay_ms)

    async def decelerate_to_stop(self, motor_nums, step=1, delay_ms=50):
        '''
        Gradually slow one or more motors to a stop (speed = 100).
        :param motor_nums: int or list/tuple of ints
        '''
        if self._verbose:
            self._log.info("decelerating motor(s) {} to stop...".format(motor_nums))
        await self.accelerate(motor_nums, target_speed=Motor.STOPPED, step=step, delay_ms=delay_ms)

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

    def disable(self):
        self.stop()
        self._enabled = False

    def close(self):
        self.disable()
        self._log.info("closed.")

#EOF
