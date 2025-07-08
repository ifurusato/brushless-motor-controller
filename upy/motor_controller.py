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
from asyncio import ThreadSafeFlag
from math import isclose
import utime
from pyb import Timer
from logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor, ChannelUnavailableError
from slew_limiter import SlewLimiter
from pid import PID
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
        self._hard_reset_delay_sec = 3
        self._motors     = {}
        self._motor_list = []
        self._motor_numbers = [0, 1, 2, 3]
        _cfg             = config["kros"]["motor_controller"]
        _app_cfg         = config["kros"]["application"]
        _motor_cfg       = config["kros"]["motors"]
        _slew_cfg        = config["kros"]["slew_limiter"]
        _pwm_frequency   = _cfg['pwm_frequency']
        self._use_closed_loop = _cfg.get('use_closed_loop', True)
        _max_motor_speed = _cfg.get('max_motor_speed')

        self._motor_stop_pwm_threshold = 8
        self._soft_stop_rpm_threshold = _cfg.get('soft_stop_rpm_threshold', 5.0)

        self._enable_slew_limiter     = _slew_cfg['enabled']                  # True
        self._max_delta_rpm_per_sec   = _slew_cfg['max_delta_rpm_per_sec']    # closed loop: 120.0
        self._max_delta_speed_per_sec = _slew_cfg['max_delta_speed_per_sec']  # open loop: 100.0
        self._safe_slew_threshold     = _slew_cfg['safe_slew_threshold']      # 10.0
        self._slew_limiters = {}

        if self._use_closed_loop:
             # NEW: Closed-loop flag and PID related attributes
            self._pid_controllers = {}     # PID instances
            self._motor_target_rpms = {}   # target RPM for each motor
            self._pid_gains = _cfg.get('pid_gains', {'Kp': 0.5, 'Ki': 0.01, 'Kd': 0.01}) # Default PID gains if not in config
            _pid_timer_number = _cfg['pid_timer_number']
            _pid_timer_freq   = _cfg['pid_timer_frequency']
            self._pid_timer = Timer(_pid_timer_number, freq=_pid_timer_freq)

            self._pid_signal_flag = ThreadSafeFlag()
            self._last_global_pid_cycle_time = utime.ticks_us() # NEW: Initialize last global update time
 
            asyncio.create_task(self._run_pid_task())
            self._log.info(Fore.MAGENTA + 'closed loop enabled: PID timer {} configured with frequency of {}Hz; timer: {}'.format(_pid_timer_number, _pid_timer_freq, self._pid_timer))
        else:
            self._log.info(Fore.MAGENTA + 'open-loop control enabled (PID disabled).')

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
            # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            for index in range(4):
                if not motors_enabled[index]:
                    continue
                motor_key = "motor{}".format(index)
                self._log.info('configuring {}…'.format(motor_key))
                _m_cfg    = _motor_cfg[motor_key]
                pwm_timer = Timer(_m_cfg["pwm_timer"], freq=_pwm_frequency)
                _id = _m_cfg["id"]
                _name = _m_cfg["name"]
                motor = Motor(
                    id = _id,
                    name = _name,
                    pwm_timer = pwm_timer,
                    pwm_channel = _m_cfg["pwm_channel"],
                    pwm_pin = _m_cfg["pwm_pin"],
                    pwm_pin_name = _m_cfg["pwm_pin_name"],
                    direction_pin = _m_cfg["direction_pin"],
                    direction_pin_name = _m_cfg["direction_pin_name"],
                    encoder_pin = _m_cfg["encoder_pin"],
                    encoder_pin_name = _m_cfg["encoder_pin_name"],
                    max_speed = _max_motor_speed if self._use_closed_loop else 100.0,
                    reverse = _m_cfg["reverse"]
                )
                self._motors[index] = motor
                self._motor_list.append(motor)
                # instantiate PID controller for this motor if closed-loop enabled
                if self._use_closed_loop:
                    self._pid_controllers[index] = PID(_name, config, level=level)
                    self._motor_target_rpms[index] = 0.0 # initialize target RPM
                # instantiate SlewLimiter for this motor
                if self._enable_slew_limiter:
                    self._slew_limiters[index] = SlewLimiter(
                        max_delta_per_sec=self._max_delta_rpm_per_sec if self._use_closed_loop else self._max_delta_speed_per_sec,
                        safe_threshold=self._safe_slew_threshold
                    )

            self._log.info('ready.')
        except ChannelUnavailableError:
            # hard reset ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._log.fatal('cannot start: ' + Fore.RED + 'performing hard reset in {} seconds…'.format(self._hard_reset_delay_sec))
            import machine, time
            time.sleep(self._hard_reset_delay_sec)
            machine.reset()
        except Exception as e:
            self._log.error('{} raised by motor controller constructor: {}'.format(type(e), e))
            sys.print_exception(e)

    def _rpm_timer_callback(self, timer):
        '''
        This callback's primary role is to signal when PID updates are needed. The motor's
        RPM is continuously updated by the encoder's ISR and can be directly accessed via
        the rpm() property when needed by the PID controller.
        '''
        pass

    async def _run_pid_task(self):
        self._log.info("starting asynchronous PID control task, driven by hardware timer.")
        while True:
            await self._pid_signal_flag.wait()

            current_time = utime.ticks_us()
            # calculate dt_us based on the consistent global cycle time
            global_cycle_dt_us = utime.ticks_diff(current_time, self._last_global_pid_cycle_time)
            self._last_global_pid_cycle_time = current_time # update for next cycle

            for motor in self.motors:
                motor_id = motor.id
                if motor_id in self._pid_controllers:
                    pid_ctrl = self._pid_controllers[motor_id]
                    target_rpm_signed = self._motor_target_rpms.get(motor_id, 0.0)
                    current_motor_rpm = motor.rpm
                    pid_ctrl.setpoint = target_rpm_signed
                    # call PID update
                    new_speed_percent_signed = pid_ctrl.update(current_motor_rpm, global_cycle_dt_us)
                    # Apply zero-speed deadband (retained logic)
                    if target_rpm_signed == 0.0:
                        if abs(new_speed_percent_signed) < self._motor_stop_pwm_threshold:
                            new_speed_percent_signed = 0.0  
                    motor.speed = int(round(new_speed_percent_signed))

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

    def _get_motor_target_rpms(self, motor_id):
        if self._use_closed_loop:
            return self._motor_target_rpms[motor_id]
        else:
            return -1.0

    async def _rpm_logger_coro(self, interval_ms: int):
        '''
        Periodically logs the current RPM and tick count for all motors.

        Args:
            interval_ms: The interval between calls to the logger.
        '''
        if self._verbose:
            self._log.info(Fore.MAGENTA + "called RPM logger coro with interval: {}ms.".format(interval_ms))
        try:
            while self._logging_enabled:
                if self._motor_list:
                    rpm_values = ", ".join(

                        '{}: '.format(motor.name)
                            + Style.BRIGHT + '{:6.1f} RPM '.format(motor.rpm)
                            + ( Style.NORMAL + "(target: {}); {:5d} ticks".format(self._get_motor_target_rpms(motor.id), motor.tick_count)
                            if self._use_closed_loop else 
                                Style.NORMAL + "; {:5d} ticks".format(motor.tick_count)
                              )
#                       "{}: {:.2f} RPM (target: {}); {} ticks".format(
#                               motor.name,
#                               motor.rpm,
#                               self._get_motor_target_rpms(motor.id),
#                               motor.tick_count)
                        for motor in self._motor_list
                    )
                    if self._verbose:
                        self._log.info('set: ' + Fore.MAGENTA + "{}".format(rpm_values))
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
            if self._use_closed_loop:
                # Configure the _pid_timer to call our ISR callback, set the flag from within the lambda callback
                self._pid_timer.callback(lambda t: self._pid_signal_flag.set())
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
        if self._enable_slew_limiter:
            transform = list(transform) # convert to list for mutability
            for i in range(len(transform)):
                if i in self._slew_limiters:
                    transform[i] = self._slew_limiters[i].limit(transform[i])
            self._log.debug('slew-limited transform: {}'.format(transform))
        if self._use_closed_loop:
            for i, target_rpm in enumerate(transform):
                if i in self._motor_target_rpms:
                    self._motor_target_rpms[i] = float(target_rpm)
            self._log.debug('setting target RPMs to {}'.format(transform))
        else:
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

    async def accelerate(self, target_speed, step=1, delay_ms=50):
        if self._use_closed_loop:
            self._log.warning("`accelerate` with closed-loop is a simplified placeholder. Setting all motor target RPMs to `target_speed` (magnitude).")
            for motor_id in self._motor_target_rpms:
                self._motor_target_rpms[motor_id] = abs(float(target_speed))
        else:
            done = False
            while not done:
                done = True
                for motor in self._motor_list:
                    current_pwm = motor.speed
                    target_pwm = target_speed
                    
                    if current_pwm == target_pwm:
                        continue
                    done = False
                    
                    delta = target_pwm - current_pwm
                    direction_change_step = 1 if delta > 0 else -1
                    new_pwm = current_pwm + direction_change_step * min(step, abs(delta))
                    motor.speed = new_pwm
                await asyncio.sleep_ms(delay_ms)

    async def x_accelerate(self, target_speed, step=1, delay_ms=50):
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
        if self._use_closed_loop:
            for pid_ctrl in self._pid_controllers.values():
                pid_ctrl.reset()
        if self._verbose:
            self._log.info("all motors stopped.")
        return True

    def disable(self):
        '''
        Stop and disable all motors, then disable the motor controller.
        '''
        if not self.enabled:
            self._log.warning("motor controller already disabled.")
        else:
            self._enabled = False
            _ = self.stop()
            if self._use_closed_loop:
                self._pid_timer.callback(None)
            for motor in self.motors:
                motor.disable()
                if self._enable_slew_limiter and motor.id in self._slew_limiters:
                    self._slew_limiters[motor.id].reset()
            if self._use_closed_loop and motor.id in self._pid_controllers:
                self._pid_controllers[motor.id].reset()
            self._rpm_timer.callback(None)
            self._log.info("motor controller disabled.")

    def close(self):
        '''
        Stop and disable motors, their respective callbacks, and close the motor controller.
        '''
        self.disable()
        for motor in self.motors:
            motor.close()
        self._log.info("closed.")

#EOF
