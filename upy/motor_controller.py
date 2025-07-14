#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-04
# modified: 2025-07-09

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
from zero_crossing_handler import ZeroCrossingHandler
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
        self._log.info('initialising motor controller…')
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
        _zc_cfg          = config["kros"]["zero_crossing_handler"]
        _pwm_frequency   = _cfg['pwm_frequency']
        self._use_closed_loop = _cfg.get('use_closed_loop', True)
        _max_motor_speed = _cfg.get('max_motor_speed')

        self._motor_stop_pwm_threshold = 8

        self._enable_slew_limiter     = _slew_cfg['enabled']                  # True
        self._max_delta_rpm_per_sec   = _slew_cfg['max_delta_rpm_per_sec']    # closed loop: 120.0
        self._max_delta_speed_per_sec = _slew_cfg['max_delta_speed_per_sec']  # open loop: 100.0
        self._slew_limiters = {}

        self._enable_zero_crossing    = _zc_cfg.get('enabled', False)
        self._zero_crossing_handlers  = {}
        self._zc_awaiter_tasks        = {}

        if self._use_closed_loop:
            # closed-loop flag and PID related attributes
            self._pid_controllers = {}     # PID instances
            self._motor_target_rpms = {}   # target RPM for each motor
            self._pid_gains = _cfg.get('pid_gains', {'Kp': 0.5, 'Ki': 0.01, 'Kd': 0.01}) # Default PID gains if not in config
            _pid_timer_number = _cfg['pid_timer_number']
            _pid_timer_freq   = _cfg['pid_timer_frequency']
            self._pid_timer = Timer(_pid_timer_number, freq=_pid_timer_freq)
            self._pid_signal_flag = ThreadSafeFlag()
            self._last_global_pid_cycle_time = utime.ticks_us() # NEW: Initialize last global update time
            self._log.info(Fore.MAGENTA + 'closed loop enabled: PID timer {} configured with frequency of {}Hz; timer: {}'.format(_pid_timer_number, _pid_timer_freq, self._pid_timer))
        else:
            self._log.info(Fore.MAGENTA + 'open-loop control enabled (PID disabled).')

        self._verbose    = False # _app_cfg["verbose"]
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
                    # instantiate ZCH if closed-loop and ZCH enabled
                    if self._enable_zero_crossing:
                        self._zero_crossing_handlers[index] = ZeroCrossingHandler(
                            motor_id=index,
                            config=config,
                            motor_instance=motor,
                            level=level
                        )
                # instantiate SlewLimiter for this motor
                if self._enable_slew_limiter:
                    self._slew_limiters[index] = SlewLimiter(
                        max_delta_per_sec=self._max_delta_rpm_per_sec if self._use_closed_loop else self._max_delta_speed_per_sec
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

    def enable(self):
        '''
        Enables the motor controller.
        '''
        if self.enabled:
            self._log.debug("motor controller already enabled.")
        else:
            self._log.info("enabling motor controller…")
            self._enabled = True
            for motor in self.motors:
                motor.enable()
            self._rpm_timer.callback(self._rpm_timer_callback)
            if self._use_closed_loop:
                self._pid_timer.callback(lambda t: self._pid_signal_flag.set())
                if self._enable_slew_limiter:
                    for motor_id, limiter in self._slew_limiters.items():
                        motor_instance = self._motors[motor_id]
                        limiter.enable(motor_instance.rpm)
                        self._log.info("motor {}: slew limiter enabled.".format(motor_id))
                if self._enable_zero_crossing:
                    for motor_id, handler in self._zero_crossing_handlers.items():
                        handler.reset()
                        self._log.info("motor {}: zero crossing handler enabled.".format(motor_id))
            self._pid_task = asyncio.create_task(self._run_pid_task())
            self.enable_rpm_logger()
            self._log.info("motor controller enabled.")

    def enable_rpm_logger(self, interval_ms: int = 1000):
        '''
        Starts the asynchronous RPM logger.
        '''
        if self._loop is None:
            self._loop = asyncio.get_event_loop()
        if not self._logging_enabled:
            self._log.debug("starting RPM logger with interval: {}ms.".format(interval_ms))
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

    def get_motor(self, index):
        '''
        Returns the corresponding motor.

        Args:
            index (int):  The motor number (0-3).
        '''
        if isinstance(index, int):
            return self._motors[index]
        raise ValueError('expected an int, not a {}'.format(type(index)))

    async def go(self, mode=Mode.STOP, speeds=None):
        '''
        Set the navigation mode and speeds for all motors,
        orchestrating between ZeroCrossingHandler and SlewLimiter.

        Args:
            mode:    the enumerated Mode, which includes a tuple multiplier against the speeds.
            speeds:  the four speeds of the motors, in order: pfwd, sfwd, paft, saft
        '''
        if not self._use_closed_loop:
            # For open-loop, just set speed directly (slew limiter handled in _set_motor_speed if enabled)
            transformed_speeds = MotorController._apply_mode(speeds, mode)
            # The original slew limiter logic in _set_motor_speed was commented out.
            # If slew limiting is desired for open-loop, it would need to be re-added here,
            # potentially using the new SlewLimiter methods.
            self._set_motor_speed(transformed_speeds)
            self._log.info('open-loop set speeds to {}'.format(transformed_speeds))
            return

        # closed-loop control path
        commanded_rpms_raw = MotorController._apply_mode(speeds, mode)
#       self._log.info('go: {}; speeds: {}; transform (raw): {}'.format(mode, speeds, commanded_rpms_raw))
        self._status.motors(commanded_rpms_raw) # Update status with raw command

        for i, commanded_target_rpm_raw in enumerate(commanded_rpms_raw):
            current_motor = self._motors.get(i)
            if not current_motor:
                continue # skip if motor is not enabled/configured

            # always store the raw, commanded target RPM. This is the ultimate target.
            # _run_pid_task will decide if ZCH, SlewLimiter, or this raw value is used.
            self._motor_target_rpms[i] = float(commanded_target_rpm_raw)

            zc_handler     = self._zero_crossing_handlers.get(i)
            slew_limiter   = self._slew_limiters.get(i)
            pid_controller = self._pid_controllers[i]

            if self._enable_zero_crossing and zc_handler:
                # ZCH makes the primary decision based on the raw command and actual RPM
                # handle_new_command now returns the ZC task object (or None)
                zc_transition_task = zc_handler.handle_new_command(
                    float(commanded_target_rpm_raw),  # Pass the raw, un-slew-limited command
                    current_motor.rpm,
                    pid_controller
                )
                if zc_transition_task: # if a ZC task was initiated or remains responsible
                    # ZCH is handling this transition; disables SlewLimiter for this motor
                    if self._enable_slew_limiter and slew_limiter:
                        slew_limiter.disable()
                        self._log.info("motor {}: ZC engaged, SlewLimiter disabled.".format(i))
                    # cancel any previous awaiter task for this motor
                    if i in self._zc_awaiter_tasks and self._zc_awaiter_tasks[i] and not self._zc_awaiter_tasks[i].done():
                        self._zc_awaiter_tasks[i].cancel()
                    # create a new awaiter task, directly passing the ZC transition task
                    self._zc_awaiter_tasks[i] = asyncio.create_task(
                        self._await_zc_completion_notifier(motor_id=i, zc_task_to_await=zc_transition_task)
                    )
                else:
                    # ZCH is not active for this command (e.g., was idle, or completed, or command didn't trigger it)
                    # SlewLimiter should be used if enabled.
                    if self._enable_slew_limiter and slew_limiter:
                        if not slew_limiter.enabled:
                            slew_limiter.enable(current_motor.rpm)
                            self._log.info("motor {}: ZC idle/complete, re-enabled SlewLimiter to {:.1f} RPM.".format(i, current_motor.rpm))
                        slew_limiter.set_target(float(commanded_target_rpm_raw))
                        self._log.debug("motor {}: SlewLimiter active, target set to {:.1f} RPM.".format(i, commanded_target_rpm_raw))
                    else:
                        self._log.info("motor {}: Neither ZC nor SL active, PID target is raw command: {:.1f} RPM.".format(i, commanded_target_rpm_raw))

            else: # ZCH is not enabled for this motor at all via config
                # only SlewLimiter (if enabled) or raw command is used.
                if self._enable_slew_limiter and slew_limiter:
                    if not slew_limiter.enabled:
                        slew_limiter.enable(current_motor.rpm)
                        self._log.info("motor {}: ZC disabled, re-enabled SlewLimiter to {:.1f} RPM.".format(i, current_motor.rpm))
                    slew_limiter.set_target(float(commanded_target_rpm_raw))
                    self._log.info("motor {}: ZC disabled, SL active, target set to {:.1f} RPM.".format(i, commanded_target_rpm_raw))
                else:
                    self._log.info("motor {}: Neither ZC nor SL enabled, PID target is raw command: {:.1f} RPM.".format(i, commanded_target_rpm_raw))
        self._log.debug('go: finished processing commands.')

    def stop(self):
        '''
        Stop all motors and cancel/disable/reset zero crossing handlers, slew limiters, etc.
        '''
        for motor in self.motors:
            motor.stop()
        if self._use_closed_loop:
            for pid_ctrl in self._pid_controllers.values():
                pid_ctrl.reset()
            if self._enable_zero_crossing:
                for handler in self._zero_crossing_handlers.values():
                    handler.reset()
                for task_id, task in list(self._zc_awaiter_tasks.items()):
                    if task and not task.done():
                        task.cancel()
                        del self._zc_awaiter_tasks[task_id]
            if self._enable_slew_limiter:
                for limiter in self._slew_limiters.values():
                    limiter.reset()
        if self._verbose:
            self._log.info("all motors stopped.")
        return True

    def disable(self):
        '''
        Stop and disable all motors, then disable the motor controller.
        '''
        if not self.enabled:
            self._log.debug("motor controller already disabled.")
        else:
            self._log.info('disabling motor controller…')
            self._enabled = False
            _ = self.stop()
            if self._use_closed_loop:
                self._pid_timer.callback(None)
                self._log.info("disabled PID timer callback.")
            if self._pid_task and not self._pid_task.done():
                self._pid_task.cancel()
                self._log.info("PID control task cancellation requested.")
            for motor in self.motors:
                motor.disable()
            if self._enable_slew_limiter:
                for limiter in self._slew_limiters.values():
                    limiter.disable()
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

    # support meethods ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _rpm_timer_callback(self, timer):
        '''
        This callback's primary role is to signal when PID updates are needed. The motor's
        RPM is continuously updated by the encoder's ISR and can be directly accessed via
        the rpm() property when needed by the PID controller.
        '''
        pass

    async def _run_pid_task(self):
        if not self.enabled: # TEMP
            raise RuntimeError('motor controller disabled.')
        self._log.debug("starting asynchronous PID control task, driven by hardware timer.")
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
                    zc_handler = self._zero_crossing_handlers.get(motor_id)
                    slew_limiter = self._slew_limiters.get(motor_id) # Get slew limiter instance

                    target_rpm_for_pid = 0.0
                    # orchestrate the target source for PID: ZCH (if active) > SlewLimiter (if enabled) > Raw Command
                    if zc_handler and zc_handler.is_active:
                        target_rpm_for_pid = zc_handler.get_effective_pid_target_rpm
                        self._log.debug("motor {}: ZC active, PID target from ZCH: {:.1f}".format(motor_id, target_rpm_for_pid))
                    elif slew_limiter and slew_limiter.enabled:
                        # if SlewLimiter is enabled, it should update its internal state
                        # and provide the current slew-limited target.
                        # The SlewLimiter's internal target has already been set in .go()
                        target_rpm_for_pid = slew_limiter.get_current_target()
                        self._log.debug("motor {}: SL active, PID target from SL: {:.1f}".format(motor_id, target_rpm_for_pid))
                    else:
                        # No ZC or SlewLimiter active, use the raw commanded target RPM
                        target_rpm_for_pid = self._motor_target_rpms.get(motor_id, 0.0)
                        self._log.debug("motor {}: Raw command, PID target: {:.1f}".format(motor_id, target_rpm_for_pid))

                    pid_ctrl.setpoint = target_rpm_for_pid
                    # call PID update
                    new_speed_percent_signed = pid_ctrl.update(motor.rpm, global_cycle_dt_us)
                    # Apply zero-speed deadband (retained logic)
                    if target_rpm_for_pid == 0.0:
                        if abs(new_speed_percent_signed) < self._motor_stop_pwm_threshold:
                            new_speed_percent_signed = 0.0
                    motor.speed = int(round(new_speed_percent_signed))

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

    def _get_motor_target_rpms(self, motor_id):
        '''
        Helper for logging, returns the currently active target RPM for a motor.
        '''
        if not self._use_closed_loop:
            return -1.0 # Not applicable for open loop
        zc_handler = self._zero_crossing_handlers.get(motor_id)
        slew_limiter = self._slew_limiters.get(motor_id)
        if zc_handler and zc_handler.is_active:
            return zc_handler.get_effective_pid_target_rpm
        elif slew_limiter and slew_limiter.enabled:
            return slew_limiter.get_current_target() # log the current slew-limited target
        else:
            return self._motor_target_rpms.get(motor_id, 0.0) # log the raw commanded target

    @staticmethod
    def _apply_mode(base_power, mode):
        return tuple(p * m for p, m in zip(base_power, mode.speeds))

    async def _await_zc_completion_notifier(self, motor_id, zc_task_to_await):
        '''
        Monitors the internal ZC transition task and makes sure it completes or is cancelled.
        Upon ZC completion, re-engages the SlewLimiter for that motor.
        '''
        try:
            # directly await the task passed in. We know it's not None because .go() checked.
            await zc_task_to_await 
            # retrieve the handler to log its final state if needed
            handler = self._zero_crossing_handlers.get(motor_id)
            if handler: 
                self._log.info("motor {}: ZC internal task completed/cancelled. ZCH is_active: {}".format(
                        motor_id, handler.is_active))
            else:
                self._log.info("motor {}: ZC internal task completed/cancelled (handler not found).".format(motor_id))
            # ZC task is done. Re-enable and reset SlewLimiter for this motor.
            if self._enable_slew_limiter and motor_id in self._slew_limiters:
                slew_limiter = self._slew_limiters[motor_id]
                motor_instance = self._motors[motor_id]
                # reset slew limiter to current actual RPM for smooth transition after ZC
                slew_limiter.enable(motor_instance.rpm)
                slew_limiter.set_target(self._motor_target_rpms[motor_id]) # Set its target to the last commanded RPM
                self._log.info("motor {}: Re-enabled SlewLimiter, reset to {:.1f} RPM, target {:.1f} RPM.".format(
                        motor_id, motor_instance.rpm, self._motor_target_rpms[motor_id]))
        except asyncio.CancelledError:
            self._log.debug(Fore.YELLOW + "motor {}: ZC completion notifier task cancelled.".format(motor_id))
        except Exception as e:
            self._log.error("error in ZC completion notifier for motor {}: {}".format(motor_id, e))
            # confirm SlewLimiter is correctly configured even after an error
            if self._enable_slew_limiter and motor_id in self._slew_limiters:
                slew_limiter = self._slew_limiters[motor_id]
                motor_instance = self._motors[motor_id]
                slew_limiter.enable(motor_instance.rpm)
                slew_limiter.set_target(self._motor_target_rpms[motor_id])


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

#EOF
