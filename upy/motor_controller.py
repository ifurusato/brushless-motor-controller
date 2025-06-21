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

import sys
import uasyncio as asyncio
import time
from pyb import Timer
from core.logger import Logger, Level
from config_loader import ConfigLoader
from colorama import Fore, Style
from motor import Motor

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MotorController:
    def __init__(self, config=None, motors_enabled=(True, True, True, True), level=Level.INFO):
        self._log = Logger('motor-ctrl', level=level)
        self._log.info('initialising Motor Controller…')
        if config is None:
            raise ValueError('no configuration provided.')
        self._enabled    = False
        self._motors     = {}
        self._motor_list = []
        try:
#           config = ConfigLoader.configure('motor-config.yaml')
            _cfg = config["kros"]["motor_controller"]
            _motor_cfg         = config["kros"]["motors"]
            _pwm_frequency     = _cfg['pwm_frequency']
            _encoder_frequency = _cfg['encoder_frequency']

            # motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            for index in range(4):
                if not motors_enabled[index]:
                    continue
                motor_key = "motor{}".format(index)
                self._log.info('configuring {}…'.format(motor_key))
                mcfg      = _motor_cfg[motor_key]
                pwm_timer = Timer(mcfg["pwm_timer"], freq=_pwm_frequency)
                enc_timer = Timer(mcfg["enc_timer"], freq=_encoder_frequency)
                motor = Motor(
                    name=mcfg["name"],
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
            # immediately stop all motors
            self.stop()
            self._log.debug('configuring additional timers…')
            # timers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            # RPM timer
            _rpm_timer_number  = _cfg['rpm_timer_number']
            _rpm_timer_freq    = _cfg['rpm_timer_frequency']
            self._rpm_timer    = Timer(_rpm_timer_number, freq=_rpm_timer_freq)
            self._rpm_timer.callback(self._rpm_timer_callback)
            self._log.info(Fore.MAGENTA + 'configured Timer {} for RPM calculation at {}Hz'.format(_rpm_timer_number, _rpm_timer_freq))
            # Logging timer
            _log_timer_number  = _cfg['log_timer_number']
            _log_timer_freq    = _cfg['log_timer_frequency']
            self._logging_task = None # store the logging task to manage it
            self._loop         = None # asyncio loop instance
            self._timer        = None # timer for logging
            self._asyncio_event_from_isr = asyncio.Event() # Event to signal asyncio from ISR
            # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._log.info('ready.')

        except Exception as e:
            self._log.error('{} raised by motor controller constructor: {}'.format(type(e), e))
            sys.print_exception(e)

    def _rpm_timer_callback(self, timer):
        for motor in self.motors:
            motor._calculate_rpm()

    # properties ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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

    # logging ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def enable_rpm_logger(self, interval_ms: int = 1000, timer_freq_hz: int = 100):
        '''
        Enables the RPM logger. This method manages the logger task's
        lifecycle and the pyb.Timer that drives the asyncio loop.

        Args:
            interval_ms (int): The logging interval in milliseconds.
            timer_freq_hz (int): Frequency at which the internal timer ticks (e.g., 100 Hz = every 10ms).
                                 This determines how often asyncio tasks are given a chance to run.
        '''
        # If the asyncio loop hasn't been obtained yet, get it and start the ISR event processor.
        if self._loop is None:
            self._loop = asyncio.get_event_loop()
            self._log.info("Main: asyncio event loop obtained.")
            # Start the internal ISR event processor task
            self._loop.create_task(self._isr_event_processor_coro())
            self._log.info("Main: Internal ISR event processor task started.")
        if self._logging_task is None:
            self._log.info("Main: Creating RPM logger task...")
            self._logging_task = self._loop.create_task(self._rpm_logger_coro(interval_ms))
            self._log.info("Main: RPM logger task created.")
        else:
            self._log.info("Main: RPM logger is already running.")
        if self._timer is None: # Use self._timer
            self._log.info("Main: Starting Timer(6) at {} Hz to drive asyncio loop...".format(timer_freq_hz))
            self._timer = Timer(6, freq=timer_freq_hz) # Use self._timer
            self._timer.callback(self._timer_callback_isr) # Attach ISR to self._timer
            self._log.info("Main: Timer(6) started.")
        else:
            self._log.info("Main: Timer(6) is already running.")

    def disable_rpm_logger(self):
        '''
        Disables the RPM logger. This method manages stopping the logger task
        and the Timer that drives the asyncio loop for logging.
        '''
        if self._logging_task is not None:
            self._log.info("Main: Cancelling RPM logger task...")
            self._logging_task.cancel()
            self._logging_task = None
            self._log.info("Main: RPM logger task cancelled.")
        else:
            self._log.info("Main: RPM logger is already stopped.")

        if self._timer is not None: # Use self._timer
            self._log.info("Main: Stopping Timer(6)...")
            self._timer.callback(None) # Detach ISR from self._timer
            self._timer.deinit() # De-initialize self._timer
            self._timer = None # Clear reference
            self._log.info("Main: Timer(6) stopped.")
        else:
            self._log.info("Main: Timer(6) is already stopped.")

    async def _rpm_logger_coro(self, interval_ms: int):
        '''
        The asynchronous coroutine that performs the RPM logging.
        This runs within the asyncio event loop.
        It now formats the log message using the self._motor_list.
        '''
        self._log.info("logger task started, logging every {}ms".format(interval_ms))
        try:
            while True:
                if self._motor_list: # Iterate over the populated list
                    # log the RPM values
                    rpm_values = ", ".join(
                        "{}: {:.2f} RPM; {} ticks; ".format(motor.name, motor.rpm, motor.tick_count)
                        for motor in self.motors
                    )
                    self._log.info(Fore.MAGENTA + "current RPM: {}".format(rpm_values))
                else:
                    self._log.warning('no motors configured for RPM logging.')
                await asyncio.sleep_ms(interval_ms) # Wait asynchronously
        except asyncio.CancelledError:
            self._log.info(Style.DIM + "asyncio: logger task cancelled.")
        except Exception as e:
            self._log.error("Asyncio: Logger task error: {}".format(e))
        finally:
            self._log.info("Asyncio: Logger task finished.")

    def _timer_callback_isr(self, timer):
        '''
        This is the Interrupt Service Routine (ISR) for Timer 6.
        It signals the asyncio event, which is then handled by an async task.
        '''
        self._asyncio_event_from_isr.set()

    async def _isr_event_processor_coro(self):
        '''
        An internal async coroutine that continuously waits for the ISR event
        and ensures the asyncio loop gets a chance to run.
        This effectively acts as the 'tick' driven by the timer.
        '''
        while True:
            await self._asyncio_event_from_isr.wait()
            self._asyncio_event_from_isr.clear()
            await asyncio.sleep_ms(0) # Yield control
            if self._logging_task and self._logging_task.done():
                self._logging_task = None
            await asyncio.sleep_ms(0) # Briefly yield again

    def delay_and_tick(self, duration_ms=50, tick_interval_ms=10):
        '''
        A Helper for internal delays that also tick asyncio.

        Performs a delay in smaller chunks, calling self.tick() repeatedly.
        Used internally by blocking methods like accelerate().
        '''
        num_ticks = duration_ms // tick_interval_ms
        remaining_ms = duration_ms % tick_interval_ms
        for _ in range(num_ticks):
            time.sleep_ms(tick_interval_ms)
            self.tick() # give asyncio a chance to run
        if remaining_ms > 0:
            time.sleep_ms(remaining_ms)
            self.tick() # one final tick after the remainder

    def tick(self):
        '''
        Processes internal MotorController events, including allowing its
        asyncio tasks to run briefly. This method is synchronous.
        Your main application must call this periodically.
        '''
        if self._loop is not None:
            try:
                # Define a tiny async helper function internally that just yields
                async def _internal_tick_coro():
                    await asyncio.sleep_ms(0)
                # Now, run this helper coroutine until it completes.
                # This ensures run_until_complete receives a proper coroutine object.
                self._loop.run_until_complete(_internal_tick_coro())
            except Exception as e:
                self._log.error("Error processing internal asyncio events: {} - {}".format(type(e).__name__, e))
                sys.print_exception(e)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def enable(self):
        if self.enabled:
            self._log.warning("motor controller already enabled.")
        else:
            for motor in self.motors:
                motor.enable()
            self._enabled = True
            self._log.info("motor controller enabled.")

    def disable(self):
        if self.enabled:
            self._log.warning("motor controller already disabled.")
        else:
            for motor in self.motors:
                motor.disable()
            self._encoder_channel.callback(None)
        self._enabled = False

    def get_motor(self, motor_num):
        if isinstance(motor_num, int):
            return self._motors[motor_num]
        raise ValueError('expected an int.')

    def set_motor_speed(self, motor_nums, speed):
        '''
        Set speed for one or more motors.
        :param motor_nums: int or list/tuple of ints (motor numbers 0–3)
        :param speed: speed percentage (0–100)
        '''
        self._log.info(Fore.WHITE + Style.BRIGHT + 'set motor(s) {} speed to {}'.format(motor_nums, speed))
        if not self.enabled:
            raise RuntimeError('motor controller not enabled.')
        if isinstance(motor_nums, int):
            motor_nums = [motor_nums]
        for motor in self.iter_valid_motors(motor_nums):
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

    def accelerate(self, motor_nums, target_speed, step=1, delay_ms=50):
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
            self.delay_and_tick(delay_ms)

    def decelerate_to_stop(self, motor_nums, step=1, delay_ms=50):
        '''
        Gradually slow one or more motors to a stop (speed = 100).
        :param motor_nums: int or list/tuple of ints
        '''
        self._log.info("Decelerating motor(s) {} to stop...".format(motor_nums))
        self.accelerate(motor_nums, target_speed=Motor.STOPPED, step=step, delay_ms=delay_ms)

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
        self._log.info("all motors stopped.")

    def disable(self):
        self.stop()
        self._enabled = False

    def close(self):
        self.disable()
        self._log.info("closed.")

#EOF
